# カメラなしで認識を作る手順

RealSense を使えるときに一度だけ撮っておき、あとはその記録と写真で認識のプログラムを作る手順です。カメラが1台しかなく、ロボットとカメラを使える時間が限られているので、カメラが必要な作業（手順1）は短く済ませ、残りはどの PC でもできるようにします。

| 手順 | やること | 必要なもの |
|---|---|---|
| 1 | rosbag で画像・深度・tf を記録する | RealSense（ロボットもあるとよい） |
| 2 | 記録を再生して写真（png）を保存する | 手順1の記録 |
| 3 | HSV で色ごとのしきい値を探す | 写真 |
| 4 | 輪郭からピースの位置と角度を求める | 写真 |
| 5 | 深度とカメラの情報で 3D の位置にする | 手順1の記録 |
| 6 | tf2 でロボットの座標にする | 手順1の記録 |

手順3・4は OpenCV だけで動くので、ROS もカメラもいりません。手順5・6はお手本の `color_detection.cpp` と同じことをします。

RealSense がないとできないのは、実際の照明でのしきい値の最終調整、カメラの校正（チェッカーボード）、ロボットと組み合わせた通しの確認です。

Gazebo はカメラの画像を作らないので、認識の練習には使えません（→ [software_design.md](software_design.md)）。

## 1. rosbag で記録する（カメラがあるとき）

### カメラ付きで起動する

実機を動かす手順（→ [real_robot.md](real_robot.md)）に `use_d435:=true` を付けます。カメラの画像と深度に加えて、ロボットから見たカメラの位置（tf）も記録できます。

```bash
ros2 launch crane_x7_examples demo.launch.py port_name:=/dev/ttyUSB0 use_d435:=true
```

ロボットを使えず、色の調整用の写真だけほしいときは、カメラだけ起動しても構いません（この場合 tf は記録されないので、手順6はできません）。

```bash
ros2 launch realsense2_camera rs_launch.py camera_namespace:='' align_depth.enable:=true
```

### 撮影姿勢にする

RViz でアームを動かし、カメラが机を真上から見る姿勢にします。撮影姿勢（#47）が決まっていればその姿勢にします。

映り方は別のターミナルで確認できます。`/camera/color/image_raw` を選んでください。

```bash
ros2 run rqt_image_view rqt_image_view
```

D435 は近すぎると深度が出ません。お手本の `color_detection.cpp` は、物体までの距離が 0.2〜0.5m のときだけ位置を出します。

### 記録する

机にピースを並べ、別のターミナルで実行します。

```bash
cd ~
ros2 bag record -o pieces_01 \
  /camera/color/image_raw /camera/color/camera_info \
  /camera/aligned_depth_to_color/image_raw /camera/aligned_depth_to_color/camera_info \
  /tf /tf_static /joint_states
```

5秒ほどたったら `Ctrl + C` で止めます。10秒で数百 MB になるので、短く撮ってください。

ピースの位置・向き・照明（部屋の明かり、窓際、影がかかる場所など）を変えて、`pieces_02`、`pieces_03` … と何回か撮ります。何を変えたかをメモしておくと、あとで比べやすくなります。

`pieces_01` などのフォルダは大きいので **GitHub には上げません**（`.gitignore` で除外しています）。Google Drive に置いて、リンクをチームに共有してください。

## 2. 記録を再生して写真を保存する

ロボットとカメラはいりません。Drive から記録のフォルダをダウンロードして、再生します。

```bash
ros2 bag play ~/pieces_01 --loop
```

別のターミナルで `rqt_image_view` を開き、`/camera/color/image_raw` を選びます。

```bash
ros2 run rqt_image_view rqt_image_view
```

画面の上にある保存ボタン（フロッピーの絵）を押すと、表示している画像を png で保存できます。いろいろなパターンの写真を何枚か作っておきます。

## 3. 色のしきい値を探す

次の内容を `~/hsv_tuner.py` として保存します。

```python
import sys

import cv2
import numpy as np

img = cv2.imread(sys.argv[1])
if img is None:
    sys.exit(f'画像を読み込めません: {sys.argv[1]}')
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

names = ['H_min', 'H_max', 'S_min', 'S_max', 'V_min', 'V_max']
init = [0, 179, 0, 255, 0, 255]
maxv = [179, 179, 255, 255, 255, 255]
cv2.namedWindow('mask')
for n, i, m in zip(names, init, maxv):
    cv2.createTrackbar(n, 'mask', i, m, lambda x: None)

while True:
    v = [cv2.getTrackbarPos(n, 'mask') for n in names]
    lower = np.array([v[0], v[2], v[4]])
    upper = np.array([v[1], v[3], v[5]])
    mask = cv2.inRange(hsv, lower, upper)
    cv2.imshow('image', img)
    cv2.imshow('mask', cv2.bitwise_and(img, img, mask=mask))
    key = cv2.waitKey(30) & 0xFF
    if key == ord('p'):
        print('lower =', lower.tolist(), 'upper =', upper.tolist())
    elif key == ord('q'):
        break
cv2.destroyAllWindows()
```

保存した写真を指定して実行します。

```bash
python3 ~/hsv_tuner.py ~/写真の名前.png
```

1. スライダーを動かして、`mask` の画面に **1色のピースだけが残る** ようにする
2. ちょうどよくなったら `p` を押す。ターミナルに値が出るのでメモする
3. 水色・緑・オレンジの3色で繰り返す。終わったら `q` で閉じる
4. **別の写真でも同じ値で切り出せるか** を確かめる（照明が変わっても大丈夫か）

OpenCV の H は 0〜179 です。目安は次のとおりです。

| 色（ピース） | H の目安 |
|---|---|
| オレンジ（L） | 10〜25 |
| 緑（S） | 45〜75 |
| 水色（I） | 85〜105 |

S と V は、下限を 80 くらいにすると机や影が消えやすくなります。

スマホやノート PC のカメラで撮った写真でも練習はできますが、RealSense とは色の出方が違うので、最後の調整は RealSense の写真で行います。

## 4. 輪郭からピースの位置と角度を求める

手順3で決めた値を使い、色ごとに切り出したあと、輪郭から中心と角度を求めます。次の内容を `~/detect_pieces_image.py` として保存し、`COLORS` の値を手順3の値に書き換えます。

```python
import sys

import cv2
import numpy as np

# hsv_tuner.py で探した値に書き換える
COLORS = {
    'I': ([85, 80, 80], [105, 255, 255]),   # 水色
    'S': ([45, 80, 80], [75, 255, 255]),    # 緑
    'L': ([10, 80, 80], [25, 255, 255]),    # オレンジ
}
MIN_AREA = 500  # これより小さい輪郭はゴミとして捨てる（画素数）

img = cv2.imread(sys.argv[1])
if img is None:
    sys.exit(f'画像を読み込めません: {sys.argv[1]}')
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
kernel = np.ones((5, 5), np.uint8)

for kind, (lower, upper) in COLORS.items():
    mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)   # 小さな点を消す
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # 小さな穴を埋める
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in contours:
        if cv2.contourArea(c) < MIN_AREA:
            continue
        m = cv2.moments(c)
        u, v = int(m['m10'] / m['m00']), int(m['m01'] / m['m00'])
        _, _, angle = cv2.minAreaRect(c)
        print(f'{kind}: 中心 ({u}, {v}) 画素, 角度 {angle:.1f} 度')
        cv2.drawContours(img, [c], -1, (0, 0, 255), 2)
        cv2.circle(img, (u, v), 5, (0, 0, 255), -1)
        cv2.putText(img, kind, (u + 8, v), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

cv2.imshow('result', img)
cv2.waitKey(0)
```

```bash
python3 ~/detect_pieces_image.py ~/写真の名前.png
```

ピースのまわりに赤い線と中心の点が描かれ、ターミナルに種類・中心・角度が出れば OK です。何かキーを押すと閉じます。

| うまくいかないとき | 直し方 |
|---|---|
| 1つのピースが細切れになる | `kernel` を大きくする（`(7, 7)` など）、または S・V の下限を下げる |
| 机の汚れや影まで拾う | `MIN_AREA` を大きくする、または S・V の下限を上げる |
| 隣り合うピースが1つにくっつく | 同じ色のピースを離して置く（中間発表では L が2個とも同じ色） |

### 気をつけること

- **種類は色で分かる**：中間発表のピース（I・S・L・L）は色ごとに種類が決まるので、形から種類を見分ける処理はいりません。L は2個あるので、輪郭が2つ出ます
- **角度は I と L・S で扱いが違う**：I は細長い長方形なので `minAreaRect` の角度がそのまま使えます。L と S は対称でないので、`minAreaRect` では向きが 90° ごとにしか区別できません（L の出っ張りがどちら側か分からない）。中間発表では **ピースを決まった向きで置いておく** ことにし、余裕があれば `minAreaRect` の枠を 20mm のマスに区切って、どのマスに色があるかで向きを決めます
- **取っ手が輪郭に影響しないか**：真上から見たとき、取っ手がピースと同じ色かどうかで輪郭の形が変わります。写真で確かめてください

## 5. 3D の位置にする

手順4の中心 (u, v) は画像の上の位置（画素）です。深度画像で同じ画素の距離 Z を読み、`camera_info` に入っているカメラの値（fx, fy, cx, cy）で、カメラから見た 3D の位置にします。

```
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
```

- 深度画像は `/camera/aligned_depth_to_color/image_raw` を使う（カラー画像と画素の位置がそろっている）
- 値の単位は mm なので、1000 で割って m にする
- お手本の `color_detection.cpp` は `image_geometry::PinholeCameraModel` でこの計算をしています

## 6. ロボットの座標にする

手順5の位置はカメラから見た位置（カラー画像のフレーム）です。tf2 でロボットの `base_link` から見た位置に変えます。カメラがハンドに付いているので、アームの姿勢によって変換が変わりますが、tf2 が自動で計算します。

お手本の `color_detection.cpp` は、見つけた位置を tf（`target_0`）として出し、`pick_and_place_tf.cpp` が `base_link` から見た `target_0` の位置を読んでつかみに行きます。手順5・6はこのお手本からほぼそのまま使えるので、ノードにするときはお手本をもとに C++ で書きます（パッケージは C++ のため）。

カメラの付いたロボットの見た目と tf は、ロボットなしでも RViz で確認できます。

```bash
ros2 launch crane_x7_description display.launch.py use_d435:=true
```

## 参考

- お手本：`crane_x7_ros/crane_x7_examples/src/color_detection.cpp`（色の切り出し、深度からの位置、tf での出力）、`aruco_detection.cpp`（マーカーの位置と向き）
- 担当と作るもの：[software_design.md](software_design.md) の「B：認識」
- 関係する Issue：#21（HSV で切り出す）、#22（輪郭から種類・位置・角度）、#23（ロボットの座標に変換）
