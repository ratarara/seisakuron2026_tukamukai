# シミュレーション（Gazebo・RViz）の使い方

ロボットがなくても、Gazebo のシミュレーションで CRANE-X7 を動かせます。動作計画のプログラムは、まずシミュレーションで試してから実機で動かします。

## 全体の関係

```
RViz（操作・表示）──指示──▶ MoveIt（経路を計算）──▶ コントローラ ──▶ Gazebo（仮想のロボット）
                                                                        │
RViz に今の姿勢が表示される ◀──────────── 関節の角度 ◀────────────┘
```

実機のときは、Gazebo のところが本物の CRANE-X7 に入れ替わるだけです。RViz の操作も、自分で書くプログラムも同じものが使えます。

## 起動と終了

**普通のターミナル（`Ctrl + Alt + T`）** で起動します。VSCode の中のターミナルから起動すると RViz が落ちます（→ [troubleshooting](troubleshooting.md) の「VSCode のターミナルから起動すると RViz が落ちる」）。

```bash
ros2 launch crane_x7_gazebo crane_x7_with_table.launch.py
```

- Gazebo と RViz の2つのウィンドウが開きます。初回は机と箱のモデルをダウンロードするので、表示に時間がかかります
- ターミナルに `You can start planning now!` と `Configured and activated crane_x7_arm_controller` が出たら準備完了です
- 終わるときは、起動したターミナルで **`Ctrl + C`** を押します。**ウィンドウを閉じるだけでは止まりません**

## Gazebo（物理シミュレーション）

机・箱・CRANE-X7 が置かれた仮想の世界です。重力やぶつかりも計算されます。基本的には見るだけで、操作は RViz から行います。

| 操作 | やり方 |
|---|---|
| 視点の回転 | 左ドラッグ |
| 視点の平行移動 | 右ドラッグ（またはホイールを押したままドラッグ） |
| ズーム | ホイール |
| 再生／一時停止 | 左下の ▶ ／ ⏸。一時停止中はロボットも止まるので、動かないときはここを確認する |

## RViz（MoveIt でアームを動かす）

### アームを好きな姿勢に動かす

1. アームの先にある **色つきの矢印（平行移動）と輪（回転）** をドラッグする。オレンジ色のアームが目標の姿勢
2. 左の「MotionPlanning」パネルの「Planning」タブで、どちらかを押す
   - **Plan**：経路を計算して、半透明のアームで動きを再生するだけ（まだ動かさない）
   - **Plan & Execute**：計算して、そのまま動かす（Gazebo のアームも動く）

届かない位置までマーカーを引っぱると、経路の計算に失敗して動きません。少し手前に戻してください。

### 決まった姿勢に戻す

「Planning」タブの「Goal State」で姿勢を選び、「Plan & Execute」を押します。

| 名前 | 姿勢 |
|---|---|
| `home` | 腕を折りたたんだ待機姿勢 |
| `vertical` | まっすぐ上に伸ばした姿勢 |

### ハンドを開閉する

ハンドには名前のついた姿勢がないので、関節の角度で指定します。

1. 「Planning Group」を `arm` から **`gripper`** に切り替える
2. 「Joints」タブで `crane_x7_gripper_finger_a_joint` のスライダーを動かす（0 で閉じる、大きくすると開く）
3. 「Planning」タブに戻って「Plan & Execute」

終わったら「Planning Group」を `arm` に戻します。

## サンプルプログラム

シミュレーションを起動した状態で、**別のターミナル** で実行します。シミュレーションでは必ず `use_sim_time:=true` を付けます（付けないと時計が合わずに動きません）。

```bash
ros2 launch crane_x7_examples example.launch.py example:=pick_and_place use_sim_time:=true
```

`example:=` を変えると、ほかのサンプルも動かせます。コードは `~/ros2_ws/src/crane_x7_ros/crane_x7_examples/src/<名前>.cpp` にあります。

| example | 内容 | 参考になる場面 |
|---|---|---|
| `gripper_control` | ハンドを開いたり閉じたりする | ハンドの開閉（#26） |
| `pose_groupstate` | `home` や `vertical` に順に動く | 決まった姿勢への移動 |
| `joint_values` | 関節の角度を1つずつ変える | 関節を直接指定する方法 |
| `cartesian_path` | 手先で円を描く | 手先をまっすぐ下ろす・持ち上げる動き（#27, #28） |
| `pick_and_place` | 箱をつかんで、持ち上げて、運んで、置く | ピースをつかんで置く動作全体（#25〜#28） |

RealSense を使うサンプル（`aruco_detection`・`color_detection`・`point_cloud_detection`）もあります。認識（#20, #21）の参考になりますが、カメラをつけた実機が必要です（`camera_example.launch.py`、詳しくは crane_x7_examples の README）。

## 実機で動かすとき

シミュレーションの代わりに実機を起動します。サンプルは `use_sim_time:=true` を **外して** 実行します。USB の設定（毎回必要）や終了のしかたを含めた手順は [real_robot.md](real_robot.md) にあります。

```bash
# ターミナル1
ros2 launch crane_x7_examples demo.launch.py port_name:=/dev/ttyUSB0

# ターミナル2
ros2 launch crane_x7_examples example.launch.py example:=pick_and_place
```

実機では、まわりに人や物がないことを確かめてから動かしてください。

## うまくいかないとき

[troubleshooting](troubleshooting.md) の「シミュレーション」を見てください。
