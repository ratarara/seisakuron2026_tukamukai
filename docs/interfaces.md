# ノード間のデータの形式

ノード同士でやりとりするデータの型と、座標・向きの決まりです（#29）。型の定義ファイルは `msg/`・`srv/`・`action/` にあります。

**この内容はたたき台です。** 堀さん（①）・石川さん（④）が見て、足りないものや変えたいところがあれば PR か #29 にコメントしてください。全員が合意したら確定します。

## 全体の流れとやりとり

```
③ 司令塔（D）
   │ 1. MoveToObservation（アクション）──▶ ④ 動作（C）    撮影姿勢へ動いて止まる
   │ 2. LocateCase（サービス）         ──▶ ② ケース認識（B）ケースの位置と向き
   │ 3. DetectPieces（サービス）       ──▶ ① ピース認識（B）机の上のピースの一覧
   │ 4. ソルバーで解き、置く順番を決める（D の中）
   │ 5. PickPlace（アクション）        ──▶ ④ 動作（C）    1個つかんで置く
   └─ 1 に戻る（全部置き終わるまで）
```

| 名前 | 種類 | 誰が作る（サーバ） | 誰が呼ぶ（クライアント） | 名前（案） |
|---|---|---|---|---|
| `tukamukai/srv/DetectPieces` | サービス | ① ピース認識（B） | ③ 司令塔（D） | `/detect_pieces` |
| `tukamukai/srv/LocateCase` | サービス | ② ケース認識（B） | ③ 司令塔（D） | `/locate_case` |
| `tukamukai/action/MoveToObservation` | アクション | ④ 動作（C） | ③ 司令塔（D） | `/move_to_observation` |
| `tukamukai/action/PickPlace` | アクション | ④ 動作（C） | ③ 司令塔（D） | `/pick_place` |
| `tukamukai/msg/Piece` | メッセージ | （DetectPieces の中で使う） | | |

- **サービス**：頼んだらすぐ答えが返るもの（認識）
- **アクション**：時間がかかり、途中経過（feedback）を返したり中断したりできるもの（アームの動作）

中身は `ros2 interface show tukamukai/action/PickPlace` のように表示できます。

## 座標と向きの決まり（全員共通）

- **基準のフレーム**：すべて `base_link`（ロボットの土台）
- **単位**：長さはメートル、角度はラジアン
- **角度の向き**：上から見て **反時計回りが正**（ROS の標準、REP-103）

### ピースの向き（yaw）

ソルバー（`include/tukamukai/solver.hpp`）の形を「基準の向き」とします。ピースに固定した座標軸を次のように決めます。

- ピースの **x 軸**：基準の向きで、行が増える向き（ソルバーの図で下向き）
- ピースの **y 軸**：基準の向きで、列が増える向き（ソルバーの図で右向き）

`yaw` は、`base_link` の x 軸から、ピースの x 軸までの角度（反時計回りが正）です。

- 取っ手の板の長手方向は、ピースの **y 軸** の向き（基準の向きでの長い方向。[hardware/handle_design.md](../hardware/handle_design.md)）
- そのため、ハンドの指はピースの **x 軸の向き** に閉じてつかむ（④ はこの関係で手首の角度を決める）
- 対称なピースは、同じ見た目になる角度が複数ある（O は 90°、I・S・Z は 180° ごと）。① はどれを返してもよい。③ がそろえる

### ケースの座標系

`LocateCase` が返す `case_pose` と、tf のフレーム `tukamukai_case` です。

- **原点**：マス (0, 0)（ソルバーの図で左上）の外側の角
- **x 軸**：行が増える向き、**y 軸**：列が増える向き、**z 軸**：上向き（右手系になる）
- マス (行 r, 列 c) の中心：ケース座標で x = (r + 0.5) × 0.020、y = (c + 0.5) × 0.020（1マス 20mm）

ソルバーの回転角は「図の上で時計回り」なので、ケース座標での yaw は **符号が逆**（−回転角）になります。置く位置と向きの計算（#31）では次のようにします。

1. ソルバーの重心 `center`（マス単位。マス (r, c) の中心を (r, c) とした値）に **0.5 を足してから** 0.020 倍してケース座標にする。x = (center の行 + 0.5) × 0.020、y = (center の列 + 0.5) × 0.020。0.5 を足さないと半マス（1cm）ずれる
2. yaw = −（ソルバーの回転角）
3. `tukamukai_case` → `base_link` の変換（tf2）で、ロボットの座標に直す

## 各ノードでの使い方（C++）

同じパッケージの中で定義しているので、ノードを作るときは `CMakeLists.txt` に次を足します。

```cmake
add_executable(piece_detector src/piece_detector.cpp)
target_link_libraries(piece_detector ${cpp_typesupport_target} rclcpp::rclcpp)
# アクションを使うノードは rclcpp_action も（find_package(rclcpp_action REQUIRED) も必要）
```

ヘッダは次のように読み込みます。

```cpp
#include "tukamukai/msg/piece.hpp"
#include "tukamukai/srv/detect_pieces.hpp"
#include "tukamukai/action/pick_place.hpp"
```

## 決めていないこと（相談したい）

- サービスとアクションの名前（上の表は案）
- 撮影姿勢が複数必要になったときの扱い（`pose_name` で名前を渡す案）
- 置いたあとに、ずれていないか確かめるかどうか（撮り直して比べる、など）
- 失敗したときにやり直す回数と、誰がやり直しを判断するか（案：③ が判断する）
