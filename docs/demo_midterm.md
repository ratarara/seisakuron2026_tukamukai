# 中間発表のデモ：当日の手順と確認項目

カメラで 4 ピース（I・S・L・L）を見つけ、4×4 のケースに敷き詰めるデモの手順です。上から順に進めてください。**実機での通しはまだ試していません。** 1〜4 の確認で問題が出たら、「うまくいかないとき（予備の案）」に切り替えます。

使うもの：CRANE-X7（D435 マウンタつき）、D435、PC、ピース 4 個、4×4 のケース（なければ 80 mm 角のマスを印刷した紙）、メジャー、非常停止

## 0. 前日まで（PC ごとに 1 回）

```bash
cd ~/ros2_ws/src/seisakuron2026_tukamukai
git fetch && git switch feature/piece-tf && git pull
cd ~/ros2_ws && colcon build --symlink-install --packages-select tukamukai
source install/setup.bash
```

- USB の `latency_timer` を自動で 1 にする設定（[troubleshooting.md](troubleshooting.md) の「`Overrun detected!`」）をしておく
- 画面が開くもの（RViz・rqt_image_view）は、**普通のターミナル（`Ctrl + Alt + T`）** で起動する（VSCode の中だと落ちる）

## 1. つなぐ・置く

1. ロボットと D435 を USB でつなぐ。D435 は **USB 3** で：`rs-enumerate-devices | grep "Usb Type"` が `3.2` なら OK（`2.1` なら挿し直す・ケーブルを替える）
2. **ケース**：マス (0, 0) の外側の角を、土台の中心から **前 22 cm・右 4 cm**（x = 0.22, y = -0.04）に合わせる。ケースの辺はロボットの前後・左右にそろえる
3. **ピース**：置き場（土台の中心から **前 17〜31 cm・左 9〜23 cm** の 14 cm 四方）に、4 つを重ならないように置く。向きはばらばらでよい
4. カメラに写る範囲（前 14〜35 cm・左 0〜28 cm）に、**ほかのオレンジ・緑・青の物を置かない**（色で見分けるため）

```
        左（+y）
          ↑
   0.23 ┌──────┐
        │置き場│      ← ピース 4 つ（14 cm 四方）
   0.09 └──────┘
   0.04 ┌────┐
        │ケース│     ← 8 cm 四方。角 (0.22, -0.04)
  -0.04 └────┘
        0.17  0.31  → 前（+x）  単位 m、土台の中心から
```

## 2. 起動する

```bash
# ターミナル1：実機とカメラ
ros2 launch crane_x7_examples demo.launch.py port_name:=/dev/ttyUSB0 use_d435:=true
```

RViz にロボットとカメラが出て、赤いエラーが続いていなければ OK。`START_STATE_INVALID` で動かないときは [troubleshooting.md](troubleshooting.md)。

## 3. 認識だけ確かめる（ロボットは撮影姿勢まで動くだけ）

```bash
# ターミナル2：撮影姿勢 A へ動いて止まる
ros2 launch tukamukai pick_place.launch.py mode:=hand_down down_x:=0.29 down_y:=0.17 down_z:=0.30 speed_scale:=0.3
# ターミナル3：認識のノード
ros2 run tukamukai piece_tf_node.py
# ターミナル4：確認用の画像
ros2 run rqt_image_view rqt_image_view /piece_tf_node/image
```

確認すること：

| 確認 | OK の目安 | だめなとき |
|---|---|---|
| 4 ピースに黒い形が重なり、`piece_I_0` `piece_S_0` `piece_L_0` `piece_L_1` と出る | 4 つとも出る | 色が取れない → 照明、ほかの色の物。端で切れている → ピースを置き場の内側へ |
| 画像の左上に赤い文字が出ていない | 何も出ない | `camera moving`：アームが止まるのを待つ。`no camera_info` `no tf`：ターミナル1 を確認 |
| 位置が合っているか | 下の表で 5 mm 以内 | 下の「位置のずれ」 |

**位置のずれを測る**：1 つのピースの取っ手の中心を、メジャーで土台の中心から測り、表示と比べる。

```bash
ros2 run tf2_ros tf2_echo base_link piece_I_0    # Translation の x, y [m] が取っ手の位置
```

| ピース | メジャー x, y [m] | 表示 x, y [m] | ずれ [mm] |
|---|---|---|---|
| I | | | |
| L | | | |

- ずれが **5 mm 以内**：4 へ
- **一定の向きにずれる**（全部のピースで x が +1 cm など）：カメラの位置のずれ。PC で Claude Code に「認識した位置が x に +1 cm ずれる。補正を入れて」と頼む（`piece_tf_node.py` に補正の値を足す）
- **高さによるずれ**（遠くのピースほど大きくずれる）：`plane_z`（ピースの上面の高さ、初期値 0.005 m）が合っていない。ターミナル3 を `ros2 run tukamukai piece_tf_node.py --ros-args -p plane_z:=0.010` のように変えて試す

確認が終わったら、ターミナル2〜4 を `Ctrl + C` で止める（ターミナル1 はそのまま）。

## 4. 1 動作ずつで 1 回通す（初回だけ）

```bash
ros2 launch tukamukai demo_4x4.launch.py step_by_step:=true
```

xterm が開くので、動作ごとに Enter を押す（`q` + Enter で中止）。指が取っ手の **広い面** を挟むか、となりのピースや枠に当たらないか、置いた向きが合っているかを見る。

## 5. 本番（通しで動かす）

ピースを置き場に並べ直してから：

```bash
ros2 launch tukamukai demo_4x4.launch.py
```

1 個ごとに「撮影姿勢 A → 探す → つかむ → 手首を回す → 置く」を繰り返し、最後に撮影姿勢 B でケースを見せてホームに戻る。終わったら `Ctrl + C`。**非常停止にいつでも手が届くように。**

- 速さは 0.3（`speed_scale:=0.3`）。慣れたら 0.5 くらいまで上げてよい
- 1 回にかかる時間を測っておく（発表の時間に入るか）
- **動画を撮っておく**（本番で失敗したときの予備。#51）

## うまくいかないとき（予備の案）

| 症状 | まずやること |
|---|---|
| `no I piece found outside the case` で止まる | 続けて出る `seen:` の行を見る。何も出ない → 認識できていない（3 に戻る）。`(in the case)` → ピースがケースに近すぎる（置き場の内側へ） |
| つかめない・指がピースに乗る | 位置のずれ（3）を測る。取っ手の向きが違うなら 4 で確かめる |
| つかむときに手が傾く（前が下がる） | 傾きは自動で補正する（下の「手の傾きの補正」）。ターミナルの `[comp] ... tilt rx=... ry=...` を見る |
| アームの力が抜けた（`Communication timeout!`） | アームを支えて止める → 電源を入れ直す → [troubleshooting.md](troubleshooting.md) |

**予備の案 A：認識と敷き詰めを別々に見せる**

認識を使った通しがうまくいかないときは、次の 2 つを別々に見せる。

1. 認識：3 の画面（4 ピースに形と角度が描かれる）を見せる
2. 敷き詰め：ピースを決め打ちの置き場（[real_robot.md](real_robot.md) の配置図、向き 0°）に置き、カメラを使わずに動かす

```bash
ros2 launch tukamukai pick_place.launch.py mode:=fill_4x4 speed_scale:=0.3
```

## 手の傾きの補正

腕や手の重さで手首が少し下がり、つかむときに手の前側が下がるように傾くことがある（11/08）。真上と下ろしたところで、モーターが読んだ手の向きと目標の向きの差（傾き）を測り、**逆に傾けた向きを指令して動き直す**（位置の補正と同じ考え方）。ターミナルに次のように出る。

```
[comp] go down: try 0 error dx=... [mm], tilt rx=+0.10 ry=-2.30 [deg]   ← 傾きを測った
[comp] go down: try 1 error dx=... [mm], tilt rx=+0.02 ry=-0.20 [deg]   ← 補正して小さくなった
```

- `try 1` で 0.5° 未満になっていれば補正できている
- **見た目は傾いているのに、`tilt` がほぼ 0 のまま**：モーターの角度には表れない傾き（取り付けのガタ、マウンタの重さでハンドがずれるなど）。ソフトでは測れないので、ハンドとマウンタの取り付けを確かめる
- 補正を切るとき：`comp_use_tilt:=false`。補正の上限は 6°（`comp_max_tilt_deg`）

## まだ確かめていないこと（当日わかったら記録する）

- 実機での通し（認識 → 撮り直し → 敷き詰め）。Mock と録画では 4 ピースとも通った（2026-11-08）
- 撮影姿勢 A で、腕のねじれの関節が条件の上限（30°）近く（29°）。実機で動きにくければ `observe_y:=0.16` などで少し変える
- 認識した位置のずれ（3 の表）。結果は [docs/tests/](tests/) に残す
- つかむときの手の向き（`hand_offset_deg` = -90°。11/02 に石川さんが実機で確認）
- 手の傾きの補正（Mock で、わざと傾きを入れて 1 回で直ることだけ確認）
