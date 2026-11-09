# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

ロボット設計製作論実習3（2026年度 知能コース）、チーム「つかむ会」の開発リポジトリ `ratarara/seisakuron2026_tukamukai`。README・docs・コメントは日本語で書く。

## やりたいこと

テトリミノ型のピースを机の上にばらまき、CRANE-X7 が RealSense で認識してつかみ、ケースの枠にぴったり詰めるパズルを解く。

- ピースは**一段（2D）**のみ。積み上げはしない
- ピースの種類は**色と形**で認識する（テトリス標準色：I=水色、O=黄、T=紫、S=緑、Z=赤、J=青、L=オレンジ）。赤・オレンジ・黄は照明で混ざりやすいので、輪郭（マス数・形）で二重チェックする
- ケースは **8×8マス**（ピース16個分）の予定。デモ時間を考え、一部を最初から埋めた状態から残りを解く案も検討中
- **中間発表の目標**：4×4 をピース4つ（I・S・L・L の3種類：水色・緑・オレンジ）で埋める。色相が離れていて認識しやすく、S・L は非対称なので回転の確認にもなる（2026-10-05 決定）

## 処理の流れ

1. **キャリブレーション**：市松模様（チェッカーボード）でカメラを校正。ケースの3つの角（マーカー、ArUco も候補）で原点とx・y軸を決め、ケース座標系を求める
2. **認識**：HSV で色ごとに切り出し → 輪郭から種類と姿勢（x, y, 回転角）を推定 → 深度とキャリブレーション結果でロボット座標へ変換
3. **ソルバー**：手持ちのピースで枠を埋める配置を探索（完全被覆問題。小さい盤面はバックトラッキング、大きければ Dancing Links）
4. **動作計画**：解いた配置に従い、ピースを順につかんで指定位置・向きに置く

ソフトの設計（ノードの分け方、担当、データ形式の案）は `docs/software_design.md`。要点：

- カメラ（RealSense D435）は **ハンドに付ける**（rt-net 公式マウンタ、`demo.launch.py use_d435:=true`）。カメラ→ロボットの座標変換は tf2 で自動。撮るたびにアームを撮影姿勢に動かして止めてから撮る
- ノードは ①② 認識（B）、③ 司令塔（D、ソルバーを含む）、④ 動作（C）。認識はサービス、動作はアクションの案
- A（ハード）はソフトの動作確認（テスト）も担当する（TA・先生の助言）。記録は `docs/tests/`
- お手本は crane_x7_examples の `color_detection` + `pick_and_place_tf`
- Gazebo では D435 の画像は出ない（センサーのプラグインがない）

### ソルバーの制約

- 盤面を市松に塗ると T ピースは必ず 3:1 になるため、**T の個数が偶数でないと 8×8 は埋まらない**
- 行ごとの縞で塗ると J・L は向きに関係なく 3:1 になるため、**T を使わない場合は J+L の個数も偶数**でないと 8×8 は埋まらない
- S/Z、L/J は鏡像。裏返し動作はしない前提なので、**回転だけで解ける**ピースの組み合わせに限定する（ソルバーは鏡映を候補に入れない）
- 実装は `include/tukamukai/solver.hpp` と `src/solver.cpp`（ROS 非依存のライブラリ `tukamukai_solver`）。盤面を 64 ビットのマスクで表し、候補が最も少ないマスから埋めるバックトラッキングで、上の条件を探索中にも枝刈りに使う。8×8 のランダムな組み合わせで最悪 0.03 秒程度。`ros2 run tukamukai solver_demo` でデモ、テストは `test/test_solver.cpp`（gtest）

## 精度を上げるための機構（ハードウェア）

- **取っ手**：全ピースの上面、重心の真上に共通形状の取っ手をつける。つかみ方を1種類に揃え、指がケースや隣のピースに当たらないようにする。向きが決まるよう板状・角形にする（球形は不可）
  - CRANE-X7 のハンドは片側の指だけが動く構造のはずなので、固定側の指の面で位置が決まる形にする（実機で要確認）
- **位置決め（ビー玉の案）**：ピースの裏に突起（ビー玉を埋め込む or 3Dプリントの半球）、ケースの全マスにすり鉢状のくぼみをつけ、置いたときに重力で中心に寄るようにする
  - 向きも決めるため、1ピースにつき突起は2個以上（離れたマスに）
  - すり鉢の斜面は 45° 程度（PLA 同士の摩擦角は最大でも約 27° なので余裕がある）
  - 補正できるのは半マス未満のずれまで
  - ピース外形はマスより片側 0.5〜1mm 小さくして横に動ける余地を残す
  - ロボットは目標の数mm上でハンドを開いて離す（握ったままだと機構が働かない）。動作計画の配置高さはこれを前提にする
- 枠の入り口は面取りする
- 次の作業：1マス分だけ試作し、わざとずらして置いてどこまで戻るか測定する

## 動作環境

- Ubuntu 24.04 LTS / ROS 2 Jazzy
- `~/ros2_ws/src/` に本リポジトリと `rt-net/crane_x7_ros`、`rt-net/crane_x7_description`（`-b $ROS_DISTRO`）を並べて置く
- Intel RealSense（`ros-jazzy-realsense2-camera`。`package.xml` の依存に `realsense2_camera` を入れる）
- アームは `crane_x7_examples`（MoveIt ベース）で別に起動し、本パッケージのノードから指令する

## パッケージ構成

`ament_cmake` パッケージ `tukamukai`。**言語は C++17（rclcpp）**。チームメンバーが慣れていて、MoveIt・crane_x7_examples も C++ のため（2026-10-02 に Python から変更）。README の「ディレクトリ構成」に従い、新しい構成を発明しない。

- `package.xml`, `CMakeLists.txt`
- 新しいノードは `CMakeLists.txt` に `add_executable` を追加し、`install(TARGETS ... DESTINATION lib/${PROJECT_NAME})` に足す。ROS に依存しない処理はライブラリに分けて gtest でテストする
- `include/tukamukai/`：ヘッダ、`src/`：実装と各ノードの main
- `test/`：gtest。lint は ament_lint_common（uncrustify・lint_cmake・xmllint など）。コードの書式は `ament_uncrustify --reformat <ファイル>` で自動で直せる
- `launch/` のファイルは C++ パッケージでも Python で書く（ROS 2 の標準）
- `launch/`：エントリポイントは `main.launch.py`
- `perception/`：認識（B・堀さん）の試作コード・写真・メモの置き場。`colcon test` の lint 対象外（CMakeLists.txt の AMENT_LINT_AUTO_FILE_EXCLUDE、pep257 は launch/・test/ のみ）。完成したら `src/` のノードにする
- `config/`：カメラ・パラメータ設定
- `docs/`：発表資料・議事録・環境構築手順

## コマンド

```bash
# ビルド（ワークスペースのルートで）
cd ~/ros2_ws
rosdep install -r -y -i --from-paths src
colcon build --symlink-install --packages-select tukamukai
source install/setup.bash

# 実機を起動し、別ターミナルで本パッケージを起動
ros2 launch crane_x7_examples demo.launch.py port_name:=/dev/ttyUSB0
ros2 launch tukamukai main.launch.py

# ソルバーのデモ
ros2 run tukamukai solver_demo

# テスト（gtest と lint）
colcon test --packages-select tukamukai && colcon test-result --verbose
```

## 開発ルール

- ライセンスは **Apache-2.0**（`LICENSE`、著作権の表示は `NOTICE`、2026-11-09 決定）
- PR と main への push のたびに、GitHub Actions（`.github/workflows/build_and_test.yml`）が `ros:jazzy-ros-base` の上でビルドと `colcon test` を行う。crane_x7_* と realsense2_camera は rosdep で入らない・起動にしか使わないので飛ばしている。ビルドに新しい依存を足すときは、rosdep で入るか確かめる
- `main` に直接 push しない。作業ごとに `feature/<内容>` ブランチを切って PR でマージ
- PR の説明に `Closes #<Issue番号>` を書いて Issue と紐づける
- Issue は1回の講義で終わるサイズに切る。ラベル：認識／ソルバー／動作計画／ハード／調査／バグ
- 進捗管理：スケジュール（ガントチャート）は Backlog、個別タスクと記録は GitHub Issues/Projects
- Milestone はプラン発表（第3回、2026-10-12）・中間発表・最終発表に対応
- 講義時間内で終わる計画にする（見積もりの3倍かかる前提）

## リポジトリに残すもの

- コード（途中でも Draft PR で小さく共有）
- `docs/notes/`：調査メモ
- `docs/troubleshooting.md`：つまずいたことと解決方法
- `docs/minutes/YYYY-MM-DD.md`：議事録と決定事項（判断の理由も）
- `hardware/`：ピース・ケースの CAD（STEP など）と STL、設計値
- 実験記録（条件と結果を表で）
- 入れないもの：`build/` `install/` `log/`（.gitignore）、動画や rosbag などの大きいファイル（Drive に置いてリンク）

## 未決定・TODO

- 全体が CRANE-X7 の作業範囲とカメラ画角に収まるか（1マスは 20mm に決定済み。8×8 のケース内寸は 160mm）
- 取っ手の形と位置（J・L は重心が角に寄っていて、重心を中心にした土台は 10mm 角まで。#13）
- ピースの種類ごとの個数（T は偶数。T なしなら J+L も偶数）
- メンバーの担当、Backlog の URL、中間・最終発表の日程（README に TODO として残っている）
- 先輩のリポジトリをベースにする場合、元リポジトリ・ライセンス・差分を README に記載
- `.gitignore` がまだない（`build/` `install/` `log/` を入れる）
- 余裕があれば：認識結果と解の画面表示（デモ用）
- 著作権表示のチェック（ament_copyright）はスキップ中。全ファイルの先頭に Apache-2.0 の著作権表示を入れたら、CMakeLists.txt のスキップを外す
