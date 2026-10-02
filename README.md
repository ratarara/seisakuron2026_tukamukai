# つかむ会（seisakuron2026_tukamukai）

千葉工業大学 未来ロボティクス学科「ロボット設計製作論実習3（2026年度 知能コース）」のチーム「つかむ会」のリポジトリです。

## 概要

CRANE-X7 と RealSense を使って、机の上にばらまいたテトリミノ型のピースを認識・把持し、8×8マスのケースにぴったり詰めるパズルを解くロボットを作ります。

1. カメラのキャリブレーションと、マーカーによるケース座標系の推定
2. 色（HSV）と輪郭でピースの種類と位置・向きを認識
3. ソルバーで枠を埋める配置を探索
4. CRANE-X7 でピースを順につかみ、指定の位置・向きに置く

- デモ動画：（最終発表後に追加）
- 発表資料：[docs/](docs/)

## メンバー

| 名前 | GitHub | 担当 |
|------|--------|------|
| 比屋根佳歩 | [@ratarara](https://github.com/ratarara) | |
| 石川奈々美 | [@73lkj](https://github.com/73lkj) | |
| 堀円香 | [@](https://github.com/) | |
| キンジュンウ | [@](https://github.com/) | |

## 動作環境

- Ubuntu 24.04 LTS
- ROS 2 Jazzy
- CRANE-X7（7軸マニピュレータ）
- Intel RealSense（深度カメラ）

## セットアップ

ROS 2 Jazzy と GitHub の設定がまだの人は、先に [ROS 2 環境構築](docs/ros2_setup.md) を済ませてください。

このリポジトリと CRANE-X7 パッケージの取得・ビルド：

```bash
cd ~/ros2_ws/src
gh repo clone ratarara/seisakuron2026_tukamukai

# CRANE-X7 の ROS 2 パッケージ（rt-net）
git clone -b $ROS_DISTRO https://github.com/rt-net/crane_x7_ros.git
git clone -b $ROS_DISTRO https://github.com/rt-net/crane_x7_description.git

# RealSense の ROS 2 パッケージ
sudo apt install -y ros-jazzy-realsense2-camera

# 依存パッケージのインストールとビルド（時間がかかります）
cd ~/ros2_ws
rosdep install -r -y -i --from-paths src
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
```

CRANE-X7 の USB ポートを使えるように、自分のユーザーを `dialout` グループに追加します（最初の1回だけ。実行後にログアウト→ログインし直すと有効になります）。

```bash
sudo usermod -aG dialout $USER
```

## 使い方

```bash
# 実機の起動
ros2 launch crane_x7_examples demo.launch.py port_name:=/dev/ttyUSB0

# 別ターミナルで本パッケージを起動
ros2 launch tukamukai main.launch.py
```

<!-- launchファイル名・引数は実装に合わせて書き換える -->

## ディレクトリ構成

```
seisakuron2026_tukamukai/
├── README.md            # このファイル
├── CLAUDE.md            # Claude Code 向けのプロジェクトメモ
├── .gitignore           # Git で管理しないファイルの一覧
├── package.xml          # ROS 2 パッケージの情報と依存関係
├── setup.py             # Python パッケージのビルド・インストール設定
├── setup.cfg            # 実行ファイルのインストール先
├── resource/
│   └── tukamukai        # パッケージ登録用の空ファイル
├── tukamukai/           # プログラム本体
│   ├── __init__.py
│   └── solver.py        # パズルのソルバー
├── launch/
│   └── main.launch.py   # 全体の起動ファイル
├── config/              # 設定ファイル置き場（今は空）
├── test/                # 自動テスト
└── docs/
    └── ros2_setup.md    # ROS 2 環境構築の手順
```

### 各ファイルの説明

**ドキュメント**

| ファイル | 役割 | 詳細・いつ触るか |
|---|---|---|
| `README.md` | リポジトリの説明書 | 概要・セットアップ・使い方・ルールをまとめたもの。GitHub のトップに表示される。手順や構成が変わったら更新する |
| `CLAUDE.md` | Claude Code 向けのメモ | やりたいこと、処理の流れ、ソルバーの制約、ハードの案、開発ルール、TODO。Claude Code が作業前に読む。方針が決まったら更新する |
| `docs/ros2_setup.md` | ROS 2 環境構築手順 | 何も入っていない Ubuntu 24.04 に ROS 2 Jazzy の開発環境を作る手順。新しいメンバーや PC の再インストール時に使う |

**ROS 2 パッケージの設定**（パッケージ名 `tukamukai`、ament_python）

| ファイル | 役割 | 詳細・いつ触るか |
|---|---|---|
| `package.xml` | パッケージの情報 | 名前・説明・管理者・ライセンス・依存パッケージ（`rclpy`、`realsense2_camera` など）。`rosdep install` はここを見て必要なものを入れる。**新しいパッケージを使うときは依存を追加する** |
| `setup.py` | ビルド・インストール設定 | `colcon build` がどのファイルをどこに入れるかを決める。**ノードを作ったら `entry_points` の `console_scripts` に登録する**（登録しないと `ros2 run` で起動できない）。`launch/` と `config/` もここでインストールしている |
| `setup.cfg` | 実行ファイルの置き場所 | `ros2 run` が実行ファイルを見つけられるようにする設定。基本的に触らない |
| `resource/tukamukai` | パッケージ登録用 | 中身は空。ROS 2 がこのパッケージを見つけるための目印。消さない・触らない |
| `.gitignore` | Git の除外設定 | ビルドで生成される `build/` `install/` `log/`、Python のキャッシュ、動画や rosbag などの大きいファイルをコミットしないようにする |

**プログラム**

| ファイル | 役割 | 詳細・いつ触るか |
|---|---|---|
| `tukamukai/` | プログラム本体の置き場 | 認識・ソルバー・動作計画のコードをここに置く。ノードを追加したら `setup.py` にも登録する |
| `tukamukai/__init__.py` | Python パッケージの目印 | 中身は空。このフォルダを Python のモジュールとして読み込めるようにする。消さない |
| `tukamukai/solver.py` | パズルのソルバー | 手持ちのピースで盤面を埋める配置（種類・位置・回転角・重心）を探す。ROS に依存しないので `python3 -m tukamukai.solver` で単体で試せる |
| `launch/main.launch.py` | 全体の起動ファイル | `ros2 launch tukamukai main.launch.py` で使う。認識・ソルバー・動作計画のノードができたらここに追加する（今は空） |
| `config/` | 設定ファイル置き場 | カメラのパラメータや HSV のしきい値などを YAML で置く予定。`.gitkeep` は空のフォルダを Git に残すための空ファイル |

**テスト**（`colcon test --packages-select tukamukai` で実行）

| ファイル | 役割 | 詳細 |
|---|---|---|
| `test/test_solver.py` | ソルバーのテスト | 8×8 が埋まるか、解けない組み合わせを正しく判定するか、鏡像を使っていないかなどを確かめる |
| `test/test_flake8.py` | コードの書き方チェック | 1行の長さ、インデント、使っていない import などを検査する |
| `test/test_pep257.py` | docstring の書き方チェック | 関数の説明文（docstring）の形式を検査する |
| `test/test_copyright.py` | 著作権表示のチェック | 各ファイル先頭の著作権表示を検査する。ライセンス未定のため今はスキップされる |

### 今後追加する予定のもの

| 場所 | 中身 |
|---|---|
| `LICENSE` | ライセンス（種類は未定） |
| `docs/notes/` | 調査メモ |
| `docs/minutes/YYYY-MM-DD.md` | 議事録と決定事項（判断の理由も） |
| `docs/troubleshooting.md` | つまずいたことと解決方法 |
| `hardware/` | ピース・ケースの CAD（STEP など）と STL、設計値 |

## 進捗管理

進捗は [Issues](../../issues) と [Projects](../../projects) で管理します。
Milestone は3回の発表に対応しています。

| Milestone | 内容 | 日程 |
|-----------|------|------|
| プラン発表 | 作るものの計画 | 2026年10月12日（月）第3週 |
| 中間発表 | デモ付き | |
| 最終発表 | デモ付き | |

## 開発ルール

- `main` には直接pushせず、作業ごとにブランチを切って Pull Request でマージする
- ブランチ名は `feature/<内容>`（例：`feature/block-detection`）
- 作業を始めるときは対応する Issue に自分を Assign する
- 詰まったら Issue にコメントして早めに相談する

## ライセンス・謝辞

- 本リポジトリは以下を利用しています。
  - [rt-net/crane_x7_ros](https://github.com/rt-net/crane_x7_ros)：Apache License 2.0
  - [rt-net/crane_x7_description](https://github.com/rt-net/crane_x7_description)：RT社の独自ライセンス（[LICENSE](https://github.com/rt-net/crane_x7_description/blob/HEAD/LICENSE)）。研究開発目的での使用は許諾されていますが、商業目的での使用には RT社への問い合わせが必要です
- 先輩のコードを参考・流用した場合は、ここに元リポジトリとライセンスを記載する。

© 2026 つかむ会
