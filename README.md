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

ここまでの手順は、ロボットやカメラをつながなくてもできます。実際に行ったときの記録は [docs/notes/setup-log-2026-10-02.md](docs/notes/setup-log-2026-10-02.md) にあります。

> [!WARNING]
> `sudo` を含むコマンドは **1行ずつ** 貼り付けてください。まとめて貼り付けると、パスワードを聞かれている間に次の行がパスワードとして読み込まれ、残りが実行されません。

**1. このリポジトリと CRANE-X7 パッケージの取得**

```bash
cd ~/ros2_ws/src
gh repo clone ratarara/seisakuron2026_tukamukai

# CRANE-X7 の ROS 2 パッケージ（rt-net）
git clone -b $ROS_DISTRO https://github.com/rt-net/crane_x7_ros.git
git clone -b $ROS_DISTRO https://github.com/rt-net/crane_x7_description.git
```

**2. 依存パッケージのインストール**（1行ずつ実行。`rosdep install` は時間がかかります）

```bash
# rosdep をまだ初期化していない人だけ（"already exists" と出たら初期化済み）
sudo rosdep init
rosdep update
```

```bash
sudo apt install -y ros-jazzy-realsense2-camera
```

```bash
cd ~/ros2_ws && rosdep install -r -y -i --from-paths src
```

最後に `#All required rosdeps installed successfully` と出れば OK です。

**3. CRANE-X7 の USB ポートを使えるようにする**（最初の1回だけ。ログアウト→ログインし直すと有効になります）

```bash
sudo usermod -aG dialout $USER
```

**4. ビルド**

```bash
cd ~/ros2_ws
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
```

`Summary: 〇 packages finished` と出れば成功です。crane_x7_examples で `Policy CMP0144 is not set` という警告が出ますが、無視して大丈夫です。

**5. 動作確認**（ロボットなしでできる範囲）

```bash
# CRANE-X7・RealSense・tukamukai のパッケージが表示されれば OK
ros2 pkg list | grep -E "^crane_x7|^realsense2|^tukamukai"
```

ロボットがなくても、Gazebo のシミュレーションで CRANE-X7 を動かせます（初回はモデルのダウンロードに時間がかかります）。

```bash
ros2 launch crane_x7_gazebo crane_x7_with_table.launch.py
```

うまくいかないときは [docs/troubleshooting.md](docs/troubleshooting.md) を見てください。

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
├── README.md
├── LICENSE
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── tukamukai  # ament_python のパッケージ登録用（空ファイル）
├── launch/        # 起動用launchファイル
├── tukamukai/    # ノード本体（認識・動作計画など）
├── config/        # カメラ・パラメータ設定
└── docs/          # 発表資料・議事録・ROS 2 環境構築手順
```

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
