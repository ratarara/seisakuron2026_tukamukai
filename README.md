# つかむ会（seisakuron2026_tukamukai）

千葉工業大学 未来ロボティクス学科「ロボット設計製作論実習3（2026年度 知能コース）」のチーム「つかむ会」のリポジトリです。

## 概要

CRANE-X7 と RealSense を使って、〇〇をするロボットを作ります。

<!-- 例：カメラでブロックの色と形を認識し、CRANE-X7で実物のテトリスをプレイする -->

- デモ動画：（最終発表後に追加）
- 発表資料：[docs/](docs/)

## メンバー

| 名前 | GitHub | 担当 |
|------|--------|------|
| 比屋根佳歩 | [@ratarara](https://github.com/ratarara) | |
| 石川奈々美 | [@](https://github.com/) | |
| 堀円香 | [@](https://github.com/) | |
| キンジュンウ | [@](https://github.com/) | |

## 動作環境

- Ubuntu 24.04 LTS
- ROS 2 Jazzy
- CRANE-X7（7軸マニピュレータ）
- Intel RealSense（深度カメラ）

## セットアップ

授業で先生が示した手順に沿って、実際に行った環境構築の手順です。

### 1. 事前準備

```bash
# （Windowsとデュアルブートの場合）時計のずれを防ぐ
timedatectl set-local-rtc 1 --adjust-system-clock

sudo apt update && sudo apt upgrade -y
sudo apt install -y software-properties-common curl git
sudo add-apt-repository universe
```

### 2. ROS 2 Jazzy のインストール

```bash
# ROS 2 の apt リポジトリを追加
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

# ROS 2 Jazzy と開発ツールのインストール
sudo apt update && sudo apt upgrade -y
sudo apt install -y ros-jazzy-desktop ros-dev-tools

# 端末を開くたびに ROS 2 を読み込む
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
# 通信を自分のPC内だけに限定する（教室で他の人のノードと混ざらないように）
echo "export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST" >> ~/.bashrc
source ~/.bashrc
```

動作確認（2つの端末で実行し、talker の数字が listener に届けばOK）：

```bash
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_py listener
```

### 3. ワークスペースの作成と自作パッケージでの動作確認

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
ros2 pkg create mypkg --build-type ament_python
```

`mypkg/mypkg/` に `talker.py`（`countup` トピックに `Int16` を0.5秒ごとに送る）と `listener.py`（受け取って表示する）を作成し、以下を編集します。

- `package.xml`：`<exec_depend>rclpy</exec_depend>` と `<exec_depend>std_msgs</exec_depend>` を追加
- `setup.py`：`entry_points` の `console_scripts` に `'talker = mypkg.talker:main'` と `'listener = mypkg.listener:main'` を追加

```bash
cd ~/ros2_ws
colcon build
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 2つの端末で実行
ros2 run mypkg talker
ros2 run mypkg listener
```

### 4. Git / GitHub の設定

```bash
git config --global user.name "GitHubのユーザー名"
git config --global user.email "GitHubに登録したメールアドレス"
git config --global init.defaultBranch main

# SSH鍵を作成し、公開鍵（~/.ssh/id_ed25519.pub）を GitHub の Settings > SSH keys に登録
ssh-keygen -t ed25519 -C "GitHubに登録したメールアドレス"
ssh -T git@github.com

# GitHub CLI
sudo apt update
sudo apt install -y gh git
gh auth login
```

### 5. このリポジトリのクローン

```bash
cd ~/ros2_ws/src
gh repo clone ratarara/seisakuron2026_tukamukai
```

CRANE-X7 を動かすための依存パッケージとビルド：

```bash
cd ~/ros2_ws/src
git clone -b $ROS_DISTRO https://github.com/rt-net/crane_x7_ros.git
git clone -b $ROS_DISTRO https://github.com/rt-net/crane_x7_description.git

cd ~/ros2_ws
rosdep install -r -y -i --from-paths src
colcon build --symlink-install
source install/setup.bash
```

### 6. Claude Code のインストール（任意）

```bash
curl -fsSL https://claude.ai/install.sh | bash
cd ~/ros2_ws/src/seisakuron2026_tukamukai
claude
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
├── README.md
├── LICENSE
├── package.xml
├── setup.py
├── launch/        # 起動用launchファイル
├── tukamukai/    # ノード本体（認識・動作計画など）
├── config/        # カメラ・パラメータ設定
└── docs/          # 発表資料・議事録
```

## 進捗管理

進捗は [Issues](../../issues) と [Projects](../../projects) で管理します。
Milestone は3回の発表に対応しています。

| Milestone | 内容 | 日程 |
|-----------|------|------|
| プラン発表 | 作るものの計画 | |
| 中間発表 | デモ付き | |
| 最終発表 | デモ付き | |

## 開発ルール

- `main` には直接pushせず、作業ごとにブランチを切って Pull Request でマージする
- ブランチ名は `feature/<内容>`（例：`feature/block-detection`）
- 作業を始めるときは対応する Issue に自分を Assign する
- 詰まったら Issue にコメントして早めに相談する

## ライセンス・謝辞

- 本リポジトリは [rt-net/crane_x7_ros](https://github.com/rt-net/crane_x7_ros)（Apache License 2.0）を利用しています。
- 先輩のコードを参考・流用した場合は、ここに元リポジトリとライセンスを記載する。

© 2026 つかむ会
