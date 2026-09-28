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

### 1. ROS 2 Jazzy のインストール

Ubuntu 24.04 に ROS 2 Jazzy をインストールします（[公式手順](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)）。インストール済みの場合は飛ばしてください。

```bash
# ロケールをUTF-8に設定
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# universe リポジトリの有効化
sudo apt install -y software-properties-common
sudo add-apt-repository universe

# ROS 2 の apt リポジトリを追加
sudo apt update && sudo apt install -y curl
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

# ROS 2 Jazzy と開発ツールのインストール
sudo apt update && sudo apt upgrade -y
sudo apt install -y ros-jazzy-desktop ros-dev-tools

# 端末を開くたびに ROS 2 を読み込む
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc

# rosdep の初期化（初回のみ）
sudo rosdep init
rosdep update
```

動作確認（2つの端末で実行し、メッセージが届けばOK）：

```bash
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_py listener
```

### 2. RealSense パッケージのインストール

```bash
sudo apt install -y ros-jazzy-realsense2-camera
```

### 3. ワークスペースの作成とビルド

```bash
# ワークスペースの作成
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 依存パッケージ
git clone -b $ROS_DISTRO https://github.com/rt-net/crane_x7_ros.git
git clone -b $ROS_DISTRO https://github.com/rt-net/crane_x7_description.git

# このリポジトリ
git clone https://github.com/ratarara/seisakuron2026_tukamukai.git

# 依存関係のインストールとビルド
cd ~/ros2_ws
rosdep install -r -y -i --from-paths src
colcon build --symlink-install
source install/setup.bash
```

### 4. CRANE-X7 の USB ポート権限

実機を接続したら、USB ポートへのアクセス権限を付与します（接続のたびに必要）。

```bash
sudo chmod 666 /dev/ttyUSB0
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
