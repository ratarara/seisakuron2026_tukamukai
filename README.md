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

Ubuntu 24.04 をインストールしただけの、何も設定していないPCから始める手順です。上から順番に実行してください。

### 0. 始める前に

- **Ubuntu 24.04 LTS** がインストール済みであること
- **GitHub アカウント**を持っていること

端末の基本操作：

- 端末を開く：`Ctrl + Alt + T`
- 端末に貼り付け：`Ctrl + Shift + V`（`Ctrl + V` では貼り付けできません）
- `sudo` のコマンドではパスワードを聞かれます。入力しても画面には何も表示されませんが、そのまま入力して Enter を押してください
- 途中で `[Y/n]` と聞かれたら `Y` で Enter

### 1. 事前準備

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y software-properties-common curl git
sudo add-apt-repository -y universe
```

Windows とデュアルブートしている場合だけ、時計がずれないように次も実行します。

```bash
timedatectl set-local-rtc 1 --adjust-system-clock
```

> [!TIP]
> Claude の有料プランを持っている人は、ここで Claude Code を入れておくと、この先エラーが出たときに端末で相談できて便利です。
>
> ```bash
> curl -fsSL https://claude.ai/install.sh | bash
> ```

### 2. ROS 2 Jazzy のインストール

```bash
# ROS 2 の apt リポジトリを追加
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

# ROS 2 Jazzy と開発ツールのインストール（時間がかかります）
sudo apt update && sudo apt upgrade -y
sudo apt install -y ros-jazzy-desktop ros-dev-tools

# 端末を開くたびに ROS 2 を読み込む
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
# 通信を自分のPC内だけに限定する（教室で他の人のノードと混ざらないように）
echo "export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST" >> ~/.bashrc
source ~/.bashrc

# 依存パッケージを自動で入れるツール（rosdep）の初期化
sudo rosdep init
rosdep update
```

動作確認：端末を2つ開き、それぞれで次を実行します。listener 側に `I heard: [Hello World: 1]` のような表示が出ればOKです（`Ctrl + C` で止めます）。

```bash
# 端末1
ros2 run demo_nodes_cpp talker
```

```bash
# 端末2
ros2 run demo_nodes_py listener
```

### 3. ワークスペースの作成と自作パッケージでの動作確認

ワークスペース `~/ros2_ws` を作り、練習用パッケージ `mypkg` を作成します。

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
ros2 pkg create mypkg --build-type ament_python --dependencies rclpy std_msgs
cd ~/ros2_ws/src/mypkg

# talker と listener をコマンドとして登録
sed -i "s/'console_scripts': \[/'console_scripts': [\n            'talker = mypkg.talker:main',\n            'listener = mypkg.listener:main',/" setup.py
```

talker（`countup` トピックに数字を0.5秒ごとに送る）を作成：

```bash
cat > ~/ros2_ws/src/mypkg/mypkg/talker.py << 'EOF'
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16


def main():
    rclpy.init()
    node = Node("talker")
    pub = node.create_publisher(Int16, "countup", 10)
    n = 0

    def cb():
        nonlocal n
        msg = Int16()
        msg.data = n
        pub.publish(msg)
        n += 1

    node.create_timer(0.5, cb)
    rclpy.spin(node)


if __name__ == '__main__':
    main()
EOF
```

listener（受け取った数字を表示する）を作成：

```bash
cat > ~/ros2_ws/src/mypkg/mypkg/listener.py << 'EOF'
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16


def main():
    rclpy.init()
    node = Node("listener")

    def cb(msg):
        node.get_logger().info("Listen: %d" % msg.data)

    node.create_subscription(Int16, "countup", cb, 10)
    rclpy.spin(node)


if __name__ == '__main__':
    main()
EOF
```

ビルドして動作確認：

```bash
cd ~/ros2_ws
colcon build
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

端末を2つ開き、listener 側に `Listen: 0`, `Listen: 1`, ... と表示されればOKです。

```bash
# 端末1
ros2 run mypkg talker
```

```bash
# 端末2
ros2 run mypkg listener
```

### 4. Git / GitHub の設定

```bash
# 自分の名前とメールアドレスに書き換えて実行
git config --global user.name "GitHubのユーザー名"
git config --global user.email "GitHubに登録したメールアドレス"
git config --global init.defaultBranch main

# GitHub CLI のインストールとログイン
sudo apt install -y gh
gh auth login
```

`gh auth login` では次のように選びます（矢印キーで選択、Enter で決定）。

1. `Where do you use GitHub?` → **GitHub.com**
2. `What is your preferred protocol for Git operations?` → **SSH**
3. `Generate a new SSH key to add to your GitHub account?` → **Y**（パスフレーズは空のまま Enter でOK）
4. `Title for your SSH key` → そのまま Enter
5. `How would you like to authenticate GitHub CLI?` → **Login with a web browser**
6. 表示された8桁のコードをコピーし、Enter でブラウザを開いてコードを入力・承認

接続確認（`Hi ユーザー名! You've successfully authenticated` と出ればOK。初回に `Are you sure you want to continue connecting` と聞かれたら `yes`）：

```bash
ssh -T git@github.com
```

### 5. このリポジトリと CRANE-X7 パッケージの取得・ビルド

```bash
cd ~/ros2_ws/src
gh repo clone ratarara/seisakuron2026_tukamukai

# CRANE-X7 の ROS 2 パッケージ（rt-net）
git clone -b $ROS_DISTRO https://github.com/rt-net/crane_x7_ros.git
git clone -b $ROS_DISTRO https://github.com/rt-net/crane_x7_description.git

# 依存パッケージのインストールとビルド（時間がかかります）
cd ~/ros2_ws
rosdep install -r -y -i --from-paths src
colcon build --symlink-install
source ~/.bashrc
```

## 使い方

CRANE-X7 を USB で接続したら、ポートへのアクセス権限を付与します（接続するたびに必要）。

```bash
sudo chmod 666 /dev/ttyUSB0
```

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
