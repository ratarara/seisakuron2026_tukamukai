# ROS 2 環境構築

Ubuntu 24.04 をインストールしただけの、何も設定していないPCに ROS 2 Jazzy の開発環境を作る手順です。上から順番に実行してください。

## 0. 始める前に

- **Ubuntu 24.04 LTS** がインストール済みであること
- **GitHub アカウント**を持っていること

ターミナルの基本操作：

- ターミナルを開く：`Ctrl + Alt + T`
- ターミナルに貼り付け：`Ctrl + Shift + V`（`Ctrl + V` では貼り付けできません）
- `sudo` のコマンドではパスワードを聞かれます。入力しても画面には何も表示されませんが、そのまま入力して Enter を押してください
- 途中で `[Y/n]` と聞かれたら `Y` で Enter

## 1. 事前準備

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
> Claude の有料プランを持っている人は、ここで Claude Code を入れておくと、この先エラーが出たときにターミナルで相談できて便利です。
>
> ```bash
> curl -fsSL https://claude.ai/install.sh | bash
> ```

## 2. ROS 2 Jazzy のインストール

```bash
# ROS 2 の apt リポジトリを追加
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

# ROS 2 Jazzy と開発ツールのインストール（時間がかかります）
sudo apt update && sudo apt upgrade -y
sudo apt install -y ros-jazzy-desktop ros-dev-tools

# ターミナルを開くたびに ROS 2 を読み込む
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
# 通信を自分のPC内だけに限定する（教室で他の人のノードと混ざらないように）
echo "export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST" >> ~/.bashrc
source ~/.bashrc

# 依存パッケージを自動で入れるツール（rosdep）の初期化
sudo rosdep init
rosdep update
```

動作確認：ターミナルを2つ開き、それぞれで次を実行します。listener 側に `I heard: [Hello World: 1]` のような表示が出ればOKです（`Ctrl + C` で止めます）。

```bash
# ターミナル1
ros2 run demo_nodes_cpp talker
```

```bash
# ターミナル2
ros2 run demo_nodes_py listener
```

## 3. ワークスペースの作成と自作パッケージでの動作確認

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

ターミナルを2つ開き、listener 側に `Listen: 0`, `Listen: 1`, ... と表示されればOKです。

```bash
# ターミナル1
ros2 run mypkg talker
```

```bash
# ターミナル2
ros2 run mypkg listener
```

## 4. Git / GitHub の設定

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

## 次にやること

ROS 2 の環境ができたら、[README のセットアップ](../README.md#セットアップ) に戻って、このリポジトリと CRANE-X7 パッケージを取得・ビルドしてください。
