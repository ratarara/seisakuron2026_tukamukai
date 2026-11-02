# RealSense 環境構築
### 1.パッケージ情報の更新
パッケージを最新の状態に更新します。  
```sudo apt-get update```

システムの全体更新やパッケージの依存関係の整理が必要な人は以下を実行してください。問題のない人は省略が可能です。  
```sudo apt-get dist-upgrade```

### 2.パッケージのインストール
RealSense関連のライブラリのビルドなどに必要な開発用パッケージをインストールします。  
```sudo apt-get install automake libtool vim cmake libusb-1.0-0-dev libx11-dev xorg-dev libglu1-mesa-dev```

ROS2環境では、通常TBBのインストールは不要ですが、以降の手順でTBBが要求された場合は以下を実行してTBBのインストールを行ってください。  
```sudo apt-get install libtbb-dev```

### 3.RealSenseのソースコードの取得
RealSenseのライブラリと設定フォルダを取得します。  
```
cd ~
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
```

### 4.USBアクセスルールの設定
RealSenseカメラにアクセスできるように、udevルールを登録します。  
```
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger
```
### 5.ライブラリ検索パスの設定
以下の人はライブラリの検索パスを設定してください。  
・RealSenseの共有ライブラリを手動でインストールした人  
・プログラム実行時にlibrealsenseなどのライブラリが見つからない人  
・ROS2やRealSenseをパッケージマネージャーで導入した人  
```
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
source ~/.bashrc
```
### 6.RealSenseの導入
librealsenseをビルドしてシステムにインストールします。  
```
cd ~/librealsense
sudo apt-get install libusb-1.0-0-dev libudev-dev pkg-config
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=false
cmake --build . -j$(nproc)
sudo cmake --install .
sudo ldconfig
```
###7.ROS2用RealSenseドライバの取得  
ROS2からRealSenseを利用するためのドライバを取得します。  
```
source /opt/ros/jazzy/setup.bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/realsenseai/realsense-ros.git -b ros2-master
```
### 8.ROS2ドライバの依存関係のインストール
ドライバのビルドに必要な依存パッケージをインストールします。  
`sudo apt-get install python3-rosdep python3-colcon-common-extensions`
rosdepの初期化がまだの場合は以下を実行してください。  
```
sudo rosdep init
rosdep update
```
次に依存関係をインストールします。 
```
cd ~/ros2_ws
rosdep install -i --from-path src --rosdistro jazzy --skip-keys=librealsense2 -y
```

### 9.ROS2ドライバのビルド
取得したドライバをビルドします。  
```
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build
```
ビルド後、環境を読み込みます。  
```
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/local_setup.bash
```
### 10.RealSenseカメラの軌道と動作確認
RealSenseカメラをUSBポートに接続してROS2から起動します。  
`ros2 launch realsense2_camera rs_launch.py`
別のターミナルを用意して以下のトピックをそれぞれ確認します。  
```
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/local_setup.bash
ros2 topic list
```
終了させるときはCtrl+Cを押します。  
