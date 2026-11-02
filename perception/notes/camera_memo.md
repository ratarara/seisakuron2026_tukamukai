## カメラの起動方法

カメラを接続したあと、ターミナルを開いたら、以下のコマンドを順に実行してください。  
```
source /opt/ros/jazzy/setup.bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
```
以下のような表示があれば成功です。  
RealSense Node Is Up!  

別のターミナルを開いて以下のコードを順に実行してください。  
```
source /opt/ros/jazzy/setup.bash
ros2 topic list
```
トピックの一覧が表示されます。以下のトピックが含まれているか確認してください。  
  
・/camera/camera/color/image_raw  
(カラー画像)  
・/camera/camera/aligned_depth_to_color/image_raw  
(深度画像)



## RealSenseカメラの録画
あらかじめ動画を撮っておくと、RealSenseが手元にない状況でも自作のコードを試すことができます。録画方法、及び再生方法は以下のとおりです。  

### 録画方法
まず、上記のカメラの起動方法の通りにカメラを起動させてください。  

次に新しくターミナルを開いて以下のコマンドを順に行ってください。  
```
source /opt/ros/jazzy/setup.bash

mkdir -p ~/realsense_bags
cd ~/realsense_bags

ros2 bag record -o d435_recording   --topics   /camera/camera/color/image_raw   /camera/camera/depth/image_rect_raw
```
`[INFO] [rosbag2_recorder]: Recording...`と表示されれば録画できています。  

録画はCtrl+cで終了させることができます。  
録画したデータは~/realsense_bags/d435_recording/に保存されています。以下のように録画されているか確認してください。  
```
cd ~/realsense_bags
ros2 bag info d435_recording
```
トピックの中にカラー画像と深度画像が表示されていれば成功です。  

### 再生方法
以下のコマンドを順に行ってください。  
```
source /opt/ros/jazzy/setup.bash
cd ~/realsense_bags
ros2 bag play ~/realsense_bags/d435_recording --loop
```
録画した動画が繰り返し再生されるようになります。  
次に、別のターミナルで以下を実行してください。  
```
source /opt/ros/jazzy/setup.bash
source ~/realsense_ws/install/setup.bash
ros2 run realsense_image_viewer image_listener
```

### 録画データの共有方法
録画データを共有する場合、非常に大きいサイズとなっている可能性があるのでgoogle driveでのリンク共有をおすすめします。  

録画データがきちんと残っているか`ls -lh ~/realsense_bags/d435_recording`で確認してから以下の手順に沿ってzipファイルを作成ください。  
```
sudo apt update
sudo apt install zip

cd ~/realsense_bags
zip -r d435_recording.zip d435_recording
```
zipファイルの作成には時間がかかることがあります。  
作成できたら、`ls -lh ~/realsense_bags/d435_recording.zip`で確認してください。  

google driveを開いたら、d435_recording.zipの名前で保存されているファイルをアップロードし、共有設定を変更してください。その後、共有リンクを共有してください。  
  
なお、zip形式にしてもサイズが小さくなるとは限りません。  

  
  
## 異なる色を認識したいとき

色相の値を変更することによって指定する色を変更することができます。  
  
例:  
colorecogにおいて
```
#色の設定（HSV）
self.colors = {
    'ORANGE': [
        ([5, 100, 100], [25, 255, 255])
    ],
    'GREEN': [
        ([30, 50, 40], [90, 255, 255])
    ],
    'BLUE': [
        ([90, 70, 50], [130, 255, 255])
    ]
}
```
とありますが、np.arrayの最初の値が色相、2番目が彩度、3番目が明度です。  
それぞれの上限は180、255、255となっています。  
  
認識したい色によっては色相だけではなく彩度、明度も変更する必要があります。
  
  
  
## 白が認識しづらいとき

彩度の高い反射光によって白が認識できないことがあります。  
状況にあった色相、彩度、明度の値を変更することによって認識できるようになることもあります。  

例:  
colorecogにおいて
```
#白い領域を検出

white_mask = cv2.inRange(
    roi_smooth,
    np.array([0, 0, 59]),
    np.array([179, 55, 255])
)
```

異なる色の認識と同じく、np.arrayの最初の値が色相、2番目が彩度、3番目が明度です。  
それぞれの上限は180、255、255となっています。
