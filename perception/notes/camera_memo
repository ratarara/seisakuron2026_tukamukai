##カメラの起動方法

カメラを接続したあと、ターミナルを開いたら、以下のコマンドを順に実行してください。  
'''
source /opt/ros/jazzy/setup.bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
'''
以下のような表示があれば成功です。  
RealSense Node Is Up!  

別のターミナルを開いて以下のコードを順に実行してください。  
'''
source /opt/ros/jazzy/setup.bash
ros2 topic list
'''
トピックの一覧が表示されます。以下のトピックが含まれているか確認してください。  
・/camera/camera/color/image_raw  
(カラー画像)  
・/camera/camera/aligned_depth_to_color/image_raw  
(深度画像)



##異なる色を認識したいとき

色相の値を変更することによって指定する色を変更することができます。  
例:  
colorecogにおいて
'''
# 色の設定（HSV）
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
'''
とありますが、np.arrayの最初の値が色相、2番目が彩度、3番目が明度です。  
それぞれの上限は180、255、255となっています。  

認識したい色によっては色相だけではなく彩度、明度も変更する必要があります。


  
##白が認識しづらいとき

彩度の高い反射光によって白が認識できないことがあります。　　
状況にあった色相、彩度、明度の値を変更することによって認識できるようになることもあります。  

例:  
colorecogにおいて
'''
#白い領域を検出

white_mask = cv2.inRange(
    roi_smooth,
    np.array([0, 0, 59]),
    np.array([179, 55, 255])
)
'''
色の設定と同じく、np.arrayの最初の値が色相、2番目が彩度、3番目が明度です。  
それぞれの上限は180、255、255となっています。
