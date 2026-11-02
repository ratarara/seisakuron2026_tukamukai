# 中間発表のデモ：カメラでピースを見つけ、4×4 のケースに敷き詰める
#
# 先に実機とカメラを起動し、アームを撮影姿勢（カメラが机のピースを真上から見る姿勢）に
# してから、このファイルを起動する。
#
#   ros2 launch crane_x7_examples demo.launch.py port_name:=/dev/ttyUSB0 use_d435:=true
#   ros2 launch tukamukai pick_place.launch.py mode:=observe step_by_step:=true speed_scale:=0.3
#   ros2 launch tukamukai demo_4x4.launch.py
#
# 起動するもの：
#   1. 認識のノード piece_tf_node.py（tf piece_<種類>_<番号> を出す）
#   2. 確認用の画像（rqt_image_view /piece_tf_node/image）。view:=false で出さない
#   3. start_delay_sec 秒待ってから、fill_4x4（piece_source:=tf）
#      fill_4x4 は始めにホーム姿勢へ動くので、撮影姿勢で止まっている間に認識が tf を出せるよう、
#      先に待つ。ピースは動かないので、撮影姿勢で出した tf を使えるよう、tf の古さの基準は長くする
#
# 最初は step_by_step:=true（初期値）のまま、1 動作ずつ Enter で確かめる。

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

ARGS = {
    'step_by_step': ('true', 'true: confirm each motion with Enter (opens xterm)'),
    'speed_scale': ('0.3', 'speed scale 0.0-1.0'),
    'view': ('true', 'true: show the recognition image (rqt_image_view)'),
    'start_delay_sec': ('5.0', 'wait this long [s] before starting fill_4x4'),
    'tf_wait_sec': ('10.0', 'fill_4x4: wait this long [s] for all pieces'),
    'tf_max_age_sec': ('60.0', 'fill_4x4: ignore tf older than this [s]'),
    'image_topic': ('/camera/color/image_raw', 'color image topic'),
    'plane_z': ('0.005', 'height [m] of the top of the pieces (from base_link)'),
    'case_x': ('0.22', 'x [m] of the case corner (outside of cell 0,0)'),
    'case_y': ('-0.04', 'y [m] of the case corner'),
    'case_yaw_deg': ('0.0', 'case yaw [deg]'),
}


def generate_launch_description():
    args = [
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, (default, description) in ARGS.items()
    ]

    recognizer = Node(
        package='tukamukai',
        executable='piece_tf_node.py',
        name='piece_tf_node',
        output='screen',
        parameters=[{
            'image_topic': LaunchConfiguration('image_topic'),
            'plane_z': ParameterValue(LaunchConfiguration('plane_z'), value_type=float),
        }],
    )

    viewer = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        arguments=['/piece_tf_node/image'],
        condition=IfCondition(LaunchConfiguration('view')),
    )

    fill = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('tukamukai'), 'launch', 'pick_place.launch.py')
        ),
        launch_arguments={
            'mode': 'fill_4x4',
            'piece_source': 'tf',
            'step_by_step': LaunchConfiguration('step_by_step'),
            'speed_scale': LaunchConfiguration('speed_scale'),
            'tf_wait_sec': LaunchConfiguration('tf_wait_sec'),
            'tf_max_age_sec': LaunchConfiguration('tf_max_age_sec'),
            'case_x': LaunchConfiguration('case_x'),
            'case_y': LaunchConfiguration('case_y'),
            'case_yaw_deg': LaunchConfiguration('case_yaw_deg'),
        }.items(),
    )

    return LaunchDescription(args + [
        recognizer,
        viewer,
        TimerAction(period=LaunchConfiguration('start_delay_sec'), actions=[fill]),
    ])
