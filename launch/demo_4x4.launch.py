# 中間発表のデモ：カメラでピースを見つけ、4×4 のケースに敷き詰める（Enter なしで通しで動く）
#
#   ros2 launch crane_x7_examples demo.launch.py port_name:=/dev/ttyUSB0 use_d435:=true
#   ros2 launch tukamukai demo_4x4.launch.py
#
# 起動するもの：
#   1. 認識のノード piece_tf_node.py（tf piece_<種類>_<番号> を出す）
#   2. 確認用の画像（rqt_image_view /piece_tf_node/image）。view:=false で出さない
#   3. fill_4x4（piece_source:=camera）：1 個ごとに
#      「撮影姿勢 A へ動く → 止まってから撮った画像で、次に置く種類のピースを探す → つかんで置く」
#      を繰り返し、最後に撮影姿勢 B でケースを見せてから、ホーム姿勢に戻る
#
# 撮影姿勢 A・B の値は config/observation_pose.yaml。初めての場所や照明では、
# step_by_step:=true で 1 動作ずつ確かめてから、通しで動かす。

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# 認識のノードに渡すもの
RECOGNIZER_ARGS = {
    'image_topic': ('/camera/color/image_raw', 'color image topic'),
    'plane_z': ('0.005', 'height [m] of the top of the pieces (from base_link)'),
}

# fill_4x4 に渡すもの
FILL_ARGS = {
    'step_by_step': ('false', 'true: confirm each motion with Enter (opens xterm)'),
    'speed_scale': ('0.3', 'speed scale 0.0-1.0'),
    'observe_x': ('0.29', 'hand x [m] to look at the pieces (pose A)'),
    'observe_y': ('0.17', 'hand y [m] to look at the pieces (pose A)'),
    'observe_z': ('0.30', 'hand height [m] to look at the pieces (pose A)'),
    'show_x': ('0.31', 'hand x [m] to show the case at the end (pose B)'),
    'show_y': ('0.02', 'hand y [m] to show the case at the end (pose B)'),
    'show_z': ('0.30', 'hand height [m] to show the case at the end (pose B)'),
    'show_at_end': ('true', 'move to pose B at the end'),
    'observe_wait_sec': ('5.0', 'wait this long [s] for the next piece at pose A'),
    'case_x': ('0.22', 'x [m] of the case corner (outside of cell 0,0)'),
    'case_y': ('-0.04', 'y [m] of the case corner'),
    'case_yaw_deg': ('0.0', 'case yaw [deg]'),
}


def generate_launch_description():
    args = [
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, (default, description) in {**RECOGNIZER_ARGS, **FILL_ARGS}.items()
    ]
    args.append(DeclareLaunchArgument(
        'view', default_value='true', description='true: show the recognition image'))

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

    fill_args = {'mode': 'fill_4x4', 'piece_source': 'camera'}
    fill_args.update({name: LaunchConfiguration(name) for name in FILL_ARGS})
    fill = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('tukamukai'), 'launch', 'pick_place.launch.py')
        ),
        launch_arguments=fill_args.items(),
    )

    # 認識のノードが起動してから動き始める
    return LaunchDescription(args + [
        recognizer,
        viewer,
        TimerAction(period=2.0, actions=[fill]),
    ])
