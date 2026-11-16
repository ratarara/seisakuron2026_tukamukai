# ハンドに RealSense D415 を付けたときに、カメラを起動し、カメラの位置を tf で出す
#
#   ros2 launch crane_x7_examples demo.launch.py port_name:=/dev/ttyUSB0      # use_d435 は付けない
#   ros2 launch tukamukai camera_d415.launch.py
#   ros2 launch tukamukai demo_4x4.launch.py
#
# なぜ：demo.launch.py の use_d435:=true は、カメラのドライバーに device_type: d435 を渡すので、
# D415 では起動しない。ロボットのモデル（URDF）のカメラの位置も D435 の寸法で計算している。
# そこで use_d435 なしで起動し、カメラとその位置はここで出す。
#
# カメラの位置：
#   1. crane_x7_gripper_base_link → camera_bottom_screw_frame（カメラの底のねじ穴）
#      crane_x7_description の use_d435 と同じ値。D415 も同じマウンタに、底のねじで付けている前提
#   2. camera_bottom_screw_frame → camera_link
#      realsense2_description の _d415.urdf.xacro の値（D435 とは前に約 10 mm、横に 2.5 mm 違う）
#   camera_link から先（color の光軸など）は、カメラのドライバーが本体の校正値で出す
#
# 取り付け方が違うときは、mount_* の引数で 1 の値を変える。決め方は docs/real_robot.md

import math

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 名前 -> (既定値, 説明)。もとは crane_x7_description（use_d435）と同じ (0, 0.032, 0.0191)。
# 2026-11-16 に、定規で測った 4 ピースの位置と、撮影姿勢 A で認識した位置を比べ、
# 認識が平均 (-50.5, -53.3) mm ずれていたので直し、もう一度測って残り (+5, +5) mm も直した
MOUNT_ARGS = {
    'mount_x': ('-0.048', 'x [m] of the camera bottom screw in crane_x7_gripper_base_link'),
    'mount_y': ('-0.014', 'y [m] of the camera bottom screw'),
    'mount_z': ('0.0191', 'z [m] of the camera bottom screw'),
    'mount_roll': (str(math.radians(-90)), 'roll [rad] of the camera'),
    'mount_pitch': (str(math.radians(-90)), 'pitch [rad] of the camera'),
    'mount_yaw': ('0.0', 'yaw [rad] of the camera'),
}

# realsense2_description/urdf/_d415.urdf.xacro：ねじ穴 → camera_link
D415_SCREW_TO_LINK = ('0.0', '0.020', '0.0115')


def generate_launch_description():
    args = [
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, (default, description) in MOUNT_ARGS.items()
    ]

    mount_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_mount_tf',
        arguments=[
            '--x', LaunchConfiguration('mount_x'),
            '--y', LaunchConfiguration('mount_y'),
            '--z', LaunchConfiguration('mount_z'),
            '--roll', LaunchConfiguration('mount_roll'),
            '--pitch', LaunchConfiguration('mount_pitch'),
            '--yaw', LaunchConfiguration('mount_yaw'),
            '--frame-id', 'crane_x7_gripper_base_link',
            '--child-frame-id', 'camera_bottom_screw_frame',
        ],
    )

    screw_to_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_screw_to_link_tf',
        arguments=[
            '--x', D415_SCREW_TO_LINK[0],
            '--y', D415_SCREW_TO_LINK[1],
            '--z', D415_SCREW_TO_LINK[2],
            '--frame-id', 'camera_bottom_screw_frame',
            '--child-frame-id', 'camera_link',
        ],
    )

    # 認識（piece_tf_node.py）はカラー画像だけを使う（深度は使わない）。
    # 2026-11-16 にカラー 1280×720 と深度を出していたら PC が固まったので（原因は未確定）、
    # 深度を切り、カラーも 640×480・15 fps にして、USB と PC の負荷を下げる。
    # 撮影姿勢 A（config/observation_pose.yaml）は 640×480 の画角（横 約 55°）で決めてある。
    # D415 は 848×480・15 fps のカラーを受け付けず、640×480 になった（2026-11-16）
    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory('realsense2_camera'), '/launch/rs_launch.py']
        ),
        launch_arguments={
            'camera_namespace': '',
            'device_type': 'd415',
            'enable_depth': 'false',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            'pointcloud.enable': 'false',
            'align_depth.enable': 'false',
            'rgb_camera.color_profile': '640,480,15',
        }.items(),
    )

    return LaunchDescription(args + [mount_tf, screw_to_link_tf, realsense])
