# 認識のノードの代わりに、ピースの位置と向きの tf を決め打ちで出す（認識ができる前の確認用）
#
# 認識のノードと同じく、base_link から見た取っ手の位置と、ピースの向き（上から見て反時計回り）を
# フレーム piece_<種類>_<番号> で出す。fill_4x4 の piece_source:=tf と組み合わせて使う。
#
#   ros2 launch tukamukai fake_pieces.launch.py
#   ros2 launch tukamukai fake_pieces.launch.py L_1_yaw_deg:=90.0   # 1つだけ向きを変える
#
# 初期値は、fill_4x4 の決め打ちの置き場（tray_x=0.20, tray_y=0.12, tray_dx=0.08, tray_dy=0.09,
# tray_piece_deg=0）と同じ位置

import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# フレーム名：(x [m], y [m], 向き [deg])
PIECES = {
    'I_0': (0.20, 0.12, 0.0),
    'S_0': (0.28, 0.12, 0.0),
    'L_0': (0.20, 0.21, 0.0),
    'L_1': (0.28, 0.21, 0.0),
}


def make_publishers(context):
    nodes = []
    for name in PIECES:
        x = float(LaunchConfiguration(f'{name}_x').perform(context))
        y = float(LaunchConfiguration(f'{name}_y').perform(context))
        yaw = math.radians(float(LaunchConfiguration(f'{name}_yaw_deg').perform(context)))
        nodes.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'fake_piece_{name}',
            arguments=[
                '--x', str(x), '--y', str(y), '--z', '0.0', '--yaw', str(yaw),
                '--frame-id', 'base_link', '--child-frame-id', f'piece_{name}',
            ],
        ))
    return nodes


def generate_launch_description():
    args = []
    for name, (x, y, yaw_deg) in PIECES.items():
        args += [
            DeclareLaunchArgument(f'{name}_x', default_value=str(x),
                                  description=f'piece_{name}: x [m] of the handle'),
            DeclareLaunchArgument(f'{name}_y', default_value=str(y),
                                  description=f'piece_{name}: y [m] of the handle'),
            DeclareLaunchArgument(f'{name}_yaw_deg', default_value=str(yaw_deg),
                                  description=f'piece_{name}: piece yaw [deg] (CCW)'),
        ]
    return LaunchDescription(args + [OpaqueFunction(function=make_publishers)])
