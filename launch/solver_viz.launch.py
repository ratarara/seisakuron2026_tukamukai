# ソルバーの解を RViz に表示する（solver_viz ノード）
#
# 中間発表の 4×4（I・S・L・L）:
#   ros2 launch tukamukai solver_viz.launch.py
# 最終の 8×8（決まった16ピース）:
#   ros2 launch tukamukai solver_viz.launch.py preset:=8x8
# Gazebo や実機と一緒に（ロボットのモデルはそちらのものを使う）:
#   ros2 launch tukamukai solver_viz.launch.py with_robot:=false
# プリセットの一部だけ変える:
#   ros2 launch tukamukai solver_viz.launch.py preset:=8x8 step_time:=0.8

from ament_index_python.packages import get_package_share_directory
from crane_x7_description.robot_description_loader import RobotDescriptionLoader
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# crane_x7_moveit_config の SRDF にある home 姿勢
HOME_POSE = {
    'crane_x7_lower_arm_fixed_part_joint': 0.0563,
    'crane_x7_lower_arm_revolute_part_joint': 0.4817,
    'crane_x7_shoulder_fixed_part_pan_joint': 0.0,
    'crane_x7_shoulder_revolute_part_tilt_joint': 0.3979,
    'crane_x7_upper_arm_revolute_part_rotate_joint': -2.1994,
    'crane_x7_upper_arm_revolute_part_twist_joint': 0.0,
    'crane_x7_wrist_joint': 0.0,
}

# 盤面・ピース・ケースの位置の組み合わせ。ケースの位置は base_link から見たケースの角 [m]
# rviz は config/ の RViz 設定（カメラの位置だけが違う）
PRESETS = {
    # 中間発表の目標。ケースはロボットの正面 16〜24cm
    '4x4': {
        'rows': 4, 'cols': 4, 'pieces': 'I:1,S:1,L:2',
        'case_x': 0.16, 'case_y': -0.04,
        'step_time': 2.0,
        'rviz': 'solver_viz.rviz',
    },
    # 最終の 8×8。ピースは solver_demo と同じ16個（T は偶数）。ケースはロボットの正面 15〜31cm
    '8x8': {
        'rows': 8, 'cols': 8, 'pieces': 'I:2,O:2,T:4,S:2,Z:2,J:2,L:2',
        'case_x': 0.15, 'case_y': -0.08,
        'tray_columns': 6,
        'step_time': 1.2,
        'rviz': 'solver_viz_8x8.rviz',
    },
}

# preset の値を上書きできる引数と、その型
OVERRIDES = {'rows': int, 'cols': int, 'pieces': str, 'step_time': float}


def make_nodes(context):
    params = dict(PRESETS[LaunchConfiguration('preset').perform(context)])
    rviz_config = params.pop('rviz')
    for name, cast in OVERRIDES.items():
        value = LaunchConfiguration(name).perform(context)
        if value:
            params[name] = cast(value)

    solver_viz = Node(
        package='tukamukai',
        executable='solver_viz',
        output='screen',
        parameters=[params],
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='solver_viz_rviz',
        arguments=['-d', get_package_share_directory('tukamukai') + '/config/' + rviz_config],
        output='log',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )
    return [solver_viz, rviz]


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            'preset', default_value='4x4', choices=list(PRESETS),
            description='盤面とピースの組み合わせ'),
        DeclareLaunchArgument(
            'with_robot', default_value='true', choices=['true', 'false'],
            description='CRANE-X7 のモデルを home 姿勢で出す。Gazebo・実機と一緒に使うときは false'),
        DeclareLaunchArgument(
            'rviz', default_value='true', choices=['true', 'false'],
            description='RViz を開く'),
        DeclareLaunchArgument('rows', default_value='', description='盤面の行数（preset を上書き）'),
        DeclareLaunchArgument('cols', default_value='', description='盤面の列数（preset を上書き）'),
        DeclareLaunchArgument(
            'pieces', default_value='', description='使うピース 例 "I:1,S:1,L:2"（preset を上書き）'),
        DeclareLaunchArgument(
            'step_time', default_value='', description='1ピースを運ぶ時間 [s]（preset を上書き）'),
    ]

    with_robot = IfCondition(LaunchConfiguration('with_robot'))
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': RobotDescriptionLoader().load()}],
        condition=with_robot,
    )
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'zeros': HOME_POSE}],
        condition=with_robot,
    )

    return LaunchDescription(
        args + [
            OpaqueFunction(function=make_nodes),
            robot_state_publisher, joint_state_publisher,
        ])
