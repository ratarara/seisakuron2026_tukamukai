# ソルバーの解を RViz に表示する（solver_viz ノード）
#
# ロボットなし（CRANE-X7 のモデルを home 姿勢で表示）:
#   ros2 launch tukamukai solver_viz.launch.py
# Gazebo や実機と一緒に（ロボットのモデルはそちらのものを使う）:
#   ros2 launch tukamukai solver_viz.launch.py with_robot:=false
# ピースや盤面を変える:
#   ros2 launch tukamukai solver_viz.launch.py rows:=8 cols:=8 \
#     pieces:="I:2,O:2,T:4,S:2,Z:2,J:2,L:2"

from ament_index_python.packages import get_package_share_directory
from crane_x7_description.robot_description_loader import RobotDescriptionLoader
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

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


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            'with_robot', default_value='true', choices=['true', 'false'],
            description='CRANE-X7 のモデルを home 姿勢で出す。Gazebo・実機と一緒に使うときは false'),
        DeclareLaunchArgument(
            'rviz', default_value='true', choices=['true', 'false'],
            description='RViz を開く'),
        DeclareLaunchArgument('rows', default_value='4', description='盤面の行数'),
        DeclareLaunchArgument('cols', default_value='4', description='盤面の列数'),
        DeclareLaunchArgument(
            'pieces', default_value='I:1,S:1,L:2', description='使うピース（種類:個数）'),
        DeclareLaunchArgument(
            'step_time', default_value='2.0', description='1ピースを運ぶ時間 [s]'),
    ]

    solver_viz = Node(
        package='tukamukai',
        executable='solver_viz',
        output='screen',
        parameters=[{
            'rows': ParameterValue(LaunchConfiguration('rows'), value_type=int),
            'cols': ParameterValue(LaunchConfiguration('cols'), value_type=int),
            'pieces': ParameterValue(LaunchConfiguration('pieces'), value_type=str),
            'step_time': ParameterValue(LaunchConfiguration('step_time'), value_type=float),
        }],
    )

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

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='solver_viz_rviz',
        arguments=['-d', get_package_share_directory('tukamukai') + '/config/solver_viz.rviz'],
        output='log',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription(
        args + [solver_viz, robot_state_publisher, joint_state_publisher, rviz])
