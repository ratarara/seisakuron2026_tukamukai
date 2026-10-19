# Based on crane_x7_examples/launch/example.launch.py
# Copyright 2022 RT Corporation, licensed under the Apache License, Version 2.0

from crane_x7_description.robot_description_loader import RobotDescriptionLoader
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder

# ノードに渡す設定: 名前 -> (既定値, 型, 説明)
# 実行時に  名前:=値  の形で変えられる。 例: ros2 launch tukamukai pick_place.launch.py mode:=corners
NODE_ARGS = {
    'mode': (
        'single', str,
        'home | single | corners | rotations | gripper | gripper_sweep | gripper_cycle | '
        'hand_down | lower | lift_place | pick_move',
    ),
    'speed_scale': ('1.0', float, 'speed scale 0.0-1.0 (use 0.1-0.3 on the real robot)'),
    'step_by_step': ('false', bool, 'true: confirm each motion with Enter (opens xterm)'),
    'origin_x': ('0.19', float, 'robot x [m] of the center of cell (row 0, col 0)'),
    'origin_y': ('-0.105', float, 'robot y [m] of the center of cell (row 0, col 0)'),
    'cell_size': ('0.03', float, 'size of one cell [m]'),
    'yaw_at_rot0': ('-90.0', float, 'hand yaw [deg] when the solver rotation is 0'),
    'yaw_sign': ('-1.0', float, '+1 or -1: direction of yaw for solver clockwise rotation'),
    'row': ('0.0', float, 'target row (single / rotations mode)'),
    'col': ('0.0', float, 'target column (single / rotations mode)'),
    'rotation': ('0.0', float, 'solver rotation [deg] (single mode)'),
    'place_z': ('0.13', float, 'hand height [m] when placing'),
    'pick_x': ('0.25', float, 'single / pick_move mode: hand x [m] when picking'),
    'pick_y': ('0.0', float, 'single / pick_move mode: hand y [m] when picking'),
    'pick_z': ('0.105', float, 'single / pick_move mode: hand height [m] when picking'),
    'pick_yaw': ('-90.0', float, 'single / pick_move mode: hand yaw [deg] when picking'),
    'lift_z': ('0.3', float, 'hand height [m] when moving above the pieces'),
    'gripper_open_deg': ('20.0', float, 'gripper angle [deg] when open (0 = closed)'),
    'gripper_grasp_deg': ('10.0', float, 'gripper angle [deg] when grasping the handle'),
    'sweep_step_deg': ('1.0', float, 'gripper_sweep mode: closing step [deg] per motion'),
    'sweep_interval_sec': ('0.5', float, 'gripper_sweep mode: wait [s] after each motion'),
    'cycle_max_deg': ('90.0', float, 'gripper_cycle mode: max gripper angle [deg] (limit: 90)'),
    'cycle_min_deg': ('-2.0', float, 'gripper_cycle mode: min gripper angle [deg] (0 = closed)'),
    'cycle_count': ('1', int, 'gripper_cycle mode: number of repetitions'),
    'down_x': ('0.25', float, 'hand_down mode: hand x [m]'),
    'down_y': ('0.0', float, 'hand_down mode: hand y [m]'),
    'down_z': ('0.2', float, 'hand_down mode: hand height [m]'),
    'down_gripper_deg': ('-2.0', float, 'hand_down mode: gripper angle [deg] while moving'),
    'down_end_gripper_deg': ('10.0', float, 'hand_down mode: gripper angle [deg] at the end'),
    'down_elbow_min_deg': ('-135.0', float, 'hand_down / lower mode: min elbow angle [deg]'),
    'down_elbow_max_deg': ('-45.0', float, 'hand_down / lower mode: max elbow angle [deg]'),
    'lower_start_z': ('0.2', float, 'lower mode: start hand height [m]'),
    'lower_min_z': ('0.09', float, 'lower mode: lowest hand height [m] (fingertip: ~6 mm up)'),
    'lower_speed': ('0.01', float, 'lower mode: going-down speed [m/s]'),
    'lift_dz': ('0.05', float, 'lift_place / pick_move mode: how high to lift [m]'),
    'lift_speed': ('0.02', float, 'lift_place / pick_move mode: lifting speed [m/s]'),
    'place_dx': ('0.0', float, 'pick_move mode: move forward by [m] before placing'),
    'place_dy': ('0.13', float, 'pick_move mode: move left by [m] before placing'),
    'pick_grasp_deg': ('-2.0', float, 'pick_move mode: gripper angle [deg] to grasp'),
}


def generate_launch_description():
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        choices=['true', 'false'],
        description='Set true when using the gazebo simulator.',
    )

    declare_node_args = []
    node_params = {}
    for name, (default, value_type, description) in NODE_ARGS.items():
        declare_node_args.append(
            DeclareLaunchArgument(name, default_value=default, description=description)
        )
        node_params[name] = ParameterValue(LaunchConfiguration(name), value_type=value_type)

    description_loader = RobotDescriptionLoader()

    moveit_config = MoveItConfigsBuilder('crane_x7').to_moveit_configs()
    moveit_config.robot_description = {
        'robot_description': description_loader.load(),
    }

    # Enter の入力を受け取るモードかどうか（ステップ実行、または Enter で止めるモード）
    use_keyboard = PythonExpression(
        [
            "'", LaunchConfiguration('step_by_step'), "' == 'true' or '",
            LaunchConfiguration('mode'), "' in ('gripper_sweep', 'lower')",
        ]
    )

    def make_node(**kwargs):
        return Node(
            name='tukamukai_pick_place',
            package='tukamukai',
            executable='pick_place_node',
            output='screen',
            parameters=[moveit_config.to_dict(), node_params],
            **kwargs,
        )

    # 通常: そのまま起動する
    normal_node = make_node(condition=UnlessCondition(use_keyboard))

    # キーボード入力を使うとき: ros2 launch はキーボード入力をノードに渡せないので、
    # xterm の新しいウィンドウの中でノードを起動し、そこで Enter を受け取る。
    # -hold は、終了後もウィンドウを閉じずに、ログを読めるようにするため。
    step_node = make_node(
        condition=IfCondition(use_keyboard),
        prefix='xterm -hold -e',
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
            *declare_node_args,
            normal_node,
            step_node,
        ]
    )
