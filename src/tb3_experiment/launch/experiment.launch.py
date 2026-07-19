#!/usr/bin/env python3
"""Phase 4 - Integrated experiment launch (A/B controlled).

Single entry point that runs the full pipeline. The key switch is `use_gt`:

    use_gt:=false  (Mode B, default) -> robot_localization EKF owns the
                                        odom->base_footprint TF (real state
                                        estimation under noise + slip).
    use_gt:=true   (Mode A)          -> gt_bridge_node owns that TF from Gazebo
                                        ground truth (a "perfect estimator").

In BOTH modes the gt_bridge still publishes /gt_odom (with publish_tf matched to
the mode) so the evaluator and evo always have a ground-truth reference.

Everything else (Gazebo world, Nav2, evaluator, automatic goal, optional rosbag)
is identical between modes, so any difference in avoidance performance is
attributable to the state estimator.

Common invocations:
    ros2 launch tb3_experiment experiment.launch.py use_gt:=false record:=true
    ros2 launch tb3_experiment experiment.launch.py use_gt:=true  record:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_sim = get_package_share_directory('tb3_disturbance_gazebo')
    pkg_est = get_package_share_directory('tb3_state_estimation')
    pkg_exp = get_package_share_directory('tb3_experiment')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    use_gt = LaunchConfiguration('use_gt')
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    record = LaunchConfiguration('record')
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    nav2_params = LaunchConfiguration('nav2_params')
    csv_dir = LaunchConfiguration('csv_dir')

    # 'A' when use_gt is true, else 'B'. Used for CSV/bag naming.
    mode_label = PythonExpression(["'A' if '", use_gt, "' == 'true' else 'B'"])
    bag_uri = PythonExpression(
        ["'", csv_dir, "' + '/rosbag_mode_' + ('A' if '", use_gt,
         "' == 'true' else 'B')"])

    declare_args = [
        DeclareLaunchArgument(
            'use_gt', default_value='false',
            description='true=Mode A (ground-truth TF), false=Mode B (EKF TF).'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument(
            'record', default_value='false',
            description='If true, run `ros2 bag record -a` (SQLite3).'),
        DeclareLaunchArgument('goal_x', default_value='5.0',
                              description='Straight-ahead goal distance [m].'),
        DeclareLaunchArgument('goal_y', default_value='0.0'),
        DeclareLaunchArgument(
            'nav2_params',
            default_value=os.path.join(pkg_exp, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument(
            'csv_dir',
            default_value=os.path.join(os.path.expanduser('~'), 'tb3_eval')),
    ]

    # ---- 1. Simulator (Gazebo + spawn noisy/slip robot) ----
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sim, 'launch', 'disturbance_world.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'gui': gui,
        }.items(),
    )

    # ---- 2. Static identity map->odom (map-less Nav2) ----
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ---- 3a. Mode B: EKF (only when use_gt == false) ----
    ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_est, 'launch', 'ekf.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=UnlessCondition(use_gt),
    )

    # ---- 3b. Ground-truth bridge ----
    # Mode A: publishes /gt_odom AND the odom->base_footprint TF (perfect loc).
    gt_bridge_mode_a = Node(
        package='tb3_experiment',
        executable='gt_bridge_node',
        name='gt_bridge_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'publish_tf': True}],
        condition=IfCondition(use_gt),
    )
    # Mode B: still publishes /gt_odom as the evaluation reference, but the EKF
    # owns the TF, so publish_tf=false here.
    gt_bridge_mode_b = Node(
        package='tb3_experiment',
        executable='gt_bridge_node',
        name='gt_bridge_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'publish_tf': False}],
        condition=UnlessCondition(use_gt),
    )

    # ---- 4. Nav2 (map-less; global_frame=odom) ----
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params,
            'autostart': 'true',
        }.items(),
    )

    # ---- 5. Clearance evaluator (logs CSV, both modes) ----
    evaluator = Node(
        package='tb3_experiment',
        executable='evaluator_node',
        name='evaluator_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'mode_label': mode_label,
            'rate_hz': 10.0,
            'output_csv': PythonExpression(
                ["'", csv_dir, "' + '/clearance_' + ('A' if '", use_gt,
                 "' == 'true' else 'B') + '.csv'"]),
        }],
    )

    # ---- 6. Automatic goal (start after the stack settles) ----
    goal_client = TimerAction(
        period=8.0,
        actions=[Node(
            package='tb3_experiment',
            executable='goal_pose_client',
            name='goal_pose_client',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'goal_x': goal_x,
                'goal_y': goal_y,
                'frame_id': 'map',
                'delay_s': 1.0,
            }],
        )],
    )

    # ---- 7. rosbag record -a (SQLite3), optional ----
    rosbag = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a', '-s', 'sqlite3', '-o', bag_uri],
        output='screen',
        condition=IfCondition(record),
    )

    return LaunchDescription([
        *declare_args,
        sim,
        map_to_odom,
        ekf,
        gt_bridge_mode_a,
        gt_bridge_mode_b,
        nav2,
        evaluator,
        goal_client,
        rosbag,
    ])
