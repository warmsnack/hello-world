#!/usr/bin/env python3
"""Phase 1 bring-up: start Gazebo Classic with the disturbance world and spawn
the noisy / low-friction TurtleBot3 Waffle.

Responsibilities:
    * launch gzserver + gzclient with worlds/disturbance.world
    * publish the robot description (robot_state_publisher) for TF of the
      fixed/wheel frames (the diff_drive plugin no longer publishes them)
    * spawn the custom model.sdf (sensor noise + wheel slip + no odom TF)

This launch file only builds the *simulator*. The state-estimation pipeline
(EKF vs. ground-truth bridge), goal client and rosbag recording live in
tb3_experiment/launch/experiment.launch.py, which includes this file.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_this = get_package_share_directory('tb3_disturbance_gazebo')

    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    world = LaunchConfiguration('world')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use the /clock published by Gazebo.')
    declare_gui = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Launch the Gazebo client GUI (set false for headless).')
    declare_world = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_this, 'worlds', 'disturbance.world'),
        description='Full path to the world file to load.')
    declare_x = DeclareLaunchArgument('x_pose', default_value='0.0')
    declare_y = DeclareLaunchArgument('y_pose', default_value='0.0')

    # --- Robot description (URDF) from turtlebot3_description, waffle variant. ---
    # Requires TURTLEBOT3_MODEL=waffle in the environment.
    tb3_desc = get_package_share_directory('turtlebot3_description')
    urdf_path = os.path.join(tb3_desc, 'urdf', 'turtlebot3_waffle.urdf')
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    # --- Gazebo server + client ---
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world, 'verbose': 'true'}.items(),
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(gui),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
        }],
    )

    # Spawn the *custom* disturbance SDF (noise + slip + publish_odom_tf=false).
    model_sdf = os.path.join(
        pkg_this, 'models', 'turtlebot3_waffle', 'model.sdf')
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_turtlebot3',
        output='screen',
        arguments=[
            '-entity', 'turtlebot3_waffle',
            '-file', model_sdf,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01',
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_gui,
        declare_world,
        declare_x,
        declare_y,
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_entity,
    ])
