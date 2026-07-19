#!/usr/bin/env python3
"""Phase 2 - launch the robot_localization EKF (Mode B: real state estimation).

Subscribes to /odom_unfiltered (wheel encoders) and /imu (gyro), and publishes:
    * /odometry/filtered  (nav_msgs/Odometry)
    * odom -> base_footprint TF
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('tb3_state_estimation')
    default_ekf = os.path.join(pkg, 'config', 'ekf.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    ekf_config = LaunchConfiguration('ekf_config')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('ekf_config', default_value=default_ekf),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config, {'use_sim_time': use_sim_time}],
            remappings=[
                # Publish the fused estimate on the conventional /odom topic so
                # Nav2 and RViz consume the EKF output, not the raw wheel odom.
                ('odometry/filtered', '/odom'),
            ],
        ),
    ])
