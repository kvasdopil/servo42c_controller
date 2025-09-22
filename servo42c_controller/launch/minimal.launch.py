#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_dir = get_package_share_directory('servo42c_controller')
    config_file = os.path.join(pkg_dir, 'config', 'servo_config.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        ),
        Node(
            package='servo42c_controller',
            executable='servo42c_controller',
            name='servo42c_controller',
            output='screen',
            parameters=[config_file]
        ),
    ])



