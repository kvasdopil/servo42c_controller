#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('servo42c_controller')
    
    # Define paths for config and URDF files
    urdf_file = os.path.join(pkg_dir, 'description', 'arm.urdf')
    config_file = os.path.join(pkg_dir, 'config', 'servo_config.yaml')
    controllers_file = os.path.join(pkg_dir, 'config', 'controllers.yaml')
    
    # Create the launch description with all actions
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[urdf_file, controllers_file],
            output="screen"
        ),
        # Joint state broadcaster must be started before trajectory controller
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["arm_trajectory_controller"],
            output="screen",
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time', default='false'), 
                'robot_description': open(urdf_file).read()
            }]
        ),
        Node(
            package='servo42c_controller',
            executable='servo42c_controller',
            name='servo42c_controller',
            output='screen',
            parameters=[config_file]
        )
    ])
