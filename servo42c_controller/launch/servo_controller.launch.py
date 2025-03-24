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
    
    # Create launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Declare the launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Load URDF into parameter server
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': open(urdf_file).read()}]
    )
    
    # Launch servo controller node
    servo_controller_node = Node(
        package='servo42c_controller',
        executable='servo42c_controller',
        name='servo42c_controller',
        output='screen',
        parameters=[config_file]
    )
    
    # Create the launch description with all actions
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_use_sim_time)
    
    # Add nodes to launch description
    ld.add_action(robot_state_publisher_node)
    ld.add_action(servo_controller_node)
    
    return ld 