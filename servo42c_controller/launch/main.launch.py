#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('servo42c_controller')
    package_name = 'servo42c_controller'
    
    # Define paths for config and URDF files
    urdf_file = os.path.join(pkg_dir, 'description', 'arm.urdf')
    config_file = os.path.join(pkg_dir, 'config', 'servo_config.yaml')
    controllers_file = os.path.join(pkg_dir, 'config', 'controllers.yaml')
    
    # --- MoveIt Configuration --- 
    moveit_config = (
        MoveItConfigsBuilder(package_name, package_name=package_name)
        .robot_description(file_path=os.path.join(pkg_dir, "description", "arm.urdf"))
        .robot_description_semantic(file_path=os.path.join(pkg_dir, "config", "servo42c_controller.srdf"))
        .trajectory_execution(file_path=os.path.join(pkg_dir, "config", "moveit_controllers.yaml"))
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    
    # Get the dictionary, ensure 'ompl' key exists, and add planning_plugins
    moveit_configs_dict = moveit_config.to_dict()
    ompl_config = moveit_configs_dict.setdefault("ompl", {})
    ompl_config["planning_plugins"] = ["ompl_interface/OMPLPlanner"]

    # --- End MoveIt Configuration ---

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
            parameters=[moveit_config.robot_description, controllers_file],
            output="screen"
        ),
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
            parameters=[controllers_file],
            output="screen",
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time', default='false'), 
                'robot_description': moveit_config.robot_description["robot_description"]
            }]
        ),
        Node(
            package='servo42c_controller',
            executable='servo42c_controller',
            name='servo42c_controller',
            output='screen',
            parameters=[config_file]
        ), # Removed standalone node
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                moveit_configs_dict,
                {"moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager"}
            ],
        )
    ])
