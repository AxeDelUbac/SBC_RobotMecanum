#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    pkg_mecanum_robot = get_package_share_directory('mecanum_robot')
    
    # Launch arguments
    use_lidar_arg = DeclareLaunchArgument(
        'use_lidar',
        default_value='true',
        description='Whether to start the lidar node'
    )
    
    robot_mode_arg = DeclareLaunchArgument(
        'robot_mode',
        default_value='raspberry',
        description='Mode: raspberry (sur RPi) ou monitoring (machine distante)'
    )
    
    # Configuration files
    rplidar_config = os.path.join(pkg_mecanum_robot, 'config', 'rplidar_config.yaml')
    robot_config = os.path.join(pkg_mecanum_robot, 'config', 'robot_config.yaml')
    
    # RPlidar node
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        parameters=[rplidar_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_lidar'))
    )
    
    # Robot controller (sur Raspberry Pi)
    robot_controller_node = Node(
        package='mecanum_robot',
        executable='robot_controller.py',
        name='robot_controller',
        parameters=[robot_config],
        output='screen',
        condition=IfCondition(
            PythonExpression([
                '"', LaunchConfiguration('robot_mode'), '" == "raspberry"'
            ])
        )
    )
    
    # Lidar processor
    lidar_processor_node = Node(
        package='mecanum_robot',
        executable='lidar_processor.py',
        name='lidar_processor',
        parameters=[robot_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_lidar'))
    )
    
    # Communication bridge
    communication_bridge_node = Node(
        package='mecanum_robot',
        executable='communication_bridge.py',
        name='communication_bridge',
        parameters=[robot_config],
        output='screen'
    )
    
    return LaunchDescription([
        use_lidar_arg,
        robot_mode_arg,
        rplidar_node,
        robot_controller_node,
        lidar_processor_node,
        communication_bridge_node,
    ])