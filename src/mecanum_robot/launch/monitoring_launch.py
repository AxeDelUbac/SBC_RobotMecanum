#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Launch file pour la machine de monitoring.
    Ce fichier lance uniquement les outils de visualisation et de monitoring,
    sans les capteurs physiques.
    """
    
    # Configuration réseau ROS2 automatique pour monitoring
    ros_domain_id = SetEnvironmentVariable('ROS_DOMAIN_ID', '42')
    ros_discovery_range = SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'SUBNET')
    ros_localhost_only = SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '0')
    
    # Get package directory
    pkg_mecanum_robot = get_package_share_directory('mecanum_robot')
    
    # Launch arguments
    robot_mode_arg = DeclareLaunchArgument(
        'robot_mode',
        default_value='monitoring',
        description='Mode: monitoring (machine distante de visualisation)'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz for visualization'
    )
    
    # RViz pour visualisation
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )
    
    # Static transform publisher pour visualisation dans RViz
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
        output='screen'
    )
    
    return LaunchDescription([
        # Configuration réseau
        ros_domain_id,
        ros_discovery_range,
        ros_localhost_only,
        # Arguments de lancement
        robot_mode_arg,
        use_rviz_arg,
        # Nodes
        static_transform_publisher,
        # rviz_node,  # Décommenté si vous voulez lancer RViz automatiquement
    ])