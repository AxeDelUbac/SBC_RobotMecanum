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
    monitoring_ip_arg = DeclareLaunchArgument(
        'monitoring_ip',
        default_value='192.168.1.100',
        description='IP address of the monitoring machine'
    )
    
    # Configuration files
    robot_config = os.path.join(pkg_mecanum_robot, 'config', 'robot_config.yaml')
    
    # Remote monitor node (sur machine de monitoring)
    remote_monitor_node = Node(
        package='mecanum_robot',
        executable='remote_monitor.py',
        name='remote_monitor',
        parameters=[
            robot_config,
            {'monitoring_ip': LaunchConfiguration('monitoring_ip')}
        ],
        output='screen'
    )
    
    return LaunchDescription([
        monitoring_ip_arg,
        remote_monitor_node,
    ])