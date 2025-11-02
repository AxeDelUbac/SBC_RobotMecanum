#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
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
    
    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='false',
        description='Use GUI monitor (true) or console monitor (false)'
    )
    
    # Console monitor node (par défaut)
    console_monitor_node = Node(
        package='mecanum_robot',
        executable='console_monitor.py',
        name='console_monitor',
        parameters=[
            {'monitoring_ip': LaunchConfiguration('monitoring_ip')}
        ],
        output='screen'
    )
    
    return LaunchDescription([
        monitoring_ip_arg,
        use_gui_arg,
        console_monitor_node,
    ])