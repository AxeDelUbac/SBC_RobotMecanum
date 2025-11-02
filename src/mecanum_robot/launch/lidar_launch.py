#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Configuration réseau ROS2 automatique
    ros_domain_id = SetEnvironmentVariable('ROS_DOMAIN_ID', '42')
    ros_discovery_range = SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'SUBNET')
    ros_localhost_only = SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '0')
    
    # Get package directory
    pkg_mecanum_robot = get_package_share_directory('mecanum_robot')
    
    # Launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for the RPlidar'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='laser_frame',
        description='Frame ID for the laser scan'
    )
    
    # Configuration file
    rplidar_config = os.path.join(pkg_mecanum_robot, 'config', 'rplidar_config.yaml')
    
    # RPlidar node
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        parameters=[
            rplidar_config,
            {
                'serial_port': LaunchConfiguration('serial_port'),
                'frame_id': LaunchConfiguration('frame_id')
            }
        ],
        output='screen',
        remappings=[
            # Utiliser le topic standard /scan (pas de remapping nécessaire)
        ]
    )
    
    # Static transform publisher pour le frame du lidar
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_frame_publisher',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser_frame'],
        output='screen'
    )
    
    return LaunchDescription([
        # Configuration réseau
        ros_domain_id,
        ros_discovery_range,
        ros_localhost_only,
        # Arguments de lancement
        serial_port_arg,
        frame_id_arg,
        rplidar_node,
        static_transform_publisher,
    ])