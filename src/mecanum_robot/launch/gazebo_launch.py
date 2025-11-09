#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    
    # Configure ROS arguments
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=os.path.join(
            get_package_share_directory('mecanum_robot'), 'urdf', 'mecanum_robot.urdf.xacro'),
        description='Absolute path to robot urdf file')
    
    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        choices=['true', 'false'],
        description='Flag to enable use_sim_time')

    # Robot description
    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Start Gazebo
    gazebo_process = ExecuteProcess(
        cmd=['gz', 'sim', 'empty.sdf', '-v', '4'],
        output='screen'
    )

    # Spawn robot in Gazebo (delay to ensure Gazebo is running)
    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'mecanum_robot',
            '-z', '0.1'
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    return LaunchDescription([
        model_arg,
        use_sim_time_arg,
        robot_state_publisher_node,
        gazebo_process,
        spawn_entity_node
    ])