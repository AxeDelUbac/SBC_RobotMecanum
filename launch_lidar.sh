#!/bin/bash

# Script de lancement du lidar avec environnement correct
cd /home/maxence/Documents/lerobot/SBC_RobotMecanum
source install/setup.bash
ros2 launch mecanum_robot lidar_launch.py