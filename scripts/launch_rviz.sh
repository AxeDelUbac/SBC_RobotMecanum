#!/bin/bash

# Script simplifié pour lancer RViz
echo "Lancement de RViz..."

# Supprimer les références problématiques temporairement
export SHELL=/bin/bash

# Aller dans le bon répertoire
cd /home/maxence/Documents/Dev/LEROBOT/SBC_RobotMecanum

# Source uniquement ROS2
bash -c "source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch mecanum_robot rviz_launch.py"