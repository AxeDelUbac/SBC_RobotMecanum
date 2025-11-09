#!/bin/bash

# Script pour lancer Gazebo et RViz simultanément
echo "Lancement de Gazebo et RViz simultanément..."

# Supprimer les références problématiques temporairement
export SHELL=/bin/bash

# Aller dans le bon répertoire
cd /home/maxence/Documents/Dev/LEROBOT/SBC_RobotMecanum

# Function pour arrêter tous les processus en cas d'interruption
cleanup() {
    echo "Arrêt de tous les processus..."
    pkill -f "gazebo_launch.py"
    pkill -f "rviz_launch.py" 
    pkill -f "gz sim"
    pkill -f "rviz2"
    exit 0
}

# Intercepter Ctrl+C pour nettoyer
trap cleanup SIGINT

echo "Lancement de Gazebo en arrière-plan..."
# Lancer Gazebo en arrière-plan
bash -c "source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch mecanum_robot gazebo_launch.py" &
GAZEBO_PID=$!

# Attendre que Gazebo soit initialisé
echo "Attente de l'initialisation de Gazebo (5 secondes)..."
sleep 5

echo "Lancement de RViz..."
# Lancer RViz au premier plan
bash -c "source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch mecanum_robot rviz_launch.py" &
RVIZ_PID=$!

echo "Gazebo et RViz lancés ! Appuyez sur Ctrl+C pour arrêter les deux."
echo "PID Gazebo: $GAZEBO_PID"
echo "PID RViz: $RVIZ_PID"

# Attendre que l'un des processus se termine
wait