#!/bin/bash

# Script de démonstration pour tester la configuration simplifiée
# Execute ce script sur la machine de monitoring

echo "=== Test de la configuration simplifiée (Gazebo/RViz) ==="

# Couleurs
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

# Vérifier ROS2
echo -e "${YELLOW}[TEST]${NC} Vérification de ROS2..."
if command -v ros2 > /dev/null; then
    echo -e "${GREEN}✓${NC} ROS2 trouvé"
    ros2 --version
else
    echo -e "${RED}✗${NC} ROS2 non trouvé"
    exit 1
fi

# Vérifier la compilation
echo -e "${YELLOW}[TEST]${NC} Vérification de la compilation..."
cd ~/Documents/Dev/LEROBOT/SBC_RobotMecanum
if colcon build --symlink-install; then
    echo -e "${GREEN}✓${NC} Compilation réussie"
else
    echo -e "${RED}✗${NC} Erreur de compilation"
    exit 1
fi

# Vérifier les outils de visualisation
echo -e "${YELLOW}[TEST]${NC} Vérification des outils de visualisation..."

if command -v rviz2 > /dev/null; then
    echo -e "${GREEN}✓${NC} RViz2 disponible"
else
    echo -e "${RED}✗${NC} RViz2 non trouvé - installer avec: sudo apt install ros-jazzy-rviz2"
fi

if command -v gazebo > /dev/null; then
    echo -e "${GREEN}✓${NC} Gazebo disponible"
else
    echo -e "${YELLOW}!${NC} Gazebo non trouvé - installer avec: sudo apt install ros-jazzy-gazebo-ros-pkgs"
fi

if command -v rqt > /dev/null; then
    echo -e "${GREEN}✓${NC} rqt disponible"
else
    echo -e "${YELLOW}!${NC} rqt non trouvé - installer avec: sudo apt install ros-jazzy-rqt"
fi

# Test du lancement des nodes
echo -e "${YELLOW}[TEST]${NC} Test de lancement des nodes..."
source install/setup.bash

# Tester le lancement (sans l'exécuter complètement)
if ros2 launch mecanum_robot console_monitoring_launch.py --dry-run; then
    echo -e "${GREEN}✓${NC} Fichier de lancement valide"
else
    echo -e "${RED}✗${NC} Problème avec le fichier de lancement"
fi

# Vérifier les scripts
echo -e "${YELLOW}[TEST]${NC} Vérification des scripts créés..."

scripts=("start_monitoring.sh" "start_gazebo_rviz.sh" "check_robot_connection.sh")
for script in "${scripts[@]}"; do
    if [ -f ~/"$script" ]; then
        echo -e "${GREEN}✓${NC} $script créé"
    else
        echo -e "${RED}✗${NC} $script manquant"
    fi
done

echo ""
echo -e "${GREEN}=== Configuration simplifiée prête ! ===${NC}"
echo ""
echo "Utilisation recommandée :"
echo "1. Pour le monitoring console : ~/start_monitoring.sh"
echo "2. Pour RViz/Gazebo : ~/start_gazebo_rviz.sh"
echo "3. Pour vérifier la connexion : ~/check_robot_connection.sh"
echo ""
echo "Topics ROS2 à surveiller :"
echo "  - /scan (données lidar)"
echo "  - /cmd_vel (commandes vitesse)"
echo "  - /robot_status (état robot)"