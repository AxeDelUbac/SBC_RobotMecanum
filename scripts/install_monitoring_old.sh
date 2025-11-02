#!/bin/bash

# Script d'installation pour la machine de monitoring
# À exécuter sur la machine de monitoring distant

set -e

echo "=== Installation Machine de Monitoring ==="

# Couleurs pour l'affichage
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Fonction d'affichage
print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Vérifier si ROS2 est installé
check_ros2() {
    print_status "Vérification de l'installation ROS2 Jazzy..."
    if [ -f "/opt/ros/jazzy/setup.bash" ]; then
        print_status "ROS2 Jazzy trouvé"
        source /opt/ros/jazzy/setup.bash
    else
        print_error "ROS2 Jazzy non trouvé. Veuillez l'installer d'abord."
        exit 1
    fi
}

# Installer les dépendances système
install_system_dependencies() {
    print_status "Installation des dépendances système..."
    sudo apt update
    
    # Packages essentiels pour monitoring avec Gazebo/RViz
    sudo apt install -y \
        python3-pip \
        python3-numpy \
        git
}

# Installer les dépendances Python
install_python_dependencies() {
    print_status "Installation des dépendances Python..."
    pip3 install --user \
        numpy \
        matplotlib
}

# Compiler le workspace ROS2
build_workspace() {
    print_status "Compilation du workspace ROS2..."
    cd ~/Documents/Dev/LEROBOT/SBC_RobotMecanum
    
    # Source ROS2
    source /opt/ros/jazzy/setup.bash
    
    # Compiler
    colcon build --symlink-install
    
    if [ $? -eq 0 ]; then
        print_status "Compilation réussie"
    else
        print_error "Erreur lors de la compilation"
        exit 1
    fi
}

# Configurer l'environnement
setup_environment() {
    print_status "Configuration de l'environnement..."
    
    # Ajouter au bashrc
    if ! grep -q "# Robot Mecanum Monitoring Environment" ~/.bashrc; then
        echo "" >> ~/.bashrc
        echo "# Robot Mecanum Monitoring Environment" >> ~/.bashrc
        echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
        echo "source ~/Documents/Dev/LEROBOT/SBC_RobotMecanum/install/setup.bash" >> ~/.bashrc
        echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
        echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc
    fi
    
    # Créer le script de lancement pour monitoring console
    cat > ~/start_monitoring.sh << 'EOF'
#!/bin/bash
source /opt/ros/jazzy/setup.bash
source ~/Documents/Dev/LEROBOT/SBC_RobotMecanum/install/setup.bash
export ROS_DOMAIN_ID=42

echo "Démarrage du monitoring console..."
echo "Utilisez Gazebo et RViz pour la visualisation"
echo "Tapez 'help' pour voir les commandes disponibles"
ros2 launch mecanum_robot console_monitoring_launch.py
EOF
    chmod +x ~/start_monitoring.sh
    
    # Script pour lancer Gazebo et RViz
    cat > ~/start_gazebo_rviz.sh << 'EOF'
#!/bin/bash
source /opt/ros/jazzy/setup.bash
source ~/Documents/Dev/LEROBOT/SBC_RobotMecanum/install/setup.bash
export ROS_DOMAIN_ID=42

echo "Démarrage de Gazebo et RViz pour monitoring du robot..."

# Lancer RViz en arrière-plan
rviz2 &

echo "RViz lancé. Configurez-le pour afficher:"
echo "  - /scan (sensor_msgs/LaserScan)"
echo "  - /robot_status (pour le statut)"
echo "  - /tf (pour les transformations)"
echo ""
echo "Pour Gazebo, utilisez: gazebo --verbose"
EOF
    chmod +x ~/start_gazebo_rviz.sh
    
    # Script pour vérifier la connexion réseau
    cat > ~/check_robot_connection.sh << 'EOF'
#!/bin/bash
ROBOT_IP="192.168.1.101"
ROBOT_PORT="11511"

echo "Vérification de la connexion au robot..."
echo "IP: $ROBOT_IP, Port: $ROBOT_PORT"

if timeout 5 bash -c "</dev/tcp/$ROBOT_IP/$ROBOT_PORT"; then
    echo "✓ Robot accessible"
else
    echo "✗ Robot non accessible"
    echo "Vérifiez:"
    echo "  - La connexion réseau"
    echo "  - L'IP du robot dans la configuration"
    echo "  - Que le robot est démarré"
fi

echo ""
echo "Test de connectivité ROS2:"
timeout 10 ros2 topic list
EOF
    chmod +x ~/check_robot_connection.sh
}

# Créer les raccourcis bureau
create_desktop_shortcuts() {
    print_status "Création des raccourcis bureau..."
    
    # Raccourci pour le monitoring
    cat > ~/Desktop/Robot_Monitoring.desktop << EOF
[Desktop Entry]
Version=1.0
Type=Application
Name=Robot Monitoring
Comment=Interface de monitoring du robot Mecanum
Exec=$HOME/start_monitoring.sh
Icon=applications-development
Terminal=true
Categories=Development;
EOF
    chmod +x ~/Desktop/Robot_Monitoring.desktop
    
    # Raccourci pour vérifier la connexion
    cat > ~/Desktop/Check_Robot_Connection.desktop << EOF
[Desktop Entry]
Version=1.0
Type=Application
Name=Check Robot Connection
Comment=Vérifier la connexion au robot
Exec=$HOME/check_robot_connection.sh
Icon=network-wired
Terminal=true
Categories=Development;
EOF
    # Raccourci pour le monitoring console
    cat > ~/Desktop/Robot_Console_Monitoring.desktop << EOF
[Desktop Entry]
Version=1.0
Type=Application
Name=Robot Console Monitoring
Comment=Interface console de monitoring du robot Mecanum
Exec=$HOME/start_console_monitoring.sh
Icon=utilities-terminal
Terminal=true
Categories=Development;
EOF
    chmod +x ~/Desktop/Robot_Console_Monitoring.desktop
}

# Configuration réseau
configure_network() {
    print_status "Configuration réseau..."
    print_warning "Configuration manuelle requise:"
    echo ""
    echo "1. Assurez-vous que cette machine et la Raspberry Pi sont sur le même réseau"
    echo "2. Notez l'IP de cette machine: $(ip route get 1 | awk '{print $7}' | head -1)"
    echo "3. Configurez l'IP de la Raspberry Pi dans le fichier:"
    echo "   ~/Documents/Dev/LEROBOT/SBC_RobotMecanum/src/mecanum_robot/config/robot_config.yaml"
    echo ""
}

# Fonction principale
main() {
    print_status "Début de l'installation pour la machine de monitoring..."
    
    check_ros2
    install_system_dependencies
    install_python_dependencies
    build_workspace
    setup_environment
    create_desktop_shortcuts
    configure_network
    
    print_status "Installation terminée avec succès!"
    print_warning "Redémarrez votre session ou exécutez 'source ~/.bashrc' pour charger l'environnement"
    print_status "Pour démarrer le monitoring: ~/start_monitoring.sh"
    print_status "Pour vérifier la connexion: ~/check_robot_connection.sh"
}

# Exécuter si appelé directement
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi