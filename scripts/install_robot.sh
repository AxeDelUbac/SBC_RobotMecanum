#!/bin/bash

# Script d'installation et de configuration pour le robot Mecanum
# À exécuter sur la Raspberry Pi

set -e

echo "=== Installation Robot Mecanum - Raspberry Pi ==="

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
    sudo apt install -y \
        python3-pip \
        python3-serial \
        python3-psutil \
        python3-numpy \
        python3-tkinter \
        udev \
        git
}

# Installer les packages ROS2
install_ros2_packages() {
    print_status "Installation des packages ROS2..."
    sudo apt install -y \
        ros-jazzy-rplidar-ros \
        ros-jazzy-laser-geometry \
        ros-jazzy-pcl-conversions \
        ros-jazzy-tf2-tools \
        ros-jazzy-robot-state-publisher \
        ros-jazzy-joint-state-publisher
}

# Installer les dépendances Python
install_python_dependencies() {
    print_status "Installation des dépendances Python..."
    pip3 install --user \
        pyserial \
        psutil \
        numpy
}

# Configurer les permissions série
setup_serial_permissions() {
    print_status "Configuration des permissions série..."
    sudo usermod -a -G dialout $USER
    
    # Créer les règles udev pour les périphériques
    sudo tee /etc/udev/rules.d/99-robot-devices.rules > /dev/null <<EOF
# RPlidar A1
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="rplidar", GROUP="dialout", MODE="0666"

# Microcontrôleur (Arduino/ESP32)
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0043", SYMLINK+="microcontroller", GROUP="dialout", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="microcontroller", GROUP="dialout", MODE="0666"
EOF

    sudo udevadm control --reload-rules
    sudo udevadm trigger
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
    if ! grep -q "# Robot Mecanum Environment" ~/.bashrc; then
        echo "" >> ~/.bashrc
        echo "# Robot Mecanum Environment" >> ~/.bashrc
        echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
        echo "source ~/Documents/Dev/LEROBOT/SBC_RobotMecanum/install/setup.bash" >> ~/.bashrc
        echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
        echo "export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp" >> ~/.bashrc
    fi
    
    # Créer le script de lancement
    cat > ~/start_robot.sh << 'EOF'
#!/bin/bash
source /opt/ros/jazzy/setup.bash
source ~/Documents/Dev/LEROBOT/SBC_RobotMecanum/install/setup.bash
export ROS_DOMAIN_ID=42

echo "Démarrage du robot..."
ros2 launch mecanum_robot robot_launch.py robot_mode:=raspberry
EOF
    chmod +x ~/start_robot.sh
}

# Créer le service systemd
create_systemd_service() {
    print_status "Création du service systemd..."
    
    sudo tee /etc/systemd/system/mecanum-robot.service > /dev/null <<EOF
[Unit]
Description=Robot Mecanum ROS2 Service
After=network.target
Wants=network.target

[Service]
Type=simple
User=$USER
Group=$USER
WorkingDirectory=$HOME
Environment="HOME=$HOME"
Environment="ROS_DOMAIN_ID=42"
ExecStart=$HOME/start_robot.sh
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

    sudo systemctl daemon-reload
    print_status "Service créé. Utilisez 'sudo systemctl enable mecanum-robot' pour l'activer au démarrage"
}

# Fonction principale
main() {
    print_status "Début de l'installation..."
    
    check_ros2
    install_system_dependencies
    install_ros2_packages
    install_python_dependencies
    setup_serial_permissions
    build_workspace
    setup_environment
    create_systemd_service
    
    print_status "Installation terminée avec succès!"
    print_warning "Redémarrez votre session ou exécutez 'source ~/.bashrc' pour charger l'environnement"
    print_status "Pour démarrer le robot: ~/start_robot.sh"
    print_status "Pour activer le service au démarrage: sudo systemctl enable mecanum-robot"
}

# Exécuter si appelé directement
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi