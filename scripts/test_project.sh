#!/bin/bash

# Script de test de compilation (sans ROS2 installé)
# Ce script vérifie la syntaxe Python des nodes

echo "=== Test de syntaxe des nodes Python ==="

# Vérifier Python3
if ! command -v python3 &> /dev/null; then
    echo "❌ Python3 non trouvé"
    exit 1
fi

echo "✓ Python3 trouvé: $(python3 --version)"

# Répertoire des nodes
NODES_DIR="src/mecanum_robot/mecanum_robot"

if [ ! -d "$NODES_DIR" ]; then
    echo "❌ Répertoire des nodes non trouvé: $NODES_DIR"
    exit 1
fi

echo "✓ Répertoire des nodes trouvé"

# Tester chaque node Python
echo ""
echo "Test de syntaxe des nodes:"

for node_file in "$NODES_DIR"/*.py; do
    if [ -f "$node_file" ]; then
        node_name=$(basename "$node_file")
        echo -n "  $node_name... "
        
        if python3 -m py_compile "$node_file" 2>/dev/null; then
            echo "✓"
        else
            echo "❌"
            echo "    Erreur de syntaxe dans $node_file"
            python3 -m py_compile "$node_file"
        fi
    fi
done

echo ""
echo "Test de syntaxe des fichiers launch:"

for launch_file in src/mecanum_robot/launch/*.py; do
    if [ -f "$launch_file" ]; then
        launch_name=$(basename "$launch_file")
        echo -n "  $launch_name... "
        
        if python3 -m py_compile "$launch_file" 2>/dev/null; then
            echo "✓"
        else
            echo "❌"
            echo "    Erreur de syntaxe dans $launch_file"
            python3 -m py_compile "$launch_file"
        fi
    fi
done

echo ""
echo "Test de validation des fichiers YAML:"

for yaml_file in src/mecanum_robot/config/*.yaml; do
    if [ -f "$yaml_file" ]; then
        yaml_name=$(basename "$yaml_file")
        echo -n "  $yaml_name... "
        
        if python3 -c "import yaml; yaml.safe_load(open('$yaml_file'))" 2>/dev/null; then
            echo "✓"
        else
            echo "❌"
            echo "    Erreur de syntaxe YAML dans $yaml_file"
        fi
    fi
done

echo ""
echo "✓ Tests de syntaxe terminés"

echo ""
echo "=== Structure du projet ==="
echo "Pour compiler avec ROS2, utilisez:"
echo "  source /opt/ros/jazzy/setup.bash"
echo "  colcon build --symlink-install"
echo ""
echo "Pour installer ROS2 Jazzy:"
echo "  curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg"
echo "  echo 'deb [arch=\$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu \$(. /etc/os-release && echo \$UBUNTU_CODENAME) main' | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null"
echo "  sudo apt update"
echo "  sudo apt install ros-jazzy-desktop"