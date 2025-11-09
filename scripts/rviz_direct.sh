#!/bin/bash

# Script direct pour lancer RViz avec visualisation lidar
echo "🚀 === Lancement RViz Direct ==="

export ROS_DOMAIN_ID=42

echo "📡 Vérification des topics..."
ros2 topic list

echo ""
echo "🎯 Lancement de RViz avec configuration permanente..."

# Configuration RViz permanente
RVIZ_CONFIG="$(dirname "$(dirname "$0")")/config/mecanum_robot.rviz"

if [ ! -f "$RVIZ_CONFIG" ]; then
    echo "❌ Configuration RViz non trouvée: $RVIZ_CONFIG"
    echo "📝 Utilisation d'une configuration par défaut..."
    RVIZ_CONFIG=""
fi

echo "📊 Configuration RViz: $RVIZ_CONFIG"
echo "🎯 Démarrage de RViz..."

# Démarrer les transformations en arrière-plan
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map laser_frame &
TF_PID=$!

# Lancer RViz avec ou sans configuration
if [ -n "$RVIZ_CONFIG" ] && [ -f "$RVIZ_CONFIG" ]; then
    rviz2 -d "$RVIZ_CONFIG"
else
    rviz2
fi

# Nettoyer à la fermeture
echo "🧹 Nettoyage..."
kill $TF_PID 2>/dev/null