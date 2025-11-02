# Quick Start Guide - Robot Mecanum

## 🚀 Démarrage Rapide

### Sur Raspberry Pi (Robot)

```bash
# 1. Cloner et installer
git clone https://github.com/AxeDelUbac/SBC_RobotMecanum.git
cd SBC_RobotMecanum
./scripts/install_robot.sh

# 2. Configurer les ports série
# Éditer src/mecanum_robot/config/robot_config.yaml
# Éditer src/mecanum_robot/config/rplidar_config.yaml

# 3. Démarrer le robot
~/start_robot.sh
```

### Sur Machine de Monitoring

```bash
# 1. Cloner et installer
git clone https://github.com/AxeDelUbac/SBC_RobotMecanum.git
cd SBC_RobotMecanum
./scripts/install_monitoring.sh

# 2. Vérifier la connexion
~/check_robot_connection.sh

# 3. Démarrer le monitoring
~/start_monitoring.sh
```

## 📁 Structure du Projet

```
SBC_RobotMecanum/
├── README.md              # Documentation complète
├── scripts/               # Scripts d'installation
│   ├── install_robot.sh   # Installation RPi
│   ├── install_monitoring.sh # Installation monitoring
│   └── test_project.sh    # Test de syntaxe
└── src/mecanum_robot/     # Package ROS2
    ├── mecanum_robot/     # Nodes Python
    ├── launch/            # Fichiers de lancement
    ├── config/            # Configurations YAML
    ├── msg/               # Messages personnalisés
    └── srv/               # Services personnalisés
```

## ⚡ Commandes Essentielles

```bash
# Compiler le projet
colcon build --symlink-install

# Lancer le robot (RPi)
ros2 launch mecanum_robot robot_launch.py robot_mode:=raspberry

# Lancer le monitoring (PC)
ros2 launch mecanum_robot monitoring_launch.py

# Voir les topics
ros2 topic list

# Contrôler manuellement
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"

# Changer le mode
ros2 service call /set_robot_mode mecanum_robot/srv/SetRobotMode "{mode: 'manual'}"
```

## 🔧 Configuration Réseau

1. **Même domaine ROS2** : `export ROS_DOMAIN_ID=42`
2. **IPs configurées** dans `config/robot_config.yaml`
3. **Port TCP** : 11511 (configurable)

## 📋 Checklist de Démarrage

- [ ] ROS2 Jazzy installé sur les deux machines
- [ ] Lidar RPlidar A1 connecté en USB sur RPi
- [ ] Microcontrôleur connecté sur RPi (optionnel)
- [ ] Même réseau WiFi pour les deux machines
- [ ] Permissions série configurées (`dialout` group)
- [ ] IPs configurées dans les fichiers YAML

## 🆘 Dépannage Rapide

| Problème | Solution |
|----------|----------|
| Lidar non détecté | `sudo chmod 666 /dev/ttyUSB0` |
| Pas de communication | Vérifier `ROS_DOMAIN_ID` |
| Permission denied | `sudo usermod -a -G dialout $USER` |
| Node ne démarre pas | Vérifier `colcon build` |

---
**Projet LEROBOT - Robot Mecanum ROS2 Jazzy**