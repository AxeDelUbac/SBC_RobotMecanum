# Robot Mecanum avec Communication Multi-Machine

Projet ROS2 Jazzy pour robot mecanum équipé d'un lidar RPlidar A1 avec communication entre Raspberry Pi et machine de monitoring distante.

## 📋 Table des Matières

- [Description](#description)
- [Architecture](#architecture)
- [Prérequis](#prérequis)
- [Installation](#installation)
- [Configuration](#configuration)
- [Utilisation](#utilisation)
- [Dépannage](#dépannage)
- [Développement](#développement)
- [Licence](#licence)

## 🎯 Description

# SBC_RobotMecanum

Projet ROS2 Jazzy pour robot à roues mecanum équipé d'un lidar RPlidar A1. Ce projet permet la communication entre deux machines : la Raspberry Pi embarquée et une machine de monitoring distant via les topics ROS2.

## 🤖 Description

Ce projet implémente un système de contrôle et de monitoring pour un robot à roues mecanum avec les fonctionnalités suivantes :

- **Contrôle du robot** : Gestion des mouvements omnidirectionnels
- **Acquisition lidar** : Traitement des données du RPlidar A1
- **Communication réseau** : Pont TCP pour communication inter-machines
- **Monitoring** : Interface console + visualisation Gazebo/RViz

## 🏗️ Architecture

### Machines
- **Raspberry Pi** : Contrôle robot + acquisition capteurs
- **Machine de monitoring** : Surveillance à distance + visualisation

### Nodes ROS2
- `robot_controller.py` : Contrôle des moteurs et mouvements
- `lidar_processor.py` : Traitement des données lidar
- `console_monitor.py` : Interface de monitoring console
- `communication_bridge.py` : Pont de communication TCP

## 📦 Installation

### Prérequis
- ROS2 Jazzy
- Python 3.10+
- Raspberry Pi OS (pour la Pi) ou Ubuntu 22.04+ (monitoring)

### Raspberry Pi
```bash
cd SBC_RobotMecanum
./scripts/install_robot.sh
```

### Machine de monitoring
```bash
cd SBC_RobotMecanum
./scripts/install_monitoring.sh
```

## 🚀 Utilisation

### Démarrage du robot (Raspberry Pi)
```bash
~/start_robot.sh
```

### Monitoring distant
```bash
# Interface console
~/start_monitoring.sh

# Visualisation avec RViz et Gazebo
~/start_gazebo_rviz.sh

# Vérification de connexion
~/check_robot_connection.sh
```

### Topics ROS2 principaux
- `/scan` : Données lidar (sensor_msgs/LaserScan)
- `/cmd_vel` : Commandes de vitesse (geometry_msgs/Twist)
- `/robot_status` : État du robot

## 🔧 Configuration

La configuration se trouve dans `src/mecanum_robot/config/robot_config.yaml` :
- Adresses IP des machines
- Paramètres du lidar
- Configuration des moteurs
- Ports de communication

## 🎯 Visualisation

Ce projet utilise les outils standard ROS2 pour la visualisation :
- **RViz2** : Visualisation des données de capteurs et état du robot
- **Gazebo** : Simulation (optionnelle)
- **rqt** : Outils de debug et monitoring ROS2

### Configuration RViz recommandée
- Ajouter `/scan` pour visualiser les données lidar
- Ajouter `/tf` pour les transformations
- Ajouter `/cmd_vel` pour les commandes de vitesse

## 🌐 Communication

Le système utilise deux modes de communication :
- **ROS2 DDS** : Communication principale via topics
- **TCP Bridge** : Communication de secours et données spécifiques

Port par défaut : 11511

## 🛠️ Développement

### Compilation
```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Structure du projet
```
src/mecanum_robot/
├── mecanum_robot/           # Code Python des nodes
├── launch/                  # Fichiers de lancement
├── config/                  # Fichiers de configuration
└── package.xml             # Métadonnées du package
```

## 📋 TODO
- [ ] Intégration encodeurs roues
- [ ] Navigation autonome
- [ ] Cartographie SLAM
- [ ] Interface web (optionnelle)

## 🔗 Liens utiles
- [ROS2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [RPlidar A1 Manual](https://www.slamtec.com/en/Lidar/A1)
- [Gazebo Documentation](https://gazebosim.org/docs)

### Composants Principaux

1. **Robot Controller** (Raspberry Pi) - Contrôle principal du robot
2. **Remote Monitor** (Machine distante) - Interface de monitoring et contrôle
3. **Lidar Processor** - Traitement des données lidar
4. **Communication Bridge** - Communication réseau entre machines

## 🏗️ Architecture

```
┌─────────────────────┐         ┌─────────────────────┐
│   Raspberry Pi      │  TCP    │  Machine Monitoring │
│                     │◄──────►│                     │
│ ┌─────────────────┐ │         │ ┌─────────────────┐ │
│ │ Robot Controller│ │         │ │ Remote Monitor  │ │
│ │ Lidar Processor │ │         │ │ Communication   │ │
│ │ Comm. Bridge    │ │         │ │ Bridge          │ │
│ └─────────────────┘ │         │ └─────────────────┘ │
│         │           │         │                     │
│    ┌────▼────┐      │         │                     │
│    │ RPlidar │      │         │                     │
│    │   A1    │      │         │                     │
│    └─────────┘      │         │                     │
│         │           │         │                     │
│  ┌──────▼──────┐    │         │                     │
│  │Microcontrôleur│  │         │                     │
│  │(Moteurs+Odom) │  │         │                     │
│  └─────────────┘    │         │                     │
└─────────────────────┘         └─────────────────────┘
```

### Topics ROS2

- `/robot_status` - Statut général du robot
- `/sensor_data` - Données consolidées des capteurs
- `/scan` - Données brutes du lidar
- `/scan_filtered` - Données lidar filtrées
- `/obstacles` - Obstacles détectés
- `/cmd_vel` - Commandes de vitesse

### Services ROS2

- `/set_robot_mode` - Changement de mode du robot

## 🔧 Prérequis

### Matériel

- **Raspberry Pi 4** (recommandé) avec Ubuntu 22.04
- **Lidar RPlidar A1** connecté en USB
- **Microcontrôleur** (Arduino/ESP32) pour contrôle moteurs
- **Machine de monitoring** avec Ubuntu 22.04
- **Réseau WiFi** commun aux deux machines

### Logiciels

- **ROS2 Jazzy** installé sur les deux machines
- **Python 3.10+**
- **Git**

## 🚀 Installation

### 1. Installation sur Raspberry Pi

```bash
# Cloner le dépôt
git clone https://github.com/AxeDelUbac/SBC_RobotMecanum.git
cd SBC_RobotMecanum

# Exécuter le script d'installation
./scripts/install_robot.sh
```

### 2. Installation sur Machine de Monitoring

```bash
# Cloner le dépôt
git clone https://github.com/AxeDelUbac/SBC_RobotMecanum.git
cd SBC_RobotMecanum

# Exécuter le script d'installation
./scripts/install_monitoring.sh
```

### 3. Installation Manuelle (optionnelle)

Si vous préférez installer manuellement :

#### Dépendances ROS2
```bash
sudo apt install ros-jazzy-rplidar-ros ros-jazzy-laser-geometry
```

#### Compilation du workspace
```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## ⚙️ Configuration

### 1. Configuration du Robot

Éditez le fichier `src/mecanum_robot/config/robot_config.yaml` :

```yaml
robot:
  ros__parameters:
    robot_id: "mecanum_robot_01"
    
    # Paramètres réseau
    network:
      monitoring_ip: "192.168.1.100"  # IP de votre machine de monitoring
      robot_ip: "192.168.1.101"       # IP de votre Raspberry Pi
      communication_port: 11511
    
    # Capteurs
    sensors:
      lidar_enabled: true
      microcontroller_port: "/dev/ttyACM0"  # Port de votre microcontrôleur
```

### 2. Configuration du Lidar

Éditez le fichier `src/mecanum_robot/config/rplidar_config.yaml` :

```yaml
rplidar:
  ros__parameters:
    serial_port: "/dev/ttyUSB0"  # Port de votre lidar
    serial_baudrate: 115200
    frame_id: "laser_frame"
    scan_mode: "Boost"
```

### 3. Permissions Série

Assurez-vous que l'utilisateur a les permissions pour les ports série :

```bash
sudo usermod -a -G dialout $USER
# Redémarrez votre session après cette commande
```

## 🎮 Utilisation

### 1. Démarrage du Robot (Raspberry Pi)

```bash
# Option 1: Script automatique
~/start_robot.sh

# Option 2: Commande manuelle
source install/setup.bash
ros2 launch mecanum_robot robot_launch.py robot_mode:=raspberry
```

### 2. Démarrage du Monitoring (Machine distante)

```bash
# Option 1: Script automatique
~/start_monitoring.sh

# Option 2: Commande manuelle
source install/setup.bash
ros2 launch mecanum_robot monitoring_launch.py
```

### 3. Interface de Contrôle

L'interface de monitoring fournit :

- **Statut en temps réel** du robot
- **Contrôle manuel** avec boutons directionnels
- **Changement de mode** (manuel, auto, scan, stop)
- **Logs** des événements
- **Informations capteurs** (batterie, température, etc.)

### 4. Commandes ROS2 Utiles

```bash
# Voir les topics actifs
ros2 topic list

# Monitorer le statut du robot
ros2 topic echo /robot_status

# Envoyer une commande de vitesse
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Changer le mode du robot
ros2 service call /set_robot_mode mecanum_robot/srv/SetRobotMode "{mode: 'manual'}"

# Visualiser les données lidar
ros2 topic echo /scan --once
```

### 5. Service Automatique (optionnel)

Pour démarrer le robot automatiquement au boot :

```bash
sudo systemctl enable mecanum-robot
sudo systemctl start mecanum-robot
```

## 🔍 Dépannage

### Problèmes Courants

#### 1. Lidar non détecté

```bash
# Vérifier la connexion USB
lsusb | grep Silicon

# Vérifier les permissions
ls -la /dev/ttyUSB*

# Tester la connexion
sudo chmod 666 /dev/ttyUSB0
ros2 run rplidar_ros rplidar_composition --ros-args -p serial_port:=/dev/ttyUSB0
```

#### 2. Communication réseau

```bash
# Tester la connectivité
~/check_robot_connection.sh

# Vérifier les topics ROS2
ros2 topic list
ros2 node list

# Vérifier la configuration réseau
echo $ROS_DOMAIN_ID  # Doit être identique sur les deux machines
```

#### 3. Microcontrôleur non connecté

```bash
# Vérifier la connexion série
ls -la /dev/ttyACM*
ls -la /dev/ttyUSB*

# Tester la communication
screen /dev/ttyACM0 115200
# Ou
minicom -D /dev/ttyACM0 -b 115200
```

### Messages d'Erreur Fréquents

| Erreur | Solution |
|--------|----------|
| `Permission denied: '/dev/ttyUSB0'` | Ajouter l'utilisateur au groupe dialout |
| `No module named 'mecanum_robot'` | Recompiler avec `colcon build` |
| `Failed to connect to robot` | Vérifier la configuration réseau |
| `Lidar not responding` | Vérifier l'alimentation et la connexion USB |

### Logs de Débogage

```bash
# Logs du système
sudo journalctl -u mecanum-robot -f

# Logs ROS2
ros2 run rqt_console rqt_console

# Logs des nodes
ros2 node info /robot_controller
```

## 👨‍💻 Développement

### Structure du Projet

```
SBC_RobotMecanum/
├── src/mecanum_robot/
│   ├── mecanum_robot/          # Nodes Python
│   │   ├── robot_controller.py
│   │   ├── remote_monitor.py
│   │   ├── lidar_processor.py
│   │   └── communication_bridge.py
│   ├── launch/                 # Fichiers de lancement
│   ├── config/                 # Configurations YAML
│   ├── msg/                    # Messages personnalisés
│   └── srv/                    # Services personnalisés
├── scripts/                    # Scripts d'installation
└── README.md
```

### Ajouter un Nouveau Node

1. Créer le fichier Python dans `src/mecanum_robot/mecanum_robot/`
2. Ajouter l'exécutable dans `CMakeLists.txt`
3. Recompiler avec `colcon build`

### Créer un Nouveau Message

1. Définir le message dans `src/mecanum_robot/msg/`
2. Ajouter à la liste dans `CMakeLists.txt`
3. Recompiler le workspace

### Tests

```bash
# Lancer les tests
colcon test

# Tests spécifiques
ros2 run mecanum_robot test_communication
```

### Contribuer

1. Fork le projet
2. Créer une branche feature (`git checkout -b feature/AmazingFeature`)
3. Commit les changements (`git commit -m 'Add AmazingFeature'`)
4. Push vers la branche (`git push origin feature/AmazingFeature`)
5. Ouvrir une Pull Request

## 📈 Performances

### Fréquences de Publication

- **Status robot** : 1 Hz
- **Données capteurs** : 10 Hz
- **Scan lidar** : 10 Hz (configurable)
- **Heartbeat réseau** : 1 Hz

### Optimisations

- Utilisation de QoS profiles adaptés
- Filtrage des données lidar pour réduire le bruit
- Communication réseau avec compression optionnelle

## 🔒 Sécurité

- Communication réseau non chiffrée (LAN privé recommandé)
- Authentification basique pour les services
- Validation des commandes de vitesse

## 📝 Licence

Ce projet est sous licence MIT. Voir le fichier `LICENSE` pour plus de détails.

## 🤝 Support

Pour obtenir de l'aide :

1. Consultez la section [Dépannage](#dépannage)
2. Ouvrez une issue sur GitHub
3. Consultez la documentation ROS2 Jazzy

## 🔄 Versions

- **v0.1.0** - Version initiale avec lidar RPlidar A1
- **Prochaines versions** - Integration caméra, navigation autonome

---

**Développé avec ❤️ pour le projet LEROBOT**