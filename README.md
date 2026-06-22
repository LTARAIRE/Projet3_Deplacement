# Projet 3 — Déplacement Hexapode

Paquet ROS2 permettant de commander un hexapode via `geometry_msgs/TwistStamped`.

> Robotique avancée — Mastère 2 Expert Mécatronique — YNOV

## Packages

| Package | Description |
|---------|-------------|
| `hexapod_interfaces` | Messages, services et actions custom |
| `hexapod_locomotion` | Cinématique inverse, générateur de démarche, noeuds ROS2 |

## Prérequis

- **Linux** (testé sur Linux Mint / Ubuntu 22.04+)
- **ROS2 Humble** ou **Jazzy**
- Python 3.10+

## Installation rapide

```bash
git clone git@github.com:LTARAIRE/Projet3_Deplacement.git
cd Projet3_Deplacement
chmod +x scripts/setup_and_launch.sh
./scripts/setup_and_launch.sh
```

Le script installe automatiquement les dépendances, build le workspace et affiche les commandes de lancement.

## Installation manuelle

```bash
# Sourcer ROS2
source /opt/ros/humble/setup.bash

# Dépendances
sudo apt install python3-colcon-common-extensions python3-numpy \
  ros-humble-xacro ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher-gui ros-humble-rviz2

# Build
colcon build --symlink-install
source install/setup.bash
```

## Lancement

```bash
# Visualiser le modèle URDF dans RViz (avec sliders pour tester les joints)
ros2 launch hexapod_locomotion display.launch.py

# Locomotion complète + RViz
ros2 launch hexapod_locomotion locomotion.launch.py

# Contrôle clavier + RViz
ros2 launch hexapod_locomotion teleop.launch.py
```

## Contrôle clavier (teleop)

| Touche | Action |
|--------|--------|
| `Z` / `S` | Avancer / Reculer |
| `Q` / `D` | Strafe gauche / droite |
| `A` / `E` | Rotation gauche / droite |
| `Espace` | Stop |
| `+` / `-` | Vitesse +/- |

## Interfaces ROS2

### Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/cmd_vel` | `geometry_msgs/TwistStamped` | Input |
| `/joint_states` | `sensor_msgs/JointState` | Output |

### Services

| Service | Type |
|---------|------|
| `/hexapod/set_gait` | `hexapod_interfaces/SetGait` |
| `/hexapod/get_leg_ik` | `hexapod_interfaces/GetLegIK` |

### Action

| Action | Type |
|--------|------|
| `/hexapod/walk` | `hexapod_interfaces/Walk` |

## Architecture

```
src/
├── hexapod_interfaces/          # Interfaces custom (cmake)
│   ├── msg/                     # LegAngles, HexapodState
│   ├── srv/                     # SetGait, GetLegIK
│   └── action/                  # Walk
└── hexapod_locomotion/          # Package Python
    ├── hexapod_locomotion/
    │   ├── leg_ik.py            # Cinématique inverse 3-DOF
    │   ├── gait_generator.py    # Démarches : tripod, wave, ripple
    │   ├── locomotion_node.py   # Noeud principal
    │   └── teleop_key_node.py   # Contrôle clavier
    ├── urdf/                    # URDF/Xacro 6 pattes
    ├── meshes/                  # Fichiers STL
    ├── launch/                  # Fichiers launch
    ├── config/                  # Paramètres YAML
    └── rviz/                    # Configuration RViz
```

## Équipe

- Lucas Taraire
