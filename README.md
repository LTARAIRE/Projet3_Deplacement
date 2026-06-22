# Projet 3 — Déplacement Hexapode

Paquet ROS2 permettant de commander un hexapode via `geometry_msgs/TwistStamped`.

> Robotique avancée — Mastère 2 Expert Mécatronique — YNOV

## Packages

| Package | Description |
|---------|-------------|
| `hexapod_interfaces` | Messages, services et actions custom |
| `hexapod_locomotion` | Cinématique inverse, générateur de démarche, noeuds ROS2 |
| `phantomx_description` | Modèle 3D réel de l'hexapode **PhantomX** (URDF + meshes STL) |

> Le modèle visualisé est le **PhantomX** (3 DOF/patte : `j_c1`/`j_thigh`/`j_tibia`).
> La cinématique inverse et la démarche tripode sont reprises du modèle de
> simulation PhantomX de référence (constantes `PHANTOMX_SIMULATION`).

## Prérequis

- **Linux** (testé sur Linux Mint 22.x / Ubuntu 24.04)
- **ROS2 Jazzy** (Ubuntu 24.04 / Mint 22.x) ou **Humble** (Ubuntu 22.04 / Mint 21.x)
- Python 3.10+

> Si ROS2 n'est pas installé, le script `setup_and_launch.sh` l'installe
> automatiquement (détection de la base Ubuntu, gestion du codename Mint).

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
| `/tf` (`odom`→`base_link`) | `tf2_msgs/TFMessage` | Output (odométrie intégrée) |
| `/environment` | `visualization_msgs/MarkerArray` | Output (sol) |

Le robot **se déplace réellement** dans le repère `odom` (intégration de la
consigne Twist) et **repose sur le sol** (paramètre `base_height`). La démarche
tripode est générée en repère corps : à l'appui le pied pousse à l'opposé de la
vitesse commandée (le corps avance dans le sens commandé), au transfert il se lève.

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
├── phantomx_description/        # Modèle 3D PhantomX (ament_cmake)
│   ├── urdf/phantomx.urdf       # 6 pattes × 3 DOF
│   └── meshes/                  # STL (body, connect, thigh, tibia)
└── hexapod_locomotion/          # Package Python
    ├── hexapod_locomotion/
    │   ├── phantom_kinematics.py # FK exacte (URDF) + IK numérique + démarche
    │   ├── locomotion_node.py    # Noeud principal (TwistStamped → joints + odom)
    │   ├── world_node.py         # Publie le sol (/environment)
    │   ├── teleop_key_node.py    # Contrôle clavier
    │   ├── leg_ik.py             # (ancien modèle analytique simplifié)
    │   └── gait_generator.py     # (ancien générateur de démarche)
    ├── launch/                   # Fichiers launch
    ├── config/                   # Paramètres YAML
    └── rviz/                     # Configuration RViz (fixed frame: base_link)
```

## Équipe

- Lucas Taraire
