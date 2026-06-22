#!/bin/bash
set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

info()  { echo -e "${GREEN}[INFO]${NC} $1"; }
warn()  { echo -e "${YELLOW}[WARN]${NC} $1"; }
error() { echo -e "${RED}[ERREUR]${NC} $1"; exit 1; }

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"

# ───────────────────────────────────────────
# 1. Détection de ROS2
# ───────────────────────────────────────────
ROS_DISTRO=""
for distro in jazzy humble iron; do
    if [ -f "/opt/ros/$distro/setup.bash" ]; then
        ROS_DISTRO="$distro"
        break
    fi
done

if [ -z "$ROS_DISTRO" ]; then
    error "Aucune installation ROS2 détectée dans /opt/ros/.
Installez ROS2 Humble ou Jazzy d'abord :
  https://docs.ros.org/en/humble/Installation.html"
fi

info "ROS2 $ROS_DISTRO détecté"
source "/opt/ros/$ROS_DISTRO/setup.bash"

# ───────────────────────────────────────────
# 2. Installation des dépendances système
# ───────────────────────────────────────────
info "Installation des dépendances apt..."
sudo apt update -qq
sudo apt install -y -qq \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-numpy \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-tf2-ros \
    xterm

# ───────────────────────────────────────────
# 3. Initialisation de rosdep
# ───────────────────────────────────────────
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    info "Initialisation de rosdep..."
    sudo rosdep init || true
fi
rosdep update --rosdistro="$ROS_DISTRO" || true

# ───────────────────────────────────────────
# 4. Installation des dépendances ROS
# ───────────────────────────────────────────
info "Résolution des dépendances du workspace..."
cd "$WS_DIR"
rosdep install --from-paths src --ignore-src -r -y || true

# ───────────────────────────────────────────
# 5. Build
# ───────────────────────────────────────────
info "Build du workspace avec colcon..."
cd "$WS_DIR"
colcon build --symlink-install

info "Build terminé avec succès."

# ───────────────────────────────────────────
# 6. Source du workspace
# ───────────────────────────────────────────
source "$WS_DIR/install/setup.bash"

# ───────────────────────────────────────────
# 7. Lancement (optionnel)
# ───────────────────────────────────────────
if [ "$1" = "--launch" ]; then
    info "Lancement de display.launch.py (RViz + sliders)..."
    ros2 launch hexapod_locomotion display.launch.py
elif [ "$1" = "--teleop" ]; then
    info "Lancement de teleop.launch.py (locomotion + clavier + RViz)..."
    ros2 launch hexapod_locomotion teleop.launch.py
else
    echo ""
    info "Setup terminé. Pour lancer :"
    echo "  source install/setup.bash"
    echo "  ros2 launch hexapod_locomotion display.launch.py      # Visualisation URDF"
    echo "  ros2 launch hexapod_locomotion locomotion.launch.py   # Locomotion + RViz"
    echo "  ros2 launch hexapod_locomotion teleop.launch.py       # Teleop clavier + RViz"
    echo ""
    echo "Ou relancez ce script avec une option :"
    echo "  ./scripts/setup_and_launch.sh --launch    # display"
    echo "  ./scripts/setup_and_launch.sh --teleop    # teleop"
fi
