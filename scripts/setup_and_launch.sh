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
# 0. Détermination du codename Ubuntu / distro ROS2
# ───────────────────────────────────────────
# Sur Linux Mint, VERSION_CODENAME vaut "zena"/"virginia"/... ce qui casse
# le dépôt apt ROS. On se base donc sur UBUNTU_CODENAME (base Ubuntu réelle).
UBUNTU_CODENAME_DETECTED="$(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}")"

case "$UBUNTU_CODENAME_DETECTED" in
    noble)  TARGET_DISTRO="jazzy" ;;   # Ubuntu 24.04 (et Mint 22.x)
    jammy)  TARGET_DISTRO="humble" ;;  # Ubuntu 22.04 (et Mint 21.x)
    *)
        warn "Codename Ubuntu '$UBUNTU_CODENAME_DETECTED' non reconnu, tentative avec Jazzy."
        UBUNTU_CODENAME_DETECTED="noble"
        TARGET_DISTRO="jazzy"
        ;;
esac

# ───────────────────────────────────────────
# 1. Détection / installation de ROS2
# ───────────────────────────────────────────
ROS_DISTRO=""
for distro in jazzy humble iron; do
    if [ -f "/opt/ros/$distro/setup.bash" ]; then
        ROS_DISTRO="$distro"
        break
    fi
done

install_ros2() {
    local distro="$1"
    local codename="$2"
    warn "Aucune installation ROS2 détectée — installation de ROS2 $distro ($codename)..."

    # Locale UTF-8 (requis par ROS2)
    sudo apt update -qq
    sudo apt install -y -qq locales curl gnupg lsb-release software-properties-common
    sudo locale-gen en_US en_US.UTF-8 || true
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 || true

    # Dépôt "universe" (best-effort : peut être géré différemment sous Mint)
    sudo add-apt-repository -y universe 2>/dev/null || true

    # Paquet officiel ros-apt-source qui configure le dépôt + la clé
    local ros_apt_ver
    ros_apt_ver="$(curl -sSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
        | grep -F '"tag_name"' | awk -F'"' '{print $4}')"
    [ -z "$ros_apt_ver" ] && ros_apt_ver="1.2.0"

    local deb="/tmp/ros2-apt-source.deb"
    info "Téléchargement de ros2-apt-source ${ros_apt_ver} pour ${codename}..."
    curl -sSL -o "$deb" \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_apt_ver}/ros2-apt-source_${ros_apt_ver}.${codename}_all.deb" \
        || error "Échec du téléchargement du dépôt apt ROS2."
    sudo apt install -y "$deb"
    rm -f "$deb"

    info "Installation de ros-${distro}-ros-base (cela peut prendre plusieurs minutes)..."
    sudo apt update -qq
    sudo apt install -y ros-dev-tools "ros-${distro}-ros-base"

    [ -f "/opt/ros/${distro}/setup.bash" ] \
        || error "Installation de ROS2 ${distro} terminée mais /opt/ros/${distro}/setup.bash introuvable."
}

if [ -z "$ROS_DISTRO" ]; then
    install_ros2 "$TARGET_DISTRO" "$UBUNTU_CODENAME_DETECTED"
    ROS_DISTRO="$TARGET_DISTRO"
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
# Linux Mint n'est pas reconnu par rosdep : on force la base Ubuntu détectée.
export ROS_OS_OVERRIDE="ubuntu:${UBUNTU_CODENAME_DETECTED}"
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
