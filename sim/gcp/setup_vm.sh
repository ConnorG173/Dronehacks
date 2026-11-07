#!/usr/bin/env bash
# One-time setup of the GCP sim VM (Ubuntu 22.04 + NVIDIA L4, driver preinstalled).
# Installs: Docker + NVIDIA container toolkit, ROS 2 Humble, Gazebo Harmonic + ros_gz,
# PX4-Autopilot v1.16.2 SITL (same firmware as the drone), and builds the organizers'
# skybeta-sim container (cuVSLAM + PX4 bridge + flight interface, identical to the drone).
# Idempotent: safe to rerun. Log: ~/setup_vm.log
set -euo pipefail
exec > >(tee -a "$HOME/setup_vm.log") 2>&1
step() { echo; echo "=== [$(date +%H:%M:%S)] $*"; }
export DEBIAN_FRONTEND=noninteractive
wait_apt() { while sudo fuser /var/lib/dpkg/lock-frontend /var/lib/apt/lists/lock >/dev/null 2>&1; do sleep 5; done; }

PX4_TAG=v1.16.2
SKYBETA_REPO=https://github.com/Artash-N/skybeta.git
SKYBETA_BRANCH=fleet-ready

step "base packages"
wait_apt
sudo apt-get update -q
sudo apt-get install -yq curl git build-essential tmux htop software-properties-common gnupg lsb-release

step "NVIDIA OpenGL/EGL userspace (the GCP accelerator image ships compute-only; Gazebo headless rendering needs EGL)"
DRV_VER="$(nvidia-smi --query-gpu=driver_version --format=csv,noheader | cut -d. -f1)"
dpkg -s "libnvidia-gl-${DRV_VER}-server" >/dev/null 2>&1 || { wait_apt; sudo apt-get install -yq "libnvidia-gl-${DRV_VER}-server"; }
sudo usermod -aG video,render "$USER"
ls /usr/share/glvnd/egl_vendor.d/

step "docker + nvidia container toolkit"
if ! command -v docker >/dev/null; then
  curl -fsSL https://get.docker.com | sudo sh
fi
sudo usermod -aG docker "$USER"
if ! dpkg -s nvidia-container-toolkit >/dev/null 2>&1; then
  curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor --yes -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
  curl -fsSL https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list |
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#' |
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list >/dev/null
  wait_apt; sudo apt-get update -q; sudo apt-get install -yq nvidia-container-toolkit
  sudo nvidia-ctk runtime configure --runtime=docker
  sudo systemctl restart docker
fi
sudo docker run --rm --runtime nvidia --gpus all ubuntu:22.04 nvidia-smi -L

step "ROS 2 Humble"
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" |
    sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
  wait_apt; sudo apt-get update -q
  sudo apt-get install -yq ros-humble-ros-base ros-humble-rviz2 python3-colcon-common-extensions python3-rosdep ros-humble-foxglove-bridge
fi

step "PX4-Autopilot ${PX4_TAG} (installs Gazebo Harmonic via PX4's ubuntu.sh)"
if [[ ! -d "$HOME/PX4-Autopilot" ]]; then
  git clone https://github.com/PX4/PX4-Autopilot.git "$HOME/PX4-Autopilot"
fi
cd "$HOME/PX4-Autopilot"
git fetch --tags -q
git checkout -q "$PX4_TAG"
git submodule update --init --recursive -q
wait_apt
bash Tools/setup/ubuntu.sh --no-nuttx
# ubuntu.sh pip-installs into ~/.local; make sure the build sees it
export PATH="$HOME/.local/bin:$PATH"
make px4_sitl -j"$(nproc)"

step "ros_gz bridge for Humble + Harmonic"
wait_apt; sudo apt-get update -q
sudo apt-get install -yq ros-humble-ros-gzharmonic || sudo apt-get install -yq ros-humble-ros-gzharmonic-bridge ros-humble-ros-gzharmonic-image

step "organizer stack container (skybeta-sim)"
if [[ ! -d "$HOME/skybeta" ]]; then
  git clone -b "$SKYBETA_BRANCH" "$SKYBETA_REPO" "$HOME/skybeta"
fi
cd "$HOME/skybeta"
git pull -q --ff-only || true
VERSION="$(cat VERSION)"
if ! sudo docker image inspect "skybeta-sim:${VERSION}" >/dev/null 2>&1; then
  # Organizer bug (their sim profile is marked "never executed"): the loop that COLCON_IGNOREs every
  # top-level dir without a package.xml also hits px4-ros2-interface-lib (its package.xml is in
  # px4_ros2_cpp/), so competitor_advanced_cpp cannot find px4_ros2_cpp. Build a patched copy; their
  # repo stays untouched.
  sed 's#if \[ ! -f "${d}package.xml" \]; then#if [ ! -f "${d}package.xml" ] \&\& [ "${d}" != "px4-ros2-interface-lib/" ]; then#' \
    docker/Dockerfile.sim > /tmp/Dockerfile.sim.team
  grep -q 'px4-ros2-interface-lib/" ]' /tmp/Dockerfile.sim.team || { echo "patch did not apply"; exit 1; }
  sudo docker build -f /tmp/Dockerfile.sim.team -t "skybeta-sim:${VERSION}" .
fi
sudo docker image ls | grep skybeta-sim

step "shell setup"
grep -q "ros/humble/setup.bash" "$HOME/.bashrc" || cat >> "$HOME/.bashrc" <<'EOF'
source /opt/ros/humble/setup.bash
export PATH="$HOME/.local/bin:$PATH"
export GZ_SIM_RESOURCE_PATH="$HOME/PX4-Autopilot/Tools/simulation/gz/models:$HOME/PX4-Autopilot/Tools/simulation/gz/worlds:${GZ_SIM_RESOURCE_PATH:-}"
EOF

step "DONE"
echo "Setup finished. Log out and back in once so the docker group applies."
