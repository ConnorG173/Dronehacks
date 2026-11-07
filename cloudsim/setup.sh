#!/usr/bin/env bash
# One-time setup of the cloud sim VM (Ubuntu 24.04 + NVIDIA driver image). Idempotent: re-run
# after a failure. Run as your normal (OS Login) user from the pushed repo:
#     bash ~/Dronehacks/cloudsim/setup.sh
# Installs: ROS 2 Jazzy (ros-base + ros_gz_bridge), PX4 SITL v1.16.2 with Gazebo Harmonic (PX4's
# own ubuntu.sh), Micro XRCE-DDS Agent, px4_msgs release/1.16, /etc/flyx/mission.yaml
# (use_sim_time) and the 30-minute idle shutdown.
set -euo pipefail

PX4_TAG=v1.16.2                 # the real drone's PX4 version
PX4_MSGS_BRANCH=release/1.16
XRCE_TAG=v2.4.3                 # the agent version PX4's docs use
HERE=$(cd "$(dirname "$0")" && pwd)
export DEBIAN_FRONTEND=noninteractive

echo "== base packages"
sudo apt-get update
sudo apt-get install -y git curl tmux build-essential cmake python3-pip python3-numpy python3-pytest

echo "== idle shutdown (first, so a stuck setup still powers off)"
sudo install -m 755 "$HERE/idle-shutdown.sh" /usr/local/bin/cloudsim-idle-shutdown
sudo install -m 644 "$HERE/cloudsim-idle.service" "$HERE/cloudsim-idle.timer" /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now cloudsim-idle.timer

echo "== ROS 2 Jazzy"
if [ ! -f /opt/ros/jazzy/setup.bash ]; then
    ver=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
          | grep -F '"tag_name"' | awk -F'"' '{print $4}')
    curl -fsSL -o /tmp/ros2-apt-source.deb \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ver}/ros2-apt-source_${ver}.noble_all.deb"
    sudo dpkg -i /tmp/ros2-apt-source.deb
    sudo apt-get update
fi
sudo apt-get install -y ros-jazzy-ros-base ros-dev-tools ros-jazzy-ros-gz-bridge

echo "== PX4 $PX4_TAG + Gazebo Harmonic"
if [ ! -d ~/PX4-Autopilot ]; then
    git clone --recursive --depth 1 -b "$PX4_TAG" https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot
fi
bash ~/PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx
(cd ~/PX4-Autopilot && make px4_sitl)

echo "== Micro XRCE-DDS Agent $XRCE_TAG"
if ! command -v MicroXRCEAgent >/dev/null; then
    rm -rf /tmp/xrce
    git clone --depth 1 -b "$XRCE_TAG" https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/xrce
    cmake -S /tmp/xrce -B /tmp/xrce/build
    cmake --build /tmp/xrce/build -j"$(nproc)"
    sudo cmake --install /tmp/xrce/build
    sudo ldconfig /usr/local/lib/
fi

echo "== px4_msgs $PX4_MSGS_BRANCH"
mkdir -p ~/ros2_ws/src
if [ ! -d ~/ros2_ws/src/px4_msgs ]; then
    git clone --depth 1 -b "$PX4_MSGS_BRANCH" https://github.com/PX4/px4_msgs.git ~/ros2_ws/src/px4_msgs
fi
set +u; source /opt/ros/jazzy/setup.bash; set -u
(cd ~/ros2_ws && colcon build --packages-select px4_msgs)

echo "== sim-time parameter file (read by takeoff_land.py and localizer.sim)"
sudo install -D -m 644 "$HERE/mission.yaml" /etc/flyx/mission.yaml

echo "== done. Start the sim with: bash ~/Dronehacks/cloudsim/sim.sh start"
