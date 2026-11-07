#!/usr/bin/env bash
# Gazebo -> ROS 2 topics (cameras, camera_info, IMU, clock, ground truth).
set -eo pipefail
source "$(dirname "$0")/common.sh"
source /opt/ros/humble/setup.bash
exec ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:="$REPO/sim/gz/bridge.yaml"
