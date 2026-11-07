#!/usr/bin/env bash
# Gazebo -> ROS 2 topics (cameras, camera_info, IMU, clock, ground truth) for this slot.
set -eo pipefail
source "$(dirname "$0")/common.sh"
source /opt/ros/humble/setup.bash
CFG="/tmp/sim${SIM_ID}_bridge.yaml"
sed -e "s#@WORLD@#$WORLD#g" -e "s#@MODEL@#$MODEL_NAME#g" "$REPO/sim/gz/bridge.yaml" > "$CFG"
exec ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:="$CFG"
