#!/usr/bin/env bash
# The organizers' stack, unchanged, in their x86 container: uXRCE-DDS agent + cuVSLAM + PX4 odometry bridge.
# Same code as the drone. The repo is mounted read-only at /root/team.
set -eo pipefail
source "$(dirname "$0")/common.sh"
VERSION="$(cat "$SKYBETA_DIR/VERSION")"
docker rm -f skybeta-sim >/dev/null 2>&1 || true
exec docker run --rm --name skybeta-sim --runtime nvidia --gpus all --network host --ipc host \
  -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" -v "$REPO:/root/team:ro" "skybeta-sim:$VERSION" \
  bash -lc 'source /opt/ros/humble/setup.bash && source /root/skybeta_ws/install/setup.bash &&
            ros2 launch /root/skybeta_ws/sim/skybeta_sim.launch.py enable_slam_visualization:=False \
              enable_landmarks_view:=False enable_observations_view:=False'
