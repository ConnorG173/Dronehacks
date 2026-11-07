#!/usr/bin/env bash
# The organizers' stack, unchanged, in their x86 container: uXRCE-DDS agent + cuVSLAM + PX4 odometry bridge.
# Same code as the drone. This slot's repo checkout is mounted at /root/team (read-write, so you can build there).
set -eo pipefail
source "$(dirname "$0")/common.sh"
VERSION="$(cat "$SKYBETA_DIR/VERSION")"
docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
exec docker run --rm --name "$CONTAINER" --runtime nvidia --gpus all --network host --ipc host \
  -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" -e FASTRTPS_DEFAULT_PROFILES_FILE=/root/team/sim/dds/fastdds_udp.xml -v "$REPO:/root/team" "skybeta-sim:$VERSION" \
  bash -lc "source /opt/ros/humble/setup.bash && source /root/skybeta_ws/install/setup.bash &&
            ros2 launch /root/skybeta_ws/sim/skybeta_sim.launch.py xrce_udp_port:=$XRCE_PORT camera_x:=$CAMERA_X camera_z:=$CAMERA_Z \
              enable_slam_visualization:=False enable_landmarks_view:=False enable_observations_view:=False"
