#!/usr/bin/env bash
# The organizers' stack, unchanged, in their x86 container: uXRCE-DDS agent + cuVSLAM + PX4 odometry bridge.
# Same code as the drone. This slot's repo checkout is mounted at /root/team (read-write, so you can build there).
set -eo pipefail
source "$(dirname "$0")/common.sh"
VERSION="$(cat "$SKYBETA_DIR/VERSION")"
# VIO_IMU=1: our copy of the organizers' sim launch with cuVSLAM IMU fusion on (A/B test); default = theirs.
# VSLAM_PATCHED=1: overlay our patched visual_slam (vio/build_vslam.sh: IMU timestamp fix) from <repo>/vio_ws.
# VIO_IMU_ARGS: extra launch arguments for it, e.g. "imu_yaw:=1.5708 gyro_noise_density:=0.0005".
LAUNCH=/root/skybeta_ws/sim/skybeta_sim.launch.py
EXTRA=""
OVERLAY=""
[[ "${VSLAM_PATCHED:-0}" == "1" ]] && OVERLAY="source /root/team/vio_ws/install/setup.bash &&"
[[ "${VIO_IMU:-0}" == "1" ]] && LAUNCH=/root/team/sim/launch/skybeta_sim_imu.launch.py && EXTRA="${VIO_IMU_ARGS:-}"
docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
exec docker run --rm --name "$CONTAINER" --runtime nvidia --gpus all --network host --ipc host \
  -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" -e FASTRTPS_DEFAULT_PROFILES_FILE=/root/team/sim/dds/fastdds_udp.xml -v "$REPO:/root/team" "skybeta-sim:$VERSION" \
  bash -lc "source /opt/ros/humble/setup.bash && source /root/skybeta_ws/install/setup.bash && $OVERLAY
            ros2 launch $LAUNCH xrce_udp_port:=$XRCE_PORT camera_x:=$CAMERA_X camera_z:=$CAMERA_Z \
              enable_slam_visualization:=False enable_landmarks_view:=False enable_observations_view:=False $EXTRA"
