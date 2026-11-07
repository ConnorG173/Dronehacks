#!/usr/bin/env bash
# Health check for one sim slot, run on the VM:  SIM_ID=0 bash sim/tools/check_sim.sh
# Prints topic rates for the whole chain and whether PX4's EKF2 is fusing vision (xy valid).
source "$(dirname "$0")/../scripts/common.sh"
docker exec -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" -e FASTRTPS_DEFAULT_PROFILES_FILE=/root/team/sim/dds/fastdds_udp.xml "$CONTAINER" bash -lc '
source /opt/ros/humble/setup.bash; source /root/skybeta_ws/install/setup.bash
for t in /drone1/left/color/image_raw /oak/left/image_mono /visual_slam/tracking/odometry /fmu/in/vehicle_visual_odometry; do
  r=$(timeout 8 ros2 topic hz "$t" 2>&1 | grep -m1 average); echo "$t: ${r:-NONE}"
done
echo "--- PX4 local position (best effort):"
timeout 6 ros2 topic echo --once --qos-reliability best_effort /fmu/out/vehicle_local_position px4_msgs/msg/VehicleLocalPosition 2>/dev/null \
  | grep -E "^(xy_valid|z_valid|v_xy_valid|x|y|z|dist_bottom|dist_bottom_valid):" || echo "no vehicle_local_position"
echo "--- flight interface state:"
timeout 4 ros2 topic echo --once /skybeta/state 2>/dev/null | head -5 || echo "flight interface not running"
'
