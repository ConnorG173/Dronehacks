#!/usr/bin/env bash
# Health check for one sim slot, run on the VM:  SIM_ID=0 bash sim/tools/check_sim.sh
# Prints topic rates for the whole chain and whether PX4's EKF2 is fusing vision (xy valid).
source "$(dirname "$0")/../scripts/common.sh"
docker exec -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" -e FASTRTPS_DEFAULT_PROFILES_FILE=/root/team/sim/dds/fastdds_udp.xml "$CONTAINER" bash -lc '
source /opt/ros/humble/setup.bash; source /root/skybeta_ws/install/setup.bash
for t in /drone1/left/color/image_raw /oak/left/image_mono /visual_slam/tracking/odometry /fmu/in/vehicle_visual_odometry; do
  r=$(timeout 8 ros2 topic hz "$t" 2>&1 | grep -m1 average); echo "$t: ${r:-NONE}"
done
echo "--- camera image contrast (a blank/white frame makes cuVSLAM report success with zero motion):"
python3 - <<'"'"'PY'"'"'
import numpy as np, rclpy, time
from rclpy.node import Node
from sensor_msgs.msg import Image
rclpy.init(); n = Node("img_chk"); got = {}
n.create_subscription(Image, "/oak/left/image_mono", lambda m: got.setdefault("m", m), 2)
t0 = time.time()
while "m" not in got and time.time() - t0 < 8: rclpy.spin_once(n, timeout_sec=0.1)
m = got.get("m")
if m is None: print("FAIL no /oak/left/image_mono")
else:
    a = np.frombuffer(bytes(m.data), np.uint8)[: m.height * m.step].reshape(m.height, m.step)[:, : m.width]
    ok = 10 < a.mean() < 245 and a.std() > 5
    print(("OK" if ok else "FAIL"), f"mean {a.mean():.1f} std {a.std():.1f}")
PY
echo "--- PX4 local position (best effort):"
timeout 6 ros2 topic echo --once --qos-reliability best_effort /fmu/out/vehicle_local_position px4_msgs/msg/VehicleLocalPosition 2>/dev/null \
  | grep -E "^(xy_valid|z_valid|v_xy_valid|x|y|z|dist_bottom|dist_bottom_valid):" || echo "no vehicle_local_position"
echo "--- flight interface state:"
timeout 4 ros2 topic echo --once /skybeta/state 2>/dev/null | head -5 || echo "flight interface not running"
'
