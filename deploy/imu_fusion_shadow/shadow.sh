#!/bin/bash
# SHADOW cuVSLAM with IMU fusion on drone 12, for comparison only. Run on the Jetson HOST as dronehacks:
#   bash ~/team_ws/team_staging/imu_fusion/shadow.sh check            read-only: preconditions, nothing started
#   bash ~/team_ws/team_staging/imu_fusion/shadow.sh start [oak|px4]  start shadow + logger (refuses unless
#                                                                      PX4 is disarmed AND landed)
#   bash ~/team_ws/team_staging/imu_fusion/shadow.sh stop | status
# What it starts, all inside the running skybeta-stack container (docker exec; the container, the flight
# cuVSLAM, the leveller, the bridge and the competitor/dashboard launches are never touched):
#   team_imu_prep.py   /oak/imu/data (or PX4 sensor_combined) -> /team_imu/imu
#   static TF          base_link -> team_imu_frame (the measured IMU rotation, calib.json); node team_imu_tf
#   cuVSLAM            node /team_imu/team_imu_vio: IMU fusion ON, its own odom frame team_imu_odom,
#                      publishes NO TF, topics only under /team_imu/visual_slam/*. nice 10.
#   team_imu_log.py    read-only logger + GUARD: if PX4 reports armed or not landed it stops the shadow.
# Nothing reaches PX4: the bridge reads /claude/vio_leveled/odometry (from /visual_slam/tracking/odometry).
set -o pipefail
KIT=/root/team_ws/team_staging/imu_fusion          # same directory, seen from inside the container
HOSTKIT="$(cd "$(dirname "$0")" && pwd)"
C=skybeta-stack
ENVS="source /opt/ros/humble/setup.bash; source /root/skybeta_ws/install/local_setup.bash; cd $KIT"
dx() { docker exec "$C" bash -c "$ENVS; $*"; }
dxd() { docker exec -d "$C" bash -c "$ENVS; $*"; }
PAT='__node:=team_imu_vio|team_imu_prep.py|__node:=team_imu_tf|team_imu_log.py|grab_frames.py'

stop_all() {
  # bracketed patterns ([t]eam...) never match the pkill's own bash -c command line
  docker exec "$C" bash -c "pkill -INT -f '[t]eam_imu_log.py'; sleep 1; pkill -f '__node:=[t]eam_imu_vio'; pkill -f '[t]eam_imu_prep.py'; pkill -f '__node:=[t]eam_imu_tf'; pkill -f '[g]rab_frames.py'" 2>/dev/null
  sleep 2
  docker exec "$C" bash -c "pkill -9 -f '__node:=[t]eam_imu_vio'; pkill -9 -f '[t]eam_imu_prep.py'" 2>/dev/null
  true
}

case "${1:-}" in
check)
  docker ps --format '{{.Names}}' | grep -qx "$C" || { echo "FAIL: $C not running"; exit 1; }
  free -m | awk '/Mem:/ {print "memory available MB:", $7; if ($7 < 1200) {print "FAIL: < 1200 MB free"; exit 1}}' || exit 1
  dx "python3 $KIT/team_imu_log.py --check" ; rc=$?
  echo "--- shadow processes now:"; docker exec "$C" bash -c "ps -eo pid,etime,pcpu,args | grep -E '$PAT' | grep -v grep | cut -c1-140"
  exit $rc ;;
status)
  docker exec "$C" bash -c "ps -eo pid,etime,pcpu,rss,args | grep -E '$PAT' | grep -v grep | cut -c1-150"
  ls -t "$HOSTKIT/logs" 2>/dev/null | head -1 | sed 's/^/latest log dir: /'
  exit 0 ;;
stop) stop_all; echo "shadow stopped"; exit 0 ;;
record)
  # Diagnostics only, NO second cuVSLAM: logger (flight VIO raw + levelled, PX4 attitude, PX4/OAK gyro) and
  # left-camera JPEGs at 2 Hz. Read-only. Guard: stops if PX4 ARMS (carrying the drone is fine).
  docker exec "$C" bash -c "$ENVS; python3 $KIT/team_imu_log.py --check" | head -1
  docker exec "$C" bash -c "$ENVS; python3 - <<'PY'
import rclpy, time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from px4_msgs.msg import VehicleStatus
rclpy.init(); n = Node('team_imu_armchk'); st = {}
n.create_subscription(VehicleStatus, '/claude/usb/out/vehicle_status_v1', lambda m: st.update(a=int(m.arming_state)), QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=5))
t0 = time.time()
while 'a' not in st and time.time() - t0 < 6: rclpy.spin_once(n, timeout_sec=0.1)
raise SystemExit(0 if st.get('a') == 1 else 1)
PY" || { echo "REFUSING: PX4 not reporting disarmed"; exit 1; }
  stop_all
  STAMP="$(date -u +%Y%m%dT%H%M%SZ)"; LOG="logs/${STAMP}_record"; mkdir -p "$HOSTKIT/$LOG"
  dxd "exec nice -n 10 python3 -u $KIT/grab_frames.py --out $KIT/$LOG/frames --hz 2 > $KIT/$LOG/grab.log 2>&1"
  dxd "exec python3 -u $KIT/team_imu_log.py --out $KIT/$LOG --guard-armed > $KIT/$LOG/logger.log 2>&1"
  sleep 8; tail -1 "$HOSTKIT/$LOG/logger.log"; ls "$HOSTKIT/$LOG/frames" 2>/dev/null | wc -l | sed 's/$/ frames so far/'
  echo "recording to $HOSTKIT/$LOG  (stop: shadow.sh stop)"; exit 0 ;;
start) ;;
*) sed -n 2,16p "$0"; exit 1 ;;
esac

SRC="${2:-oak}"
[[ "$SRC" == oak || "$SRC" == px4 ]] || { echo "source must be oak or px4"; exit 1; }
[[ -f "$HOSTKIT/calib.json" ]] || { echo "no calib.json (see RUNBOOK step 0)"; exit 1; }
# The PATCHED node (build_vslam.sh): the stock 3.2.6 binary's IMU fusion is broken (IMU-timestamp bug).
VSLAM_BIN=$KIT/vio_ws/install/isaac_ros_visual_slam/lib/isaac_ros_visual_slam/isaac_ros_visual_slam
VSLAM_LIB=$KIT/vio_ws/install/isaac_ros_visual_slam/lib
if [[ "${STOCK:-0}" == 1 ]]; then VSLAM_BIN=/opt/ros/humble/lib/isaac_ros_visual_slam/isaac_ros_visual_slam; VSLAM_LIB=/opt/ros/humble/lib
elif [[ ! -f "$HOSTKIT/vio_ws/install/isaac_ros_visual_slam/lib/isaac_ros_visual_slam/isaac_ros_visual_slam" ]]; then
  echo "patched cuVSLAM not built: run build_vslam.sh first (STOCK=1 forces the stock, IMU-broken binary)"; exit 1; fi
# preconditions (disarmed + landed + fresh telemetry), checked inside the container
dx "python3 $KIT/team_imu_log.py --check" || { echo "REFUSING to start: preconditions failed"; exit 1; }
stop_all
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"; LOG="logs/${STAMP}_$SRC"; mkdir -p "$HOSTKIT/$LOG"
read QX QY QZ QW TX TY TZ < <(python3 -c "
import json; c=json.load(open('$HOSTKIT/calib.json')); q=c['quat_xyzw'] if '$SRC'=='oak' else [0,0,0,1]
t=c['tf_translation'] if '$SRC'=='oak' else [0,0,0]; print(*q, *t)")
# IMU noise model (continuous-time densities) and rate, per source; see RUNBOOK "parameters"
if [[ "$SRC" == oak ]]; then FREQ=200.0; GN=0.0012; GRW=0.0002; AN=0.02; ARW=0.01
else FREQ=100.0; GN=0.001; GRW=0.0001; AN=0.01; ARW=0.002; fi
[[ -n "${SHADOW_NOISE:-}" ]] && read GN GRW AN ARW <<< "$SHADOW_NOISE"
[[ -n "${SHADOW_FREQ:-}" ]] && FREQ="$SHADOW_FREQ"   # measured: /fmu/out/sensor_combined ~99 Hz (2026-11-08)
echo "src=$SRC imu_q=($QX $QY $QZ $QW) t=($TX $TY $TZ) freq=$FREQ noise gn=$GN grw=$GRW an=$AN arw=$ARW" | tee "$HOSTKIT/$LOG/config.txt"
cp "$HOSTKIT/calib.json" "$HOSTKIT/$LOG/"

dxd "exec python3 -u $KIT/team_imu_prep.py $SRC --calib $KIT/calib.json --px4-topic ${PX4_IMU_TOPIC:-/fmu/out/sensor_combined} > $KIT/$LOG/prep.log 2>&1"
dxd "exec /opt/ros/humble/lib/tf2_ros/static_transform_publisher --x $TX --y $TY --z $TZ --qx $QX --qy $QY --qz $QZ --qw $QW \
  --frame-id base_link --child-frame-id team_imu_frame --ros-args -r __node:=team_imu_tf > $KIT/$LOG/tf.log 2>&1"
sleep 2
# Our build has no RUNPATH (the stock binary carries the GXF/NITROS library dirs in its own): take the stock
# library's RUNPATH, with the patched lib dir in front so OUR libvisual_slam_node.so wins.
RP=$(docker exec "$C" bash -c "readelf -d /opt/ros/humble/lib/libvisual_slam_node.so | sed -n 's/.*runpath: \\[\\(.*\\)\\]/\\1/p'")
[[ -n "$RP" ]] || { echo "could not read the stock RUNPATH"; stop_all; exit 1; }
LDP="$VSLAM_LIB:$RP"
if ! docker exec "$C" bash -c "LD_LIBRARY_PATH=$LDP:\$LD_LIBRARY_PATH ldd $VSLAM_BIN" > "$HOSTKIT/$LOG/ldd.txt" 2>&1 \
   || grep -q "not found" "$HOSTKIT/$LOG/ldd.txt"; then
  echo "patched cuVSLAM has unresolved libraries:"; grep "not found" "$HOSTKIT/$LOG/ldd.txt"; stop_all; exit 1
fi
grep "libvisual_slam_node" "$HOSTKIT/$LOG/ldd.txt" | tee -a "$HOSTKIT/$LOG/config.txt"
dxd "export LD_LIBRARY_PATH=$LDP:\$LD_LIBRARY_PATH; \
 exec nice -n 10 $VSLAM_BIN --ros-args \
  -r __node:=team_imu_vio -r __ns:=/team_imu \
  -r visual_slam/image_0:=/oak/left/image_mono -r visual_slam/camera_info_0:=/oak/left/camera_info \
  -r visual_slam/image_1:=/oak/right/image_mono -r visual_slam/camera_info_1:=/oak/right/camera_info \
  -r visual_slam/imu:=/team_imu/imu \
  -p enable_image_denoising:=false -p rectified_images:=false -p image_qos:=SENSOR_DATA -p imu_qos:=SENSOR_DATA \
  -p enable_imu_fusion:=true -p imu_frame:=team_imu_frame -p imu_jitter_threshold_ms:=20.0 \
  -p gyro_noise_density:=$GN -p gyro_random_walk:=$GRW \
  -p accel_noise_density:=$AN -p accel_random_walk:=$ARW -p calibration_frequency:=$FREQ \
  -p base_frame:=base_link -p map_frame:=team_imu_map -p odom_frame:=team_imu_odom \
  -p 'camera_optical_frames:=[oak_left_camera_optical_frame,oak_right_camera_optical_frame]' \
  -p publish_odom_to_base_tf:=false -p publish_map_to_odom_tf:=false \
  -p enable_localization_n_mapping:=false -p image_jitter_threshold_ms:=40.0 \
  -p enable_slam_visualization:=false -p enable_landmarks_view:=false -p enable_observations_view:=false \
  > $KIT/$LOG/vslam.log 2>&1"
dxd "exec python3 -u $KIT/team_imu_log.py --out $KIT/$LOG --guard > $KIT/$LOG/logger.log 2>&1"
sleep 10
echo "shadow up ($SRC IMU), logs in $HOSTKIT/$LOG"
grep -aE "Enable IMU Fusion|frequency|noise|ERROR|rror" "$HOSTKIT/$LOG/vslam.log" | head -8
tail -2 "$HOSTKIT/$LOG/prep.log"
P=$(docker exec "$C" pgrep -f "__node:=team_imu_vio" | head -1)
echo "shadow cuVSLAM pid $P loads: $(docker exec "$C" bash -c "grep -o '/[^ ]*libvisual_slam_node.so' /proc/$P/maps 2>/dev/null | sort -u")" \
  | tee -a "$HOSTKIT/$LOG/config.txt"
