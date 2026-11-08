#!/usr/bin/env bash
# Re-run cuVSLAM offline on a recorded input bag (slot1.sh fly with RECORD_INPUT=1), on the VM, in its own
# container and ROS domain (41: no slot uses it), then score it against the bag's ground truth.
#   bash sim/imu_ab/replay.sh runs_imu/<run>/input <out_name> [launch args, e.g. enable_imu_fusion:=True imu_yaw:=0.1]
# Output: runs_imu/replay/<out_name>/{bag,launch.log,metrics.txt}. Same input every time -> a same-data A/B.
set -eo pipefail
cd "$(dirname "$0")/../.."
src="${1:?input bag}"; name="${2:?out name}"; shift 2
out="runs_imu/replay/$name"; rm -rf "$out"; mkdir -p "$out"
RID="${REPLAY_ID:-0}"; C=skybeta-replay-$RID; DOM=$((41 + RID));   # REPLAY_ID: parallel replays
 V="$(cat ~/skybeta/VERSION)"
E="-e ROS_DOMAIN_ID=$DOM -e FASTRTPS_DEFAULT_PROFILES_FILE=/root/team/sim/dds/fastdds_udp.xml"
IN="source /opt/ros/humble/setup.bash && source /root/skybeta_ws/install/setup.bash && cd /root/team"
# VSLAM_PATCHED=1: our patched visual_slam (vio/build_vslam.sh -> vio_ws) instead of the image's 3.2.6 binary
[ "${VSLAM_PATCHED:-0}" = 1 ] && IN="$IN && source /root/team/vio_ws/install/setup.bash"
docker rm -f $C >/dev/null 2>&1 || true
docker run -d --rm --name $C -e VSLAM_PATCHED=${VSLAM_PATCHED:-0} ${DH_IMU_DEBUG:+-e DH_IMU_DEBUG=1} --runtime nvidia --gpus all --network host --ipc host $E -v "$PWD:/root/team" "skybeta-sim:$V" \
  bash -lc "$IN && ros2 launch sim/imu_ab/replay_vslam.launch.py $* > $out/launch.log 2>&1" >/dev/null
for _ in $(seq 1 60); do grep -q "successfully initialized\|Waiting for" "$out/launch.log" 2>/dev/null && break; sleep 1; done
sleep 3
docker exec -d $E $C bash -lc "$IN && ros2 bag record -s mcap -o $out/bag /visual_slam/tracking/odometry /visual_slam/status > $out/rec.log 2>&1"
sleep 2
docker exec $E $C bash -lc "$IN && timeout ${PLAY_SECS:-600} ros2 bag play $src --start-offset ${START_OFFSET:-0} --topics /clock /oak/left/image_mono /oak/right/image_mono \
  /drone1/left/color/camera_info /drone1/right/color/camera_info /drone1/sensors/imu ${PLAY_ARGS:-}" > "$out/play.log" 2>&1 || true
sleep 2
docker exec $C pkill -INT -f "ros2 bag record" || true
sleep 3
grep -a -E "Enable IMU Fusion|frequency:|noise_density" "$out/launch.log" | sed 's/.*\]: //' | tr -s ' ' | head -4
echo "VSLAM_PATCHED=${VSLAM_PATCHED:-0} $*" > "$out/variant.txt"
docker rm -f $C >/dev/null 2>&1 || true
docker run --rm -v "$PWD:/root/team" "skybeta-sim:$V" bash -lc "$IN && python3 sim/imu_ab/vio_drift.py $src --vio-bag $out/bag \
  --json $out/vio_drift.json" | tee "$out/metrics.txt"
