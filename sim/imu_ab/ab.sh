#!/usr/bin/env bash
# IMU-fusion A/B in slot 1, run ON THE VM from ~/slot1/Dronehacks (after sim/gcp/sync.sh 1 + sim/imu_ab/sync_pf.sh
# + `bash sim/imu_ab/slot1.sh build`). Gate B course, tracker on ground truth, so the only difference is VIO.
#   nohup bash sim/imu_ab/ab.sh <tag> 1 0 1 0 > /tmp/ab_<tag>.log 2>&1 &
# Each argument is one run: VIO_IMU value; a fresh reset (world + PX4 + stack) before every run.
# VIO_IMU_ARGS (env) goes to the fusion launch, e.g. VIO_IMU_ARGS="imu_yaw:=1.5708" for the negative control.
set -o pipefail
source /opt/ros/humble/setup.bash
cd "$(dirname "$0")/../.."
tag="${1:?tag}"; shift
for v in "$@"; do
  n=$(( $(ls -d runs_imu/${tag}_imu${v}_* 2>/dev/null | wc -l) + 1 ))
  name="${tag}_imu${v}_$n"
  echo "=== $(date -u +%T) $name  VIO_IMU=$v ${VIO_IMU_ARGS:+args: $VIO_IMU_ARGS}"
  VIO_IMU=$v bash sim/imu_ab/slot1.sh reset 2>&1 | tail -6
  mkdir -p "runs_imu/$name"
  tmux capture-pane -J -p -t sim1:stack -S -5000 | grep -a -E "Enable IMU Fusion|noise_density|random_walk|frequency:" \
    | sed 's/.*\]: //' | tee "runs_imu/$name/fusion_check.txt"
  echo "VIO_IMU=$v VIO_IMU_ARGS=${VIO_IMU_ARGS:-}" >> "runs_imu/$name/fusion_check.txt"
  RECORD_INPUT=${RECORD_INPUT:-0} bash sim/imu_ab/slot1.sh fly gt "$name" 2>&1 | tail -40 > "runs_imu/$name/fly_tail.txt"
  grep -a -E "^gazebo RTF|^vio_drift|horiz ATE|attitude drift|^at-go|checkpoints|clean" "runs_imu/$name/fly_tail.txt" "runs_imu/$name/metrics.txt" 2>/dev/null | head -12
done
echo "AB_DONE $(date -u +%T)"
