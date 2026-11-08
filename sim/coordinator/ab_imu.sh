#!/usr/bin/env bash
# IMU-fusion A/B in slot 2: same Gate B course, tracker on ground truth, cuVSLAM fusion off then on.
cd ~/slot2/Dronehacks
VIO_IMU=0 bash path_following/sim/slot2.sh reset 2>&1 | tail -1
bash path_following/sim/slot2.sh build 2>&1 | tail -4
docker exec skybeta-sim-2 bash -lc "ls /root/team/install | grep team_path_following" || { echo BUILD_FAILED; exit 1; }
for v in 0 1 0 1; do
  n=$(( $(ls -d path_following/runs/sim/imu${v}_* 2>/dev/null | wc -l) + 1 ))
  echo "=== VIO_IMU=$v run $n"
  VIO_IMU=$v bash path_following/sim/slot2.sh reset 2>&1 | tail -2
  VIO_IMU=$v bash path_following/sim/slot2.sh fly gt imu${v}_$n 2>&1 | tail -3
  head -1 path_following/runs/sim/imu${v}_$n/metrics.txt 2>/dev/null
  grep -E "^(at-go|best fit yaw\+t |horizontal path|VIO latency|window)" path_following/runs/sim/imu${v}_$n/metrics.txt 2>/dev/null
done
bash sim/run_sim.sh 2 stop
echo AB_DONE
