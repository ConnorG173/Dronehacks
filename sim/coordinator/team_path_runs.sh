#!/usr/bin/env bash
# Teammates' path (localizer/data/course_v3_path.bin, window route) + raw cuVSLAM + conservative tracker, 3 runs.
cd ~/slot2/Dronehacks
ARGS="path_file:=/root/team/team_course_v3_path.bin tracker_params:=/root/team/path_following/team_path_following/config/tracker_conservative.yaml time_scale:=1.4 speed_factor:=1.0 max_speed_mps:=1.5 setpoint_rate_hz:=10.0 auto_offboard_on_arm:=true z_ceiling_m:=1.6 return_config:="
VIO_IMU=0 bash path_following/sim/slot2.sh reset 2>&1 | tail -1
bash path_following/sim/slot2.sh build 2>&1 | tail -3
docker exec skybeta-sim-2 bash -lc "ls /root/team/install | grep -q team_path_following" || { echo BUILD_FAILED; exit 1; }
for i in 1 2 3; do
  echo "=== teampath run $i"
  [ "$i" -gt 1 ] && VIO_IMU=0 bash path_following/sim/slot2.sh reset 2>&1 | tail -1
  TEAMPATH=1 COURSE=planner/config/course_v3.yaml bash path_following/sim/slot2.sh fly vio teampath_c$i $ARGS 2>&1 | tail -2
  head -1 path_following/runs/sim/teampath_c$i/metrics.txt 2>/dev/null
  grep -aE "^at-go|^horizontal path" path_following/runs/sim/teampath_c$i/metrics.txt 2>/dev/null
done
echo RUNS_DONE
