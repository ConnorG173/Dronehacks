#!/usr/bin/env bash
# Candidate 2 repeats in slot 2: Gate B, raw VIO, tracker_sim_x500, time_scale 1.0, sphere dilation R 0.35.
# Waits for ~/c2_ready (the coordinator touches it after syncing path-following 2f108ba), then rebuilds.
cd ~/slot2/Dronehacks
while [ ! -f ~/c2_ready ]; do sleep 15; done
bash path_following/sim/slot2.sh reset 2>&1 | tail -1
bash path_following/sim/slot2.sh build 2>&1 | tail -3
for i in 1 2 3; do
  echo "=== c2 rep $i"
  [ "$i" -gt 1 ] && bash path_following/sim/slot2.sh reset 2>&1 | tail -1
  bash path_following/sim/slot2.sh fly vio c2rep$i tracker_params:=/root/team/path_following/team_path_following/config/tracker_sim_x500.yaml "planner_overrides:=MaxVelMag=1.2@DilateRadius=0.35@DilateShape=sphere" 2>&1 | tail -2
  head -1 path_following/runs/sim/c2rep$i/metrics.txt 2>/dev/null
done
echo C2_DONE
