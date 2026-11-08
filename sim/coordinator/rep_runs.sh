#!/usr/bin/env bash
# Repeats of path following's 30 s candidate (Gate B, raw VIO, time_scale 1.0, tracker_sim_x500) in slot 2.
cd ~/slot2/Dronehacks
bash path_following/sim/slot2.sh build 2>&1 | tail -2
for i in 1 2 3; do
  echo "=== rep $i"
  bash path_following/sim/slot2.sh reset 2>&1 | tail -1
  bash path_following/sim/slot2.sh fly vio rep$i tracker_params:=/root/team/path_following/team_path_following/config/tracker_sim_x500.yaml 2>&1 | tail -2
  head -1 path_following/runs/sim/rep$i/metrics.txt 2>/dev/null
  grep -aE "rtf|RTF" path_following/runs/sim/rep$i/metrics.txt 2>/dev/null | head -2
done
echo REPS_DONE
