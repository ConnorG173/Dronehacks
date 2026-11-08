#!/usr/bin/env bash
# Candidate 3 repeats in slot 2 (path-following 5d62099): course_v3_gateb2 (chicane vias), tracker_sim_c3 (clearance
# re-timing), sphere dilation R 0.35, raw VIO. Lets c2 rep 2 finish, skips c2 rep 3 (candidate 2 retired), then
# installs ~/c3.tgz into slot 2, rebuilds, flies 3 fresh-reset runs.
until grep -aqE "=== c2 rep 3|C2_DONE" ~/rep2_runs.log || ! pgrep -f "[r]ep2_runs" >/dev/null; do sleep 10; done
pkill -f "[r]ep2_runs" || true; sleep 1; pkill -f "[s]lot2.sh fly" || true; pkill -f "[s]lot2.sh reset" || true; sleep 3
tar xzf ~/c3.tgz -C ~/slot2/Dronehacks
cd ~/slot2/Dronehacks
bash path_following/sim/slot2.sh reset 2>&1 | tail -1
bash path_following/sim/slot2.sh build 2>&1 | tail -3
for i in 1 2 3; do
  echo "=== c3 rep $i"
  [ "$i" -gt 1 ] && bash path_following/sim/slot2.sh reset 2>&1 | tail -1
  COURSE=planner/config/course_v3_gateb2.yaml CP_VIA=5,6,7 bash path_following/sim/slot2.sh fly vio c3rep$i \
    tracker_params:=/root/team/path_following/team_path_following/config/tracker_sim_c3.yaml \
    "planner_overrides:=MaxVelMag=1.2@DilateRadius=0.35@DilateShape=sphere" 2>&1 | tail -2
  head -1 path_following/runs/sim/c3rep$i/metrics.txt 2>/dev/null
done
echo C3_DONE
