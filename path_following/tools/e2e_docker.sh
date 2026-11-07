#!/usr/bin/env bash
# Offline ROS 2 end-to-end test in the pf-dev:humble image: build team_path_following, run
# mission_tracker against kinsim_ros.py (flight interface + PX4 + VIO stand-in).
#   bash path_following/tools/e2e_docker.sh [extra mission_tracker --ros-args ...]
set -eo pipefail
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
docker run --rm --network none -e ROS_DOMAIN_ID=77 -v "$REPO:/repo" pf-dev:humble bash -c '
set -e
source /opt/ros/humble/setup.bash
mkdir -p /ws/src && ln -s /repo/path_following/team_path_following /ws/src/team_path_following
cd /ws && colcon build --symlink-install --packages-select team_path_following >/tmp/build.log 2>&1 || { cat /tmp/build.log; exit 1; }
source /ws/install/setup.bash
[ -x /repo/planner/build-docker/gcopter_planner ] || { echo "build the planner first: python3 path_following/tools/plan.py ..."; exit 1; }
python3 /repo/path_following/tools/kinsim_ros.py /repo/planner/config/course_v3.yaml --exit-when-done &
SIM=$!
timeout 120 ros2 run team_path_following mission_tracker --ros-args \
  -p course_config:=/repo/planner/config/course_v3.yaml \
  -p planner_binary:=/repo/planner/build-docker/gcopter_planner \
  -p out_dir:=/tmp/pf_out \
  --params-file /repo/path_following/team_path_following/config/tracker.yaml '"$*"' &
TRK=$!
wait $SIM; echo "sim exit $?"
kill -INT $TRK 2>/dev/null; wait $TRK 2>/dev/null || true
'
