#!/usr/bin/env bash
# Runs inside pf-dev:humble (see e2e_docker.sh). Env: SIM_ARGS, LAUNCH_ARGS, COURSE.
COURSE="${COURSE:-/repo/planner/config/course_v3_gateb.yaml}"
LAUNCH_FILE="${LAUNCH_FILE:-mission.launch.py}"
set -e
source /opt/ros/humble/setup.bash
mkdir -p /ws/src
ln -s /repo/path_following/team_path_following /ws/src/team_path_following
cp -r /repo/path_following/tools/stub_fi /ws/src/skybeta_flight_interface
rm /ws/src/skybeta_flight_interface/COLCON_IGNORE
cd /ws
colcon build --symlink-install --packages-select team_path_following skybeta_flight_interface >/tmp/build.log 2>&1 || { cat /tmp/build.log; exit 1; }
source /ws/install/setup.bash
if [ ! -x /repo/planner/build-docker/gcopter_planner ]; then
  echo "build the planner first: python3 path_following/tools/plan.py ..."; exit 1
fi
python3 /repo/path_following/tools/kinsim_ros.py $COURSE --exit-when-done $SIM_ARGS &
SIM=$!
# shellcheck disable=SC2086
if [ "$LAUNCH_FILE" = mission.launch.py ]; then
  BASE_ARGS="allow_mode_request:=true mission_preset:=course course_config:=$COURSE"
else
  BASE_ARGS="repo_root:=/repo"   # a frozen profile: it fixes its own course and settings
fi
timeout 150 ros2 launch team_path_following $LAUNCH_FILE $BASE_ARGS \
  planner_binary:=/repo/planner/build-docker/gcopter_planner \
  out_dir:=/tmp/pf_out $LAUNCH_ARGS &
TRK=$!
set +e
wait $SIM
echo "sim exit $?"
kill -INT $TRK 2>/dev/null
wait $TRK 2>/dev/null
exit 0
