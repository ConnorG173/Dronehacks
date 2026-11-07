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
cp -r /repo/path_following/tools/stub_fi /ws/src/skybeta_flight_interface && rm /ws/src/skybeta_flight_interface/COLCON_IGNORE
cd /ws && colcon build --symlink-install --packages-select team_path_following skybeta_flight_interface >/tmp/build.log 2>&1 || { cat /tmp/build.log; exit 1; }
source /ws/install/setup.bash
[ -x /repo/planner/build-docker/gcopter_planner ] || { echo "build the planner first: python3 path_following/tools/plan.py ..."; exit 1; }
python3 /repo/path_following/tools/kinsim_ros.py /repo/planner/config/course_v3.yaml --exit-when-done &
SIM=$!
timeout 150 ros2 launch team_path_following mission.launch.py allow_mode_request:=true \
  course_config:=/repo/planner/config/course_v3.yaml planner_binary:=/repo/planner/build-docker/gcopter_planner \
  out_dir:=/tmp/pf_out pose_topic:=/visual_slam/tracking/odometry pose_topic_frame:=odom '"$*"' &
TRK=$!
wait $SIM; echo "sim exit $?"
kill -INT $TRK 2>/dev/null; wait $TRK 2>/dev/null || true
'
