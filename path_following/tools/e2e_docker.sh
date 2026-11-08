#!/usr/bin/env bash
# Offline ROS 2 end-to-end test in the pf-dev:humble image: build team_path_following (+ an
# interface stub), launch mission.launch.py against kinsim_ros.py (flight interface + PX4 + VIO,
# and with MODE=loc also the localization chain: TF map->odom, TF course->map, /team/odom_map).
#   [COURSE=/repo/planner/config/<cfg>.yaml] bash path_following/tools/e2e_docker.sh [loc|vio] [extra launch args ...]
set -eo pipefail
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
MODE="${1:-loc}"; shift || true
case "$MODE" in
  loc) SIM_ARGS="--with-loc"; POSE_ARGS="" ;;   # launch defaults: /team/odom_map + TF course->odom
  vio) SIM_ARGS=""; POSE_ARGS="pose_topic:=/visual_slam/tracking/odometry pose_topic_frame:=odom" ;;
  *) echo "mode: loc | vio"; exit 1 ;;
esac
docker run --rm --network none -e ROS_DOMAIN_ID=77 -e SIM_ARGS="$SIM_ARGS" -e LAUNCH_ARGS="$POSE_ARGS $*" \
  -e COURSE="${COURSE:-/repo/planner/config/course_v3_gateb.yaml}" \
  -v "$REPO:/repo" pf-dev:humble bash /repo/path_following/tools/e2e_inside.sh
