#!/bin/bash
# Start / stop EGO-Planner in its own container next to skybeta-stack. Nothing in the stack changes;
# `run_ego.sh stop` (or just not starting it) is the full revert.
# usage: run_ego.sh start | stop | status
set -euo pipefail
NAME=skybeta-ego
TEAM=/home/dronehacks/team_ws
case "${1:-status}" in
  start)
    docker rm -f $NAME >/dev/null 2>&1 || true
    docker run -d --rm --name $NAME --network host --ipc host \
      -e ROS_DOMAIN_ID=12 -e SKYBETA_DDS_SCOPE=local \
      -v /home/dronehacks/skybeta/host/dds/fastdds_localhost.xml:/etc/skybeta/fastdds.xml:ro \
      -e FASTRTPS_DEFAULT_PROFILES_FILE=/etc/skybeta/fastdds.xml -e FASTDDS_DEFAULT_PROFILES_FILE=/etc/skybeta/fastdds.xml \
      -v $TEAM:/root/team_ws skybeta:0.3.2 \
      bash -lc 'source /opt/ros/humble/setup.bash && source /root/skybeta_ws/install/setup.bash && \
                source /root/team_ws/claude/ego_ws/install/setup.bash && \
                exec nice -n 5 ros2 launch /root/team_ws/claude/ego/ego.launch.py' >/dev/null
    echo "started $NAME"; ;;
  stop)
    docker rm -f $NAME >/dev/null 2>&1 && echo "stopped $NAME" || echo "not running"; ;;
  status)
    docker ps --filter name=$NAME --format '{{.Names}} {{.Status}}'; ;;
esac
