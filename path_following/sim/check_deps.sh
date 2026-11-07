#!/usr/bin/env bash
# On the VM: what the stack container and the host have for building the planner.
source "$(dirname "$0")/../../sim/scripts/common.sh"
echo "--- container $CONTAINER"
docker exec "$CONTAINER" bash -lc 'ls -d /opt/ros/humble/share/ompl /usr/share/ompl /usr/lib/*/cmake/ompl 2>/dev/null; dpkg -l | grep -iE "ompl|libeigen" | cut -c1-80; which cmake g++'
echo "--- host"
ls -d /opt/ros/humble/share/ompl 2>/dev/null; dpkg -l | grep -iE "ompl|libeigen" | cut -c1-80; which cmake g++
echo "--- team build"
ls /root 2>/dev/null; docker exec "$CONTAINER" bash -lc 'ls /root/team/install 2>/dev/null | head'
