#!/usr/bin/env bash
# Run a competitor launch inside the stack container (default: the organizers' example, which starts
# the flight interface + a square). Pass a different launch:  competitor.sh <pkg> <launch.py> [args]
set -eo pipefail
PKG="${1:-competitor_example}"; LAUNCH="${2:-competitor.launch.py}"; shift 2 2>/dev/null || true
exec docker exec -it skybeta-sim bash -lc "source /opt/ros/humble/setup.bash && source /root/skybeta_ws/install/setup.bash &&
  ros2 launch $PKG $LAUNCH $*"
