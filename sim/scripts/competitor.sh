#!/usr/bin/env bash
# Run a competitor launch inside this slot's stack container (default: the organizers' example, which starts
# the flight interface + a square).   SIM_ID=0 bash sim/scripts/competitor.sh [<pkg> <file.launch.py> [args]]
set -eo pipefail
source "$(dirname "$0")/common.sh"
PKG="${1:-competitor_example}"; LAUNCH="${2:-competitor.launch.py}"; shift 2 2>/dev/null || true
exec docker exec -it -e FASTRTPS_DEFAULT_PROFILES_FILE=/root/team/sim/dds/fastdds_udp.xml "$CONTAINER" bash -lc "source /opt/ros/humble/setup.bash && source /root/skybeta_ws/install/setup.bash &&
  [ -f /root/team/install/setup.bash ] && source /root/team/install/setup.bash; ros2 launch $PKG $LAUNCH $*"
