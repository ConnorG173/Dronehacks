#!/usr/bin/env bash
# Restart the gyro heading fix's bias calibration WITHOUT the dashboard's Reset button (which also sends the
# interface a Hold). Drone on the pad, disarmed, nobody touching it. The heading node itself refuses unless PX4
# reports landed. Publishes one String on /team_heading/calibrate; no flight command. Waits until calibrated.
set -uo pipefail
source "$(dirname "$0")/common.sh"
ct "test -e /root/team_ws/claude/mavlink_usb/heading_fix_enabled" || { echo "heading fix disabled: nothing to calibrate"; exit 0; }
ct "$ENV_CT; ros2 topic pub --once /team_heading/calibrate std_msgs/msg/String \"{data: 'cv1 calibrate.sh'}\" >/dev/null"
echo "calibration restarted: keep the drone still"
for i in $(seq 1 30); do
  sleep 2
  st=$(ct "$ENV_CT; timeout 3 ros2 topic echo --once /team_heading/status 2>/dev/null" | head -1)
  echo "  $(echo "$st" | grep -o '"still_s": [0-9.]*\|"calibrated": [a-z]*' | tr '\n' ' ')"
  echo "$st" | grep -q '"calibrated": true' && echo "calibrated: pilot may arm" && exit 0
done
echo "NOT calibrated after 60 s (drone moving, not landed, or gyro stale)"; exit 1
