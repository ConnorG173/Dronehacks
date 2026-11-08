#!/usr/bin/env bash
set -e
SKYBETA_UNDERLAY_ONLY=1
source skybeta_env.sh
if [[ -f /root/team_ws/claude/mavlink_usb/enabled ]]; then
    exec ros2 launch /root/team_ws/claude/mavlink_usb/bridge.launch.py
fi
exec ros2 launch /root/team_ws/claude/startup/bridge.launch.py
