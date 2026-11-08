#!/usr/bin/env bash
set -e
source skybeta_env.sh
if [[ -f /root/team_ws/claude/mavlink_usb/enabled ]]; then
    exec ros2 launch /root/team_ws/claude/mavlink_usb/dashboard.launch.py
fi
exec ros2 launch /root/team_ws/claude/startup/dashboard.launch.py
