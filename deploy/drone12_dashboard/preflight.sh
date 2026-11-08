#!/usr/bin/env bash
# Read-only checks before a drone-conservative-v1 flight on the dashboard stack. Sends nothing.
source "$(dirname "$0")/common.sh"
ct "$ENV_CT; set +e
echo '--- built';  ls $KIT_CT/repo/planner/build/gcopter_planner >/dev/null 2>&1 && echo planner OK || echo 'planner MISSING (run build.sh)'
ros2 pkg executables team_path_following 2>/dev/null | grep -q mission_tracker && echo package OK || echo 'package MISSING (run build.sh)'
echo '--- exactly one flight_interface (the dashboard one)'; ros2 node list --no-daemon --spin-time 5 2>/dev/null | grep -c flight_interface
echo '--- our mission_tracker must NOT already run'; ros2 node list --no-daemon --spin-time 5 2>/dev/null | grep mission_tracker || echo none
echo '--- leveled VIO (expect 30-60 Hz)'; timeout 5 ros2 topic hz /claude/vio_leveled/odometry 2>&1 | grep average | tail -1
echo '--- PX4 fuses (bridge input_topic; run.sh steers on the same topic)'; timeout 20 ros2 param get --no-daemon /px4_odom_bridge input_topic 2>/dev/null || echo 'bridge param unreadable'
echo '--- heading fix (only if enabled): want calibrated true, gyro_age_s < 0.5'; if [ -e /root/team_ws/claude/mavlink_usb/heading_fix_enabled ]; then timeout 5 ros2 topic hz /team_heading/odometry 2>&1 | grep average | tail -1; timeout 10 ros2 topic echo --once --no-daemon /team_heading/status 2>/dev/null | head -3 | cut -c1-500; else echo 'disabled (PX4 fuses leveled VIO)'; fi
echo '--- /skybeta/state (want px4_link true, armed false, landed true, nav_state OFFBOARD before arming)'
timeout 10 ros2 topic echo --once --no-daemon /skybeta/state 2>/dev/null | head -1 | cut -c1-700
echo '--- dashboard must be idle: no /skybeta/setpoint/pose in 10 s'; timeout 10 ros2 topic echo --once --no-daemon /skybeta/setpoint/pose >/dev/null 2>&1 && echo 'POSE SEEN: the dashboard is commanding, release control first' || echo 'none (good)'
echo '--- PX4 local position'; timeout 4 ros2 topic hz /fmu/out/vehicle_local_position 2>&1 | grep average | tail -1
"
