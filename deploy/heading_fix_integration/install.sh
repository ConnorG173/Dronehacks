#!/bin/bash
# Install the heading-fix integration into the teammates' dashboard/bridge files on drone 12 (HOST, dronehacks).
#   bash install.sh            check originals unchanged, back up, install (does NOT enable or restart anything)
#   bash install.sh enable     touch the marker (PX4 gets /team_heading/odometry after the bridge launch restarts)
#   bash install.sh disable    remove the marker (previous chain after the bridge launch restarts)
#   bash install.sh rollback   remove the marker and restore the backed-up originals
set -e
K=~/team_ws/team_staging/imu_fusion; C=~/team_ws/claude; B=$K/backups/heading_fix_integration
MARK=$C/mavlink_usb/heading_fix_enabled
FILES="mavlink_usb/bridge.launch.py dash/dash_node.py dash/dash.html"
case "${1:-install}" in
enable)  touch "$MARK"; echo "enabled (restart the bridge launch or reboot to apply)"; exit 0 ;;
disable) rm -f "$MARK"; echo "disabled (restart the bridge launch or reboot to apply)"; exit 0 ;;
rollback) rm -f "$MARK"; for f in $FILES; do cp -p "$B/$f" "$C/$f"; done; echo "originals restored"; exit 0 ;;
esac
cd "$C"
# refuse if a teammate changed these files since they were reviewed (md5 of the versions the patch was made from)
EXPECT="1841cf3734ca1145f6946cdd77fd4ce9 d13d8ec8d316b32e9d07b51ca5e8af37 0d9ef9a0566afe2f6b848234a66f6285 "
i=0; for f in $FILES; do e=$(echo $EXPECT | cut -d' ' -f$((i+1))); a=$(md5sum "$f" | cut -d' ' -f1)
  [ "$a" = "$e" ] || { echo "REFUSING: $f changed since review ($a != $e)"; exit 1; }; i=$((i+1)); done
mkdir -p "$B/mavlink_usb" "$B/dash"
for f in $FILES; do [ -f "$B/$f" ] || cp -p "$f" "$B/$f"; done
for f in $FILES; do cp "$K/integration/$f" "$f"; done
docker exec skybeta-stack bash -c "cd /root/team_ws/claude && python3 -m py_compile mavlink_usb/bridge.launch.py dash/dash_node.py /root/team_ws/team_staging/imu_fusion/heading_fix.py && echo compile OK"
echo "installed; marker $( [ -f "$MARK" ] && echo PRESENT || echo absent ) (absent = behaviour unchanged)"
