#!/bin/bash
# Powers the VM off (stopped = no compute charge, disk kept) after IDLE_MIN minutes with no
# established SSH (22, which is how IAP arrives) or noVNC (6080) connection. Run every 5 min by
# cloudsim-idle.timer. A running sim with nobody connected counts as idle on purpose.
IDLE_MIN=${IDLE_MIN:-30}
STAMP=/run/cloudsim-last-active          # /run is emptied at boot, so a fresh boot gets a full grace period

n=$(ss -Htn state established '( sport = :22 or sport = :6080 )' | wc -l)
if [ "$n" -gt 0 ] || [ ! -f "$STAMP" ]; then
    touch "$STAMP"
    exit 0
fi
idle=$(( ($(date +%s) - $(stat -c %Y "$STAMP")) / 60 ))
if [ "$idle" -ge "$IDLE_MIN" ]; then
    logger -t cloudsim "idle for ${idle} min, powering off"
    systemctl poweroff
fi
