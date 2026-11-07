#!/usr/bin/env python3
"""Virtual RC transmitter for one sim slot: centred sticks over MAVLink MANUAL_CONTROL at 20 Hz, forever.

On the drone a safety pilot's transmitter is always on; Position mode (the arming mode and the go
signal) needs a live manual-control signal. Centred sticks = "hold", so they never trigger the
drone's stick-override takeover (COM_RC_OVERRIDE). Sends on PX4's onboard MAVLink link
(14580 + N) so it does not collide with sim/pilot.py on the GCS link.

    python3 sim/rc.py --sim 0          (run_sim.sh starts this in tmux window "rc")
"""
import socket
import sys
import time

from pymavlink import mavutil

sim_id = int(sys.argv[2]) if len(sys.argv) >= 3 and sys.argv[1] == "--sim" else 0
m = mavutil.mavlink_connection(f"udpout:127.0.0.1:{14580 + sim_id}", source_system=254)
m.port.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
m.port.bind(("127.0.0.1", 24580 + sim_id))   # fixed partner port: PX4 locks onto the first one it hears
print(f"virtual RC for sim {sim_id} -> udp {14580 + sim_id}", flush=True)
last_hb = 0.0
target = 1 + sim_id          # PX4 sets MAV_SYS_ID = instance + 1
while True:
    now = time.time()
    if now - last_hb > 1.0:
        m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        last_hb = now
    # x, y, r in [-1000, 1000]; z (throttle) in [0, 1000], 500 = centre (hold altitude in Position).
    m.mav.manual_control_send(target, 0, 0, 500, 0, 0)
    time.sleep(0.05)
