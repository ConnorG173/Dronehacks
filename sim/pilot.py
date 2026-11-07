#!/usr/bin/env python3
"""Sim stand-in for the safety pilot (PX4 SITL over MAVLink, GCS port 14550).

On the drone the pilot arms, then switches to Position; the organizers' flight interface
treats that as the go signal and enters Offboard once. This does the same in sim.

    python3 sim/pilot.py --sim 0 go        # wait for a position estimate, arm, switch to Position
    python3 sim/pilot.py --sim 0 land      # PX4 Land mode (the pilot's abort)
    python3 sim/pilot.py --sim 0 kill      # disarm immediately (the kill switch; only on the ground!)
    python3 sim/pilot.py --sim 0 status    # mode, armed
--sim N talks to PX4 instance N on its own GCS port (18570 + N), so two sims never cross.
"""
import sys
import time

from pymavlink import mavutil

PX4_MAIN_MODE = {"position": 3, "land": None}


def connect(sim_id, timeout=30):
    url = f"udpout:127.0.0.1:{18570 + sim_id}"
    m = mavutil.mavlink_connection(url, source_system=255)
    t0 = time.time()
    hb = None
    while hb is None and time.time() - t0 < timeout:
        # udpout: PX4 only replies once it has heard from us
        m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        hb = m.wait_heartbeat(timeout=1)
    if hb is None:
        sys.exit("no heartbeat from PX4 on " + url)
    return m


def wait_position_valid(m, timeout=120):
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = m.recv_match(type="ESTIMATOR_STATUS", blocking=True, timeout=2)
        if msg is None:
            continue
        flags = msg.flags
        horiz_rel = bool(flags & mavutil.mavlink.ESTIMATOR_POS_HORIZ_REL)
        vert = bool(flags & mavutil.mavlink.ESTIMATOR_POS_VERT_ABS)
        if horiz_rel and vert:
            return True
        print(f"waiting for EKF position (horiz_rel={horiz_rel} vert={vert}) ...", flush=True)
    return False


def command(m, cmd, *params):
    p = list(params) + [0] * (7 - len(params))
    m.mav.command_long_send(m.target_system, m.target_component, cmd, 0, *p)
    ack = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
    ok = ack is not None and ack.command == cmd and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
    print(f"cmd {cmd}: {'ACCEPTED' if ok else ack}")
    return ok


def set_px4_mode(m, main_mode, sub_mode=0):
    return command(m, mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                   mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, main_mode, sub_mode)


def status(m):
    hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=3)
    armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) if hb else None
    print("mode:", m.flightmode, "armed:", armed)


def main():
    args = sys.argv[1:]
    sim_id = 0
    if len(args) >= 2 and args[0] == "--sim":
        sim_id, args = int(args[1]), args[2:]
    if len(args) != 1 or args[0] not in ("go", "land", "kill", "status"):
        sys.exit(__doc__)
    m = connect(sim_id)
    what = args[0]
    if what == "status":
        status(m)
    elif what == "go":
        if not wait_position_valid(m):
            sys.exit("EKF never reported a valid position: is cuVSLAM tracking and the bridge running?")
        # Preflight checks flap for a few seconds after start (accel bias, position stability), so retry
        # like a pilot would: every 2 s for up to 90 s.
        t0 = time.time()
        while not command(m, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1):
            if time.time() - t0 > 90:
                sys.exit("arming refused for 90 s (see: SIM_ID=N bash sim/tools/px4_cmd.sh commander check)")
            time.sleep(2)
        time.sleep(0.5)
        set_px4_mode(m, PX4_MAIN_MODE["position"])
        status(m)
    elif what == "land":
        command(m, mavutil.mavlink.MAV_CMD_NAV_LAND)
    elif what == "kill":
        command(m, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 21196)


if __name__ == "__main__":
    main()
