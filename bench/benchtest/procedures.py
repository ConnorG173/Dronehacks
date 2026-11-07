"""bench.procedures - the bench and tethered tests, run from bench.py on the Jetson.

Each test returns True (pass) / False (fail).  Order and procedures: README.md.
"""
from __future__ import annotations

import csv
import json
import math
import os
import time
from datetime import datetime

from . import analysis as A
from .console import Operator, ask, confirm, in_multiplexer, say, status_line
from .guard import Envelope, Guard
from .profiles import Circle, Hover, PathFile, Steps, extent, wrap
from .sequencer import FINAL, FlightParams, FlightSequencer

NAV_NAMES = {0: "MANUAL", 1: "ALTCTL", 2: "POSCTL", 3: "AUTO_MISSION", 4: "AUTO_LOITER", 5: "AUTO_RTL",
             10: "ACRO", 12: "DESCEND", 13: "TERMINATION", 14: "OFFBOARD", 15: "STAB", 17: "AUTO_TAKEOFF",
             18: "AUTO_LAND", 19: "AUTO_FOLLOW", 20: "AUTO_PRECLAND", 21: "ORBIT", 22: "AUTO_VTOL_TAKEOFF"}
TICK_S = 0.02     # 50 Hz control / setpoint stream


def nav_name(n):
    return NAV_NAMES.get(n, str(n)) if n is not None else "?"


def _fmt3(v, nd=2):
    return "(" + ", ".join(f"{c:+.{nd}f}" for c in v) + ")" if v else "(?)"


def new_log_path(log_dir, name, ext):
    os.makedirs(log_dir, exist_ok=True)
    return os.path.join(log_dir, f"{datetime.now():%Y%m%d-%H%M%S}_{name}.{ext}")


def wait_for_px4(link, timeout=5.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        st, _ = link.snapshot()
        now = time.monotonic()
        if st.t_pos and st.t_status and now - st.t_pos < 0.5 and now - st.t_status < 0.5:
            return True
        time.sleep(0.05)
    say("ERROR: no PX4 telemetry. Check: MicroXRCEAgent running on the right serial port and baud, "
        "same ROS_DOMAIN_ID, px4_msgs matching the PX4 firmware version, UXRCE_DDS_CFG on the Pixhawk.")
    return False


def describe(st, ex):
    """One-line live state."""
    return (f"{nav_name(st.nav_state):10s} {'ARMED' if st.armed else 'disarmed':8s} "
            f"pos {_fmt3(st.pos)} tilt {st.tilt_deg or 0:4.1f}deg "
            f"est {'ok' if st.health else ('BAD' if st.health is False else '?')} "
            f"bat {('%3.0f%%' % (st.battery_frac * 100)) if st.battery_frac is not None and st.battery_frac >= 0 else '  ?'}"
            f" {ex.get('battery_v', float('nan')):.1f}V")


# =========================================================================== status
def status(link, args):
    """Live display until Ctrl-C. Safe: publishes nothing."""
    if not wait_for_px4(link):
        return False
    try:
        while True:
            st, ex = link.snapshot()
            status_line(describe(st, ex))
            time.sleep(0.2)
    except KeyboardInterrupt:
        say()
    return True


# =========================================================================== 1. link
REQUIRED_RATES = [  # key, min Hz, required
    ("vehicle_local_position", 10.0, True), ("vehicle_attitude", 10.0, True), ("vehicle_status", 1.0, True),
    ("vehicle_land_detected", 0.0, True), ("estimator_status_flags", 0.5, False),
    ("battery_status", 0.5, False), ("timesync_status", 0.5, False), ("failsafe_flags", 0.5, False),
    ("sensor_combined", 50.0, False),
    ("vio_odometry", 10.0, False), ("ev_to_px4", 10.0, False), ("estimator_health", 1.0, False),
]


def link_test(link, args):
    """Props off. Are all the topics there, at sane rates, and what does PX4 say about itself?"""
    say("Listening for 5 s ...")
    time.sleep(5.0)
    now = time.monotonic()
    ok = True
    say(f"\n{'topic':28s} {'ROS topic':48s} {'Hz':>7s}  result")
    for key, min_hz, required in REQUIRED_RATES:
        r = link.rates.get(key)
        if r is None:
            say(f"{key:28s} {'(not subscribed: message type missing)':48s} {'':7s}  "
                + ("FAIL" if required else "skipped"))
            ok &= not required
            continue
        hz = r.hz(now)
        seen = r.count > 0
        good = seen and hz >= min_hz
        if key in ("vio_odometry", "ev_to_px4", "estimator_health") and args.no_health:
            required = False
        res = "ok" if good else ("FAIL" if required else "warn")
        if not good and required:
            ok = False
        say(f"{key:28s} {link.topics[key]:48s} {hz:7.1f}  {res}" + ("" if seen else " (never received)"))
    st, ex = link.snapshot()
    say("\nPX4 state:")
    say(f"  mode {nav_name(st.nav_state)}, {'ARMED' if st.armed else 'disarmed'}, landed={st.landed}")
    say(f"  preflight checks pass: {ex.get('preflight_ok')}   failsafe active: {ex.get('failsafe')}")
    if ex.get("failsafe_flags"):
        say(f"  failsafe flags set: {', '.join(ex['failsafe_flags'])}")
    say(f"  local position valid xy={ex.get('xy_valid')} z={ex.get('z_valid')} heading_good={ex.get('heading_good')}"
        f"  eph={ex.get('eph', float('nan')):.2f} m epv={ex.get('epv', float('nan')):.2f} m")
    flags = [k for k in ("cs_tilt_align", "cs_yaw_align", "cs_ev_pos", "cs_ev_yaw", "cs_ev_hgt", "cs_ev_vel",
                         "cs_baro_hgt", "cs_mag_hdg", "cs_mag_3d", "cs_gps") if ex.get(k)]
    say(f"  EKF2 fusing: {', '.join(flags) if flags else '(no estimator_status_flags)'}")
    bv = ex.get("battery_v")
    say(f"  battery {bv if bv is not None else float('nan'):.2f} V, remaining "
        f"{(st.battery_frac * 100) if st.battery_frac is not None and st.battery_frac >= 0 else float('nan'):.0f}%")
    if "rtt_ms" in ex:
        rtt = ex["rtt_ms"]
        say(f"  timesync round trip {rtt:.1f} ms" + ("   <-- high: check the serial link / baud" if rtt > 20 else ""))
    if not st.armed and st.armed is not None and st.landed is False:
        say("  WARNING: PX4 says not landed while disarmed - land detector / IMU problem?")
    say("\nLINK: " + ("PASS" if ok else "FAIL"))
    return ok


# =========================================================================== 2. estimator
def estimator_test(link, args):
    """Props off, drone standing still. VIO health, EKF2 fusion flags, static drift and noise.
    Optional: cover the camera to check that unhealthy is detected and recovers."""
    if not wait_for_px4(link):
        return False
    dur = args.duration
    say(f"Keep the drone STILL for {dur:.0f} s (camera seeing a textured, lit scene) ...")
    px4, vio, health, valid, ev = [], [], [], [], []
    t0 = time.monotonic()
    while time.monotonic() - t0 < dur:
        st, ex = link.snapshot()
        now = time.monotonic()
        if st.pos:
            px4.append((now, st.pos))
        if link.last_vio and now - link.last_vio[0] < 0.5:
            vio.append(link.last_vio)
        health.append(st.health is True and st.t_health is not None and now - st.t_health < 1.0)
        valid.append(st.pos_ok)
        ev.append(bool(ex.get("cs_ev_pos")))
        status_line(f"{now - t0:5.1f} s  " + describe(st, ex))
        time.sleep(0.05)
    say()
    ok = True
    d = A.drift(px4)
    hf = sum(health) / max(len(health), 1)
    vf = sum(valid) / max(len(valid), 1)
    ef = sum(ev) / max(len(ev), 1)
    vio_hz = link.rates["vio_odometry"].hz(time.monotonic()) if "vio_odometry" in link.rates else 0.0

    def check(name, good, detail):
        nonlocal ok
        ok &= good
        say(f"  {'ok  ' if good else 'FAIL'} {name:30s} {detail}")

    say("\nResults:")
    if not args.no_health:
        check("estimator healthy", hf >= 0.99, f"{hf * 100:.1f}% of the time")
    check("PX4 local position valid", vf >= 0.99, f"{vf * 100:.1f}% of the time")
    check("EKF2 fusing vision position", ef >= 0.99 or "cs_ev_pos" not in link.snapshot()[1],
          f"{ef * 100:.1f}% of the time (cs_ev_pos)")
    check("VIO rate", vio_hz >= args.min_vio_hz or args.no_health, f"{vio_hz:.1f} Hz (need {args.min_vio_hz:.0f})")
    if d:
        check("PX4 position noise", d["noise_std_m"] <= args.max_noise, f"std {d['noise_std_m'] * 100:.2f} cm "
              f"(limit {args.max_noise * 100:.1f})")
        check("PX4 position drift", d["net_drift_m"] <= args.max_drift, f"{d['net_drift_m'] * 100:.1f} cm over "
              f"{d['duration_s']:.0f} s (limit {args.max_drift * 100:.0f})")
    else:
        check("PX4 position samples", False, "none")
    dv = A.drift(vio)
    if dv:
        say(f"       VIO raw: drift {dv['net_drift_m'] * 100:.1f} cm, noise {dv['noise_std_m'] * 100:.2f} cm "
            f"(VIO frame; compare with PX4 above)")

    if args.cover:
        say("\nCOVER TEST: cover the camera lens(es) completely with your hand when asked.")
        if ask("Press Enter, then cover the camera for ~5 s and uncover it: ") is None:
            return False
        t_start, went_bad, recovered = time.monotonic(), None, None
        while time.monotonic() - t_start < 25.0:
            st, ex = link.snapshot()
            now = time.monotonic()
            healthy = st.health is True and st.t_health is not None and now - st.t_health < 1.0
            if not healthy and went_bad is None:
                went_bad = now - t_start
            if went_bad is not None and healthy and recovered is None:
                recovered = now - t_start
                break
            status_line(f"{now - t_start:5.1f} s  health {'ok' if healthy else 'BAD'}  pos_valid {st.pos_ok}")
            time.sleep(0.05)
        say()
        check("unhealthy when covered", went_bad is not None,
              f"after {went_bad:.1f} s" if went_bad is not None else "health never went false: "
              "the guard would not catch a blinded camera")
        check("recovers when uncovered", recovered is not None,
              f"healthy again at {recovered:.1f} s" if recovered is not None else
              "did not recover within 25 s (VIO lost track? needs a reset?)")
        st, _ = link.snapshot()
        say(f"       position jump after recovery: check the plot; reset counter {link.snapshot()[1].get('ev_reset_counter')}")
    say("\nESTIMATOR: " + ("PASS" if ok else "FAIL"))
    return ok


# =========================================================================== 3. frames
FRAME_MOVES = [("forward", "in the direction the drone's nose points"), ("right", "to the drone's right"),
               ("up", "straight up")]


def frames_test(link, args):
    """Props off. Hand-carry the drone and check every axis has the right sign in PX4's frame."""
    if not wait_for_px4(link):
        return False
    say("Hold the drone level. For each move: press Enter, carry it ~0.5 m, hold still, press Enter.")
    ok = True
    for direction, how in FRAME_MOVES:
        if ask(f"\n[{direction.upper()}] Put it at a start spot and press Enter ") is None:
            return False
        st0, _ = link.snapshot()
        vio0 = link.last_vio
        if not st0.pos_ok:
            say("  FAIL: PX4 local position not valid - fix the estimator first (bench.py estimator)")
            return False
        if ask(f"  now move it ~0.5 m {how}, hold still, press Enter ") is None:
            return False
        st1, _ = link.snapshot()
        vio1 = link.last_vio
        disp = A.body_displacement(st0.pos, st1.pos, st0.yaw)
        good, msg = A.frame_check(direction, disp)
        ok &= good
        say(f"  {'ok  ' if good else 'FAIL'} PX4 moved fwd/right/down {_fmt3(disp)}: {msg}")
        if vio0 and vio1:
            vm = math.dist(vio0[1], vio1[1])
            pm = math.dist(st0.pos, st1.pos)
            if pm > 0.05:
                ratio = vm / pm
                scale_ok = 0.8 <= ratio <= 1.25
                ok &= scale_ok
                say(f"  {'ok  ' if scale_ok else 'FAIL'} VIO moved {vm:.2f} m, PX4 {pm:.2f} m (ratio {ratio:.2f}; "
                    f"far from 1 = scale or units problem)")
    if ask("\n[YAW] Put it down facing forward and press Enter ") is None:
        return False
    st0, _ = link.snapshot()
    if ask("  now rotate it ~90 deg CLOCKWISE seen from above, press Enter ") is None:
        return False
    st1, _ = link.snapshot()
    dyaw = math.degrees(wrap(st1.yaw - st0.yaw))
    good = 60.0 <= dyaw <= 120.0
    ok &= good
    if good:
        say(f"  ok   PX4 heading changed {dyaw:+.0f} deg")
    else:
        hint = "negative = yaw sign flipped" if dyaw < -45 else "too small/large = heading not tracking the rotation"
        say(f"  FAIL PX4 heading changed {dyaw:+.0f} deg (expected about +90; {hint})")
    say("\nFRAMES: " + ("PASS" if ok else "FAIL"))
    return ok


# =========================================================================== 4. motors
QUAD_X = {1: "front-right, CCW", 2: "rear-left, CCW", 3: "front-left, CW", 4: "rear-right, CW"}


def motors_test(link, args):
    """PROPS OFF. Spin each motor alone; the operator confirms position and direction."""
    if not wait_for_px4(link):
        return False
    st, _ = link.snapshot()
    if st.armed:
        say("FAIL: vehicle is armed - motor test only runs disarmed")
        return False
    if not confirm("MOTOR TEST. All PROPELLERS must be REMOVED. Needs PX4 param COM_MOT_TEST_EN=1.", "PROPS OFF"):
        return False
    say("Expected for PX4's generic Quadrotor X (check your Actuators setup if different):")
    for i in range(1, args.count + 1):
        say(f"  motor {i}: {QUAD_X.get(i, '(see your geometry)')}")
    results = {}
    for i in range(1, args.count + 1):
        if ask(f"\nPress Enter to spin MOTOR {i} for {args.seconds:.0f} s at {args.value:.2f} ") is None:
            break
        t = time.monotonic()
        link.actuator_test(i, args.value, args.seconds)
        time.sleep(0.5)
        acks = link.ack_since(t, 310)
        if acks and acks[-1] != 0:
            say(f"  PX4 rejected the motor test (MAV_RESULT {acks[-1]}): COM_MOT_TEST_EN=1? disarmed?")
        time.sleep(max(0.0, args.seconds - 0.5))
        link.actuator_test(i, float("nan"), 0.0)
        ans = ask(f"  Which motor spun and which way? (expected '{QUAD_X.get(i, '?')}'; Enter = as expected, "
                  "or type what you saw): ")
        if ans is None:
            break
        results[i] = "as expected" if ans == "" else ans
    say("\nMotor results:")
    ok = len(results) == args.count
    for i in range(1, args.count + 1):
        r = results.get(i, "not tested")
        ok &= r == "as expected"
        say(f"  motor {i}: {r}")
    say("\nMOTORS: " + ("PASS" if ok else "CHECK: fix motor order (Actuators outputs) or direction "
                                          "(swap two motor wires / ESC setting) before flying"))
    return ok


# =========================================================================== 5. arm / kill / link-loss
def _stream_until(link, sp, cond, timeout, every=TICK_S, stream=True):
    t0 = last = time.monotonic()
    while time.monotonic() - t0 < timeout:
        st, ex = link.snapshot()
        if cond(st):
            return time.monotonic() - t0
        if stream:
            link.send_setpoint(sp)
        if time.monotonic() - last >= 0.25:
            last = time.monotonic()
            status_line(describe(st, ex))
        time.sleep(every)
    return None


def arm_test(link, args):
    """PROPS OFF. Arm in Offboard and: (a) normal disarm, (b) KILL, (c) setpoint-stream loss failsafe."""
    from .profiles import Setpoint
    if not wait_for_px4(link):
        return False
    if not confirm("ARM TEST. Motors WILL spin. All PROPELLERS must be REMOVED.", "PROPS OFF"):
        return False
    ok = True
    for part in ("disarm", "kill", "stream-loss"):
        st, ex = link.snapshot()
        if st.armed:
            say("vehicle armed before the test step - sending disarm")
            link.do("disarm")
            time.sleep(1.0)
            continue
        if not st.pos_ok:
            say("FAIL: no valid PX4 local position - Offboard needs it (run the estimator test)")
            return False
        sp = Setpoint(tuple(st.pos), yaw=st.yaw)
        say(f"\n--- ({part}) streaming setpoints, then Offboard + arm")
        _stream_until(link, sp, lambda s: False, 1.5)
        link.do("offboard")
        link.do("arm")
        t_arm = _stream_until(link, sp, lambda s: s.armed and s.nav_state == 14, 6.0)
        if t_arm is None:
            st, ex = link.snapshot()
            say(f"\n  FAIL: did not arm in Offboard (mode {nav_name(st.nav_state)}, armed {st.armed}, "
                f"preflight ok {ex.get('preflight_ok')}, failsafe flags {ex.get('failsafe_flags')})")
            link.do("disarm")
            return False
        say(f"\n  armed in Offboard after {t_arm:.2f} s; holding 3 s")
        _stream_until(link, sp, lambda s: False, 3.0)
        if part == "disarm":
            link.do("disarm")
            dt = _stream_until(link, sp, lambda s: s.armed is False, 3.0, stream=False)
            good = dt is not None
            say(f"\n  {'ok  ' if good else 'FAIL'} normal disarm " + (f"in {dt:.2f} s" if good else "not accepted"))
        elif part == "kill":
            for _ in range(3):
                link.do("kill")
            dt = _stream_until(link, sp, lambda s: s.armed is False, 3.0, stream=False)
            good = dt is not None
            say(f"\n  {'ok  ' if good else 'FAIL'} KILL (force disarm) " +
                (f"in {dt:.2f} s - this is the emergency stop you will rely on" if good else
                 "NOT ACCEPTED - do not fly until this works"))
        else:
            say("  stopping the setpoint stream (simulates the Jetson program dying) ...")
            dt = _stream_until(link, sp, lambda s: s.nav_state != 14 or s.armed is False, 5.0, stream=False)
            st, _ = link.snapshot()
            good = dt is not None
            say(f"\n  {'ok  ' if good else 'FAIL'} PX4 left Offboard " +
                (f"after {dt:.2f} s, now {nav_name(st.nav_state)}, armed {st.armed} "
                 f"(expect ~COM_OF_LOSS_T then the COM_OBL_RC_ACT action, e.g. AUTO_LAND)" if good else
                 "- PX4 stayed in Offboard: check COM_OF_LOSS_T / COM_OBL_RC_ACT"))
            if st.armed:
                _stream_until(link, sp, lambda s: s.armed is False, 8.0, stream=False)
                st, _ = link.snapshot()
                if st.armed:
                    say("  still armed after the failsafe: sending disarm")
                    link.do("disarm")
                    time.sleep(1.0)
                else:
                    say("  PX4 disarmed by itself after the failsafe landing (expected on the ground)")
        ok &= good
        time.sleep(1.0)
    say("\nARM: " + ("PASS" if ok else "FAIL"))
    return ok


# =========================================================================== 6+. tethered flight
def make_profile(args):
    if args.test == "hover":
        return Hover(args.duration)
    if args.test == "steps":
        moves = [(a, args.step_m) for a in ("f", "r")] + [("d", -args.step_z_m), ("yaw", args.step_yaw_deg)]
        return Steps(moves, hold=args.hold, move_time=args.move_time, shape=args.shape)
    if args.test == "circle":
        return Circle(radius=args.radius, period=args.period, laps=args.laps, ramp=args.ramp)
    if args.test == "path":
        return PathFile(args.file, speed=args.speed)
    raise ValueError(args.test)


def make_envelope(args):
    return Envelope(fence_xy_m=args.fence, ceiling_m=args.ceiling, track_err_m=args.max_track_err,
                    tilt_land_deg=args.max_tilt, max_flight_s=args.max_flight_s,
                    battery_land_frac=args.min_battery_land)


def fly(link, args):
    """PROPS ON, TETHERED. Take off to --height, run the profile, land. Space/l = land, k = KILL."""
    profile = make_profile(args)
    env = make_envelope(args)
    ext = extent(profile)
    top = args.height + ext["up"]
    say(f"\n{args.test.upper()} plan: hover at {args.height:.2f} m, profile {profile.duration:.1f} s, "
        f"moves up to {ext['xy']:.2f} m sideways, {top:.2f} m max height, {ext['vmax']:.2f} m/s, "
        f"{ext['amax']:.2f} m/s^2 horizontal")
    say(f"Envelope: land beyond {env.fence_xy_m:.2f} m / above {env.ceiling_m:.2f} m / tilt {env.tilt_land_deg:.0f} deg"
        f" / tracking error {env.track_err_m:.2f} m;  KILL beyond +{env.hard_margin_m:.2f} m or tilt "
        f"{env.tilt_kill_deg:.0f} deg")
    problems = []
    if ext["xy"] > env.fence_xy_m - 0.1:
        problems.append(f"profile goes {ext['xy']:.2f} m sideways; fence is {env.fence_xy_m:.2f} m (need 0.1 m margin)")
    if top > env.ceiling_m - 0.1:
        problems.append(f"profile reaches {top:.2f} m; ceiling is {env.ceiling_m:.2f} m (need 0.1 m margin)")
    if args.height - ext["down"] < 0.2:
        problems.append(f"profile comes within {args.height - ext['down']:.2f} m of the ground (need 0.2 m)")
    if problems:
        for p in problems:
            say("REFUSED: " + p)
        say("Change --height / --fence / --ceiling or the profile size (--speed for a path does not shrink it).")
        return False
    if in_multiplexer() and not args.allow_multiplexer:
        say("REFUSED: running inside tmux/screen. If SSH drops, tmux keeps this program running and the "
            "link-loss landing never triggers. Run it directly in the SSH session "
            "(or pass --allow-multiplexer if you have another way to stop it).")
        return False
    if not wait_for_px4(link):
        return False
    say("\nChecklist: tether attached and short enough that the drone cannot reach people or walls; "
        "battery strapped; area clear; second SSH terminal open with 'python3 bench.py kill' typed; "
        "hand on the battery disconnect if you have one.")
    say("During the flight:  SPACE or l = land   k = KILL (motors off, drone drops)   "
        "Ctrl-C = land, twice = kill")
    if not confirm("PROPELLERS ON. The drone will arm and take off.", "FLY"):
        return False

    guard = Guard(env, require_health=not args.no_health)
    fp = FlightParams(height_m=args.height, climb_rate_mps=args.climb_rate, settle_s=args.settle)
    seq = FlightSequencer(profile, fp, guard)
    rows, run_samples = [], []
    log_csv = new_log_path(args.log_dir, args.test, "csv")
    t0 = time.monotonic()
    next_tick = t0
    last_print = 0.0
    kill_sent = warned = False

    with Operator() as op:
        op.start_keys()
        while True:
            now = time.monotonic()
            if op.kill:
                seq.request_kill(op.kill)
            if op.land:
                seq.request_land(op.land)
            st, ex = link.snapshot()
            out = seq.step(now, st)
            for e in out.events:
                say(f"\n[{now - t0:6.2f}] {e}")
            for c in out.commands:
                link.do(c)
            if out.setpoint is not None:
                link.send_setpoint(out.setpoint)
            sp = out.setpoint
            rows.append(_row(now - t0, seq.phase, sp, st))
            if seq.phase == "run" and sp is not None and st.pos is not None:
                run_samples.append((now - seq.t_run, sp, st.pos, st.yaw))
            if now - last_print >= 0.25:
                last_print = now
                err = math.dist(sp.p, st.pos) if (sp is not None and st.pos) else float("nan")
                status_line(f"{seq.phase:8s} {describe(st, ex)} err {err:.2f} m")
            if seq.phase in FINAL:
                # PX4 is in charge (failsafe) or we are done; still honour KILL until disarmed
                if st.armed is False and (seq.phase != "yielded" or now - seq.t_phase > 1.0):
                    break
                if op.kill and st.armed:
                    link.do("kill")
                    if not kill_sent:
                        say("\nKILL sent")
                        kill_sent = True
                if st.armed is not False and now - seq.t_phase > 5.0 and not warned:
                    say("\nsequence ended but the vehicle is still armed - watching; k = KILL")
                    warned = True
                if now - seq.t_phase > 120.0:
                    break
            next_tick += TICK_S
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_tick = time.monotonic()      # overrun: do not try to catch up
    say(f"\n\nResult: {seq.result}")
    _write_csv(log_csv, rows)
    summary = _summarise(args, seq, profile, run_samples)
    summary.update(result=seq.result, log=log_csv)
    with open(log_csv[:-4] + ".json", "w") as f:
        json.dump(summary, f, indent=2, default=str)
    say(f"Log: {log_csv} (+ .json).  Also pull the PX4 ULog for this flight.")
    return seq.result in ("test complete",)


def _row(t, phase, sp, st):
    nan3 = (math.nan,) * 3
    return [round(t, 3), phase,
            *(sp.p if sp else nan3), *(sp.v if sp else nan3), *(sp.a if sp else nan3),
            sp.yaw if sp else math.nan,
            *(st.pos or nan3), *(st.vel or nan3),
            st.yaw if st.yaw is not None else math.nan, st.tilt_deg if st.tilt_deg is not None else math.nan,
            int(bool(st.armed)), st.nav_state if st.nav_state is not None else -1, int(bool(st.landed)),
            -1 if st.health is None else int(st.health),
            st.battery_frac if st.battery_frac is not None else math.nan]


CSV_HEADER = ["t", "phase", "sp_x", "sp_y", "sp_z", "sp_vx", "sp_vy", "sp_vz", "sp_ax", "sp_ay", "sp_az", "sp_yaw",
              "x", "y", "z", "vx", "vy", "vz", "yaw", "tilt_deg", "armed", "nav_state", "landed", "est_healthy",
              "battery"]


def _write_csv(path, rows):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(CSV_HEADER)
        w.writerows(rows)


def _summarise(args, seq, profile, samples):
    s = {"test": args.test, "height_m": args.height}
    if not samples:
        say("No run-phase samples (the test did not reach its profile).")
        return s
    trk = A.tracking([(t, sp.p, pos) for t, sp, pos, _ in samples])
    s["tracking"] = trk
    say(f"Tracking during the profile: RMS {trk['rms_3d'] * 100:.1f} cm, max {trk['max_3d'] * 100:.1f} cm "
        f"(x {trk['rms_x'] * 100:.1f} / y {trk['rms_y'] * 100:.1f} / z {trk['rms_z'] * 100:.1f} cm RMS)"
        + (f", lag {trk['lag_s'] * 1000:.0f} ms" if trk.get("lag_s") is not None else ""))
    if isinstance(profile, Steps):
        s["steps"] = []
        hover, yaw0 = seq.hover, seq.yaw0
        rel = []     # (t, forward, right, down, yaw_rel)
        for t, sp, pos, yaw in samples:
            f, r, d = A.body_displacement(hover, pos, yaw0)
            rel.append((t, f, r, d, wrap((yaw or 0.0) - yaw0)))
        say("Step responses (rise 10-90%, overshoot, settle to 10%, steady-state error):")
        for i, (ts, axis, x0, x1) in enumerate(profile.segments):
            tend = profile.segments[i + 1][0] if i + 1 < len(profile.segments) else profile.duration
            col = {"f": 1, "r": 2, "d": 3, "yaw": 4}[axis]
            win = [(r[0], r[col]) for r in rel if ts <= r[0] < tend]
            m = A.step_response([w[0] for w in win], [w[1] for w in win], ts, x0, x1)
            if m is None:
                continue
            unit, k = ("deg", math.degrees(1)) if axis == "yaw" else ("cm", 100.0)
            name = {"f": "forward", "r": "right", "d": "down", "yaw": "yaw"}[axis]
            say(f"  {name:7s} {x0 * k:+6.1f} -> {x1 * k:+6.1f} {unit}: rise "
                f"{'%.2f s' % m['rise_s'] if m['rise_s'] is not None else '  n/a '}, overshoot {m['overshoot'] * 100:4.0f}%, "
                f"settle {'%.2f s' % m['settle_s'] if m['settle_s'] is not None else 'never'}, "
                f"ss err {m['ss_err'] * k:.1f} {unit}")
            s["steps"].append({"axis": axis, "from": x0, "to": x1, **m})
    return s


# =========================================================================== emergency
def kill(link, args):
    """Force-disarm NOW. No questions: this is the emergency stop for a second terminal."""
    for _ in range(10):
        link.do("kill")
        time.sleep(0.05)
    t0 = time.monotonic()
    while time.monotonic() - t0 < 2.0:
        st, _ = link.snapshot()
        if st.armed is False:
            say("disarmed")
            return True
        link.do("kill")
        time.sleep(0.1)
    st, _ = link.snapshot()
    say("disarmed" if st.armed is False else
        f"NOT confirmed disarmed (armed={st.armed}; no telemetry?) - pull the battery")
    return st.armed is False


def land(link, args):
    """Ask PX4 to land where it is. Note: if a bench.py flight is running it may still be streaming;
    PX4's AUTO_LAND takes over, and the flight program sees that and stops."""
    for _ in range(3):
        link.do("land")
        time.sleep(0.1)
    say("land command sent")
    return True
