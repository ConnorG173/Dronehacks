"""Unit tests for the pure parts of benchtest/ (profiles, guard, sequencer, analysis).

Like tests/test_refnav_core.py these use a kinematic fake PX4: they test decisions,
not flight dynamics, and publish nothing.  Run:  python3 -m pytest -q tests
"""
import math
import struct

import pytest

from benchtest import analysis as A
from benchtest.guard import KILL, LAND, OK, Envelope, Guard, State, tilt_from_q
from benchtest.profiles import Circle, Hover, PathFile, Setpoint, Steps, extent, to_ned
from benchtest.sequencer import FlightParams, FlightSequencer

DT = 0.02


# ============================================================ profiles
def _check_derivatives(prof, tol=0.02):
    """v must be d/dt p and a must be d/dt v (central differences)."""
    h = 1e-4
    t = 0.013
    while t < prof.duration - 2 * h:
        s0, s1 = prof.sample(t - h), prof.sample(t + h)
        s = prof.sample(t)
        for i in range(3):
            assert abs((s1.p[i] - s0.p[i]) / (2 * h) - s.v[i]) < tol, (type(prof).__name__, t, i, "v")
            assert abs((s1.v[i] - s0.v[i]) / (2 * h) - s.a[i]) < tol * 10, (type(prof).__name__, t, i, "a")
        t += 0.037


def test_steps_smooth_feedforward_consistent_and_returns_home():
    p = Steps([("f", 0.2), ("r", -0.2), ("d", -0.1), ("yaw", 30)], hold=1.0, move_time=1.0)
    _check_derivatives(p)
    assert p.duration == pytest.approx(8 * 2.0)
    end = p.sample(p.duration)
    assert end.p == pytest.approx((0, 0, 0)) and end.yaw == pytest.approx(0)
    mid = p.sample(1.5)                       # first move done, holding
    assert mid.p[0] == pytest.approx(0.2) and mid.v == pytest.approx((0, 0, 0))
    yaw_hold = p.sample(6 * 2.0 + 1.5)
    assert yaw_hold.yaw == pytest.approx(math.radians(30))


def test_steps_raw_shape_jumps():
    p = Steps([("f", 0.3)], hold=2.0, shape="step")
    assert p.sample(0.001).p[0] == pytest.approx(0.3)
    assert p.sample(2.001).p[0] == pytest.approx(0.0)


def test_circle_feedforward_starts_and_ends_at_rest_at_home():
    c = Circle(radius=0.3, period=4.0, laps=2.0, ramp=2.0)
    _check_derivatives(c)
    for t in (0.0, c.duration):
        s = c.sample(t)
        assert s.p == pytest.approx((0, 0, 0), abs=1e-9)
        assert s.v == pytest.approx((0, 0, 0), abs=1e-9)
        assert s.a == pytest.approx((0, 0, 0), abs=1e-9)
    # cruise speed = R w0
    s = c.sample(c.ramp + 0.5)
    assert math.hypot(s.v[0], s.v[1]) == pytest.approx(0.3 * 2 * math.pi / 4.0)
    e = extent(c)
    assert e["xy"] == pytest.approx(0.6, abs=0.01)


def test_to_ned_rotates_by_start_heading():
    sp = Setpoint((1.0, 0.0, -0.5), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), 0.1, 0.2)
    n = to_ned(sp, (10.0, 20.0, -1.0), math.pi / 2)        # vehicle faces east
    assert n.p == pytest.approx((10.0, 21.0, -1.5))           # forward = +y (east)
    assert n.v == pytest.approx((0.0, 1.0, 0.0))
    assert n.a == pytest.approx((-1.0, 0.0, 0.0))             # right of east-facing = south = -x
    assert n.yaw == pytest.approx(math.pi / 2 + 0.1) and n.yaw_rate == 0.2


def _write_path(tmp_path, rows):
    f = tmp_path / "path.bin"
    data = b"DHPATH\0\0" + struct.pack("<IIQd", 1, 12, len(rows), 0.01)
    for r in rows:
        data += struct.pack("<12d", *r)
    f.write_bytes(data)
    return str(f)


def test_pathfile_relative_and_time_scaled(tmp_path):
    rows = []
    for i in range(201):                       # 2 s, constant acceleration 0.5 m/s^2 forward
        t = i * 0.01
        rows.append([t, 0.25 * t * t, 0.0, -1.5, 0.5 * t, 0, 0, 0.5, 0, 0, 0.2, 0.0])
    path = _write_path(tmp_path, rows)
    p = PathFile(path, speed=0.5)
    assert p.duration == pytest.approx(4.0)
    s0 = p.sample(0.0)
    assert s0.p == pytest.approx((0, 0, 0)) and s0.yaw == pytest.approx(0.0)
    s = p.sample(2.0)                           # tau = 1 s
    assert s.p[0] == pytest.approx(0.25, abs=1e-6)
    assert s.v[0] == pytest.approx(0.5 * 0.5)
    assert s.a[0] == pytest.approx(0.25 * 0.5)
    with pytest.raises(ValueError):
        PathFile(path, speed=1.5)


def test_pathfile_rejects_wrong_magic(tmp_path):
    f = tmp_path / "bad.bin"
    f.write_bytes(b"NOTAPATH" + bytes(64))
    with pytest.raises(ValueError):
        PathFile(str(f))


# ============================================================ guard
def _state(now, pos=(0, 0, -0.5), **kw):
    d = dict(pos=pos, vel=(0, 0, 0), yaw=0.0, tilt_deg=2.0, armed=True, nav_state=14, landed=False, pos_ok=True,
             health=True, battery_frac=0.8, t_pos=now, t_status=now, t_health=now)
    d.update(kw)
    return State(**d)


ORIGIN = (0.0, 0.0, 0.0)


def test_guard_ok_and_limits():
    g = Guard(Envelope())
    assert g.check(10, _state(10), ORIGIN, (0, 0, -0.5))[0] == OK
    assert g.check(10, _state(10, pos=(0.6, 0, -0.5)), ORIGIN)[0] == LAND          # fence 0.5
    assert g.check(10, _state(10, pos=(0.9, 0, -0.5)), ORIGIN)[0] == KILL          # fence + 0.3
    assert g.check(10, _state(10, pos=(0, 0, -1.1)), ORIGIN)[0] == LAND            # ceiling 1.0
    assert g.check(10, _state(10, pos=(0, 0, -1.4)), ORIGIN)[0] == KILL
    assert g.check(10, _state(10, tilt_deg=35), ORIGIN)[0] == LAND
    assert g.check(10, _state(10, tilt_deg=70), ORIGIN)[0] == KILL
    assert g.check(10, _state(10, battery_frac=0.1), ORIGIN)[0] == LAND
    assert g.check(10, _state(10, battery_frac=-1.0), ORIGIN)[0] == OK              # unknown battery
    assert g.check(10, _state(10, t_pos=9.0), ORIGIN)[0] == LAND                    # stale
    assert g.check(10, _state(10, pos_ok=False), ORIGIN)[0] == LAND


def test_guard_tracking_error_and_health_need_persistence():
    g = Guard(Envelope(track_err_m=0.3, track_err_s=0.4, health_bad_s=0.5))
    far = (0.4, 0, -0.5)
    assert g.check(0.0, _state(0.0), ORIGIN, far)[0] == OK          # 0.4 m away, not yet long enough
    assert g.check(0.3, _state(0.3), ORIGIN, far)[0] == OK
    assert g.check(0.45, _state(0.45), ORIGIN, far)[0] == LAND
    g.reset()
    assert g.check(1.0, _state(1.0, health=False), ORIGIN)[0] == OK
    assert g.check(1.6, _state(1.6, health=False), ORIGIN)[0] == LAND
    g2 = Guard(Envelope(), require_health=False)
    assert g2.check(5.0, _state(5.0, health=None, t_health=None), ORIGIN)[0] == OK


def test_tilt_from_quaternion():
    assert tilt_from_q((1, 0, 0, 0)) == pytest.approx(0.0)
    a = math.radians(20)
    assert tilt_from_q((math.cos(a / 2), math.sin(a / 2), 0, 0)) == pytest.approx(20.0)
    # pure yaw is not tilt
    assert tilt_from_q((math.cos(1.0), 0, 0, math.sin(1.0))) == pytest.approx(0.0, abs=1e-9)


# ============================================================ fake PX4 + sequencer
class FakePX4:
    """Kinematic stand-in: Offboard needs a recent setpoint stream, arms only in Offboard, tracks the
    setpoint with a lag, AUTO_LAND descends and auto-disarms, KILL disarms instantly."""

    def __init__(self, ground_z=0.0):
        self.pos = [0.0, 0.0, ground_z]
        self.ground = ground_z
        self.armed, self.nav, self.health, self.battery = False, 4, True, 0.9
        self.sp, self.t_sp, self.t_landed = None, None, None
        self.stream_log = []
        self.cmd_log = []
        self.tilt = 1.0
        self.of_loss_t = 0.5
        self.frozen = False          # True: vehicle cannot move (stuck on a tether)
        self.push = (0.0, 0.0, 0.0)  # disturbance added every tick

    def landed(self):
        return self.pos[2] >= self.ground - 0.02

    def state(self, now):
        return State(pos=tuple(self.pos), vel=(0, 0, 0), yaw=0.0, tilt_deg=self.tilt, armed=self.armed,
                     nav_state=self.nav, landed=self.landed(), pos_ok=True, health=self.health,
                     battery_frac=self.battery, t_pos=now, t_status=now, t_health=now)

    def apply(self, now, out):
        if out.setpoint is not None:
            self.sp, self.t_sp = out.setpoint, now
            self.stream_log.append((now, out.phase))
        for c in out.commands:
            self.cmd_log.append((now, c))
            if c == "offboard" and self.t_sp is not None and now - self.t_sp < 0.5:
                self.nav = 14
            elif c == "arm" and self.nav == 14:
                self.armed = True
            elif c == "land" and self.armed:
                self.nav = 18
            elif c == "kill":
                self.armed = False
            elif c == "disarm" and self.landed():
                self.armed = False

    def physics(self, now, dt):
        if self.nav == 14 and self.armed and (self.t_sp is None or now - self.t_sp > self.of_loss_t):
            self.nav = 18                                     # Offboard-loss failsafe
        if not self.armed or self.frozen:
            return
        if self.nav == 14 and self.sp is not None:
            for i in range(3):
                self.pos[i] += (self.sp.p[i] - self.pos[i]) * dt / 0.15 + self.sp.v[i] * dt * 0.0
        elif self.nav == 18:
            self.pos[2] = min(self.ground, self.pos[2] + 0.5 * dt)
            if self.landed():
                self.t_landed = self.t_landed if self.t_landed is not None else now
                if now - self.t_landed > 1.0:
                    self.armed = False
        for i in range(3):
            self.pos[i] += self.push[i]
        self.pos[2] = min(self.pos[2], self.ground)


def run(seq, px4, t_end=200.0, hooks=None):
    t = 0.0
    while t < t_end:
        if hooks:
            for h in hooks:
                h(t, seq, px4)
        out = seq.step(t, px4.state(t))
        px4.apply(t, out)
        px4.physics(t, DT)
        if seq.phase in ("done", "aborted", "yielded") and not px4.armed:
            break
        t += DT
    return t


def make(profile=None, **gkw):
    return FlightSequencer(profile or Hover(2.0), FlightParams(height_m=0.4, settle_s=1.0, hold_s=0.5),
                           Guard(Envelope(**gkw)))


def test_full_hover_flight_completes_and_lands():
    px4, seq = FakePX4(), make()
    run(seq, px4)
    assert seq.result == "test complete" and seq.phase == "done"
    cmds = [c for _, c in px4.cmd_log]
    assert cmds[:2] == ["offboard", "arm"]
    assert "land" in cmds and "kill" not in cmds
    first_cmd_t = px4.cmd_log[0][0]
    assert px4.stream_log[0][0] <= first_cmd_t - 1.4              # streamed before asking for Offboard
    land_seen = min(t for t, c in px4.cmd_log if c == "land")
    # stream stops once AUTO_LAND is in charge
    assert not [t for t, ph in px4.stream_log if t > land_seen + 0.1]


def test_steps_profile_runs_and_follows():
    px4, seq = FakePX4(), make(Steps([("f", 0.2), ("d", -0.1)], hold=1.0))
    run(seq, px4)
    assert seq.result == "test complete"


def test_operator_land_mid_profile():
    px4, seq = FakePX4(), make(Hover(20.0))
    run(seq, px4, hooks=[lambda t, s, p: s.request_land("key") if s.phase == "run" else None])
    assert seq.result == "operator: key" and seq.phase == "done"
    assert "kill" not in [c for _, c in px4.cmd_log]


def test_operator_kill_disarms_immediately():
    px4, seq = FakePX4(), make(Hover(20.0))
    run(seq, px4, hooks=[lambda t, s, p: s.request_kill("k") if s.phase == "run" else None])
    assert seq.result.startswith("killed") and not px4.armed
    assert px4.pos[2] < -0.3                                    # it stopped in the air (fake: no falling)


def test_kill_before_arm_never_arms():
    px4, seq = FakePX4(), make()
    seq.request_kill("changed my mind")
    run(seq, px4, t_end=5.0)
    assert seq.phase == "aborted" and not px4.cmd_log


def test_fence_breach_lands_and_hard_fence_kills():
    px4, seq = FakePX4(), make(Hover(20.0), track_err_m=5.0)

    def drift_out(t, s, p):
        if s.phase == "run":
            p.pos[0] = 0.6                                      # beyond the fence, inside the hard fence
    run(seq, px4, hooks=[drift_out])
    assert seq.result.startswith("fence") and "kill" not in [c for _, c in px4.cmd_log]
    px4b, seqb = FakePX4(), make(Hover(20.0), fence_xy_m=0.5, track_err_m=5.0)

    def teleport(t, s, p):
        if s.phase == "run":
            p.pos[0] = 1.0                                      # flyaway beyond the hard fence
    run(seqb, px4b, hooks=[teleport])
    assert seqb.result.startswith("killed: hard fence") and not px4b.armed


def test_stuck_on_tether_lands_on_tracking_error():
    px4, seq = FakePX4(), make(Steps([("f", 0.3)], hold=3.0), track_err_m=0.2)
    run(seq, px4, hooks=[lambda t, s, p: setattr(p, "frozen", s.phase == "run")])
    assert "tracking error" in seq.result


def test_estimator_loss_in_flight_lands():
    px4, seq = FakePX4(), make(Hover(20.0))
    run(seq, px4, hooks=[lambda t, s, p: setattr(p, "health", False) if s.phase == "run" else None])
    assert seq.result == "estimator unhealthy" and not px4.armed


def test_px4_leaving_offboard_is_not_fought():
    px4, seq = FakePX4(), make(Hover(20.0))

    def takeover(t, s, p):
        if s.phase == "run" and p.nav == 14:
            p.nav = 4                                           # PX4 switches to Hold
    t_end = run(seq, px4, t_end=30.0, hooks=[takeover])
    assert seq.phase == "yielded"
    t_yield = min(t for t, ph in px4.stream_log if ph == "run")
    assert not [t for t, ph in px4.stream_log if ph == "yielded"]
    assert t_end > t_yield


def test_unhealthy_estimator_never_arms():
    px4, seq = FakePX4(), make()
    px4.health = False
    seq.p.wait_s = 3.0
    run(seq, px4, t_end=5.0)
    assert seq.phase == "aborted" and "estimator" in seq.result and not px4.cmd_log


def test_low_battery_never_arms():
    px4, seq = FakePX4(), make()
    px4.battery = 0.3
    seq.p.wait_s = 2.0
    run(seq, px4, t_end=4.0)
    assert seq.phase == "aborted" and "battery" in seq.result


def test_arm_timeout_disarms_on_ground():
    px4, seq = FakePX4(), make()
    px4.apply = lambda now, out, f=px4.apply: f(now, type(out)(out.phase, out.setpoint,
                                                [c for c in out.commands if c != "offboard"], out.events))
    run(seq, px4, t_end=20.0)
    assert seq.phase == "aborted" and "timeout" in seq.result


# ============================================================ analysis
def test_frame_check_detects_flip_and_swap():
    assert A.frame_check("forward", (0.5, 0.02, 0.0))[0]
    ok, msg = A.frame_check("forward", (-0.5, 0.0, 0.0))
    assert not ok and "OPPOSITE" in msg
    ok, msg = A.frame_check("right", (0.5, 0.05, 0.0))
    assert not ok and "swapped" in msg
    assert A.frame_check("up", (0.0, 0.0, -0.45))[0]
    ok, msg = A.frame_check("forward", (0.4, 0.4, 0.0))
    assert not ok and "rotated" in msg


def test_body_displacement_uses_start_heading():
    f, r, d = A.body_displacement((0, 0, 0), (0, 1, 0), math.pi / 2)  # facing east, moved east
    assert (f, r, d) == pytest.approx((1, 0, 0))


def test_step_response_first_order():
    tau = 0.3
    ts = [i * 0.01 for i in range(400)]
    vals = [0.2 * (1 - math.exp(-t / tau)) for t in ts]
    m = A.step_response(ts, vals, 0.0, 0.0, 0.2)
    assert m["rise_s"] == pytest.approx(tau * math.log(9), abs=0.02)
    assert m["overshoot"] == pytest.approx(0.0)
    assert m["ss_err"] < 0.001


def test_tracking_lag_estimate():
    samples = []
    for i in range(500):
        t = i * 0.02
        sp = (math.sin(t), math.cos(t), 0.0)
        pos = (math.sin(t - 0.2), math.cos(t - 0.2), 0.0)
        samples.append((t, sp, pos))
    tr = A.tracking(samples)
    assert tr["lag_s"] == pytest.approx(0.2, abs=0.021)


def test_drift_stats():
    d = A.drift([(0.0, (0, 0, 0)), (10.0, (0.03, 0.04, 0))])
    assert d["net_drift_m"] == pytest.approx(0.05) and d["drift_rate_mps"] == pytest.approx(0.005)
