"""streamer.py: path.bin -> course frame, T math, pad init, path clock + lead, fence."""
import math
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(HERE, ".."), HERE]
from localizer import streamer as S  # noqa: E402
from px4_model import PX4PosModel  # noqa: E402

PATH = os.path.join(HERE, "..", "data", "course_v3_path.bin")
PAD = (-2.5146, 0.9144)
CHECKPOINTS = [   # course_v3.yaml boxes: centre, half-extents
    (-1.9304, 2.5845, 1.0, 0.10, 0.12, 0.25), (-0.8128, 0.9715, 1.0, 0.10, 0.12, 0.25),
    (0.3048, 2.5845, 1.0, 0.10, 0.12, 0.25), (2.3114, 0.4572, 0.94, 0.25, 0.30, 0.25),
    (-0.4710, -2.2715, 0.762, 0.08, 0.08, 0.12)]


@pytest.fixture(scope="module")
def cpath():
    return S.path_to_course(S.load_path(PATH), PAD, 0.0)


def test_path_starts_at_pad_hover_and_visits_checkpoints(cpath):
    assert np.allclose(cpath.xy[0], PAD, atol=1e-6) and cpath.z[0] == pytest.approx(1.0)
    assert cpath.yaw[0] == pytest.approx(0.0)
    k = 0
    for cx, cy, cz, hx, hy, hz in CHECKPOINTS:              # every box entered, in order
        inside = ((np.abs(cpath.xy[:, 0] - cx) <= hx) & (np.abs(cpath.xy[:, 1] - cy) <= hy)
                  & (np.abs(cpath.z - cz) <= hz))
        hits = np.nonzero(inside[k:])[0]
        assert len(hits), (cx, cy)
        k += int(hits[0])
    assert all(S.inside_fence(p, margin_m=0.2) for p in cpath.xy)


def test_path_to_course_matches_readme_worked_example():
    # planner/README.md: StartPos (2, 3, 0), heading 90 deg; NED (2, 0, -1.5) is map (2, 5, 1.5);
    # NED (0, 1, 0) is map (3, 3, 0); NED yaw +90 deg is map heading 0
    rows = np.zeros((2, 12))
    rows[0, 1:4] = (2, 0, -1.5)
    rows[1, 1:4] = (0, 1, 0)
    rows[:, 10] = math.pi / 2
    p = S.path_to_course(rows, (2, 3), math.pi / 2)
    assert np.allclose(p.xy, [(2, 5), (3, 3)]) and np.allclose(p.z, (1.5, 0))
    assert np.allclose(p.yaw, 0)


def test_transform_roundtrip_and_pad_init():
    T, why = S.pad_init((0.02, -0.01), math.radians(1), 1.0, 0.6, PAD, 0.0)
    assert why == "ok"
    p, yaw = T.to_course((0.02, -0.01), math.radians(1))
    assert np.allclose(p, PAD) and yaw == pytest.approx(0.0)
    q, yq = T.to_odom(*T.to_course((1.3, -0.7), 0.4))
    assert np.allclose(q, (1.3, -0.7)) and yq == pytest.approx(0.4)


@pytest.mark.parametrize("args,needle", [
    (((0.5, 0.0), 0.0, 0.5, 0.5), "not near zero"),
    (((0.0, 0.0), math.radians(10), 0.5, 0.5), "yaw"),
    (((0.0, 0.0), 0.0, 179.0, 0.5), "disagrees"),           # the 180-deg-roll garbage seen before
])
def test_pad_init_refuses(args, needle):
    T, why = S.pad_init(*args, PAD, 0.0)
    assert T is None and needle in why


def test_course_velocity_and_acceleration_are_derivatives_of_position():
    # README worked example: heading 90 deg, NED velocity (4, 0, 0) is map (0, 4, 0)
    rows = np.zeros((2, 12))
    rows[:, 4:7] = (4, 0, -1)                # 4 m/s forward, 1 m/s up
    rows[:, 7:10] = (0, 2, 0)                # 2 m/s^2 to the right
    p = S.path_to_course(rows, (2, 3), math.pi / 2)
    assert np.allclose(p.vel, [(0, 4, 1)] * 2) and np.allclose(p.acc, [(2, 0, 0)] * 2)


def test_course_path_derivatives_match_course_v3(cpath):
    pos = np.c_[cpath.xy, cpath.z]
    v_fd = np.gradient(pos, cpath.t, axis=0)
    a_fd = np.gradient(cpath.vel, cpath.t, axis=0)
    assert np.abs(v_fd - cpath.vel).max() < 1e-3 and np.abs(a_fd - cpath.acc).max() < 0.05
    p, v, a, _ = cpath.state(-1.0)
    assert np.allclose(p, (*PAD, 1.0)) and not v.any() and not a.any()


def follow(clock, vehicle, dt=0.05, n=10000):
    out = []
    for _ in range(n):
        sp = clock.step(dt, vehicle(clock))
        out.append(sp)
        if clock.done:
            break
    return out


def test_clock_ramps_runs_at_scale_and_ends_at_rest_on_the_goal(cpath):
    c = S.PathClock(cpath, scale=0.5)
    sps = follow(c, lambda c: np.r_[c.path.at(c.tau)[:2]])          # vehicle exactly on p(tau)
    assert c.done and not c.stalled
    t_wall = 0.05 * len(sps)
    assert cpath.duration / 0.5 < t_wall < cpath.duration / 0.5 + 1.0   # + the 1 s ramp from rest
    assert sps[10].rate == pytest.approx(0.05 * 11 * 0.5, rel=1e-6)      # r slews at 1/s
    last = sps[-1]
    assert last.rate == 0.0 and not last.v.any() and not last.a.any()
    assert np.allclose(last.q, last.p) and np.allclose(last.p[:2], cpath.xy[-1])


def test_lead_is_on_the_path_and_one_over_kp_ahead(cpath):
    c = S.PathClock(cpath, scale=1.0, lead_s=1 / 0.95)
    c.tau, c.r = 5.0, 1.0
    sp = c.step(0.05, cpath.state(5.0)[0])
    xy, z, _ = cpath.at(sp.tau + 1 / 0.95)
    assert np.allclose(sp.q, np.r_[xy, z])
    assert np.allclose(sp.v, cpath.state(sp.tau)[1])


def test_clock_slows_on_lag_and_stops_but_not_on_overshoot(cpath):
    c = S.PathClock(cpath, scale=1.0, e_ok_m=0.3, e_stop_m=0.6)
    c.tau, c.r = 8.0, 1.0
    p, v, _, _ = cpath.state(8.0)
    t = v / np.linalg.norm(v)
    tau0 = c.tau
    for _ in range(5):                                    # 0.5 m AHEAD along the tangent: full rate
        c.step(0.05, cpath.state(c.tau)[0] + 0.5 * t)
    assert c.r == 1.0 and c.tau == pytest.approx(tau0 + 0.25)
    for _ in range(40):                                   # 1 m BEHIND: slows at 1/s, then stops
        sp = c.step(0.05, cpath.state(c.tau)[0] - 1.0 * t)
    assert c.stalled and sp.rate == 0.0 and np.allclose(sp.q, sp.p)
    tau = c.tau
    c.step(0.05, cpath.state(c.tau)[0] + np.array([0, 0, 0.7]))  # 0.7 m off the path: stays stopped
    assert c.tau == tau
    for _ in range(40):                                   # back on the path: resumes
        c.step(0.05, cpath.state(c.tau)[0])
    assert c.r == 1.0 and c.tau > tau


def run_closed_loop(path, scale, model_kw=None, lead_s=1 / 0.95, fault_at=None):
    """20 Hz node, flight_interface 10 Hz zero-order hold, PX4 model at 200 Hz."""
    xy0, z0, _ = path.at(0.0)
    m = PX4PosModel(np.r_[xy0, z0], **(model_kw or {}))
    c = S.PathClock(path, scale, lead_s=lead_s)
    xs, errs, stalls, hold, fi = [], [], 0, None, None
    for k in range(int(80 / 0.05)):
        sp = c.step(0.05, m.x)
        if fault_at is not None and k * 0.05 >= fault_at and hold is None:
            hold = sp.p                                   # path_streamer's HOLD: the on-path point
        sent = sp.q if hold is None else hold
        stalls += c.stalled and k > 0
        for j in range(10):
            if j % 20 == 5 or fi is None:                 # flight_interface samples at 10 Hz
                fi = sent
            m.step(0.005, fi)
        xs.append(m.x.copy())
        errs.append(sp.err)
        if c.done and np.linalg.norm(m.v) < 0.05:
            break
    return np.array(xs), np.array(errs), stalls, (k + 1) * 0.05, hold


def cross_track(path, xs):
    dense = np.c_[path.xy, path.z]
    return np.linalg.norm(xs[:, None] - dense[None], axis=2).min(axis=1)


def test_px4_model_tracks_course_v3_at_1_mps_without_stopping(cpath):
    scale = S.time_scale_for(cpath, 1.0)
    xs, errs, stalls, t, _ = run_closed_loop(cpath, scale)
    assert stalls == 0 and errs.max() < 0.15
    assert cross_track(cpath, xs).max() < 0.10
    assert t < cpath.duration / scale + 1.5
    assert np.linalg.norm(xs[-1] - np.r_[cpath.xy[-1], cpath.z[-1]]) < 0.1


def test_lead_tolerates_a_wrong_kp_by_slowing_not_stopping(cpath):
    scale = S.time_scale_for(cpath, 1.0)
    for kp in (0.7, 1.3):                                 # the drone's MPC_XY_P is not the default
        xs, errs, stalls, _, _ = run_closed_loop(cpath, scale, {"kp_xy": kp})
        assert stalls == 0 and cross_track(cpath, xs).max() < 0.15


def test_fault_hold_on_the_path_point_brakes_within_half_a_metre(cpath):
    scale = S.time_scale_for(cpath, 1.0)
    xs, _, _, _, hold = run_closed_loop(cpath, scale, fault_at=24.0)   # ~1 m/s on the tube leg
    k = int(24.0 / 0.05)
    assert np.linalg.norm(xs[k:] - hold, axis=1).max() < 0.5
    assert np.linalg.norm(xs[-1] - hold) < 0.05


def test_time_scale_caps_speed(cpath):
    s = S.time_scale_for(cpath, 1.0)
    assert cpath.speed.max() == pytest.approx(2.42, abs=0.05)
    assert s * cpath.speed.max() == pytest.approx(1.0)
    assert S.time_scale_for(cpath, 10.0) == 1.0


def test_fence():
    assert S.inside_fence((2.7, -2.7)) and not S.inside_fence((2.8, 0))
    assert np.allclose(S.clamp_to_fence((5, -5)), (2.749, -2.749))
