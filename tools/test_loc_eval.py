"""Tests for tools/loc_eval.py's pure core (no ROS). Run: python3 -m pytest -q tools/test_loc_eval.py"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
import loc_eval as E  # noqa: E402


def yaw_q(yaw):
    yaw = np.asarray(yaw, float)
    z = np.zeros_like(yaw)
    return np.stack([z, z, np.sin(yaw / 2), np.cos(yaw / 2)], 1)


def gt_track():
    # 1 s on the ground, climb to 1 m by 2 s, then 10 m along x at 1 m/s, heading 0.
    t = np.arange(0, 13, 0.01)
    x = np.clip(t - 2, 0, 10)
    z = np.clip(t - 1, 0, 1)
    p = np.stack([x - 2.5, np.full_like(t, 0.9), z], 1)
    return E.Track(t, p, yaw_q(np.zeros_like(t)))


def test_interp_and_errors_zero_for_identical():
    gt = gt_track()
    ok, eh, ev, ey = E.errors(gt, gt)
    assert ok.all() and eh.max() < 1e-9 and np.abs(ev).max() < 1e-9 and np.abs(ey).max() < 1e-9


def test_yaw_interp_across_wrap():
    t = np.array([0.0, 1.0])
    tr = E.Track(t, np.zeros((2, 3)), yaw_q([math.pi - 0.1, -math.pi + 0.1]))
    _, y = tr.interp([0.5])
    assert abs(abs(y[0]) - math.pi) < 1e-9


def test_anchor_with_gt_and_drift_per_m():
    gt = gt_track()
    # VIO in its own frame (origin at the pad, rotated 0.5 rad), with 2 % scale drift along x.
    t = gt.t[::3]
    pg, _ = gt.interp(t)
    rel = pg - pg[0]
    rel[:, 0] *= 1.02
    c, s = math.cos(0.5), math.sin(0.5)
    R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    p_vio = (R @ rel.T).T + np.array([3.0, -1.0, 0.0])
    vio = E.Track(t, p_vio, yaw_q(np.full_like(t, 0.5)))
    raw = E.anchor_with_gt(vio, gt)
    s_, (_, eh, _, ey, _) = E.summarize("vio_gt0", raw, gt, [], None, None)
    assert abs(eh[-1] - 0.2) < 0.01            # 2 % of 10 m
    assert abs(s_["end_err_per_m"] - 0.02 * 10 / s_["path_m"]) < 0.002
    assert np.abs(ey).max() < 1e-9


def test_checkpoints_in_order():
    gt = gt_track()
    cps = E.checkpoint_times(gt, [[2.5, 0.9, 1.0], [-1.5, 0.9, 1.0]])
    assert abs(cps[0][0] - 7.0) < 0.02
    # The second is searched only after the first: closest approach is at the end, not t=3.
    assert cps[1][0] >= cps[0][0]


def test_latency_and_honesty():
    vio_t = np.arange(0, 10, 1 / 30)
    vio = E.Track(vio_t, np.zeros((len(vio_t), 3)), yaw_q(np.zeros(len(vio_t))), recv=vio_t + 0.010)
    est = E.Track(vio_t, np.zeros((len(vio_t), 3)), yaw_q(np.zeros(len(vio_t))), recv=vio_t + 0.0115)
    lat = E.latency_ms(est, vio)
    assert len(lat) == len(vio_t) and abs(np.median(lat) - 1.5) < 1e-6
    # Error ramps past 0.3 m at t = 6 s; WARN from t = 5 s -> lead 1 s, never OK while over.
    err_t = np.arange(0, 10, 0.1)
    err_h = np.clip((err_t - 3) * 0.1, 0, None)
    st_t = np.arange(0, 10, 0.1)
    st_l = np.where(st_t >= 5 - 1e-9, 1, 0)
    h = E.status_honesty(st_t, st_l, err_t, err_h)
    assert h["bad_ok_frac"] == 0.0 and abs(h["lead_s"] - 1.0) < 0.11
    # WARN only from 7 s: late by ~1 s, and some error samples over threshold while OK.
    h2 = E.status_honesty(st_t, np.where(st_t >= 7, 1, 0), err_t, err_h)
    assert h2["lead_s"] < -0.8 and h2["bad_ok_frac"] > 0


def test_read_course_real_config():
    here = os.path.dirname(__file__)
    cps, goal, start = E.read_course(os.path.join(here, "..", "planner", "config", "course_v3.yaml"))
    assert len(cps) >= 1 and goal is not None and abs(start[0] + 2.5146) < 1e-9
