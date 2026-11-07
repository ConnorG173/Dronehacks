"""core/follower.py: the state machine, with a scripted vehicle (no ROS, no PX4 model)."""
import math
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(HERE, "..")]
from localizer.core import frames as F  # noqa: E402
from localizer.core.follower import Config, Follower, VehicleState  # noqa: E402
from localizer.core.path import load_path, path_to_course  # noqa: E402

PATH = os.path.join(HERE, "..", "data", "course_v3_path.bin")
PAD = (-2.5146, 0.9144)
DT = 0.05


@pytest.fixture(scope="module")
def cpath():
    return path_to_course(load_path(PATH), PAD, 0.0)


def real_cfg(**kw):
    return Config(**kw)


def sim_cfg(**kw):
    base = dict(self_arm=True, land_at_end=True, z_from_pad=True, on_offboard_lost="stop",
                max_pad_offset_m=None, max_pad_yaw_deg=None, max_tilt_diff_deg=None, wait_timeout_s=45.0)
    base.update(kw)
    return Config(**base)


class Rig:
    """A vehicle that sits exactly on its last commanded position (local frame = T_true^-1 course)."""

    def __init__(self, cpath, cfg, T_true=F.Transform2D(*PAD, 0.0), z0=0.0):
        self.f = Follower(cpath, cfg)
        self.T_true, self.z0 = T_true, z0                # course <- local, and the local z of the floor
        self.t = 0.0
        self.s = VehicleState(estimate_ok=True, landed=True, local_tilt_deg=0.5, imu_tilt_deg=0.4)
        self.place_course(np.r_[PAD, 0.0], 0.0)
        self.cmds = []

    def place_course(self, xyz, yaw_course):
        xy, yaw = self.T_true.to_local(xyz[:2], yaw_course)
        self.s.pos, self.s.yaw = np.r_[xy, xyz[2] + self.z0], yaw

    def course_of(self, p_local):
        return np.r_[self.T_true.to_course(p_local[:2]), p_local[2] - self.z0]

    def step(self, track=True, fresh=True):
        if fresh:
            self.s.pose_stamp = self.s.status_stamp = self.s.landed_stamp = self.s.imu_stamp = self.t
        cmd = self.f.step(self.t, self.s)
        self.cmds.append(cmd)
        if track and cmd.publish and self.s.armed:
            self.s.pos = cmd.p.copy()
            self.s.landed = False
        self.t += DT
        return cmd

    def run_until(self, pred, n=4000, **kw):
        for _ in range(n):
            cmd = self.step(**kw)
            if pred(cmd):
                return cmd
        raise AssertionError(f"timed out in {self.f.mode}: {self.f.reason}")


# ----------------------------------------------------------------------------- WAIT / pad init

def test_wait_needs_fresh_valid_landed_disarmed(cpath):
    r = Rig(cpath, real_cfg())
    r.s.pose_stamp = None
    assert not r.f.step(0.0, r.s).publish and "position" in r.f.reason
    r.s.armed = True
    cmd = r.step()
    assert r.f.mode == "WAIT" and "landed and disarmed" in r.f.reason and not cmd.publish
    r.s.armed = False
    r.s.estimate_ok = False
    r.step()
    assert r.f.mode == "WAIT" and "estimator" in r.f.reason
    r.s.estimate_ok = True
    r.step()
    assert r.f.mode == "READY" and r.f.T is not None


def test_real_pad_init_refuses_odom_far_from_origin(cpath):
    r = Rig(cpath, real_cfg(), T_true=F.Transform2D(PAD[0] - 0.5, PAD[1], 0.0))   # odom origin 0.5 m off
    for _ in range(5):
        r.step()
    assert r.f.mode == "WAIT" and "not near zero" in r.f.reason


# ----------------------------------------------------------------------------- real: pilot arms

def test_real_never_requests_arm_and_starts_on_pilot_offboard(cpath):
    r = Rig(cpath, real_cfg())
    for _ in range(400):                                   # 20 s on the pad: streams, never requests
        cmd = r.step()
        assert not cmd.request_offboard_arm and not cmd.request_land
    assert r.f.mode == "READY" and cmd.publish and np.allclose(cmd.q, r.s.pos)
    r.s.armed = r.s.offboard = True                        # the pilot arms and switches to Offboard
    cmd = r.step()
    assert r.f.mode == "TAKEOFF"
    xy, z, _ = cpath.at(0.0)
    assert np.allclose(r.course_of(cmd.q), np.r_[xy, z])


def test_rotated_offset_local_frame_maps_the_path_start(cpath):
    # simulator-like: local frame rotated 30 deg, not at the pad, floor at local z = 0.07
    T_true = F.Transform2D(-1.0, 2.0, math.radians(30))
    r = Rig(cpath, sim_cfg(), T_true=T_true, z0=0.07)
    r.run_until(lambda c: c.request_offboard_arm)
    r.s.armed = r.s.offboard = True
    cmd = r.step()
    assert r.f.mode == "TAKEOFF"
    xy, z, yaw = cpath.at(0.0)
    assert np.allclose(r.course_of(cmd.p), np.r_[xy, z], atol=1e-9)
    assert F.wrap(cmd.yaw + T_true.yaw - yaw) == pytest.approx(0, abs=1e-9)


# ----------------------------------------------------------------------------- sim: self-arm

def test_sim_requests_offboard_arm_once_after_priming(cpath):
    r = Rig(cpath, sim_cfg())
    r.step()                                               # WAIT -> READY
    t_ready = r.t
    cmd = r.run_until(lambda c: c.request_offboard_arm)
    assert r.t - t_ready >= 2.0 and cmd.publish
    for _ in range(40):                                    # never repeated
        assert not r.step().request_offboard_arm


def test_sim_aborts_if_armed_before_our_request(cpath):
    r = Rig(cpath, sim_cfg())
    r.step()
    r.s.armed = True
    cmd = r.step()
    assert r.f.mode == "STOPPED" and not cmd.publish and "before our request" in r.f.reason


def test_sim_arm_timeout_stops(cpath):
    r = Rig(cpath, sim_cfg())
    r.run_until(lambda c: c.request_offboard_arm)
    r.run_until(lambda c: r.f.mode == "STOPPED", n=200)
    assert "timed out" in r.f.reason and not r.step().publish


def test_without_approval_no_request_and_preflight_timeout(cpath):
    r = Rig(cpath, sim_cfg(self_arm=False, wait_timeout_s=10.0))
    r.s.landed = False                                     # never ready
    r.run_until(lambda c: r.f.mode == "STOPPED", n=400)
    assert all(not c.request_offboard_arm and not c.publish for c in r.cmds)


# ----------------------------------------------------------------------------- flying

def fly_to_follow(r):
    r.run_until(lambda c: r.f.mode == "READY")
    r.s.armed = r.s.offboard = True
    r.run_until(lambda c: r.f.mode == "FOLLOW")
    for _ in range(100):                                   # 5 s into the path
        r.step()


@pytest.mark.parametrize("fault", ["stale", "jump", "estimator", "fence"])
def test_fault_holds_the_on_path_point(cpath, fault):
    r = Rig(cpath, real_cfg())
    fly_to_follow(r)
    last = r.step()
    hold_course = r.f.hold
    assert not np.allclose(last.q, last.p)                 # lead point is ahead of p(tau)
    if fault == "stale":
        r.t += 0.6                                         # no pose for 0.6 s (> pose_timeout_s)
        cmd = r.step(fresh=False, track=False)
    elif fault == "jump":
        r.s.pos = r.s.pos + np.array([0.8, 0.0, 0.0])
        cmd = r.step(track=False)
    elif fault == "estimator":
        r.s.estimate_ok = False
        cmd = r.step(track=False)
    else:
        r.s.pos = np.r_[r.T_true.to_local((2.9, 0.0)), r.s.pos[2]]
        r.s.pose_stamp = r.t
        cmd = r.f.step(r.t, r.s)
    assert r.f.mode == "HOLD" and cmd.publish
    assert np.allclose(cmd.q, cmd.p) and not cmd.v.any() and not cmd.a.any()
    if fault != "fence":                                   # the fence fault holds the previous p(tau)
        assert np.allclose(r.course_of(cmd.p)[:2], hold_course[0])
    for _ in range(20):                                    # HOLD keeps sending the same point
        assert np.allclose(r.step(track=False).p, cmd.p)


def test_end_of_path_requests_land_then_lands(cpath):
    r = Rig(cpath, sim_cfg())
    r.run_until(lambda c: c.request_offboard_arm)
    r.s.armed = r.s.offboard = True
    cmd = r.run_until(lambda c: r.f.mode == "FINISH")
    assert np.allclose(r.course_of(cmd.p)[:2], cpath.xy[-1], atol=1e-6)
    reqs = [r.step().request_land for _ in range(60)]      # 3 s: NAV_LAND re-sent every 1 s
    assert sum(reqs) == 3 and r.f.mode == "FINISH"
    r.s.offboard, r.s.auto_land = False, True              # PX4 switched to AUTO_LAND
    cmd = r.step()
    assert r.f.mode == "LANDING" and not cmd.publish
    r.s.armed, r.s.landed = False, True
    r.step()
    assert r.f.mode == "DONE" and not r.step().publish


def test_real_end_holds_goal_without_land_request(cpath):
    r = Rig(cpath, real_cfg())
    r.run_until(lambda c: r.f.mode == "READY")
    r.s.armed = r.s.offboard = True
    r.run_until(lambda c: r.f.mode == "FINISH")
    cmds = [r.step() for _ in range(40)]
    assert all(c.publish and not c.request_land for c in cmds)
    assert np.allclose(r.course_of(cmds[-1].q)[:2], cpath.xy[-1], atol=1e-6)


def test_offboard_lost_stops_commanding(cpath):
    r = Rig(cpath, sim_cfg())
    r.run_until(lambda c: c.request_offboard_arm)
    r.s.armed = r.s.offboard = True
    r.run_until(lambda c: r.f.mode == "FOLLOW")
    r.s.offboard = False                                   # pilot / PX4 took over
    cmds = [r.step() for _ in range(20)]
    assert r.f.mode == "STOPPED" and not any(c.publish or c.request_land for c in cmds)
    r.s.offboard = True                                    # Offboard back: still nothing (restart)
    assert not r.step().publish


def test_status_lost_counts_as_offboard_lost(cpath):
    r = Rig(cpath, sim_cfg())
    r.run_until(lambda c: c.request_offboard_arm)
    r.s.armed = r.s.offboard = True
    r.run_until(lambda c: r.f.mode == "FOLLOW")
    r.s.status_stamp = r.t - 2.0
    r.s.pose_stamp = r.t
    assert not r.f.step(r.t, r.s).publish and r.f.mode == "STOPPED"


def test_real_offboard_lost_holds(cpath):
    r = Rig(cpath, real_cfg())                             # on_offboard_lost="hold" (path_streamer)
    fly_to_follow(r)
    r.s.offboard = False
    cmd = r.step(track=False)
    assert r.f.mode == "HOLD" and cmd.publish and np.allclose(cmd.q, cmd.p)


def test_hold_then_land_after_timeout(cpath):
    r = Rig(cpath, sim_cfg(land_after_hold_s=3.0))
    r.run_until(lambda c: c.request_offboard_arm)
    r.s.armed = r.s.offboard = True
    r.run_until(lambda c: r.f.mode == "FOLLOW")
    r.s.estimate_ok = False
    r.step()
    assert r.f.mode == "HOLD"
    r.s.estimate_ok = True
    first = next(i for i in range(200) if r.step().request_land)
    assert first * DT == pytest.approx(3.0, abs=0.11)


def test_clock_backwards_stops(cpath):
    r = Rig(cpath, real_cfg())
    r.step()
    r.t -= 1.0
    assert not r.step().publish and r.f.mode == "STOPPED"


def test_dt_is_clamped_after_a_gap(cpath):
    r = Rig(cpath, real_cfg())
    fly_to_follow(r)
    tau = r.f.clock.tau
    r.t += 5.0                                            # the node stalled for 5 s
    r.step()
    assert r.f.clock.tau - tau <= 0.25 * r.f.scale + 1e-9


def test_fence_clamp_drops_feedforward(cpath):
    f = Follower(cpath, real_cfg())
    f.T, f.mode = F.Transform2D(0.0, 0.0, 0.0), "FOLLOW"
    cmd = f._course_cmd(np.array([2.9, 0.0, 1.0]), 0.0, v=np.ones(3), a=np.ones(3))
    assert cmd.p[0] == pytest.approx(3.049 - 0.3) and not cmd.v.any() and not cmd.a.any()
    cmd = f._course_cmd(np.array([1.0, 0.0, 1.0]), 0.0, v=np.ones(3), a=np.ones(3))
    assert cmd.v.all() and cmd.a.all()


def test_command_yaw_off(cpath):
    r = Rig(cpath, real_cfg(command_yaw=False))
    fly_to_follow(r)
    cmd = r.step()
    assert cmd.yaw is None and cmd.q_yaw is None


def test_dry_run_runs_the_path_without_a_vehicle(cpath):
    r = Rig(cpath, real_cfg(dry_run=True))
    cmd = r.run_until(lambda c: r.f.mode == "FINISH", track=False)
    assert np.allclose(r.course_of(cmd.q)[:2], cpath.xy[-1], atol=1e-6)


def test_bad_config_rejected():
    with pytest.raises(ValueError):
        Config(on_offboard_lost="land")
