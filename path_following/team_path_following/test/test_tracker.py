import numpy as np

from team_path_following.pathfile import Trajectory
from team_path_following.tracker import Tracker, TrackerParams, priority_saturate


def line_traj(speed=1.0, T=6.0, dt=0.01):
    t = np.append(np.arange(0, T, dt), T)
    s = speed * (t - (T / (2 * np.pi)) * np.sin(2 * np.pi * t / T))  # rest-to-rest
    v = speed * (1 - np.cos(2 * np.pi * t / T))
    a = speed * (2 * np.pi / T) * np.sin(2 * np.pi * t / T)
    p = np.stack([s, np.zeros_like(s), np.ones_like(s)], 1)
    return Trajectory(t, p, np.stack([v, 0 * v, 0 * v], 1), np.stack([a, 0 * a, 0 * a], 1),
                      np.zeros_like(t), np.zeros_like(t))


def test_priority_saturate():
    c = np.array([0.0, 0.5, 0.0])
    u = np.array([2.0, 0.0, 0.0])
    v, alpha = priority_saturate(c, u, 1.0)
    assert abs(np.linalg.norm(v) - 1.0) < 1e-9 and np.allclose(v[1], 0.5) and 0 < alpha < 1
    v, alpha = priority_saturate(np.array([0, 3.0, 0]), u, 1.0)
    assert alpha == 0.0 and np.allclose(v, [0, 1.0, 0])
    v, alpha = priority_saturate(c, np.array([0.1, 0, 0]), 1.0)
    assert alpha == 1.0


def test_clock_slows_with_error():
    tr = Tracker(line_traj(), TrackerParams())
    assert tr.clock_target(0.0, 0.0) == 1.0
    assert tr.clock_target(0.25, 0.0) < 1.0
    assert tr.clock_target(0.5, 0.0) == 0.0
    assert tr.clock_target(0.0, 0.0, speed_scale=0.5) == 0.5


def test_tracks_ideal_first_order_plant():
    """Plant: first-order velocity lag tau_v. Inverse feedforward should track tightly."""
    traj = line_traj(speed=0.6)   # peak 1.2 m/s, under the 1.4 budget
    tp = TrackerParams(tau_v=0.5, t_delay=0.0, ff_mode='inverse', taudot_acc=100.0)
    tr = Tracker(traj, tp)
    tr.reset(tau=0.0, taudot=1.0)
    p = traj.p[0].copy()
    v = np.zeros(3)
    dt = 0.01
    worst = 0.0
    for _ in range(int(traj.duration / dt) + 100):
        u, _ = tr.step(dt, p, 0.0)
        v += (u - v) * dt / 0.5
        p += v * dt
        worst = max(worst, float(np.linalg.norm(traj.position(tr.tau) - p)))
    assert worst < 0.03
    assert np.linalg.norm(p - traj.p[-1]) < 0.02


def test_ceiling_guard():
    traj = line_traj()
    tr = Tracker(traj, TrackerParams(z_ceiling=1.2))
    u, _ = tr.step(0.02, np.array([0.0, 0.0, 1.25]), 0.0)
    assert u[2] < 0
