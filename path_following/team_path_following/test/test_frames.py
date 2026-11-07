import math

import numpy as np

from team_path_following import frames


def test_readme_worked_example():
    # planner/README.md "Frames": StartPos (2, 3, 0), StartHeadingDeg 90
    S, psi0 = np.array([2.0, 3.0, 0.0]), math.radians(90)
    cases = [((2, 5, 1.5), (2, 0, -1.5)), ((3, 3, 0), (0, 1, 0))]
    for p_map, p_ned in cases:
        assert np.allclose(frames.map_to_planner_ned(p_map, S, psi0), p_ned, atol=1e-12)
        assert np.allclose(frames.planner_ned_to_map_points(p_ned, S, psi0), p_map, atol=1e-12)
    assert np.allclose(frames.planner_ned_to_map_vectors((4, 0, 0), psi0), (0, 4, 0), atol=1e-12)
    # map heading 0 (facing +x) is NED yaw +90 deg
    assert math.isclose(float(frames.planner_yaw_to_map(math.radians(90), psi0)), 0.0, abs_tol=1e-12)


def test_round_trip_random():
    rng = np.random.default_rng(1)
    for _ in range(50):
        S = rng.uniform(-3, 3, 3)
        psi0 = rng.uniform(-math.pi, math.pi)
        p = rng.uniform(-5, 5, (10, 3))
        back = frames.planner_ned_to_map_points(frames.map_to_planner_ned(p, S, psi0), S, psi0)
        assert np.allclose(back, p, atol=1e-12)
        theta = rng.uniform(-math.pi, math.pi)
        yaw_ned = frames.wrap_pi(psi0 - theta)   # the planner's forward map
        assert math.isclose(float(frames.wrap_pi(frames.planner_yaw_to_map(yaw_ned, psi0) - theta)), 0, abs_tol=1e-9)


def test_pose2_compose_inverse():
    a = frames.Pose2(1, 2, 0.3, 0.7)
    b = frames.Pose2(-0.5, 0.2, 0.1, -1.2)
    p = np.array([0.3, -0.4, 1.0])
    assert np.allclose(a.compose(b).apply(p), a.apply(b.apply(p)))
    assert np.allclose(a.inverse().apply(a.apply(p)), p)
    assert np.allclose(a.inverse().apply_vec(a.apply_vec(p)), p)


def test_map_odom_at_go():
    launch = frames.Pose2(-2.5, 0.9, 0.0, math.radians(30))
    odom_now = frames.Pose2(0.2, -0.1, 0.05, math.radians(10))   # stack started elsewhere
    T = frames.map_odom_at_go(launch, odom_now)
    # the vehicle's current odom pose maps to the launch pose
    assert np.allclose(T.apply(odom_now.t), launch.t)
    assert math.isclose(float(frames.wrap_pi(T.yaw + odom_now.yaw - launch.yaw)), 0, abs_tol=1e-12)
