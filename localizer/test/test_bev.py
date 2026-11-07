"""bev.py: levelling from PX4 attitude, frame_bev geometry on synthetic depth images, the frame
stacker, the LocalBevMap recentre crash (the prototype's bev_live.log), and depth.ScaleFit."""
import math
import os
import sys
import time

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(HERE, "..")]
from localizer import bev  # noqa: E402
from localizer.depth import ScaleFit  # noqa: E402

# OAK-D-S2 left camera at the matcher's 512x320, mounted forward (drone.env CAMERA_X/Z)
W, H, F = 512, 320, 341.1
K = (F, F, W / 2, H / 2)
R_BODY_CAM = np.array([[0.0, 0, 1], [-1, 0, 0], [0, -1, 0]])      # optical (x right, y down, z fwd) -> FLU
T_BODY_CAM = np.array([0.1053, 0.0, 0.0304])


def q_px4(roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0):
    """PX4 attitude quaternion (w, x, y, z), FRD -> NED, ZYX Euler (PX4 conventions:
    roll + = right wing down, pitch + = nose up, yaw + = clockwise from north)."""
    r, p, y = (math.radians(v) / 2 for v in (roll_deg, pitch_deg, yaw_deg))
    qx = np.array([math.cos(r), math.sin(r), 0, 0])
    qy = np.array([math.cos(p), 0, math.sin(p), 0])
    qz = np.array([math.cos(y), 0, 0, math.sin(y)])

    def mul(a, b):
        w1, x1, y1, z1 = a
        w2, x2, y2, z2 = b
        return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                         w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])
    return mul(qz, mul(qy, qx))


# ----------------------------------------------------------------------------- levelling

def test_level_identity():
    assert np.allclose(bev.level_rotation_from_px4((1, 0, 0, 0)), np.eye(3))


def test_level_pitch_nose_up_points_forward_axis_up():
    R = bev.level_rotation_from_px4(q_px4(pitch_deg=10))
    fwd = R @ [1, 0, 0]
    assert fwd[2] == pytest.approx(math.sin(math.radians(10)))
    assert fwd[1] == pytest.approx(0, abs=1e-12)


def test_level_roll_right_wing_down_points_left_axis_up():
    R = bev.level_rotation_from_px4(q_px4(roll_deg=10))
    left = R @ [0, 1, 0]
    assert left[2] == pytest.approx(math.sin(math.radians(10)))


@pytest.mark.parametrize("yaw", [0, 37, 90, -150, 180])
def test_level_independent_of_yaw(yaw):
    R0 = bev.level_rotation_from_px4(q_px4(4, -7, 0))
    assert np.allclose(bev.level_rotation_from_px4(q_px4(4, -7, yaw)), R0, atol=1e-12)
    assert bev.tilt_deg(bev.px4_to_enu_flu(q_px4(4, -7, yaw))) == pytest.approx(
        math.degrees(math.acos(math.cos(math.radians(4)) * math.cos(math.radians(7)))))


def test_px4_yaw_maps_to_enu_ccw():
    # PX4 yaw +90 (clockwise from north = facing east) is ENU yaw 0; yaw 0 (north) is ENU +90
    assert bev.yaw_of(bev.px4_to_enu_flu(q_px4(yaw_deg=90))) == pytest.approx(0, abs=1e-12)
    assert bev.yaw_of(bev.px4_to_enu_flu(q_px4(yaw_deg=0))) == pytest.approx(math.pi / 2)


# ----------------------------------------------------------------------------- frame_bev

def render_depth(R_level_body, agl, wall_x=2.02, wall_half_y=None):
    """Depth image of a vertical wall plane x = wall_x (level frame) and the floor z = -agl."""
    v, u = np.mgrid[0:H, 0:W].astype(float)
    d_cam = np.stack([(u - K[2]) / F, (v - K[3]) / F, np.ones_like(u)], -1)
    R = R_level_body @ R_BODY_CAM
    d = d_cam @ R.T
    o = R_level_body @ T_BODY_CAM
    with np.errstate(divide="ignore", invalid="ignore"):
        tw = (wall_x - o[0]) / d[..., 0]
        tf = (-agl - o[2]) / d[..., 2]
    hit_w = tw > 0
    if wall_half_y is not None:
        hit_w &= np.abs(o[1] + d[..., 1] * tw) <= wall_half_y
    tw = np.where(hit_w, tw, np.inf)
    tf = np.where(tf > 0, tf, np.inf)
    t = np.minimum(tw, tf)
    return np.where(np.isfinite(t), t, np.nan).astype(np.float32)


def test_wall_and_floor_level():
    R = np.eye(3)
    fb = bev.frame_bev(render_depth(R, agl=1.0), K, R_BODY_CAM, T_BODY_CAM, R, agl_m=1.0)
    assert len(fb.wall_xy) > 20
    assert np.allclose(fb.wall_xy[:, 0], 2.025)                       # the cell [2.00, 2.05)
    hfov = math.atan2(W / 2, F)
    assert np.abs(fb.wall_xy[:, 1]).max() <= 2.0 * math.tan(hfov) + 0.05
    # free space: never at or beyond the wall; the in-band wedge in front of it is fully carved
    assert fb.free_xy[:, 0].max() < 2.02 - 0.05
    free = {tuple(np.round(c / 0.05 - 0.5).astype(int)) for c in fb.free_xy}
    for x in np.arange(0.6, 1.8, 0.05):
        for y in np.arange(-0.6 * x, 0.6 * x, 0.05):                # inside the 74 deg HFOV
            assert (int(math.floor(x / 0.05)), int(math.floor(y / 0.05))) in free, (x, y)


def test_floor_alone_is_not_wall():
    R = np.eye(3)
    fb = bev.frame_bev(render_depth(R, agl=1.0, wall_x=50.0), K, R_BODY_CAM, T_BODY_CAM, R, agl_m=1.0)
    assert len(fb.wall_xy) == 0
    assert len(fb.free_xy) > 100


def test_levelling_from_attitude_keeps_wall_vertical():
    q = q_px4(roll_deg=5, pitch_deg=-8)                               # nose down 8, right wing down 5
    R_true = bev.level_rotation_from_px4(q)
    depth = render_depth(R_true, agl=1.0)
    fb = bev.frame_bev(depth, K, R_BODY_CAM, T_BODY_CAM, R_true, agl_m=1.0)
    assert len(fb.wall_xy) > 20
    assert np.allclose(fb.wall_xy[:, 0], 2.025)
    # without levelling (what a failed-VO attitude amounts to) the wall smears over several cells
    fb_bad = bev.frame_bev(depth, K, R_BODY_CAM, T_BODY_CAM, np.eye(3), agl_m=1.0)
    assert np.ptp(fb_bad.wall_xy[:, 0]) >= 0.1


def test_band_falls_back_to_body_relative_on_the_pad():
    R = np.eye(3)
    p = bev.BevParams()
    depth = render_depth(R, agl=0.08, wall_x=1.52)
    fb = bev.frame_bev(depth, K, R_BODY_CAM, T_BODY_CAM, R, agl_m=0.08)   # rangefinder on the pad
    assert bev._band(p, 0.08) == p.rel_band_m
    assert len(fb.wall_xy) > 10 and np.allclose(fb.wall_xy[:, 0], 1.525)
    assert bev._band(p, 1.0) == pytest.approx((0.3 - 1.0, 1.6 - 1.0))


def test_frame_bev_runtime():
    q = q_px4(roll_deg=2, pitch_deg=-3)
    R = bev.level_rotation_from_px4(q)
    depth = render_depth(R, agl=1.0, wall_x=2.5, wall_half_y=1.0)
    depth[::7, ::5] = np.nan                                          # holes, like SGBM
    bev.frame_bev(depth, K, R_BODY_CAM, T_BODY_CAM, R, agl_m=1.0)
    t = time.perf_counter()
    for _ in range(10):
        fb = bev.frame_bev(depth, K, R_BODY_CAM, T_BODY_CAM, R, agl_m=1.0)
    ms = (time.perf_counter() - t) / 10 * 1e3
    print(f"\n  frame_bev 512x320 stride 4: {ms:.1f} ms, {fb.n_valid} valid rays, "
          f"{len(fb.wall_xy)} wall + {len(fb.free_xy)} free cells")
    assert ms < 30


# ----------------------------------------------------------------------------- ScanStack

def wall_frame(pos, yaw, xs=3.0, ys=np.arange(-1.0, 1.0, 0.05)):
    """FrameBev of an odom-frame wall x = xs, seen from odom pose (pos, yaw)."""
    pts = np.stack([np.full_like(ys, xs), ys], 1) + 0.025
    c, s = math.cos(yaw), math.sin(yaw)
    q = (pts - pos) @ np.array([[c, -s], [s, c]])                     # R(-yaw) (p - pos)
    return bev.FrameBev(wall_xy=q, free_xy=np.zeros((0, 2)), n_rays=0, n_valid=0)


def test_stack_expresses_frames_in_newest_frame():
    st = bev.ScanStack(horizon_s=1.0)
    poses = [((0.1 * k, 0.03 * k), math.radians(4 * k)) for k in range(5)]
    for k, (pos, yaw) in enumerate(poses):
        assert st.add(0.2 * k, wall_frame(np.array(pos), yaw), pos, yaw)
    W, _ = st.scan()
    truth = wall_frame(np.array(poses[-1][0]), poses[-1][1]).wall_xy
    d = np.min(np.linalg.norm(W[:, None] - truth[None], axis=2), axis=1)
    assert d.max() < 0.05                                             # within a cell of the truth


def test_stack_horizon_and_jump():
    st = bev.ScanStack(horizon_s=1.0, max_step_m=0.5)
    for k in range(10):
        st.add(0.2 * k, wall_frame(np.zeros(2), 0.0), (0.0, 0.0), 0.0)
    assert len(st.frames) == 6                                        # t = 1.0 .. 2.0 (inclusive)
    assert not st.add(2.2, wall_frame(np.zeros(2), 0.0), (19.8, 341.0), 0.0)   # VO jump: cleared
    assert len(st.frames) == 1
    st.add(1.0, wall_frame(np.zeros(2), 0.0), (19.8, 341.0), 0.0)     # time went backwards (bag loop)
    assert len(st.frames) == 1


# ----------------------------------------------------------------------------- LocalBevMap

def test_recentre_crash_case():
    """bev_live.log: odometry jumped to (19.8, 341, 286) and _recentre raised 'could not broadcast
    (161,239) into (161,0)'. A shift >= the map size must clear the map, not crash."""
    m = bev.LocalBevMap(bev.BevParams(map_size_m=12.0))
    m.insert(0.0, wall_frame(np.zeros(2), 0.0), (0.0, 0.0), 0.0)
    for t, pos in ((0.2, (19.8, 341.0)), (0.4, (24.9, 344.6)), (0.6, (-500.0, 3.0))):
        m.insert(t, wall_frame(np.array(pos), 0.0), pos, 0.0)
        rel = (np.array(pos) - m.origin) / m.p.res_m
        assert (rel > m.n / 4).all() and (rel < 3 * m.n / 4).all()


@pytest.mark.parametrize("cells", [239, 240, 241, 480, 481])         # around n = 240 and 2n
def test_recentre_boundary_shifts(cells):
    m = bev.LocalBevMap(bev.BevParams(map_size_m=12.0))
    m.L[:] = 1.0
    m.origin = np.array([-6.0, -6.0])
    m._recentre((cells * 0.05, 0.0))
    if cells >= m.n:
        assert not m.L.any()
    else:
        assert m.L[:m.n - cells].all() and not m.L[m.n - cells:].any()


def test_recentre_keeps_cells_at_their_odom_position():
    m = bev.LocalBevMap(bev.BevParams(map_size_m=12.0, decay_per_s=0.0))
    for k in range(4):                                                # the same wall, seen 4 times
        m.insert(0.1 * k, wall_frame(np.zeros(2), 0.0), (0.0, 0.0), 0.0)
    before = np.argwhere(m.state() == bev.WALL) * m.p.res_m + m.origin
    m.insert(1.0, bev.FrameBev(np.zeros((0, 2)), np.zeros((0, 2)), 0, 0), (4.0, 1.0), 0.0)  # scroll
    after = np.argwhere(m.state() == bev.WALL) * m.p.res_m + m.origin
    assert len(before) > 10 and np.allclose(np.sort(before, 0), np.sort(after, 0))


# ----------------------------------------------------------------------------- ScaleFit

def test_scalefit_recovers_inverse_depth_scale():
    rng = np.random.default_rng(0)
    z = rng.uniform(0.4, 3.4, 5000)
    s, t = 0.8, 0.05
    rel = (1 / z - t) / s
    z_st = z * (1 + rng.normal(0, 0.02, z.size))
    z_st[:800] = rng.uniform(0.3, 3.5, 800)                           # 16% stereo mismatches
    f = ScaleFit()
    assert f.update(rel, z_st)
    assert f.s == pytest.approx(s, rel=0.03) and f.t == pytest.approx(t, abs=0.01)
    zm = f.metric(rel[800:])
    assert np.nanmedian(np.abs(zm / z[800:] - 1)) < 0.02
