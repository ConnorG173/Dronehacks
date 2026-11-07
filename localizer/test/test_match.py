"""match.py on synthetic scans rendered from the course wall grid (localizer/data/*.npz).

Property checked throughout: an ACCEPTED fix is never wrong beyond its own covariance
(d^2 = e' C^-1 e below the 3-dof 99% bound). Being refused is fine; a confident wrong answer is not.
"""
import math
import os
import sys
import time

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(HERE, ".."), HERE]
from localizer.match import CourseMap, MatchParams, gate, match  # noqa: E402
import synth  # noqa: E402

NPZ = os.path.join(HERE, "..", "data", "course_v3_wallgrid.npz")
CHI2_3_99 = 11.34


@pytest.fixture(scope="module")
def course():
    z = np.load(NPZ)
    return CourseMap.load(NPZ), z["wall"], np.array([float(z["origin_x"]), float(z["origin_y"])]), \
        float(z["res_m"]), z["dist_m"]


def err(r, truth):
    e = r.pose - np.asarray(truth, float)
    e[2] = math.atan2(math.sin(e[2]), math.cos(e[2]))
    return e


def d2(r, truth):
    e = err(r, truth)
    return float(e @ np.linalg.solve(r.cov, e))


def pose(x, y, deg):
    return (x, y, math.radians(deg))


POSES = {   # all with walls in view, away from the net
    "south_open": pose(-2.4, -1.5, 30),
    "bay_A_north": pose(-1.37, 1.0, 90),
    "angled_wall": pose(0.5, -1.8, 120),
    "bay_A_east": pose(-1.37, 1.8, 0),
    "cross_wall": pose(-2.0, -0.3, 0),
    "pad": pose(-2.5146, 0.9144, 0),
}


@pytest.mark.parametrize("name", POSES)
def test_recovers_known_pose(course, name):
    cmap, world, o, res, _ = course
    truth = POSES[name]
    W, F = synth.stacked(world, o, res, synth.track(truth), room=0.5)      # with room returns
    prior = np.array(truth) + (0.25, -0.2, math.radians(5))
    r = match(cmap, W, F, prior)
    ok, why = gate(r)
    e = err(r, truth)
    assert ok, why
    assert math.hypot(e[0], e[1]) < 0.03, e
    assert abs(math.degrees(e[2])) < 2.0, e
    assert d2(r, truth) < CHI2_3_99


def corridor_map():
    """Two parallel walls at y = +-0.525 m, x in [-3, 3]; nothing else."""
    n, res, o = 142, 0.05, -3.55
    c = o + (np.arange(n) + 0.5) * res
    X, Y = np.meshgrid(c, c, indexing="ij")
    wall = (np.isclose(np.abs(Y), 0.525)) & (np.abs(X) <= 3.0)
    d = np.minimum(np.abs(np.abs(Y) - 0.525), 0.5)
    d = np.where(np.abs(X) <= 3.0, d, 0.5).astype(np.float32)
    g = dict(wall=wall, cage=np.maximum(np.abs(X), np.abs(Y)) <= 2.9, lik=np.exp(-0.5 * (d / 0.08) ** 2),
             res_m=res, origin_x=o, origin_y=o)
    return CourseMap.from_grid(g), wall, np.array([o, o]), res


def test_corridor_gives_large_along_axis_covariance():
    cmap, world, o, res = corridor_map()
    truth = (0.0, 0.0, 0.0)
    W, F = synth.stacked(world, o, res, synth.track(truth), rmax=2.5)      # no wall ends in view
    r = match(cmap, W, F, (0.2, -0.1, math.radians(3)))
    ok, why = gate(r)
    assert ok, why                                       # degenerate is not ambiguous: keep the fix
    vals, vecs = np.linalg.eigh(r.cov[:2, :2])
    assert math.sqrt(vals[1]) > 0.15, vals               # along the corridor: ~uniform over the window
    assert math.sqrt(vals[0]) < 0.05, vals               # across: well constrained
    assert abs(vecs[0, 1]) > 0.95, vecs                  # the large axis is course x (the corridor)
    e = err(r, truth)
    assert abs(e[1]) < 0.03 and abs(math.degrees(e[2])) < 1.5, e
    assert abs(e[0] - 0.2) < 0.1, e                      # along the ridge it stays near the prior
    assert d2(r, truth) < CHI2_3_99


ALIAS_POSES = {"bay_A_east": pose(-1.37, 1.8, 0), "bay_A_north": pose(-1.37, 1.0, 90)}
BAY_M = 1.12


@pytest.mark.parametrize("name", ALIAS_POSES)
def test_one_bay_alias(course, name):
    """The scan looks almost the same one bay (1.12 m) east. The matcher alone CANNOT tell: given a
    prior one bay off, it locks onto the alias with a good score. Protection is (1) the +-0.4 m
    window, so a prior that is right to within it never reaches the alias, and (2) the Mahalanobis
    gate against the filter, which refuses an aliased fix."""
    cmap, world, o, res, _ = course
    truth = np.array(ALIAS_POSES[name])
    W, F = synth.stacked(world, o, res, synth.track(tuple(truth)), room=0.5)
    pred_cov = np.diag([0.1 ** 2, 0.1 ** 2, math.radians(3) ** 2])

    # worst-case good prior: 0.35 m toward the alias -> the true bay
    r = match(cmap, W, F, truth + (0.35, 0, 0))
    assert gate(r, pred=truth, pred_cov=pred_cov)[0]
    assert np.linalg.norm(err(r, truth)[:2]) < 0.05

    # prior one bay off: the matcher finds the alias, the filter gate must refuse it
    r = match(cmap, W, F, truth + (BAY_M, 0, 0))
    assert np.linalg.norm(err(r, truth)[:2]) > 0.8                  # it really is fooled
    ok, why = gate(r, pred=truth, pred_cov=pred_cov)
    assert not ok and "Mahalanobis" in why

    # a window wide enough to contain both bays prefers the true one
    r = match(cmap, W, F, truth, MatchParams(xy_window_m=1.2))
    assert np.linalg.norm(err(r, truth)[:2]) < 0.05


def moved(world, o, res, dx_cells):
    """The as-built world with the M2 wall (x ~ -0.81, y > 1.3) moved dx_cells cells along +x."""
    c = o[0] + (np.arange(world.shape[0]) + 0.5) * res
    X, Y = np.meshgrid(c, c, indexing="ij")
    m2 = world & (np.abs(X + 0.81) < 0.15) & (Y > 1.3)
    out = world & ~m2
    out[np.roll(m2, dx_cells, 0)] = True
    return out


@pytest.mark.parametrize("shift_cells", [2, 4])            # 10 cm, 20 cm
def test_one_moved_wall(course, shift_cells):
    """The matcher uses the CAD; the scan comes from a world where M2 moved. Seen together with
    other walls, the fix stays within 5 cm. (A view of ONLY the moved wall follows it: nothing can
    detect that from one scan; the as-built survey is the fix for it.)"""
    cmap, world, o, res, _ = course
    as_built = moved(world, o, res, shift_cells)
    truth = pose(-1.37, 2.6, -90)                         # sees M2's face, M1, its end and the cross wall
    W, F = synth.stacked(as_built, o, res, synth.track(truth), room=0.5)
    r = match(cmap, W, F, np.array(truth) + (0.2, -0.15, math.radians(4)))
    ok, why = gate(r)
    assert ok, why
    assert np.linalg.norm(err(r, truth)[:2]) < 0.05
    assert d2(r, truth) < CHI2_3_99


def free_poses(dist, o, res, n, seed):
    rng = np.random.default_rng(seed)
    out = []
    while len(out) < n:
        x, y = rng.uniform(-2.7, 2.7, 2)
        i, j = np.floor((np.array([x, y]) - o) / res).astype(int)
        if dist[i, j] >= 0.35:
            out.append((x, y, rng.uniform(-math.pi, math.pi), rng))
    return out


@pytest.mark.parametrize("edge_only", [False, True], ids=["dense", "edge_only"])
def test_random_poses_no_confident_wrong_fix(course, edge_only):
    """40 random poses, random headings (many face the net), random prior offsets. Sparse
    edge-only scans model the low-texture walls: only depth discontinuities return."""
    cmap, world, o, res, dist = course
    accepted, worst = 0, 0.0
    for x, y, t, rng in free_poses(dist, o, res, 40, seed=7 if edge_only else 3):
        truth = (x, y, t)
        W, F = synth.stacked(world, o, res, synth.track(truth), room=0.5, edge_only=edge_only)
        prior = np.array(truth) + np.r_[rng.uniform(-0.3, 0.3, 2), math.radians(rng.uniform(-6, 6))]
        r = match(cmap, W, F, prior)
        if gate(r)[0]:
            accepted += 1
            worst = max(worst, d2(r, truth))
    print(f"\n  {'edge-only' if edge_only else 'dense'}: accepted {accepted}/40, worst d2 {worst:.1f}")
    assert worst < CHI2_3_99
    assert accepted >= (8 if edge_only else 18)


def test_gates_refuse_empty_and_garbage(course):
    cmap = course[0]
    r = match(cmap, np.zeros((0, 2)), None, (0.0, 0.0, 0.0))
    assert not gate(r)[0]
    rng = np.random.default_rng(0)
    junk = rng.uniform(-3, 3, (200, 2))                    # random points: no structure
    r = match(cmap, junk, None, (-2.0, -1.5, 0.0))
    ok, why = gate(r)
    assert not ok, why


def test_runtime(course):
    cmap, world, o, res, _ = course
    truth = POSES["bay_A_north"]                           # one of the larger scans (~200 + ~600 pts)
    W, F = synth.stacked(world, o, res, synth.track(truth), room=0.5)
    match(cmap, W, F, truth)
    t = time.perf_counter()
    for _ in range(5):
        match(cmap, W, F, truth)
    ms = (time.perf_counter() - t) / 5 * 1e3
    print(f"\n  match: {len(W)} wall + {min(len(F), 600)} free pts, 17 yaw x 17 x 17 xy: {ms:.1f} ms")
    assert ms < 150
