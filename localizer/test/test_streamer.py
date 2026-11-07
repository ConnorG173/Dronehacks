"""streamer.py: path.bin -> course frame, T math, pad init, carrot, fence."""
import math
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(HERE, "..")]
from localizer import streamer as S  # noqa: E402

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


def test_carrot_pauses_when_far_and_finishes(cpath):
    c = S.Carrot(cpath, scale=0.5, max_err_m=0.6)
    (xy, z, _), err = c.step(0.1, (*PAD, 1.0))
    assert c.tau == pytest.approx(0.05) and err < 0.01
    tau = c.tau
    c.step(0.1, (*PAD, 0.0))                                 # 1 m below the carrot: pause
    assert c.paused and c.tau == tau
    for _ in range(10000):                                   # a vehicle that tracks perfectly
        (xy, z, _), _ = c.step(0.05, (*c.path.at(c.tau)[0], c.path.at(c.tau)[1]))
        if c.done:
            break
    assert c.done and np.allclose(xy, cpath.xy[-1])


def test_time_scale_caps_speed(cpath):
    s = S.time_scale_for(cpath, 1.0)
    assert cpath.speed.max() == pytest.approx(2.42, abs=0.05)
    assert s * cpath.speed.max() == pytest.approx(1.0)
    assert S.time_scale_for(cpath, 10.0) == 1.0


def test_fence():
    assert S.inside_fence((2.7, -2.7)) and not S.inside_fence((2.8, 0))
    assert np.allclose(S.clamp_to_fence((5, -5)), (2.749, -2.749))
