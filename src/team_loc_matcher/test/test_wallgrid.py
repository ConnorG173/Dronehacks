"""wallgrid on the real course map (planner/maps/course_v3.bin) and on a synthetic box.
Expected numbers come from the CAD as read in planner/config/course_v3.yaml: maze gaps 0.93 m wide,
M1/M3 gaps at y 2.121..3.048 (open onto the net), M2 at y 0.508..1.435. The net is never a wall."""
import math
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(HERE, "..")]
from team_loc_matcher import wallgrid  # noqa: E402

REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
MAP = os.path.join(REPO, "planner", "maps", "course_v3.bin")
CONFIG = os.path.join(REPO, "planner", "config", "course_v3.yaml")
GAPS = {"M1": (-1.9304, 2.121, 3.048), "M2": (-0.8128, 0.508, 1.435), "M3": (0.3048, 2.121, 3.048)}


@pytest.fixture(scope="module")
def g():
    if not os.path.exists(MAP):
        pytest.skip("course map not present")
    return wallgrid.build_from_files(MAP, CONFIG)


def cell(g, x, y):
    r = float(g["res_m"])
    return int(math.floor((x - g["origin_x"]) / r)), int(math.floor((y - g["origin_y"]) / r))


def test_counts_and_start(g):
    assert g["wall"].sum() == 846          # vertical faces in the band, net ring and its margin removed
    assert g["boundary"].sum() == 484      # the net ring
    i, j = cell(g, *g["start_xy"])
    assert g["cage"][i, j] and not g["wall"][i, j] and g["dist_m"][i, j] > 0.3


def test_net_is_never_wall_and_outside_is_not_cage(g):
    assert not (g["wall"] & g["boundary"]).any()
    i, j = cell(g, 3.15, 0.0)              # between the net and MapBound
    assert not g["cage"][i, j] and g["lik"][i, j] == 0


@pytest.mark.parametrize("name", GAPS)
def test_maze_gaps_are_open(g, name):
    x, ylo, yhi = GAPS[name]
    i, _ = cell(g, x, 0)
    for y in np.arange(ylo + 0.1, min(yhi, 2.9) - 0.1, 0.05):
        _, j = cell(g, x, y)
        assert not g["wall"][i, j], (name, y)
    # The divider itself is a wall just outside the gap (M2's continues north of its gap).
    if name == "M2":
        _, j = cell(g, x, yhi + 0.3)
        assert g["wall"][i - 1:i + 2, j].any()


def test_synthetic_box_and_leak_detection():
    # A 4 x 4 m box of "net" walls with one interior wall; points every 4 cm, 0..1.7 m high.
    pts = []
    for z in np.arange(0, 1.71, 0.04):
        for s in np.arange(-2, 2.001, 0.04):
            pts += [(s, -2, z), (s, 2, z), (-2, s, z), (2, s, z)]
        for s in np.arange(-1, 1.001, 0.04):
            pts.append((0.5, s, z))
    pts = np.array(pts)
    out = wallgrid.build_wallgrid(pts, (-1.0, 0.0))
    assert out["boundary"].any() and out["wall"].any()
    # the interior wall is a wall, the box is boundary only
    i = int((0.5 - out["origin_x"]) / out["res_m"])
    assert out["wall"][i - 1:i + 2].any()
    # open box (one side missing): inside meets outside
    leak = pts[~np.isclose(pts[:, 0], 2)]
    with pytest.raises(ValueError, match="close"):
        wallgrid.build_wallgrid(leak, (-1.0, 0.0))


def test_lik_peaks_on_walls(g):
    w = g["wall"]
    assert np.allclose(g["lik"][w & g["cage"]], 1.0, atol=1e-6)
    assert (g["lik"][~g["cage"]] == 0).all()
