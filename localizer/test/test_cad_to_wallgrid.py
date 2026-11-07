"""cad_to_wallgrid on the real course CAD (planner/maps/course_v3.bin). Laptop-only: skipped when
the planner map is not present (e.g. on the Jetson). Expected numbers come from the CAD as read in
planner/config/course_v3.yaml: maze gaps 0.93 m wide, M1/M3 at y 2.121..3.048 (open onto the net),
M2 at y 0.508..1.435 (walls on both sides). The perimeter net is never a wall."""
import importlib.util
import math
import os
import subprocess
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
TOOL = os.path.join(HERE, "..", "tools", "cad_to_wallgrid.py")
CONFIG = os.path.join(REPO, "planner", "config", "course_v3.yaml")
MAP = os.path.join(REPO, "planner", "maps", "course_v3.bin")

if not (os.path.exists(MAP) and os.path.exists(TOOL)):
    pytest.skip("laptop-only: planner map / tool not present", allow_module_level=True)

spec = importlib.util.spec_from_file_location("cad_to_wallgrid", TOOL)
cw = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cw)

GAP_M = 0.927          # 0.93 m (36.5 in) maze gaps
GAPS = {               # name: (wall x, gap y_lo, gap y_hi)
    "M1": (-1.9304, 2.121, 3.048),
    "M2": (-0.8128, 0.508, 1.435),
    "M3": (0.3048, 2.121, 3.048),
}
# 1407 vertical-face cells in the 0.3-1.6 m band (planning), minus the 484-cell net ring and the
# 77 wall-end / tube-face cells within net_margin (0.15 m) of the net
N_WALL = 846


@pytest.fixture(scope="module")
def g():
    return cw.build_wallgrid(cw.read_dhmap(MAP), cage_half_m=3.049)


def cell(g, x, y):
    r, o = float(g["res_m"]), float(g["origin_x"])
    return int(math.floor((x - o) / r)), int(math.floor((y - o) / r))


def free_run(g, x, y):
    """Cells [a, b] of the run of non-wall cells through (x, y) along y, stopping at walls or at
    the edge of the cage mask. Returns (a, b, bounded_lo_by_wall, bounded_hi_by_wall)."""
    i, j = cell(g, x, y)
    col, cage = g["wall"][i], g["cage"][i]
    assert not col[j] and cage[j]
    a = j
    while cage[a - 1] and not col[a - 1]:
        a -= 1
    b = j
    while cage[b + 1] and not col[b + 1]:
        b += 1
    return a, b, bool(col[a - 1]), bool(col[b + 1])


def test_cell_count(g):
    assert int(g["wall"].sum()) == N_WALL
    assert g["wall"].shape == (142, 142)
    assert float(g["origin_x"]) == pytest.approx(-3.55)


def test_m2_gap_width_between_two_walls(g):
    x, y_lo, y_hi = GAPS["M2"]
    r, o = float(g["res_m"]), float(g["origin_y"])
    a, b, wlo, whi = free_run(g, x, 0.5 * (y_lo + y_hi))
    assert wlo and whi, "M2 is bounded by walls on both sides"
    edge_lo, edge_hi = o + a * r, o + (b + 1) * r          # free run spans [edge_lo, edge_hi)
    assert edge_lo - r <= y_lo <= edge_lo + r / 2
    assert edge_hi - r / 2 <= y_hi <= edge_hi + r
    width = (b - a + 1) * r + r          # the free run plus the expected half cell on each side
    assert abs(width - GAP_M) <= r, width


@pytest.mark.parametrize("name", ["M1", "M3"])
def test_north_gaps_open_onto_net(g, name):
    x, y_lo, y_hi = GAPS[name]
    r, o = float(g["res_m"]), float(g["origin_y"])
    a, b, wlo, whi = free_run(g, x, 0.5 * (y_lo + y_hi))
    assert wlo and not whi, "wall end to the south, only the (ignored) net to the north"
    edge_lo = o + a * r
    assert edge_lo - r <= y_lo <= edge_lo + r / 2, (name, edge_lo)   # wall end at CAD y 2.121
    assert o + (b + 1) * r == pytest.approx(3.049 - 0.15, abs=r)     # free up to the cage mask


@pytest.mark.parametrize("name", GAPS)
def test_maze_wall_x_at_cad_coordinate(g, name):
    x, y_lo, y_hi = GAPS[name]
    r, o = float(g["res_m"]), float(g["origin_x"])
    y = y_lo - 0.3 if y_hi > 3.0 else y_hi + 0.3       # a row through the wall, outside the gap
    _, j = cell(g, x, y)
    xs = o + (np.nonzero(g["wall"][:, j])[0] + 0.5) * r
    near = xs[np.abs(xs - x) < 0.3]
    # CAD walls are sampled on their surfaces: two face cells ~0.1 m apart, hollow between
    assert 1 <= len(near) <= 4, near
    assert abs(near.mean() - x) <= r, (name, near)


def test_net_is_not_wall_and_cage_mask(g):
    h, m = 3.049, 0.15
    cheb = None
    r, o = float(g["res_m"]), float(g["origin_x"])
    c = o + (np.arange(g["wall"].shape[0]) + 0.5) * r
    cx, cy = np.meshgrid(c, c, indexing="ij")
    cheb = np.maximum(abs(cx), abs(cy))
    assert not g["wall"][cheb > h - m].any(), "no wall cell on or near the net"
    assert np.array_equal(g["cage"], cheb <= h - m)
    assert g["cage"].sum() == 116 * 116
    assert g["dist_m"][cell(g, 0.0, 3.025)] > 0.2, "nothing for a net return to match"


def test_free_points_and_likelihood_field(g):
    r = float(g["res_m"])
    for name, (x, y_lo, y_hi) in GAPS.items():
        d = g["dist_m"][cell(g, x, 0.5 * (y_lo + y_hi))]
        assert d == pytest.approx(GAP_M / 2, abs=1.5 * r), name   # gap centre ~0.46 m from walls
    assert g["dist_m"][cell(g, -2.5146, 0.9144)] > 0.3              # the pad is in open floor
    assert np.all(g["lik"][g["wall"]] == 1.0)
    assert g["dist_m"].max() == pytest.approx(0.5)
    assert np.all(g["lik"][g["dist_m"] >= 0.5] < 1e-8)


def test_cli_writes_npz_and_png(tmp_path):
    out = tmp_path / "grid.npz"
    subprocess.run([sys.executable, TOOL, CONFIG, "--out", str(out)], check=True, capture_output=True)
    z = np.load(out)
    assert int(z["wall"].sum()) == N_WALL
    assert "net" not in z.files
    assert np.allclose(z["start_xy"], (-2.5146, 0.9144))
    assert float(z["start_heading_rad"]) == 0.0
    assert z["checkpoints"].shape == (5, 6)
    assert (tmp_path / "grid.png").stat().st_size > 10_000
