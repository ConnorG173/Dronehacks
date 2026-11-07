"""Depth frame -> bird's-eye-view (BEV) wall evidence. PURE numpy. Moved from the prototype
(~/team_ws/claude/bev.py) with: the level frame changed to FLU, levelling from PX4's attitude
(not cuVSLAM's, which is garbage exactly when VO fails), cheaper free-space carving, a frame
stacker for the matcher, and the LocalBevMap recentre crash fixed.

Frames:
  body    base_link, FLU (x forward, y left, z up)
  level   origin at the body origin, z up along gravity, x = body forward projected horizontally
          (roll and pitch removed, yaw kept as "forward"); y left. match.py's scan frame.
  odom    cuVSLAM odom: x, y horizontal, yaw CCW about z (ScanStack, LocalBevMap)
  camera  rectified optical frame (x right, y down, z forward); R/t_body_cam from TF

1. `frame_bev(...)`: ONE depth frame -> wall cells (returns inside the wall-height band whose cell
   spans >= min_span of height: vertical faces only) and free-space cells (the in-band part of each
   returning ray, stopping free_stop short of the hit). Rays without a return add nothing.
2. `ScanStack`: ~1 s of frames, expressed in the newest frame's level frame using odometry's
   RELATIVE motion (short-term VO is good, long-term drifts), cleared on an odom jump.
3. `LocalBevMap`: scrolling log-odds grid in odom (survey / debugging, not the matcher's input).

Height band: above the FLOOR (default 0.3-1.6 m, the course-map recipe) when the rangefinder
height of the body origin is known and >= min_agl_m; otherwise relative to the body (on the pad
a floor-referenced band would start above the camera's near field).
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

import numpy as np

UNKNOWN, FREE, WALL = -1, 0, 1


@dataclass
class BevParams:
    res_m: float = 0.05
    rmin_m: float = 0.3               # closer returns are propeller/frame/stereo junk
    rmax_m: float = 4.0               # stereo depth error grows ~ z^2 (prototype ran 4.0 live)
    band_lo_agl_m: float = 0.3        # wall band above the floor
    band_hi_agl_m: float = 1.6
    min_agl_m: float = 0.35           # below this rangefinder height, use the body-relative band
    rel_band_m: tuple = (-0.25, 0.8)  # (lo, hi) metres along level z (up) relative to the body
    min_span_m: float = 0.15          # a frame's returns in a cell must span this much height
    min_hits: int = 2                 # ...and be at least this many
    free_stop_m: float = 0.10         # stop carving free space this short of the hit
    stride: int = 4                   # use every n-th pixel
    border_px: int = 6                # image edge ignored
    # LocalBevMap log-odds
    l_hit: float = 0.85
    l_free: float = 0.35
    l_min: float = -2.0
    l_max: float = 3.5
    l_wall: float = 1.2
    l_free_thresh: float = -0.6
    decay_per_s: float = 0.08
    map_size_m: float = 12.0


@dataclass
class FrameBev:
    """One frame's evidence in the level frame (x forward, y left), metres, cell centres."""
    wall_xy: np.ndarray
    free_xy: np.ndarray
    n_rays: int
    n_valid: int


# ----------------------------------------------------------------------------- attitude

def quat_to_mat(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


_NED_ENU = np.array([[0.0, 1, 0], [1, 0, 0], [0, 0, -1]])    # its own inverse
_FRD_FLU = np.diag([1.0, -1, -1])                            # its own inverse


def px4_to_enu_flu(q_wxyz):
    """PX4 VehicleAttitude.q (w, x, y, z; FRD body -> NED) -> rotation FLU body -> ENU."""
    w, x, y, z = (float(v) for v in q_wxyz)
    return _NED_ENU @ quat_to_mat(x, y, z, w) @ _FRD_FLU


def level_rotation(R_world_body):
    """Roll/pitch-only rotation body (FLU) -> level frame, from any z-up world rotation."""
    yaw = math.atan2(R_world_body[1, 0], R_world_body[0, 0])
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, s, 0], [-s, c, 0], [0, 0, 1.0]]) @ R_world_body


def level_rotation_from_px4(q_wxyz):
    """R_level_body from /fmu/out/vehicle_attitude (IMU, gravity-referenced)."""
    return level_rotation(px4_to_enu_flu(q_wxyz))


def tilt_deg(R_world_body):
    """Angle between body z and world z (for the pad check: odom tilt vs PX4 tilt)."""
    return math.degrees(math.acos(max(-1.0, min(1.0, float(R_world_body[2, 2])))))


def yaw_of(R_world_body):
    return math.atan2(R_world_body[1, 0], R_world_body[0, 0])


# ----------------------------------------------------------------------------- one frame

def _band(p, agl_m):
    """(lo, hi) on level z (up, relative to the body origin) for the wall band."""
    if agl_m is None or not math.isfinite(agl_m) or agl_m < p.min_agl_m:
        return p.rel_band_m
    # a point at level z is (agl + z) above the floor
    return p.band_lo_agl_m - agl_m, p.band_hi_agl_m - agl_m


def _cells_unique(xy, res):
    if len(xy) == 0:
        return np.zeros((0, 2))
    return (np.unique(np.floor(xy / res).astype(np.int64), axis=0) + 0.5) * res


def frame_bev(depth_m, K, R_body_cam, t_body_cam, R_level_body, agl_m=None, params=None) -> FrameBev:
    """One depth frame -> wall cells and free cells in the level frame.
    depth_m: float metres along the optical axis, NaN = no return. K = (fx, fy, cx, cy) of that image.
    R/t_body_cam: base_link (FLU) <- (rectified) optical frame. R_level_body: level_rotation_from_px4."""
    p = params or BevParams()
    d = np.asarray(depth_m, np.float32)
    h, w = d.shape
    b, st = int(p.border_px), int(p.stride)
    rows = np.arange(b, h - b, st)
    cols = np.arange(b, w - b, st)
    nr, nc = len(rows), len(cols)
    fx, fy, cx, cy = K
    z = d[np.ix_(rows, cols)]                                              # (nr, nc)
    uu, vv = np.meshgrid((cols - cx) / fx, (rows - cy) / fy)
    R = np.asarray(R_level_body, float) @ np.asarray(R_body_cam, float)
    dirs = np.stack([uu, vv, np.ones_like(uu)], -1) @ R.T                  # (nr, nc, 3), optical z = 1
    o = np.asarray(R_level_body, float) @ np.asarray(t_body_cam, float)
    valid = np.isfinite(z) & (z > p.rmin_m) & (z < p.rmax_m)
    lo, hi = _band(p, agl_m)
    res = p.res_m

    # --- walls: returns in the band, binned; keep cells with vertical extent (sort + reduceat)
    pts = o + dirs[valid] * z[valid, None]
    pts = pts[(pts[:, 2] >= lo) & (pts[:, 2] <= hi)]
    wall_xy = np.zeros((0, 2))
    if len(pts):
        ij = np.floor(pts[:, :2] / res).astype(np.int64)
        c0 = ij.min(0)
        n1 = int(ij[:, 1].max() - c0[1] + 1)
        lin = (ij[:, 0] - c0[0]) * n1 + (ij[:, 1] - c0[1])
        order = np.argsort(lin, kind="stable")
        ls, zs = lin[order], pts[order, 2]
        starts = np.r_[0, np.flatnonzero(np.diff(ls)) + 1]
        cnt = np.diff(np.r_[starts, len(ls)])
        span = np.maximum.reduceat(zs, starts) - np.minimum.reduceat(zs, starts)
        keep = ls[starts[(cnt >= p.min_hits) & (span >= p.min_span_m)]]
        wall_xy = (np.stack([keep // n1 + c0[0], keep % n1 + c0[1]], 1) + 0.5) * res

    # --- free space. For each returning ray: the interval of horizontal distance s along its
    # azimuth where it is inside the band, ending free_stop short of the hit. All rays of one pixel
    # column share (nearly: exactly for a level camera) one azimuth, so carve once per column, over
    # [min start, max end] of its rays (the union of intervals that, in practice, all start at the
    # camera or at the band's edge). ~nr times fewer samples than carving every ray.
    free_xy = np.zeros((0, 2))
    if valid.any():
        hlen = np.hypot(dirs[..., 0], dirs[..., 1])                         # horizontal m per unit t
        dz = dirs[..., 2]
        with np.errstate(divide="ignore", invalid="ignore"):
            t_lo = (lo - o[2]) / dz
            t_hi = (hi - o[2]) / dz
        flat = np.abs(dz) < 1e-9
        inside0 = lo <= o[2] <= hi
        t0 = np.where(flat, 0.0 if inside0 else np.inf, np.maximum(0.0, np.minimum(t_lo, t_hi)))
        t1 = np.where(flat, np.inf if inside0 else -np.inf, np.maximum(t_lo, t_hi))
        t1 = np.minimum(t1, z - p.free_stop_m / np.maximum(hlen, 1e-6))
        s0, s1 = t0 * hlen, t1 * hlen
        ok = valid & (s1 > s0)
        s0 = np.where(ok, s0, np.inf).min(0)                                 # per column
        k_end = np.where(ok, s1, -np.inf).argmax(0)
        s1 = np.where(ok, s1, -np.inf).max(0)
        has = np.isfinite(s1) & (s1 > s0)
        if has.any():
            cidx = np.nonzero(has)[0]
            u = dirs[k_end[cidx], cidx, :2] / hlen[k_end[cidx], cidx, None]   # column azimuths
            n = np.ceil((s1[cidx] - s0[cidx]) / res).astype(np.int64)
            k = np.arange(int(n.max()))
            s = s0[cidx, None] + k[None, :] * res
            m = k[None, :] < n[:, None]
            xy = o[:2] + u[:, None, :] * s[..., None]
            free_xy = _cells_unique(xy[m], res)

    return FrameBev(wall_xy=wall_xy, free_xy=free_xy, n_rays=int(z.size), n_valid=int(valid.sum()))


# ----------------------------------------------------------------------------- stacking

def _rot2(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]])


class ScanStack:
    """The last `horizon_s` of frames, each with the odom pose (x, y, yaw) at its capture time,
    expressed in the NEWEST frame's level frame: the matcher's input. An odom step larger than
    `max_step_m` between consecutive frames (a VO jump) clears the stack first."""

    def __init__(self, horizon_s=1.0, res_m=0.05, max_step_m=0.5):
        self.horizon_s, self.res, self.max_step = horizon_s, res_m, max_step_m
        self.frames = deque()

    def reset(self):
        self.frames.clear()

    def add(self, t, fb: FrameBev, odom_xy, odom_yaw):
        """Returns False if the stack was cleared because of a jump."""
        xy = np.asarray(odom_xy, float)[:2]
        jumped = bool(self.frames) and float(np.linalg.norm(xy - self.frames[-1][1])) > self.max_step
        if jumped or (self.frames and t < self.frames[-1][0]):
            self.frames.clear()
        self.frames.append((t, xy, float(odom_yaw), fb))
        while self.frames and self.frames[0][0] < t - self.horizon_s:
            self.frames.popleft()
        return not jumped

    def scan(self):
        """(wall_xy, free_xy) in the newest frame's level frame, one point per res cell."""
        if not self.frames:
            return np.zeros((0, 2)), np.zeros((0, 2))
        _, pn, yn, _ = self.frames[-1]
        Rn_T = _rot2(yn).T
        W, F = [], []
        for _, pk, yk, fb in self.frames:
            Rk = _rot2(yk)
            for src, dst in ((fb.wall_xy, W), (fb.free_xy, F)):
                if len(src):
                    dst.append((src @ Rk.T + (pk - pn)) @ Rn_T.T)
        cat = lambda L: np.concatenate(L) if L else np.zeros((0, 2))
        return _cells_unique(cat(W), self.res), _cells_unique(cat(F), self.res)


# ----------------------------------------------------------------------------- local map

def _level_to_odom(xy, pos_xy, yaw):
    return np.asarray(pos_xy, float)[:2] + xy @ _rot2(yaw).T


class LocalBevMap:
    """Scrolling log-odds wall map in odom. L[i, j]: i along odom x, j along odom y, from
    `origin` (the lower corner)."""

    def __init__(self, params=None):
        self.p = params or BevParams()
        self.n = int(round(self.p.map_size_m / self.p.res_m))
        self.L = np.zeros((self.n, self.n), np.float32)
        self.origin = None
        self.t_last = None
        self.frames = 0

    def reset(self):
        self.L[:] = 0
        self.origin = None
        self.t_last = None

    def _recentre(self, pos_xy):
        """Keep the drone within the middle half of the map, shifting by whole cells. A shift of
        at least the map size (an odometry jump) clears the map instead: nothing old overlaps,
        and the old slicing wrapped around through negative indices and crashed."""
        res, n = self.p.res_m, self.n
        pos = np.asarray(pos_xy, float)[:2]
        centred = np.floor(pos / res) * res - n * res / 2
        if self.origin is None:
            self.origin = centred
            return
        rel = (pos - self.origin) / res
        shift = np.zeros(2, np.int64)
        for a in range(2):
            if rel[a] < n / 4 or rel[a] > 3 * n / 4:
                shift[a] = int(round(rel[a] - n / 2))
        if not shift.any():
            return
        if (np.abs(shift) >= n).any():
            self.L[:] = 0
            self.origin = centred
            return
        L = np.zeros_like(self.L)
        si, sj = (int(v) for v in shift)
        L[max(-si, 0):n - max(si, 0), max(-sj, 0):n - max(sj, 0)] = \
            self.L[max(si, 0):n - max(-si, 0), max(sj, 0):n - max(-sj, 0)]
        self.L = L
        self.origin = self.origin + shift * res

    def _cells(self, xy):
        ij = np.floor((xy - self.origin) / self.p.res_m).astype(np.int64)
        ok = (ij >= 0).all(1) & (ij < self.n).all(1)
        return np.unique(ij[ok], axis=0)

    def insert(self, t, fb: FrameBev, pos_xy, yaw):
        """Add one frame. pos_xy/yaw: the odom pose at the frame's capture time t."""
        p = self.p
        self._recentre(pos_xy)
        if self.t_last is not None and t > self.t_last:
            self.L *= np.float32((1.0 - p.decay_per_s) ** (t - self.t_last))
        self.t_last = t if self.t_last is None else max(t, self.t_last)
        if len(fb.free_xy):
            ij = self._cells(_level_to_odom(fb.free_xy, pos_xy, yaw))
            self.L[ij[:, 0], ij[:, 1]] -= p.l_free
        if len(fb.wall_xy):
            ij = self._cells(_level_to_odom(fb.wall_xy, pos_xy, yaw))
            self.L[ij[:, 0], ij[:, 1]] += p.l_hit + p.l_free     # a hit cell is never also free
        np.clip(self.L, p.l_min, p.l_max, out=self.L)
        self.frames += 1

    def state(self):
        """UNKNOWN / FREE / WALL, indexed [x, y] from origin."""
        g = np.full(self.L.shape, UNKNOWN, np.int8)
        g[self.L <= self.p.l_free_thresh] = FREE
        g[self.L >= self.p.l_wall] = WALL
        return g
