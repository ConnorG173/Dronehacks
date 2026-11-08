"""Pure geometry for a display-only depth overlay in PX4 local ENU metres.

Depth is optical-axis Z, not Euclidean range. Body is base_link FLU; the
supplied PX4 quaternion rotates FRD into NED. No free-space inference or
flight-control decisions are made here.
"""
import math

import numpy as np


NED_TO_ENU = np.array([[0., 1., 0.], [1., 0., 0.], [0., 0., -1.]])
FRD_TO_FLU = np.diag([1., -1., -1.])


def nearest_sample(samples, stamp_ns, max_delta_ns=150_000_000):
    """Nearest value from (host timestamp ns, value) pairs; never use latest blindly."""
    if not samples:
        return None
    if not math.isfinite(stamp_ns) or not math.isfinite(max_delta_ns) or max_delta_ns < 0:
        raise ValueError('invalid timestamp or match tolerance')
    eligible = [s for s in samples if math.isfinite(s[0])]
    if not eligible:
        return None
    # Prefer the older sample for equal distances, irrespective of input order.
    sample = min(eligible, key=lambda s: (abs(s[0]-stamp_ns), s[0]))
    return sample[1] if abs(sample[0]-stamp_ns) <= max_delta_ns else None


def _intrinsics(K):
    k = np.asarray(K, dtype=float)
    if k.shape != (4,) or not np.isfinite(k).all() or (k[:2] <= 0).any():
        raise ValueError('expected finite positive fx/fy and principal point')
    return k


def scaled_intrinsics(K, src_size, dst_size):
    """Resize (fx, fy, cx, cy); sizes are (width, height), cv2 pixel centres."""
    fx, fy, cx, cy = _intrinsics(K)
    src, dst = np.asarray(src_size, float), np.asarray(dst_size, float)
    if (src.shape != (2,) or dst.shape != (2,) or not np.isfinite(src).all()
            or not np.isfinite(dst).all() or (src <= 0).any() or (dst <= 0).any()):
        raise ValueError('invalid image sizes')
    sx, sy = dst/src
    return (float(fx*sx), float(fy*sy), float((cx+.5)*sx-.5), float((cy+.5)*sy-.5))


scale_intrinsics = scaled_intrinsics


def _rotation_from_px4(q_wxyz):
    q = np.asarray(q_wxyz, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all():
        raise ValueError('invalid PX4 quaternion')
    norm = float(np.linalg.norm(q))
    if not .9 <= norm <= 1.1:
        raise ValueError('invalid PX4 quaternion norm')
    w, x, y, z = q/norm
    r_ned_frd = np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])
    return NED_TO_ENU @ r_ned_frd @ FRD_TO_FLU


def project_depth(depth_m, K, R_body_cam, t_body_cam, position_ned, q_wxyz,
                  cell_m=.08, max_points=1600, band_lo_m=-1.0, band_hi_m=1.2, slab_m=.20):
    """Return finite wall-like XY cells and camera origin in PX4 local ENU.

Camera extrinsics map rectified optical coordinates into base_link FLU.
Keep optical depth .3–4 m, sampling every fourth pixel after a six-pixel
border. Returns must lie in a tall band about the body (default -1.0..+1.2 m,
gravity-level). Each XY cell is reported once per occupied height slab
(slab_m) with that slab's world z range (result['z_ranges']), so a window's
sill and header are separate entries and the consumer keeps only cells at the
height the route flies there (dash/depth_height.py). Each world XY cell needs >=2 returns spanning >=.12 m vertically.
This filter suppresses flat floors; it does not detect all obstacles.
"""
    d = np.asarray(depth_m, dtype=float)
    if d.ndim != 2:
        raise ValueError('depth must be a two-dimensional image')
    fx, fy, cx, cy = _intrinsics(K)
    r_body_cam = np.asarray(R_body_cam, dtype=float)
    t_body_cam = np.asarray(t_body_cam, dtype=float)
    position = np.asarray(position_ned, dtype=float)
    if (r_body_cam.shape != (3, 3) or not np.isfinite(r_body_cam).all()
            or not np.allclose(r_body_cam.T @ r_body_cam, np.eye(3), atol=1e-3)
            or not np.isclose(np.linalg.det(r_body_cam), 1., atol=1e-3)):
        raise ValueError('camera rotation must be a finite proper rotation')
    if (t_body_cam.shape != (3,) or position.shape != (3,)
            or not np.isfinite(t_body_cam).all() or not np.isfinite(position).all()):
        raise ValueError('invalid camera translation or vehicle position')
    if not math.isfinite(cell_m) or cell_m <= 0:
        raise ValueError('cell size must be finite and positive')
    if isinstance(max_points, bool) or not isinstance(max_points, (int, np.integer)) or max_points < 1:
        raise ValueError('max_points must be a positive integer')
    r_enu_body = _rotation_from_px4(q_wxyz)
    position_enu = NED_TO_ENU @ position
    camera_enu = position_enu + r_enu_body @ t_body_cam
    result = {'points': [], 'camera_xy': camera_enu[:2].tolist(), 'view_polygon': [],
              # camera optical frame (x right, y down, z forward) in PX4-local ENU, for EGO-Planner
              'camera_enu': camera_enu.tolist(), 'R_enu_cam': (r_enu_body @ r_body_cam).tolist()}

    h, w = d.shape
    rows, cols = np.arange(6, h-6, 4), np.arange(6, w-6, 4)
    if not len(rows) or not len(cols):
        return result
    z = d[np.ix_(rows, cols)]
    valid = np.isfinite(z) & (z > .3) & (z < 4.)
    if not valid.any():
        return result
    u, v = np.meshgrid((cols-cx)/fx, (rows-cy)/fy)
    rays = np.stack([u[valid], v[valid], np.ones(int(valid.sum()))], axis=1)
    points_cam = rays*z[valid, None]
    points_body = points_cam @ r_body_cam.T + t_body_cam
    relative_enu = points_body @ r_enu_body.T
    keep = (relative_enu[:, 2] >= band_lo_m) & (relative_enu[:, 2] <= band_hi_m)
    points_enu = relative_enu[keep] + position_enu
    if not len(points_enu):
        return result

    ij = np.floor(points_enu[:, :2]/cell_m).astype(np.int64)
    cells, groups, counts = np.unique(ij, axis=0, return_inverse=True, return_counts=True)
    lo = np.full(len(cells), np.inf)
    hi = np.full(len(cells), -np.inf)
    np.minimum.at(lo, groups, points_enu[:, 2])
    np.maximum.at(hi, groups, points_enu[:, 2])
    column_ok = (counts >= 2) & (hi-lo >= .12)      # flat floor columns have no vertical spread
    keep_pt = column_ok[np.asarray(groups).reshape(-1)]
    if not keep_pt.any():
        result['z_ranges'] = []
        return result
    z = points_enu[keep_pt, 2]
    voxels = np.concatenate([ij[keep_pt], np.floor(z/slab_m).astype(np.int64)[:, None]], axis=1)
    vox, vgroups = np.unique(voxels, axis=0, return_inverse=True)
    vgroups = vgroups.reshape(-1)
    vlo = np.full(len(vox), np.inf)
    vhi = np.full(len(vox), -np.inf)
    np.minimum.at(vlo, vgroups, z)
    np.maximum.at(vhi, vgroups, z)
    if len(vox) > max_points:
        # Keep whole columns (every height slab of a kept XY cell), nearest the camera first: thinning
        # slabs at random dropped the route-height slab of many columns, and with it the obstacle.
        cols, inv, counts = np.unique(vox[:, :2], axis=0, return_inverse=True, return_counts=True)
        centres = (cols+.5)*cell_m
        order = np.argsort(np.hypot(centres[:, 0]-position_enu[0], centres[:, 1]-position_enu[1]))
        keep_cols = order[np.cumsum(counts[order]) <= max_points]
        inv = np.asarray(inv).reshape(-1)
        if len(keep_cols):
            keep = np.isin(inv, keep_cols)
        else:                                   # one column alone exceeds the cap: its slabs, up to the cap
            keep = np.zeros(len(vox), dtype=bool)
            keep[np.flatnonzero(inv == order[0])[:max_points]] = True
        vox, vlo, vhi = vox[keep], vlo[keep], vhi[keep]
    result['points'] = np.round((vox[:, :2]+.5)*cell_m,3).tolist()
    result['z_ranges'] = np.round(np.stack([vlo, vhi], axis=1),3).tolist()
    return result



def scale_support(st, rel, z_stereo, tol=.10):
    """Fraction of this frame's stereo pixels (0.3-3.5 m) that the scale 1/z = s*rel + t fits within tol."""
    ok = np.isfinite(z_stereo) & (z_stereo > .3) & (z_stereo < 3.5) & np.isfinite(rel)
    if not ok.any():
        return 0.
    iz = 1./z_stereo[ok]
    return float(np.mean(np.abs(iz-(st[0]*rel[ok]+st[1])) < tol*iz))


FAR_CAP_M = 8.0   # beyond-range (s*rel + t <= 0 means "farther than anything") counts as this far


def scale_contrast(st, q):
    """log(far/near depth) the scale implies at the frame's 10 % / 90 % relative depth. A far end that is
    beyond range (non-positive inverse depth) is capped at FAR_CAP_M, not rejected: with a negative
    offset the farthest pixels are simply out of range, which metric() already blanks."""
    inv_far, inv_near = st[0]*q[0]+st[1], st[0]*q[-1]+st[1]
    if not (math.isfinite(inv_far) and math.isfinite(inv_near)) or inv_near <= 0:
        return -math.inf                          # even the nearest pixels out of range: not a usable scale
    far = FAR_CAP_M if inv_far <= 1./FAR_CAP_M else 1./inv_far
    return float(math.log(far/min(1./inv_near, far)))


class ScaleChooser:
    """Pick the ML depth scale (1/z = s*rel + t) for each frame from that frame's own evidence.

    Every candidate, including the scale used last frame, is scored by how much of THIS frame's stereo
    it fits (Depth Anything's output magnitude changes between frames, so an old (s, t) is only kept
    while it still fits). Of the candidates within `near` of the best support: keep last frame's scale
    for stability unless one has clearly more contrast (operator: the high-contrast solution is right).
    A pick close to last frame's scale is blended for smoothness."""

    def __init__(self, near=.90, better_margin=.3, alpha=.3, tol=.15, min_inv_spread=.15):
        self.near, self.better_margin, self.alpha, self.tol = near, better_margin, alpha, tol  # knobs
        # A first scale needs stereo over a real range of depths (10-90 % of 1/z at least this apart,
        # 1/m): with one depth band every line through it fits, and a flat one misplaces everything.
        self.min_inv_spread = min_inv_spread
        self.st = None

    @staticmethod
    def _dev(a, b, q):
        with np.errstate(divide='ignore', invalid='ignore'):
            za, zb = 1./(a[0]*q+a[1]), 1./(b[0]*q+b[1])
        ok = np.isfinite(za) & np.isfinite(zb) & (za > 0) & (zb > 0)
        return float(np.max(np.abs(za[ok]-zb[ok])/zb[ok])) if ok.any() else math.inf

    def choose(self, candidates, rel, z_stereo, q):
        """candidates: [(s, t)]. Returns ((s, t), state, support) with state 'new'|'kept'|'blend'|'switch'."""
        q = np.asarray(q, dtype=float)
        pool = [(float(s), float(t)) for s, t in candidates]
        if self.st is not None:
            pool.append(self.st)
        scored = [(st, scale_support(st, rel, z_stereo), scale_contrast(st, q)) for st in pool]
        scored = [x for x in scored if math.isfinite(x[2])]
        if not scored:
            return None, 'none', 0.
        best = max(x[1] for x in scored)
        good = [x for x in scored if x[1] >= self.near*best]
        top = max(good, key=lambda x: x[2])
        prev = next((x for x in good if self.st is not None and x[0] == self.st), None)
        if self.st is None:
            ok = np.isfinite(z_stereo) & (z_stereo > .3) & (z_stereo < 3.5)
            if ok.sum() < 50:
                return None, 'flat', 0.
            q10, q90 = np.percentile(1./z_stereo[ok], [10, 90])
            if q90-q10 < self.min_inv_spread:
                return None, 'flat', 0.
            self.st = top[0]
            return self.st, 'new', top[1]
        if prev is not None and top[2] <= prev[2]+self.better_margin:
            # last frame's scale still fits this frame: stay on it, nudged toward this frame's best
            mine = max((x for x in good if x[0] != self.st), key=lambda x: x[1], default=None)
            if mine is not None and self._dev(mine[0], self.st, q) <= self.tol:
                S, T = self.st
                self.st = (S+self.alpha*(mine[0][0]-S), T+self.alpha*(mine[0][1]-T))
                return self.st, 'blend', scale_support(self.st, rel, z_stereo)
            return self.st, 'kept', prev[1]
        self.st = top[0]
        return self.st, 'switch', top[1]
