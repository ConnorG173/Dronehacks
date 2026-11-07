"""Correlative scan-to-CAD matcher: where in the course is this (stacked) BEV scan? PURE numpy.

Input: wall points and free-space samples in the drone's LEVEL frame (x forward, y left, metres,
the body origin at (0, 0)), e.g. ~1 s of frame_bev output stacked with odometry's relative motion,
and a prior course pose (x, y, yaw; yaw CCW from course +x). Output: the course pose with a
covariance taken from the score surface, plus the diagnostics the gates need.

Score of a pose: sum over wall points of log(lik + eps), where lik is the CAD likelihood field
(cad_to_wallgrid.py) and is 0 outside the cage mask, so returns from the net and the room beyond
it score the same at every pose and cannot bias the match. Each point is bounded below by log(eps),
so a moved wall or a stereo outlier costs at most a constant. Free-space samples that land in a CAD
wall cell subtract w_free each: seeing through where the CAD says wall is evidence against a pose.

Search: brute force on an xy_step lattice around the prior (integer-cell shifts of the lookup grid,
so each pose is a pure gather), yaw in yaw_step increments. The lookup grid is the CAD grid upsampled `sub` times (bilinear): with nearest 5 cm cells,
a 1 deg yaw step moves points at 1-3 m range by less than a cell and the surface is full of ties.

Covariance: the surface is turned into weights w = exp((S - S_max) * n_eff / N), i.e. at most
n_eff points' worth of information (stacked points are strongly correlated), and the covariance
is the weighted second moment over the window plus a floor. A corridor gives a ridge, so one
eigenvalue is large. A second mode counts as such only if a valley separates it from the peak.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class MatchParams:
    xy_window_m: float = 0.4          # search +- around the prior (keep < ~0.5: bays repeat at 1.12 m)
    xy_step_m: float = 0.05           # lattice step, a multiple of the lookup grid's res
    yaw_window_deg: float = 8.0
    yaw_step_deg: float = 1.0
    eps: float = 0.05                 # likelihood floor per point (outlier/moved-wall robustness)
    w_free: float = 1.0               # penalty per free sample inside a CAD wall cell (log units)
    n_eff_max: int = 50               # information cap for the POSE and mode tests: a scan is worth
                                      # at most this many points (stacked points are correlated)
    n_eff_cov: int = 25               # ...and for the COVARIANCE. One cap of 50 for both (prototype)
                                      # gave 2/400 overconfident fixes (d2 up to 48) on random
                                      # course poses; 25 for both biased the pose toward the prior.
    max_wall_pts: int = 1500          # subsample above this (deterministic stride)
    max_free_pts: int = 600
    cov_floor_xy_m: float = 0.03      # CAD vs as-built, stereo bias: never claim better than this
    cov_floor_yaw_deg: float = 0.5
    # gates
    min_wall_pts: int = 15            # in-cage wall points at the pose (edge-only scans have 10-30;
                                      # the synthetic sweep accepted none wrongly at 15)
    min_score: float = 0.5            # mean lik of those points at the matched pose
    max_second_rel: float = 0.1       # relative weight of a separated second mode
    mode_sep_xy_m: float = 0.15       # a second mode must be at least this far from the peak in xy
    max_mahal: float = 3.5            # against the filter's prediction (3 dof)


@dataclass
class CourseMap:
    """The lookup grid: cad_to_wallgrid's grid upsampled `sub` times. Cell [i, j] covers
    origin + ([i, i+1), [j, j+1)) * res, i along course +x, j along +y."""
    lik: np.ndarray                   # float32 [i, j], 0 outside the cage
    wall: np.ndarray                  # float32 1.0 on CAD wall cells
    cage: np.ndarray                  # bool, inside the cage mask (observed points elsewhere ignored)
    res: float
    origin: np.ndarray                # (2,) course x, y of the lower corner of cell [0, 0]
    start_xy: np.ndarray | None = None
    start_yaw: float | None = None

    @classmethod
    def from_grid(cls, g, sub=2):
        """From cad_to_wallgrid's arrays (an npz or the dict build_wallgrid returns)."""
        cage = np.asarray(g["cage"], bool)
        start = g["start_xy"] if "start_xy" in g else None
        yaw = float(g["start_heading_rad"]) if "start_heading_rad" in g else None
        lik = _upsample_bilinear(np.where(cage, g["lik"], 0).astype(np.float32), sub)
        rep = lambda a: np.repeat(np.repeat(a, sub, 0), sub, 1)
        return cls(lik=lik, wall=rep(np.asarray(g["wall"], bool) & cage).astype(np.float32),
                   cage=rep(cage), res=float(g["res_m"]) / sub,
                   origin=np.array([float(g["origin_x"]), float(g["origin_y"])]),
                   start_xy=None if start is None else np.asarray(start, float), start_yaw=yaw)

    @classmethod
    def load(cls, path, sub=2):
        with np.load(path) as z:
            return cls.from_grid({k: z[k] for k in z.files}, sub)


def _upsample_bilinear(a, sub):
    """Cell-centred bilinear upsampling by an integer factor (edges clamp)."""
    for axis in (0, 1):
        n = a.shape[axis]
        u = (np.arange(n * sub) + 0.5) / sub - 0.5           # fine centres in coarse index units
        i0 = np.clip(np.floor(u).astype(np.int64), 0, n - 2)
        f = np.clip(u - i0, 0.0, 1.0).astype(a.dtype)
        sh = [1, 1]
        sh[axis] = -1
        f = f.reshape(sh)
        a = np.take(a, i0, axis) * (1 - f) + np.take(a, i0 + 1, axis) * f
    return a


@dataclass
class MatchResult:
    pose: np.ndarray                  # (3,) course x, y, yaw (rad)
    cov: np.ndarray                   # (3, 3) in (m, m, rad)
    score: float                      # mean lik of in-cage wall points at the pose (0..1)
    n_wall: int                       # wall points inside the cage at the pose
    n_free_hit: int                   # free samples inside CAD wall cells at the pose
    second_rel: float                 # relative weight of the best separated second mode (0 = none)
    at_edge: bool                     # the peak is clearly outside the search window
    surface: np.ndarray               # (yaws, dx, dy) total log score, for debugging


def _subsample(p, n):
    p = np.asarray(p, float).reshape(-1, 2)
    if len(p) > n:
        p = p[np.linspace(0, len(p) - 1, n).astype(np.int64)]
    return p


def _rot(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]])


def _shifted_sums(field, cells, ks):
    """sum_k field[cells_k + (dx, dy)] for all dx, dy in ks -> (len(ks), len(ks)).
    Out-of-grid indices clip to the grid edge, which is outside the cage (lik 0, no wall)."""
    n0, n1 = field.shape
    ix = np.clip(cells[:, 0, None] + ks, 0, n0 - 1)
    iy = np.clip(cells[:, 1, None] + ks, 0, n1 - 1)
    return field[ix[:, :, None], iy[:, None, :]].sum(0)


def _cells(cmap, pts_body, x, y, yaw):
    p = pts_body @ _rot(yaw).T + (x, y)
    return np.floor((p - cmap.origin) / cmap.res).astype(np.int64)


def _local_maxima(S):
    P = np.pad(S, 1, constant_values=-np.inf)
    m = np.ones(S.shape, bool)
    a, b, c = S.shape
    for da in (-1, 0, 1):
        for db in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if da or db or dc:
                    m &= S >= P[1 + da:1 + da + a, 1 + db:1 + db + b, 1 + dc:1 + dc + c]
    return np.argwhere(m)


def match(cmap: CourseMap, wall_pts, free_pts, prior, params: MatchParams | None = None) -> MatchResult:
    """Search the course pose of a level-frame scan around prior = (x, y, yaw)."""
    p = params or MatchParams()
    W = _subsample(wall_pts, p.max_wall_pts)
    F = _subsample(free_pts if free_pts is not None else np.zeros((0, 2)), p.max_free_pts)
    x0, y0, th0 = (float(v) for v in prior)
    res = cmap.res
    sh = int(round(p.xy_step_m / res))
    if sh < 1 or abs(sh * res - p.xy_step_m) > 1e-9:
        raise ValueError(f"xy_step_m {p.xy_step_m} is not a multiple of the lookup res {res}")
    K = int(round(p.xy_window_m / p.xy_step_m))
    ks = np.arange(-K, K + 1) * sh                      # shifts in lookup cells
    J = int(round(p.yaw_window_deg / p.yaw_step_deg))
    dyaw = np.radians(np.arange(-J, J + 1) * p.yaw_step_deg)
    ell = np.log(cmap.lik + np.float32(p.eps))

    S = np.empty((len(dyaw), len(ks), len(ks)))
    for a, d in enumerate(dyaw):
        S[a] = _shifted_sums(ell, _cells(cmap, W, x0, y0, th0 + d), ks)
        if len(F):
            S[a] -= p.w_free * _shifted_sums(cmap.wall, _cells(cmap, F, x0, y0, th0 + d), ks)

    n = max(len(W), 1)
    scale = min(n, p.n_eff_max) / n                       # surface units -> weight exponent
    best = np.unravel_index(int(np.argmax(S)), S.shape)
    Smax = S[best]

    step = np.array([math.radians(p.yaw_step_deg), sh * res, sh * res])

    # pose and covariance: mean and second moment of the surface's weights. On a ridge (corridor)
    # the mean stays near the prior along the ridge instead of jumping to a noisy argmax.
    w = np.exp((S - Smax) * scale)
    w /= w.sum()
    A, X, Y = np.meshgrid(dyaw, ks * res, ks * res, indexing="ij")
    D = np.stack([X.ravel(), Y.ravel(), A.ravel()], 1)        # (x, y, yaw) offsets
    mu = w.ravel() @ D
    x, y, yaw = x0 + mu[0], y0 + mu[1], th0 + mu[2]
    # covariance from broader weights (fewer effective points), about the same mean
    wc = np.exp((S - Smax) * (min(n, p.n_eff_cov) / n))
    wc /= wc.sum()
    E = D - mu
    cov = (E * wc.ravel()[:, None]).T @ E
    cov += np.diag([step[1] ** 2 / 12 + p.cov_floor_xy_m ** 2] * 2
                   + [step[0] ** 2 / 12 + math.radians(p.cov_floor_yaw_deg) ** 2])

    # peak clearly outside the window: the best interior pose is much weaker than the edge peak
    edge = any(best[ax] in (0, S.shape[ax] - 1) for ax in range(3))
    at_edge = bool(edge and (S[1:-1, 1:-1, 1:-1].max() - Smax) * scale < -1.0)

    # separated second mode: a local maximum with a valley between it and the peak
    second_rel = 0.0
    cand = [tuple(c) for c in _local_maxima(S) if tuple(c) != best]
    cand.sort(key=lambda c: -S[c])
    b = np.array(best)
    sep = p.mode_sep_xy_m / step[1]
    for c in cand[:20]:
        rel = math.exp((S[c] - Smax) * scale)
        if rel < 1e-3:
            break
        dc = np.abs(np.array(c) - b)
        if math.hypot(dc[1], dc[2]) < sep:
            continue                  # the same mode (yaw/translation ridges are one mode)
        m = int(np.abs(np.array(c) - b).max())
        path = np.rint(b + np.outer(np.linspace(0, 1, m + 1), np.array(c) - b)).astype(np.int64)
        valley = S[path[:, 0], path[:, 1], path[:, 2]].min()
        if (S[c] - valley) * scale > 1.0:
            second_rel = rel
            break

    # diagnostics at the matched pose
    cw = _cells(cmap, W, x, y, yaw)
    ok = (cw >= 0).all(1) & (cw < cmap.lik.shape[0]).all(1)
    lk = cmap.lik[cw[ok, 0], cw[ok, 1]]
    in_cage = cmap.cage[cw[ok, 0], cw[ok, 1]]
    n_wall = int(in_cage.sum())
    score = float(lk[in_cage].mean()) if n_wall else 0.0
    n_free_hit = 0
    if len(F):
        cf = _cells(cmap, F, x, y, yaw)
        okf = (cf >= 0).all(1) & (cf < cmap.wall.shape[0]).all(1)
        n_free_hit = int(cmap.wall[cf[okf, 0], cf[okf, 1]].sum())

    return MatchResult(pose=np.array([x, y, math.atan2(math.sin(yaw), math.cos(yaw))]), cov=cov,
                       score=score, n_wall=n_wall, n_free_hit=n_free_hit, second_rel=second_rel,
                       at_edge=at_edge, surface=S)


def gate(r: MatchResult, params: MatchParams | None = None, pred=None, pred_cov=None):
    """(ok, reason). pred/pred_cov: the filter's predicted course pose and its covariance (optional).
    vo_state and other stream-level checks belong to the node."""
    p = params or MatchParams()
    if r.n_wall < p.min_wall_pts:
        return False, f"few wall points ({r.n_wall} < {p.min_wall_pts})"
    if r.score < p.min_score:
        return False, f"low score ({r.score:.2f} < {p.min_score})"
    if r.at_edge:
        return False, "peak outside the search window"
    if r.second_rel > p.max_second_rel:
        return False, f"ambiguous (second mode {r.second_rel:.2f})"
    if pred is not None and pred_cov is not None:
        e = r.pose - np.asarray(pred, float)
        e[2] = math.atan2(math.sin(e[2]), math.cos(e[2]))
        d2 = float(e @ np.linalg.solve(r.cov + np.asarray(pred_cov, float), e))
        if d2 > p.max_mahal ** 2:
            return False, f"inconsistent with the filter (Mahalanobis {math.sqrt(d2):.1f})"
    return True, "ok"
