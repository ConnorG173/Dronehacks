"""Synthetic BEV scans rendered from a 2-D wall grid, for matcher tests. PURE numpy.

A scan is what frame_bev + stacking would give: wall cells and free-space cells (5 cm cell
centres) in the level body frame (x forward, y left). Rays are cast in a horizontal fan; a ray
returns at the first wall cell it enters (the hit is the point on the ray closest to that cell's
centre, so the rendered world agrees with the CAD grid to within a cell), with range noise.

Options model the real camera: `room` (rays that leave the cage through the see-through net
sometimes return from the room/people beyond), `edge_only` (low-texture walls: only depth
discontinuities return, plus a small fraction of interior pixels), and stacking ~1 s of frames.
"""
import math

import numpy as np


def _dedupe(q, res):
    if len(q) == 0:
        return np.zeros((0, 2))
    return (np.unique(np.floor(q / res).astype(np.int64), axis=0) + 0.5) * res


def render(world, origin, res, pose, fov_deg=72.0, n_rays=120, rmin=0.3, rmax=5.0, noise=0.01,
           rng=None, room=0.0, cage_half=3.049, edge_only=False, edge_jump=0.15, keep_frac=0.03,
           free_stop=0.10, noise_k=0.0, net_return=0.0):
    """One frame from pose (x, y, yaw) -> (wall_pts, free_pts) in the body frame (deduped)."""
    rng = rng or np.random.default_rng(0)
    x, y, yaw = pose
    a = np.radians(np.linspace(-fov_deg / 2, fov_deg / 2, n_rays))
    d = np.stack([np.cos(yaw + a), np.sin(yaw + a)], 1)
    s = np.arange(rmin, rmax, res / 4)
    P = np.array([x, y]) + s[None, :, None] * d[:, None, :]              # (rays, steps, 2)
    ij = np.floor((P - origin) / res).astype(np.int64)
    n = world.shape[0]
    inside = (ij >= 0).all(2) & (ij < n).all(2)
    ijc = np.clip(ij, 0, n - 1)
    hit = inside & world[ijc[..., 0], ijc[..., 1]]
    has = hit.any(1)
    first = np.argmax(hit, 1)
    r = np.full(n_rays, np.nan)
    for k in np.nonzero(has)[0]:
        c = origin + (ijc[k, first[k]] + 0.5) * res                       # hit cell centre
        r[k] = float((c - (x, y)) @ d[k])                                 # its projection on the ray
    # the net itself may return (opaque in the sim): a ray leaving the cage returns ON it
    if net_return > 0:
        out = np.abs(P).max(2) > cage_half
        for k in np.nonzero(~has & out.any(1))[0]:
            if rng.random() < net_return:
                r[k] = s[np.argmax(out[k])]
                has[k] = True
    # see-through net: rays that leave the cage without a hit may return from beyond it
    if room > 0:
        out = np.abs(P).max(2) > cage_half
        for k in np.nonzero(~has & out.any(1))[0]:
            if rng.random() < room:
                r[k] = s[np.argmax(out[k])] + rng.uniform(0.2, 2.0)
    # stereo-like range noise: noise + noise_k * r^2 (depth error grows with the square of range)
    sig = noise + noise_k * np.nan_to_num(r) ** 2
    r = r + rng.normal(0, 1, n_rays) * sig * np.isfinite(r)
    ok = np.isfinite(r) & (r > rmin) & (r < rmax)
    if edge_only:
        rr = np.where(ok, r, np.inf)
        jump = np.zeros(n_rays, bool)
        with np.errstate(invalid="ignore"):
            dl = np.abs(np.diff(rr))                                  # inf - inf: a jump
        jump[1:] |= ~(dl < edge_jump)
        jump[:-1] |= ~(dl < edge_jump)
        ok &= jump | (rng.random(n_rays) < keep_frac)
    q = np.stack([r * np.cos(a), r * np.sin(a)], 1)[ok]
    free = [np.stack([t * np.cos(a[k]), t * np.sin(a[k])], 1)
            for k in np.nonzero(ok)[0] for t in [np.arange(rmin, r[k] - free_stop, res)] if len(t)]
    free = np.concatenate(free) if free else np.zeros((0, 2))
    return _dedupe(q, res), _dedupe(free, res)


def stacked(world, origin, res, poses, **kw):
    """Frames rendered at each pose (5 Hz), stacked by the production bev.ScanStack into the LAST
    pose's level frame (as course_matcher stacks them with odometry's relative motion)."""
    from team_loc_matcher.bev import FrameBev, ScanStack
    st = ScanStack(horizon_s=1.0, res_m=res)
    for k, (x, y, t) in enumerate(poses):
        w, f = render(world, origin, res, (x, y, t), **{**kw, "rng": np.random.default_rng(k)})
        st.add(0.2 * k, FrameBev(w, f, 0, 0), (x, y), t)
    return st.scan()


def track(pose, n=5, step=0.08, dyaw_deg=2.0):
    """n poses ending at `pose`, moving forward `step` m and turning `dyaw_deg` per frame."""
    x, y, t = pose
    out = []
    for k in range(n - 1, -1, -1):
        tk = t - math.radians(dyaw_deg) * k
        out.append((x - step * k * math.cos(t), y - step * k * math.sin(t), tk))
    return out
