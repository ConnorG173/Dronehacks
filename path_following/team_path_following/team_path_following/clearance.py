"""Distance to the nearest occupied map point, from a DHMAP point map.

A voxel grid at ``resolution`` over MapBound is marked occupied where map points
fall, then scipy's Euclidean distance transform gives each free voxel's distance
to the nearest occupied voxel centre. Used for monitoring and slow-down only.
"""
import numpy as np

try:
    from scipy.ndimage import distance_transform_edt
except ImportError:  # pragma: no cover
    distance_transform_edt = None


class ClearanceMap:

    def __init__(self, points, bound, resolution=0.05):
        if distance_transform_edt is None:
            raise RuntimeError('scipy is required for ClearanceMap')
        self.res = float(resolution)
        self.lo = np.array([bound[0], bound[2], bound[4]], float)
        hi = np.array([bound[1], bound[3], bound[5]], float)
        self.shape = np.maximum(np.ceil((hi - self.lo) / self.res).astype(int), 1)
        occ = np.zeros(self.shape, bool)
        idx = np.floor((np.asarray(points, float) - self.lo) / self.res).astype(int)
        ok = np.all((idx >= 0) & (idx < self.shape), axis=1)
        idx = idx[ok]
        occ[idx[:, 0], idx[:, 1], idx[:, 2]] = True
        self.dist = distance_transform_edt(~occ) * self.res
        self.occupied_any = bool(occ.any())

    def distance(self, p):
        """Metres from p to the nearest occupied voxel centre; 0 outside the bound."""
        p = np.asarray(p, float)
        idx = np.floor((p - self.lo) / self.res).astype(int)
        if p.ndim == 1:
            if np.any(idx < 0) or np.any(idx >= self.shape):
                return 0.0
            return float(self.dist[tuple(idx)])
        out = np.zeros(len(p))
        ok = np.all((idx >= 0) & (idx < self.shape), axis=1)
        out[ok] = self.dist[idx[ok, 0], idx[ok, 1], idx[ok, 2]]
        return out

    def speed_scale(self, p, slow_below, stop_below, min_scale=0.4):
        """1 above slow_below, min_scale at stop_below and closer."""
        d = self.distance(p)
        if d >= slow_below:
            return 1.0
        if d <= stop_below:
            return min_scale
        return min_scale + (1.0 - min_scale) * (d - stop_below) / (slow_below - stop_below)
