"""Keep only depth cells at the height the route flies near them. No ROS.

The depth worker reports each XY cell once per occupied height slab with that
slab's world z range. A cell matters only if its range overlaps the route's
height within NEAR_M of it, widened by the drone's half height plus a margin:
a beam above the route, a box below it, or a window's sill and header drop out
where the route passes over, under or through them, but still count wherever
the route reaches their height. Frames without z ranges pass through unchanged.

The filtered frame also carries every input cell as ``cells3d`` ([x, y, z_lo,
z_hi]) so pass-through centring and the guard's memory can re-filter when the
route's height changes.
"""
import numpy as np

DRONE_HALF_HEIGHT_M = .12   # calibration knob: half the drone's height, props included
HEIGHT_MARGIN_M = .13
NEAR_M = .50                # route points this close (XY) define the height there; > planning clearance
                            # (.40), so a cell nearer than that to the route always uses the route's height there
MAX_ROUTE_POINTS = 600


def keep_mask(points_xy, z_ranges, route_xyz):
    """True for each cell whose [z_lo, z_hi] overlaps the route's height band near it.
    Non-finite ranges (height unknown) are always kept."""
    pts = np.asarray(points_xy, dtype=float).reshape(-1, 2)
    zr = np.asarray(z_ranges, dtype=float).reshape(-1, 2)
    if not len(pts):
        return np.zeros(0, dtype=bool)
    route = np.asarray(route_xyz, dtype=float).reshape(-1, 3)
    if len(route) > MAX_ROUTE_POINTS:
        route = route[np.linspace(0, len(route)-1, MAX_ROUTE_POINTS).astype(int)]
    d = np.linalg.norm(pts[:, None, :]-route[None, :, :2], axis=2)          # cells x route
    near = d <= NEAR_M
    nearest = d.argmin(axis=1)
    rz = np.broadcast_to(route[:, 2], d.shape)
    zmin = np.where(near.any(axis=1), np.where(near, rz, np.inf).min(axis=1), route[nearest, 2])
    zmax = np.where(near.any(axis=1), np.where(near, rz, -np.inf).max(axis=1), route[nearest, 2])
    band = DRONE_HALF_HEIGHT_M + HEIGHT_MARGIN_M
    unknown = ~np.isfinite(zr).all(axis=1)
    return unknown | ((zr[:, 0] <= zmax+band) & (zr[:, 1] >= zmin-band))


def filter_for_route(depth, route_xyz):
    if (not isinstance(depth, dict) or depth.get('status') != 'live' or 'z_ranges' not in depth
            or route_xyz is None or len(route_xyz) < 1):
        return depth
    pts = np.asarray(depth.get('points') or [], dtype=float).reshape(-1, 2)
    zr = np.asarray(depth['z_ranges'], dtype=float).reshape(-1, 2)
    if not len(pts) or len(zr) != len(pts):
        return depth
    keep = keep_mask(pts, zr, route_xyz)
    kept = {}
    for p in pts[keep].tolist():
        kept.setdefault((round(p[0], 3), round(p[1], 3)), p)
    out = dict(depth, points=list(kept.values()))
    out.pop('z_ranges', None)
    out['cells3d'] = np.hstack([pts, zr]).tolist()
    out['height_filtered'] = int(len(pts)-int(keep.sum()))
    return out


def inside_polygon(points_xy, polygon):
    """Even-odd ray test: True for each point inside the polygon ([[x, y], ...])."""
    pts = np.asarray(points_xy, dtype=float).reshape(-1, 2)
    poly = np.asarray(polygon, dtype=float).reshape(-1, 2)
    inside = np.zeros(len(pts), dtype=bool)
    if len(poly) < 3:
        return inside
    x, y = pts[:, 0], pts[:, 1]
    for (x1, y1), (x2, y2) in zip(poly, np.roll(poly, -1, axis=0)):
        crosses = (y1 > y) != (y2 > y)
        with np.errstate(divide='ignore', invalid='ignore'):
            at = x1 + (y-y1)*(x2-x1)/(y2-y1)
        inside ^= crosses & (x < at)
    return inside


def limit_to_zones(depth, circles, polygons=()):
    """Avoid-only-in-zones: drop every cell outside the zone circles ([x, y, r]) and polygons
    ([[x, y], ...]), world ENU, so the preview, guard, nudges and pass-through see no obstacle
    there and fly the route as planned. A frame with no zones keeps nothing (nothing is avoided)."""
    if not isinstance(depth, dict) or depth.get('status') != 'live':
        return depth
    pts = np.asarray(depth.get('points') or [], dtype=float).reshape(-1, 2)
    c = np.asarray(circles if circles else [], dtype=float).reshape(-1, 3)
    keep = (np.linalg.norm(pts[:, None, :]-c[None, :, :2], axis=2) <= c[None, :, 2]).any(axis=1) \
        if len(pts) and len(c) else np.zeros(len(pts), dtype=bool)
    for poly in polygons or ():
        keep |= inside_polygon(pts, poly)
    out = dict(depth, points=pts[keep].tolist(), zone_filtered=int(len(pts)-int(keep.sum())))
    if 'z_ranges' in depth and len(depth['z_ranges']) == len(pts):
        out['z_ranges'] = np.asarray(depth['z_ranges'], dtype=float).reshape(-1, 2)[keep].tolist()
    return out


def moves_in_hole_zone(new_points, old_points, holes, radius_m, tol_m=.03):
    """True if a replanned route changes the route inside a hole-control zone. The planner does not see
    cells there (they belong to hole control), so it must never move the route there: such a plan is
    discarded and the passage stays what hole control will fly."""
    if not holes or not new_points or not old_points:
        return False
    h = np.asarray([list(x)[:2] for x in holes], dtype=float).reshape(-1, 2)
    new = np.asarray(new_points, dtype=float).reshape(-1, 3)
    inside = new[(np.linalg.norm(new[:, None, :2]-h[None, :, :], axis=2) <= radius_m).any(axis=1)]
    if not len(inside):
        return False
    old = np.asarray(old_points, dtype=float).reshape(-1, 3)
    a, b = old[:-1, :2], old[1:, :2]
    d = b-a
    for p in inside[:, :2]:
        f = np.clip(np.einsum('ij,ij->i', p-a, d)/np.maximum(np.einsum('ij,ij->i', d, d), 1e-12), 0., 1.)
        if float(np.min(np.linalg.norm(p-(a+f[:, None]*d), axis=1))) > tol_m:
            return True
    return False


def hand_to_hole_control(depth, holes, radius_m):
    """Near a hole under hole control, wall cells belong to hole control alone: drop them from the
    2D `points` the guard, nudges and replanning use (they would otherwise block the passage with
    generic clearances before hole control has measured and centred it; up close the camera cannot
    see the gate anyway). `cells3d`, which hole control measures from, is left complete."""
    if not holes or not isinstance(depth, dict) or depth.get('status') != 'live':
        return depth
    pts = np.asarray(depth.get('points') or [], dtype=float).reshape(-1, 2)
    if not len(pts):
        return depth
    h = np.asarray([list(x)[:2] for x in holes], dtype=float).reshape(-1, 2)
    keep = (np.linalg.norm(pts[:, None, :]-h[None, :, :], axis=2) > radius_m).all(axis=1)
    return dict(depth, points=pts[keep].tolist(), hole_control_cells=int(len(pts)-int(keep.sum())))
