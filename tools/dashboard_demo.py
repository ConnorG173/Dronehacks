#!/usr/bin/env python3
"""Run the real flight dashboard (flight/dash/dash.html) against a simulated flight, no drone needed.

    pixi run demo            # -> http://127.0.0.1:8099/?k=demo
    python tools/dashboard_demo.py --still    # top-down shows only the wall the detour avoids

The drone flies the dashboard planner's own detour (course_state.update_preview) around a wall
placed across the course. Depth is rendered live from the drone's pose, the way the real worker
sees the cage: a 72 x 46 deg camera ray-cast against the course voxel map (planner/maps) plus the
wall, coloured 0.3-4 m like the worker, and the top-down view gets the 8 cm surface cells the
camera hits right now. Every command is refused.
"""
import io, json, math, pathlib, struct, threading, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
from PIL import Image, ImageDraw

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
PAGE = (ROOT / 'flight' / 'dash' / 'dash.html').read_bytes()
STATE = json.loads((HERE / 'demo' / 'state.json').read_text())
ROUTE = STATE.pop('_demo_route')
T0 = time.monotonic()

# ---- the course as the camera sees it: 8 cm columns with their height span ----------------------
CELL, MAX_D, MIN_D = .08, 4.0, .30
HFOV, VFOV, W, H = math.radians(72), math.radians(46), 406, 252
PAD, PAD_HEADING = (-2.545, .895), math.radians(90.)   # course 9's launch pad in the map frame


def _course_columns():
    b = (ROOT / 'planner' / 'maps' / 'updated_course.bin').read_bytes()
    n = struct.unpack('<Q', b[16:24])[0]
    p = np.frombuffer(b, '<f4', count=n * 3, offset=24).reshape(-1, 3).astype(float)
    p = p[(p[:, 2] > .05) & (p[:, 2] < 2.9)]                       # no floor, no net roof
    dx, dy = p[:, 0] - PAD[0], p[:, 1] - PAD[1]                      # map -> course (forward, left)
    c, s = math.cos(PAD_HEADING), math.sin(PAD_HEADING)
    fwd, left = dx * c + dy * s, -(dx * s - dy * c)
    cols = {}
    for i, j, z in zip(np.floor(fwd / CELL).astype(int), np.floor(left / CELL).astype(int), p[:, 2]):
        lo, hi = cols.get((i, j), (z, z))
        cols[(i, j)] = (min(lo, z), max(hi, z))
    for x, y in STATE['depth']['points']:                            # the synthetic wall, 0-2 m
        cols[(int(math.floor(x / CELL)), int(math.floor(y / CELL)))] = (0., 2.)
    keys = np.array(list(cols))
    i0, j0 = keys.min(0) - 2
    zmin = np.full(tuple(keys.max(0) - (i0, j0) + 3), np.inf)
    zmax = np.full_like(zmin, -np.inf)
    for (i, j), (lo, hi) in cols.items():
        zmin[i - i0, j - j0], zmax[i - i0, j - j0] = lo, hi
    return zmin, zmax, (i0, j0)


ZMIN, ZMAX, ORIGIN = _course_columns()
D = np.arange(.02, MAX_D + .5, .02)                                   # samples along each ray
AZ = (np.arange(W) / (W - 1) - .5) * -HFOV                           # image left = +left
EL = (.5 - np.arange(H) / (H - 1)) * VFOV


def render(x, y, z, yaw):
    """Depth (m, forward) per pixel and the hit cells, for a camera at (x, y, z) facing yaw."""
    a = yaw + AZ                                                     # (W,)
    px = x + np.cos(a)[:, None] * D[None, :]                         # (W, S)
    py = y + np.sin(a)[:, None] * D[None, :]
    i = np.floor(px / CELL).astype(int) - ORIGIN[0]
    j = np.floor(py / CELL).astype(int) - ORIGIN[1]
    ok = (i >= 0) & (j >= 0) & (i < ZMIN.shape[0]) & (j < ZMIN.shape[1])
    i, j = np.where(ok, i, 0), np.where(ok, j, 0)
    lo, hi = np.where(ok, ZMIN[i, j], np.inf), np.where(ok, ZMAX[i, j], -np.inf)
    zr = z + np.tan(EL)[:, None, None] * D[None, None, :]            # (H, W, S) ray height
    hit = (zr >= lo[None] - .04) & (zr <= hi[None] + .04)
    first = np.where(hit.any(-1), hit.argmax(-1), -1)                 # (H, W)
    wall = np.where(first >= 0, D[np.clip(first, 0, None)], np.inf)
    floor = np.where(EL[:, None] < 0, z / np.tan(-np.minimum(EL, -1e-6))[:, None], np.inf) * np.ones((1, W))
    dist = np.minimum(wall, floor) * np.cos(AZ)[None, :]
    surf = np.where(wall <= floor, 1, 0)
    # top-down cells: the nearest wall hit per column within range
    cells = set()
    for w in range(0, W, 3):
        col = first[:, w]
        k = col[col >= 0]
        if k.size and MIN_D <= D[k.min()] <= MAX_D:
            cells.add((round(float(px[w, k.min()]) / CELL) * CELL, round(float(py[w, k.min()]) / CELL) * CELL))
    return dist, surf, sorted(cells)


STOPS = np.array([0., .2, .4, .6, .8, 1.])
RGB = np.array([[215, 48, 39], [252, 141, 89], [254, 224, 139], [145, 207, 96], [26, 152, 255], [48, 18, 59]], float)


def frame_jpeg(dist, surf):
    t = np.clip((dist - MIN_D) / (MAX_D - MIN_D), 0, 1)
    rgb = np.stack([np.interp(t, STOPS, RGB[:, k]) for k in range(3)], -1)
    rgb[(dist > MAX_D) | ~np.isfinite(dist)] = 0
    rng = np.random.default_rng(int(time.monotonic() * 10))
    grey = np.where(surf == 1, 150 - 18 * np.clip(dist, 0, 5), 120 - 10 * np.clip(dist, 0, 6))
    grey = np.clip(grey + rng.normal(0, 6, grey.shape) + 6 * np.sin(np.arange(W) / 5.)[None] * (surf == 1), 10, 235)
    img = np.concatenate([np.repeat(grey[..., None], 3, -1), rgb], 1).astype(np.uint8)
    im = Image.fromarray(img)
    ImageDraw.Draw(im).text((6, 4), 'depth 0.3-4 m, near = red  (simulated)', fill=(255, 255, 255))
    buf = io.BytesIO(); im.save(buf, 'JPEG', quality=80)
    return buf.getvalue()


# ---- simulated flight ---------------------------------------------------------------------------
LIVE = {}


def pose(now):
    k = int((now - T0) * 45) % len(ROUTE)                           # ~13 s per lap
    p, q = ROUTE[k], ROUTE[min(k + 6, len(ROUTE) - 1)]
    return k, p, math.atan2(q[1] - p[1], q[0] - p[0])


def depth_loop():
    while True:
        k, p, yaw = pose(time.monotonic())
        dist, surf, cells = render(p[0], p[1], p[2], yaw)
        fan = [[p[0] + MAX_D * math.cos(yaw + a), p[1] + MAX_D * math.sin(yaw + a)] for a in np.linspace(-HFOV / 2, HFOV / 2, 9)]
        LIVE.update(jpeg=frame_jpeg(dist, surf), cells=cells, fan=[[p[0], p[1]]] + fan, at=time.monotonic(), k=k)
        time.sleep(.5)                                               # the real worker runs at 2 Hz


def state():
    s = json.loads(json.dumps(STATE))
    k, p, yaw = pose(time.monotonic())
    s['pos'] = dict(s.get('pos') or {}, x=p[0], y=p[1], z=p[2], yaw_deg=math.degrees(yaw), valid=True, age_s=.02)
    s['target'] = s['cmd'] = [p[0], p[1], p[2]]
    s['course_run'] = dict(active=True, state='running', reason='Following the obstacle-adjusted route', progress=k / len(ROUTE),
                           controller='smooth_position', path_time_s=round(k * .0373, 2), duration_s=22.4, radius_m=7.,
                           route_source='adjusted', obstacle_guard=True)
    if STILL:
        s['depth'].update(points=BLOCKING, view_polygon=[], age_s=.2, geometry_version='demo:still:live')
    elif LIVE:
        s['depth'].update(points=[list(c) for c in LIVE['cells']], view_polygon=LIVE['fan'], camera_xy=[p[0], p[1]],
                          age_s=round(time.monotonic() - LIVE['at'], 2), geometry_version=f"demo:{LIVE['k']}:live")
    return s


class H(BaseHTTPRequestHandler):
    def _send(self, code, body, kind):
        self.send_response(code); self.send_header('Content-Type', kind); self.send_header('Cache-Control', 'no-store')
        self.end_headers(); self.wfile.write(body)

    def do_GET(self):
        path = self.path.split('?')[0]
        if path in ('/', '/index.html'):
            return self._send(200, PAGE, 'text/html; charset=utf-8')
        if path == '/api/state':
            return self._send(200, json.dumps(state()).encode(), 'application/json')
        if path == '/api/depth.jpg' and LIVE:
            return self._send(200, LIVE['jpeg'], 'image/jpeg')
        self._send(404, b'{}', 'application/json')

    def do_POST(self):
        self._send(200, json.dumps(dict(ok=False, msg='demo: commands are disabled', state=state())).encode(), 'application/json')

    def log_message(self, *a):
        pass


STILL = False
BLOCKING = json.loads((HERE / 'demo' / 'blocking_wall.json').read_text())

if __name__ == '__main__':
    import sys
    STILL = '--still' in sys.argv
    threading.Thread(target=depth_loop, daemon=True).start()
    print('dashboard demo: http://127.0.0.1:8099/?k=demo')
    ThreadingHTTPServer(('127.0.0.1', 8099), H).serve_forever()
