#!/usr/bin/env python3
"""Run the real flight dashboard (flight/dash/dash.html) against a recorded state, no drone needed.

    pixi run demo            # or: python3 tools/dashboard_demo.py  -> http://127.0.0.1:8099/?k=demo

The state is a snapshot of the live dashboard with a synthetic wall across the course and the
dashboard planner's own detour around it (course_state.update_preview). The drone dot flies the
detour; every command is refused. The depth card shows a saved Depth Anything V2 frame.
"""
import json, math, pathlib, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = pathlib.Path(__file__).resolve().parent
PAGE = (HERE.parent / 'flight' / 'dash' / 'dash.html').read_bytes()
STATE = json.loads((HERE / 'demo' / 'state.json').read_text())
ROUTE = STATE.pop('_demo_route')
DEPTH = (HERE / 'demo' / 'depth.jpg').read_bytes()
T0 = time.monotonic()


def state():
    s = json.loads(json.dumps(STATE))
    k = int((time.monotonic() - T0) * 45) % len(ROUTE)          # ~13 s per lap
    p, q = ROUTE[k], ROUTE[min(k + 3, len(ROUTE) - 1)]
    yaw = math.degrees(math.atan2(q[1] - p[1], q[0] - p[0]))
    s['pos'] = dict(s.get('pos') or {}, x=p[0], y=p[1], z=p[2], yaw_deg=yaw, valid=True, age_s=.02)
    s['target'] = s['cmd'] = [p[0], p[1], p[2]]
    s['course_run'] = dict(active=True, state='running', reason='Following the obstacle-adjusted route', progress=k / len(ROUTE),
                           controller='smooth_position', path_time_s=round(k * .0228, 2), duration_s=22.4, radius_m=7.,
                           route_source='adjusted', obstacle_guard=True)
    s['depth']['age_s'] = .2
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
        if path == '/api/depth.jpg':
            return self._send(200, DEPTH, 'image/jpeg')
        self._send(404, b'{}', 'application/json')

    def do_POST(self):
        self._send(200, json.dumps(dict(ok=False, msg='demo: commands are disabled', state=state())).encode(), 'application/json')

    def log_message(self, *a):
        pass


if __name__ == '__main__':
    print('dashboard demo: http://127.0.0.1:8099/?k=demo')
    ThreadingHTTPServer(('127.0.0.1', 8099), H).serve_forever()
