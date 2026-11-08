# Openings sim: tunnel + 80 cm window + beam, through the live dash code (preview, flight loop,
# guard, nudges, pass-through centring). Depth cells are built like depth_overlay/geometry.py
# (8 cm XY cells, one entry per 20 cm world-z slab with the real min/max return height),
# limited to what a forward camera sees (3.5 m), then height-filtered like dash_node.
# usage: python sim_openings.py <dash_src_dir> [controller]
import math, os, random, sys, tempfile
from pathlib import Path
sys.path.insert(0, sys.argv[1])
from course_state import CourseState
from course_replan import plan_remaining
from dash_core import DashCore, DashParams, Pos, Fc
from depth_height import filter_for_route

HALF_W, HALF_H = .254, .12     # 20 in drone through the wide way; props-included half height
CELL, SLAB = .08, .20
CONTROLLERS = sys.argv[2:] or ['smooth_position']


def slabbed(x, y, zs, pts, zr):
    by = {}
    for z in zs:
        by.setdefault(math.floor(z / SLAB), []).append(z)
    for zz in by.values():
        pts.append([round(x, 3), round(y, 3)]); zr.append([min(zz), max(zz)])


def tube(radius, cz=1.0, cx=0., y0=3.0, y1=4.6):
    pts, zr = [], []
    for i in range(int(2 * radius / CELL) + 2):
        x = cx - radius + CELL * i
        zs = set()
        for k in range(9):
            xx = min(abs(x - cx - CELL / 2 + CELL * k / 8), radius)
            h = math.sqrt(radius * radius - xx * xx); zs.update((cz + h, cz - h))
        for j in range(int((y1 - y0) / CELL) + 1):
            slabbed(x, y0 + CELL * j, zs, pts, zr)
    def gap(t):   # prop-edge clearance inside the tube (<0 = contact)
        if not y0 <= t[1] <= y1: return None
        return radius - math.hypot(t[0] - cx, t[2] - cz) - HALF_W   # conservative: width half as radius
    return pts, zr, gap


def window(w, h, cx=0., cz=1.0, y=3.0, wall_half=1.6, z0=.2, z1=2.0, thick=.16):
    """Wall from z0..z1, x in +-wall_half, with a w x h hole centred (cx, cz)."""
    pts, zr = [], []
    for i in range(int(2 * wall_half / CELL) + 1):
        x = -wall_half + CELL * i
        zs = [z0 + k * .05 for k in range(int((z1 - z0) / .05) + 1)]
        if abs(x - cx) < w / 2:
            zs = [z for z in zs if abs(z - cz) >= h / 2]
        for j in range(int(thick / CELL) + 1):
            slabbed(x, y + CELL * j, zs, pts, zr)
    def gap(t):
        if not y - .05 <= t[1] <= y + thick + .05: return None
        return min(w / 2 - abs(t[0] - cx) - HALF_W, h / 2 - abs(t[2] - cz) - HALF_H)
    return pts, zr, gap


def beam(z_lo, z_hi, y=3.0, depth=.16, x_half=2.0):
    pts, zr = [], []
    for i in range(int(2 * x_half / CELL) + 1):
        for j in range(int(depth / CELL) + 1):
            slabbed(-x_half + CELL * i, y + CELL * j, [z_lo + (z_hi - z_lo) * k / 6 for k in range(7)], pts, zr)
    def gap(t):
        if not y <= t[1] <= y + depth: return None
        return (z_lo - (t[2] + HALF_H)) if t[2] < z_lo else ((t[2] - HALF_H) - z_hi) if t[2] > z_hi else -1.
    return pts, zr, gap


def room():
    """Ordinary room: aisle walls at x=+-0.9, a box 0.6 m to the left at y 2-2.5, floor clutter
    (0-0.3 m) under the route at y=4, ceiling beam at 2.2 m. Nothing is tight."""
    pts, zr = [], []
    for j in range(76):
        y = CELL * j
        for x in (-.9, .9):
            slabbed(x, y, [z * .05 for z in range(45)], pts, zr)
    for i in range(4):
        for j in range(7):
            slabbed(-.6 - CELL * i, 2.0 + CELL * j, [z * .05 for z in range(25)], pts, zr)
    for i in range(-3, 4):
        slabbed(CELL * i, 4.0, [0., .1, .2, .3], pts, zr)
        slabbed(CELL * i, 3.5, [2.2, 2.3], pts, zr)
    return pts, zr, (lambda t: None)


def phantom():
    pts, zr = [], []
    for i in range(3):
        for j in range(3):
            slabbed(.18 + CELL * i, 3.0 + CELL * j, [.6 + .05 * k for k in range(17)], pts, zr)
    return pts, zr, (lambda t: None), (10, 15)     # visible ticks [10, 15): a single frame


def line(x, z, n=61):
    return [[x, 6 * i / (n - 1), z] for i in range(n)]


def dip(z_hi, z_lo, y_beam=3.0, ramp=1.2, n=121):
    out = []
    for i in range(n):
        y = 6 * i / (n - 1); d = abs(y - y_beam - .08)
        out.append([0., y, z_lo if d < .5 else (z_hi if d > .5 + ramp else z_lo + (z_hi - z_lo) * (d - .5) / ramp)])
    return out


SCENARIOS = [
    # name, obstacle, route, expect ('pass' | 'refuse' | 'clean' = flies, never engages), marked hole
    ('tunnel 1.42 m, centred, marked',        tube(.71),                 line(0., 1.0),   'pass',   [0., 3.8, 1.0]),
    ('tunnel 1.42 m, 30 cm off, marked',      tube(.71),                 line(.30, 1.0),  'pass',   [0., 3.8, 1.0]),
    ('tunnel 1.42 m, 25 cm low, marked',      tube(.71),                 line(0., .75),   'pass',   [0., 3.8, 1.0]),
    ('tunnel, 30 off + 20 high, marked',      tube(.71),                 line(-.30, 1.2), 'pass',   [0., 3.8, 1.0]),
    ('tunnel, 30 off, mark 10 cm wrong',      tube(.71),                 line(.30, 1.0),  'pass',   [.10, 3.8, .95]),
    ('window 80x80, centred, marked',         window(.80, .80),          line(0., 1.0),   'pass',   [0., 3.08, 1.0]),
    ('window 80x80, 15 cm off, marked',       window(.80, .80),          line(.15, 1.0),  'pass',   [0., 3.08, 1.0]),
    ('window 80x80, 20 cm low, marked',       window(.80, .80),          line(0., .80),   'pass',   [0., 3.08, 1.0]),
    ('window, 12 off + 15 high, marked',      window(.80, .80),          line(-.12, 1.15), 'pass',  [0., 3.08, 1.0]),
    ('window at x=+0.25, marked',             window(.80, .80, cx=.25),  line(0., 1.0),   'pass',   [.25, 3.08, 1.0]),
    ('window, 15 off, mark 8 cm wrong',       window(.80, .80),          line(.15, 1.0),  'pass',   [.08, 3.08, 1.05]),
    ('window 80x80, 15 cm off, NOT marked',   window(.80, .80),          line(.15, 1.0),  'refuse', None),
    ('window 50 cm (too narrow), marked',     window(.50, .80),          line(0., 1.0),   'refuse', [0., 3.08, 1.0]),
    ('beam above route',                      beam(1.35, 1.50),          line(0., 1.0),   'pass',   None),
    ('route dips under beam',                 beam(.95, 1.10),           dip(1.0, .6),    'pass',   None),
    ('beam at route height, not marked',      beam(.95, 1.10),           line(0., 1.0),   'refuse', None),
    ('ordinary room (must NOT engage)',       room(),                    line(0., 1.0),   'clean',  None),
    ('phantom box (1 frame) -> nudge, rejoin',  phantom(),                 line(0., 1.0),   'rejoin', None),
    ('solid wall across route, no hole',      window(0., 0., wall_half=2.0, z0=0., z1=2.4), line(0., 1.0), 'refuse', None),
]


def run(name, obstacle, route, controller, hole=None):
    pts, zr, gap = obstacle[:3]
    window_ticks = obstacle[3] if len(obstacle) > 3 else None
    random.seed(2)
    tmp = tempfile.TemporaryDirectory()
    st = CourseState(Path(tmp.name) / 'course.json')
    assert st.import_route(route)[0] and st.capture_anchor([0, 0, 0], 0)[0]
    if hole is not None:
        assert st.edit_marks('add_hole', world_xyz=hole)[0]
    # Preview sees what the camera sees from the start point (3.5 m).
    vis0 = [] if window_ticks else [i for i, w in enumerate(pts) if math.dist(w, route[0][:2]) < 3.5]
    full0 = dict(status='live', age_s=.2, cell_m=CELL, points=[pts[i] for i in vis0], z_ranges=[zr[i] for i in vis0])
    st.update_preview(filter_for_route(full0, st.world_route()), now=10)
    a = st.snapshot(10)['adjustment']
    try:
        path = st.execution_path(10)
    except ValueError as e:
        return dict(result='refused', why=str(e)[:90], preview=a['status'])
    c = DashCore(DashParams(heading_mode='hold_start', land_after_course=False))
    c.owner = 'pilot'; c.owner_hb = 10; c.home = (0, 0, 0, 0)
    x0, y0, z0 = route[0]
    c.mode = 'engaged'; c.was_offboard = True; c.cmd = [x0, y0, z0]; c.target = list(c.cmd)
    c.heading.reset(math.pi / 2); c.last_step = 10; c.course_controller = controller
    p = Pos(10, x0, y0, z0, math.pi / 2, True, 0, 0, 0); f = Fc(10, True, True, True, False)   # nose along +y
    ok, why = c.start_course(10, path, 8, p, f)
    if not ok:
        return dict(result='refused', why='start: ' + why[:80], preview=a['status'])
    truth = [x0, y0, z0]; vel = [0., 0., 0.]; worst = math.inf; pauses = nudges = 0; replan = None
    was_paused = False; fail = None; t_end = None; centred_at = None; late_dev = 0.; profile = {}
    for tick in range(1, 3000):
        now = 10 + .1 * tick; p.t = f.t = now; c.heartbeat(now, 'pilot')
        # Forward camera: 3.5 m range, +-40 deg of the nose.
        vis = [i for i, w in enumerate(pts) if math.dist(w, truth[:2]) < 3.5 and abs(math.degrees(
               (math.atan2(w[1] - truth[1], w[0] - truth[0]) - p.yaw + math.pi) % (2 * math.pi) - math.pi)) <= 40]
        if window_ticks and not window_ticks[0] <= tick < window_ticks[1]:
            vis = []
        raw = dict(status='live', age_s=.3, cell_m=CELL, points=[pts[i] for i in vis],
                   z_ranges=[zr[i] for i in vis], geometry_version=f'v{tick // 5}')
        run_ = c.course_run
        d = filter_for_route(raw, run_.points if run_ is not None and run_.active else st.world_route())
        out = c.step(now, p, f, d)
        if c.mode == 'failsafe': fail = c.reason; break
        if c.heading.yaw is not None:
            p.yaw = c.heading.yaw
        if out and out[0] == 'pose':
            want = [max(-1., min(1., (out[1][i] - truth[i]) / .3)) for i in range(3)]
        elif out:
            want = list(out[1])
        else:
            want = [0., 0., 0.]
        dv = [w - v for w, v in zip(want, vel)]; m = math.sqrt(sum(x * x for x in dv)); lim = 1.5 * .1
        if m > lim: dv = [x * lim / m for x in dv]
        vel = [v + x for v, x in zip(vel, dv)]
        truth = [a_ + v * .1 for a_, v in zip(truth, vel)]
        p.x, p.y, p.z = truth; p.vx, p.vy, p.vz = vel
        g = gap(truth)
        if 4.8 <= truth[1] <= 6.0:     # after the glide back (2 s hysteresis + 1.2 m glide)
            late_dev = max(late_dev, abs(truth[0] - route[0][0]))
        b = int(truth[1] / .5)
        profile[b] = max(profile.get(b, 0.), abs(truth[0] - route[0][0]))
        if g is not None:
            worst = min(worst, g)
            if centred_at is None: centred_at = [round(v, 2) for v in truth]
        n = c.nudge_request
        if n is not None and n.get('path') is not None:
            if c.apply_nudge(now, n['path'], 1.)[0]: nudges += 1
        if tick % 5 == 0:
            r = c.course_run
            if c.paused_course is not None:
                gd, rr = c.paused_course['guard'], c.paused_course['run']
                try:
                    replan = plan_remaining(rr.points, rr.times, c.paused_course['elapsed_s'], (p.x, p.y, p.z), rr.anchor,
                                            rr.reference, rr.radius, d if d.get('status') == 'live' else None,
                                            gd.known_walls(rr.points), known_cell_m=gd.cell_m, clearance_m=gd.clearance,
                                            floor=.3, ceiling=2.)
                except ValueError: replan = None
            elif not os.environ.get('NOREJOIN') and (job := c.rejoin_job(now)) is not None:  # = dash_node.plan_rejoin
                try:
                    pth = plan_remaining(job['points'], job['times'], job['elapsed'], job['start'], job['anchor'],
                                         job['reference'], job['radius'], d, job['walls'], known_cell_m=job['cell'],
                                         clearance_m=job['clearance'], floor=job['floor'], ceiling=job['ceiling'])
                except ValueError:
                    pth = None
                c.commit_rejoin(now, job, pth)
            elif n is not None and n.get('path') is None and r is not None and r.active:
                gd = r.obstacle_guard
                try:
                    pth = plan_remaining(list(r.points), list(r.times), r.elapsed, tuple(c.cmd), r.anchor, r.reference,
                                         r.radius, d, gd.known_walls(r.points), known_cell_m=gd.cell_m,
                                         clearance_m=gd.clearance, floor=.3, ceiling=2.)
                    if pth['source'] == 'adjusted': n.update(path=pth, planned_at=now)
                    else: c.nudge_request = None; c.nudge_retry_at = now + 1
                except ValueError:
                    c.nudge_request = None; c.nudge_retry_at = now + 1
        if c.paused_course is not None:
            if not was_paused: pauses += 1
            was_paused = True
            if replan is not None: c.resume_course(now, replan, p, f, d, 1.)
        else:
            was_paused = False
        if c.course_run is not None and not c.course_run.active and c.paused_course is None:
            t_end = round(now - 10, 1); break
    events = [e for e in (getattr(c, 'events', None) or []) if 'pening' in str(e) or 'clear again' in str(e)
              or 'nudged' in str(e)]
    state = ('FAILSAFE: ' + fail) if fail else c.course_run.state
    return dict(result=state, t=t_end, pauses=pauses, nudges=nudges, preview=a['status'],
                edge_gap=None if worst == math.inf else round(worst, 3), at_opening=centred_at,
                reached_y=round(truth[1], 2), events=[str(e)[-90:] for e in events][:4],
                late_dev=round(late_dev, 3), profile={k * .5: round(v, 2) for k, v in sorted(profile.items())})


ONLY = os.environ.get('ONLY')
bad = 0
for name, obstacle, route, expect, hole in SCENARIOS:
    if ONLY and ONLY not in name: continue
    for controller in CONTROLLERS:
        r = run(name, obstacle, route, controller, hole)
        flew = r['result'] == 'complete'
        hit = r.get('edge_gap') is not None and r['edge_gap'] < 0
        ok = ((flew and not hit) if expect == 'pass' else
              (flew and not r.get('events') and r['preview'] == 'unchanged') if expect == 'clean' else
              (flew and r['nudges'] >= 1 and any('clear again' in e for e in r['events'])
               and r['late_dev'] < .05) if expect == 'rejoin' else
              (not flew or r.get('edge_gap') is None))
        bad += not ok
        print(f"{'OK ' if ok else 'BAD'} {name:40s} {controller[:6]} -> {r['result']:9s} "
              f"edge gap {r.get('edge_gap')}  at {r.get('at_opening')}  t={r.get('t')} stops={r.get('pauses')} "
              f"nudges={r.get('nudges')} off-plan after it={r.get('late_dev')} preview={r['preview']} {r.get('why', '')}")
        for e in r.get('events', []):
            print('      ', e)
        if os.environ.get('PROFILE'):
            print('       offset from plan by y (m):', r.get('profile'))
print('ALL OK' if not bad else f'{bad} scenario(s) not as expected')
