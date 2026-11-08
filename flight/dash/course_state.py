"""Stored original routes and bounded depth-adjusted course plans.

Only the validated original document is saved. Anchors, preview settings and
adjusted geometry live in memory. Flight copies a fresh, complete validated
plan; decimated display geometry and unchecked original routes are not flown.
"""
import copy
import json
import math
import os
from pathlib import Path
import tempfile
import threading
import time
import uuid

from path_heading import tangent_yaws
from course_safety import TRACKING_RESERVE_M


MAX_POINTS = 4000
MAX_DISPLAY_POINTS = 600
MAX_DEPTH_POINTS = 1600
# Maximum observation age for a planned route; live commands additionally use
# current frame age and measured velocity in the stopping-distance guard.
MAX_DEPTH_AGE_S = 1.5
# ponytail: dashboard altitude band defaults (flight_limits.json 0.3–2.0 m); preview centring stays inside it
PREVIEW_FLOOR_M = .30
PREVIEW_CEILING_M = 2.0
MAX_FILE_BYTES = 1024 * 1024
MAX_MARKS = 20            # zones and holes each
MARKS_FILE = 'course_marks.json'
RETURN_FILE = 'course_return.json'   # return hop, course frame of the main course (same anchor)


def _number(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


def _xyz(value, limit=None):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError('waypoints must be XYZ triples')
    if not all(_number(v) and (limit is None or abs(v) <= limit) for v in value):
        raise ValueError('coordinates must be finite numbers' +
                         (f' within ±{limit:g} m' if limit is not None else ''))
    return list(value)


def validate_document(doc):
    if isinstance(doc, list):
        doc = dict(version=1, frame='course_flu', units='m', waypoints=doc)
    if not isinstance(doc, dict):
        raise ValueError('route must be an object or a list of XYZ waypoints')
    allowed = {'version', 'frame', 'units', 'name', 'waypoints', 'times_s'}
    if set(doc) - allowed:
        raise ValueError('unknown route fields: ' + ', '.join(sorted(set(doc)-allowed)))
    if type(doc.get('version')) is not int or doc['version'] != 1:
        raise ValueError('route version must be 1')
    if doc.get('frame') != 'course_flu' or doc.get('units') != 'm':
        raise ValueError('route frame must be course_flu and units must be m')
    if 'name' in doc and (not isinstance(doc['name'], str) or len(doc['name']) > 80):
        raise ValueError('route name must be at most 80 characters')
    waypoints = doc.get('waypoints')
    if not isinstance(waypoints, list) or not 2 <= len(waypoints) <= MAX_POINTS:
        raise ValueError(f'route must contain 2–{MAX_POINTS} waypoints')
    result = copy.deepcopy(doc)
    result['waypoints'] = [_xyz(p, 1000) for p in waypoints]
    if 'times_s' in doc:
        times = doc['times_s']
        if (not isinstance(times, list) or len(times) != len(waypoints)
                or not all(_number(t) for t in times)
                or any(b <= a for a, b in zip(times, times[1:]))):
            raise ValueError('times_s must be finite, strictly increasing, and match the waypoint count')
    return result


def _display_indices(count):
    if count <= MAX_DISPLAY_POINTS:
        return range(count)
    return [i * (count-1) // (MAX_DISPLAY_POINTS-1)
            for i in range(MAX_DISPLAY_POINTS)]


def _display_points(points, indices=None):
    indices = _display_indices(len(points)) if indices is None else indices
    return [list(points[i]) for i in indices]


def _atomic_save(path, document):
    encoded = json.dumps(document, ensure_ascii=False, allow_nan=False,
                         separators=(',', ':')) + '\n'
    if len(encoded.encode('utf-8')) > MAX_FILE_BYTES:
        raise ValueError('route document exceeds 1 MiB')
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8',
                dir=path.parent, prefix='.'+path.name+'.', suffix='.tmp', delete=False) as handle:
            temporary = Path(handle.name)
            handle.write(encoded)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
        temporary = None
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def _in_zones(xy, zones, polygons):
    """True if the XY point is inside any zone circle [x, y, r] or polygon [[x, y], ...]."""
    from depth_height import inside_polygon
    if any(math.hypot(xy[0]-z[0], xy[1]-z[1]) <= z[2] for z in zones):
        return True
    return any(bool(inside_polygon([xy[:2]], poly)[0]) for poly in polygons)


def control_holes(holes, zones, polygons):
    """Hole control applies only to holes marked inside an avoidance zone the operator drew."""
    return [h for h in holes if _in_zones(h, zones, polygons)]


def _validate_marks(m):
    zones = [[float(v) for v in z] for z in m.get('zones', [])]
    holes = [[float(v) for v in h] for h in m.get('holes', [])]
    polygons = [[[float(v) for v in p] for p in poly] for poly in m.get('polygons', [])]
    if (len(polygons) > MAX_MARKS or any(not 3 <= len(poly) <= 40 or any(
            len(p) != 2 or not all(math.isfinite(v) for v in p) for p in poly) for poly in polygons)):
        raise ValueError('a polygon zone needs 3-40 finite corners')
    if (len(zones) > MAX_MARKS or len(holes) > MAX_MARKS
            or any(len(z) != 3 or not all(math.isfinite(v) for v in z) or not .3 <= z[2] <= 5. for z in zones)
            or any(len(h) != 3 or not all(math.isfinite(v) for v in h) for h in holes)):
        raise ValueError('invalid zones or holes')
    return dict(zones=zones, holes=holes, polygons=polygons, zones_only=bool(m.get('zones_only', False)))


class CourseState:
    def __init__(self, path):
        # Dashboard startup is grounded. Load NumPy now, rather than adding
        # import/discovery work when the first preview is requested in flight.
        import course_geometry
        self._geometry = course_geometry
        self.path = Path(path)
        self.lock = threading.RLock()
        self._import_lock = threading.Lock()
        self._session = uuid.uuid4().hex
        self.revision = 0
        self.document = None
        self.anchor = None
        self.preview_enabled = False
        self.clearance_m = .30
        self.max_offset_m = 1.5
        self._generation = 0
        self._request = 0
        self._adjustment_request = 0
        self._original_world = []
        self._original_yaws_course = []
        self._original_yaws_world = []
        self._adjusted_course = []
        self._adjusted_world = []
        self._adjusted_yaws_world = []
        self._adjustment_status = 'waiting'
        self._adjustment_reason = 'Import a course route first'
        self._depth_at = None
        self._depth_age = None
        self._plan_walls = []
        self._plan_cell = .08
        self._preview_frames = []      # grid cells of the last 3 distinct depth frames (confirmation)
        self._preview_version = None
        self._reason = 'No course route loaded'
        # Avoidance zones ([forward, left, radius]) and hole centres ([forward, left, up]), course
        # frame (relative to the course start), kept across restarts and route imports.
        self.marks = dict(zones=[], holes=[], polygons=[], zones_only=False)
        self.return_document = None
        try:
            with self.path.with_name(RETURN_FILE).open('r', encoding='utf-8') as handle:
                self.return_document = validate_document(json.loads(handle.read(MAX_FILE_BYTES)))
        except (OSError, ValueError, TypeError, KeyError):
            pass
        try:
            with self.path.with_name(MARKS_FILE).open('r', encoding='utf-8') as handle:
                self.marks = _validate_marks(json.loads(handle.read(MAX_FILE_BYTES)))
        except (OSError, ValueError, TypeError, KeyError):
            pass
        try:
            with self.path.open('r', encoding='utf-8') as handle:
                raw = handle.read(MAX_FILE_BYTES + 1)
            if len(raw.encode('utf-8')) > MAX_FILE_BYTES:
                raise ValueError('saved route exceeds 1 MiB')
            document = validate_document(json.loads(raw))
            headings = tangent_yaws(document['waypoints'])
            self.document, self._original_yaws_course = document, headings
        except FileNotFoundError:
            pass
        except (OSError, ValueError, TypeError, UnicodeError, RecursionError) as exc:
            self._reason = 'Saved route unavailable: ' + str(exc)[:180]
        else:
            self.revision = 1
            self._reason = 'Capture a fresh course anchor; anchors are not restored after restart'
            self._adjustment_reason = 'Capture a course anchor, then enable preview'

    def _invalidate(self, reason):
        self._generation += 1
        self._request += 1
        self._adjustment_request = self._request
        self._adjusted_course, self._adjusted_world = [], []
        self._adjusted_yaws_world = []
        self._adjustment_status, self._adjustment_reason = 'waiting', reason
        self._depth_at = self._depth_age = None

    def import_route(self, doc):
        try:
            original = validate_document(doc)
            headings = tangent_yaws(original['waypoints'])
            # Serialize writers without blocking snapshot readers on fsync or
            # rename. Readers keep the old route until the new file is saved.
            with self._import_lock:
                _atomic_save(self.path, original)
                with self.lock:
                    self.document = original
                    self._original_yaws_course = headings
                    self.revision += 1
                    self.anchor = None
                    self.preview_enabled = False
                    self._original_world = []
                    self._original_yaws_world = []
                    self._reason = 'Original route saved; capture a course anchor'
                    self._invalidate('Capture a course anchor, then enable preview')
                    reason = self._reason
            return True, reason
        except (OSError, ValueError, TypeError, UnicodeError, RecursionError) as exc:
            return False, str(exc)[:240]

    def capture_anchor(self, origin_enu, yaw_rad):
        try:
            origin = _xyz(origin_enu)
            if not _number(yaw_rad):
                raise ValueError('anchor yaw must be finite radians')
            anchor = dict(valid=True, origin_enu=origin, yaw_rad=yaw_rad,
                          captured_wall_ns=time.time_ns())
            with self.lock:
                if self.document is None:
                    return False, 'Import a route before capturing its anchor'
                points = copy.deepcopy(self.document['waypoints'])
                headings = list(self._original_yaws_course)
                generation = self._generation
            world = self._geometry.course_to_world(points, anchor).tolist()
            world_headings = [yaw + yaw_rad for yaw in headings]
            with self.lock:
                if generation != self._generation:
                    return False, 'Route state changed while capturing the anchor; try again'
                self.anchor = anchor
                self._original_world = world
                self._original_yaws_world = world_headings
                self.preview_enabled = True
                self._reason = 'Course anchor captured; waiting for a fresh obstacle-adjusted plan'
                self._invalidate('Waiting for fresh wall observations')
            return True, self._reason
        except (ValueError, TypeError, OverflowError) as exc:
            return False, str(exc)[:240]

    # ---- avoidance zones and holes ---------------------------------------------------------
    def _save_marks(self, marks):
        _atomic_save(self.path.with_name(MARKS_FILE), marks)
        with self.lock:
            self.marks = marks
            self._invalidate('Avoidance zones or holes changed')

    def edit_marks(self, op, world_xyz=None, radius_m=1.5, index=None, zones_only=None, world_points=None):
        """op: add_zone / add_hole (world_xyz = PX4 local ENU point), remove_zone / remove_hole
        (index), clear, zones_only (flag). Points are stored relative to the course start."""
        try:
            with self.lock:
                marks, anchor = copy.deepcopy(self.marks), copy.deepcopy(self.anchor)
            if op in ('add_zone', 'add_hole'):
                if anchor is None:
                    return False, 'Set home / course start first (marks are stored relative to it)'
                course = self._geometry.world_to_course([_xyz(world_xyz)], anchor)[0].tolist()
                if op == 'add_zone':
                    marks['zones'].append([course[0], course[1], float(radius_m)])
                else:
                    marks['holes'].append(course)
            elif op == 'add_polygon':
                if anchor is None:
                    return False, 'Set home / course start first (marks are stored relative to it)'
                corners = [[float(p[0]), float(p[1]), 0.] for p in world_points or ()]
                if len(corners) < 3:
                    return False, 'a polygon zone needs at least 3 corners'
                course = self._geometry.world_to_course(corners, anchor)[:, :2].tolist()
                marks['polygons'].append(course)
            elif op in ('remove_zone', 'remove_hole', 'remove_polygon'):
                items = marks[{'remove_zone': 'zones', 'remove_hole': 'holes', 'remove_polygon': 'polygons'}[op]]
                if type(index) is not int or not 0 <= index < len(items):
                    return False, 'no such mark (the list changed?)'
                del items[index]
            elif op == 'clear':
                marks['zones'], marks['holes'], marks['polygons'] = [], [], []
            elif op == 'zones_only':
                marks['zones_only'] = bool(zones_only)
            else:
                return False, 'unknown mark operation'
            self._save_marks(_validate_marks(marks))
            m = self.marks
            return True, f"{len(m['zones'])+len(m['polygons'])} zone(s), {len(m['holes'])} hole(s); " + (
                'avoidance only inside zones' if m['zones_only'] else 'avoidance everywhere')
        except (OSError, ValueError, TypeError, IndexError, KeyError) as exc:
            return False, str(exc)[:200]

    def world_marks(self):
        """Zones [[x, y, r]] and holes [[x, y, z]] in PX4 local ENU, or None before an anchor."""
        with self.lock:
            marks, anchor = copy.deepcopy(self.marks), copy.deepcopy(self.anchor)
        if anchor is None:
            return None
        g = self._geometry
        zones = ([[*g.course_to_world([[z[0], z[1], 0.]], anchor)[0][:2].tolist(), z[2]] for z in marks['zones']])
        holes = g.course_to_world(marks['holes'], anchor).tolist() if marks['holes'] else []
        polygons = [g.course_to_world([[p[0], p[1], 0.] for p in poly], anchor)[:, :2].tolist()
                    for poly in marks['polygons']]
        return dict(zones=zones, holes=holes, polygons=polygons, zones_only=marks['zones_only'],
                    control_holes=control_holes(holes, zones, polygons))

    def world_route(self):
        """The saved route in PX4 local ENU (empty until the course anchor is set)."""
        with self.lock:
            return [list(p) for p in self._original_world]

    def configure_preview(self, clearance_m=.30, max_offset_m=1.5):
        if not _number(clearance_m) or not .15 <= clearance_m <= 1.0:
            return False, 'clearance must be 0.15–1.0 m'
        if not _number(max_offset_m) or not 0 <= max_offset_m <= 2.0:
            return False, 'maximum offset must be 0–2.0 m'
        with self.lock:
            if self.document is None or self.anchor is None:
                return False, 'Import a route and capture its anchor first'
            self.clearance_m, self.max_offset_m = float(clearance_m), float(max_offset_m)
            self.preview_enabled = True
            self._reason = 'Obstacle adjustment enabled; flight requires a fresh validated plan'
            self._invalidate('Waiting for fresh wall observations')
        return True, self._reason

    def invalidate_anchor(self, reason):
        """Discard a doubtful course reference while preserving the saved route."""
        with self.lock:
            self.anchor = None
            self.preview_enabled = False
            self._original_world = []
            self._original_yaws_world = []
            self._reason = str(reason)[:240] or 'Course reference changed; capture a fresh anchor'
            self._invalidate(self._reason)

    def execution_reference(self):
        with self.lock:
            return None if self.anchor is None else (
                self._session, self.revision, self.anchor['captured_wall_ns'])

    def import_return(self, doc):
        """Save the return hop (course_flu, in the MAIN course's frame: same launch pad and heading)."""
        try:
            route = validate_document(doc)
            with self._import_lock:
                _atomic_save(self.path.with_name(RETURN_FILE), route)
                with self.lock:
                    self.return_document = route
            return True, f"Return hop saved ({len(route['waypoints'])} points)"
        except (OSError, ValueError, TypeError, UnicodeError, RecursionError) as exc:
            return False, str(exc)[:240]

    def return_path(self):
        """Anchored return hop, flown as an original route (no obstacle adjustment)."""
        with self.lock:
            if self.return_document is None:
                raise ValueError('No return hop loaded')
            if self.anchor is None:
                raise ValueError('Set home / course start before starting the course')
            doc, anchor = copy.deepcopy(self.return_document), copy.deepcopy(self.anchor)
            reference = self.execution_reference()
        world = self._geometry.course_to_world(doc['waypoints'], anchor).tolist()
        return dict(points=world, source='original', times_s=doc.get('times_s'), anchor=anchor,
                    reference=reference, avoidance=None, marks=None)

    def return_status(self):
        with self.lock:
            doc, anchor = self.return_document, copy.deepcopy(self.anchor)
        if doc is None:
            return None
        wp = doc['waypoints']
        out = dict(name=doc.get('name'), count=len(wp), duration_s=(doc.get('times_s') or [None])[-1],
                   max_height_m=round(max(p[2] for p in wp), 2))
        if anchor is not None:
            world = self._geometry.course_to_world(wp, anchor)
            out['world_enu'] = _display_points(world.tolist())
        return out

    def original_path(self):
        """Anchored original route, executable only in explicit shadow mode."""
        with self.lock:
            if self.document is None or self.anchor is None:
                raise ValueError('Set home / course start before starting the course')
            return dict(points=copy.deepcopy(self._original_world), source='original',
                        times_s=copy.deepcopy(self.document.get('times_s')),
                        anchor=copy.deepcopy(self.anchor), reference=self.execution_reference())

    def execution_path(self, now=None):
        """Full validated adjusted geometry, with no unchecked-route fallback."""
        now = time.monotonic() if now is None else now
        with self.lock:
            age = (None if self._depth_at is None or self._depth_age is None else
                   self._depth_age + max(0., now-self._depth_at))
            if self.anchor is None or not self.preview_enabled:
                raise ValueError('Set home / course start and enable obstacle adjustment first')
            if (age is None or age > MAX_DEPTH_AGE_S or
                    self._adjustment_status not in ('adjusted', 'unchanged') or
                    len(self._adjusted_world) < 2):
                raise ValueError('No fresh collision-checked course: '+self._adjustment_reason)
            indices = _display_indices(len(self._adjusted_world))
            return dict(points=copy.deepcopy(self._adjusted_world), times_s=None,
                anchor=copy.deepcopy(self.anchor), reference=self.execution_reference(),
                avoidance=dict(clearance_m=max(.30, self.clearance_m),
                               planned_clearance_m=max(.30, self.clearance_m)+TRACKING_RESERVE_M,
                               walls=copy.deepcopy(self._plan_walls), cell_m=self._plan_cell),
                source=self._adjustment_status, marks=self.world_marks(),
                display=dict(points=_display_points(self._adjusted_world, indices),
                             yaws=[self._adjusted_yaws_world[i] for i in indices],
                             version=f'{self._session}:{self._generation}:{self._adjustment_request}'))

    def flight_path(self, now=None, allow_unchecked=True):
        """Route to fly: the fresh obstacle-checked one when there is one; otherwise the planned route
        (bent through marked holes) flown with in-flight avoidance only (guard stops, nudges, replans,
        hole control). Operator choice: don't wait on the ground for a checked preview."""
        try:
            return self.execution_path(now)
        except ValueError:
            if not allow_unchecked:
                raise
        with self.lock:
            if self.document is None or self.anchor is None:
                raise ValueError('Set home / course start before starting the course')
            points = copy.deepcopy(self.document['waypoints'])
            anchor = copy.deepcopy(self.anchor)
            holes = control_holes(self.marks['holes'], self.marks['zones'], self.marks['polygons'])
            clearance = self.clearance_m
        if holes:
            from pass_through import through_holes
            points = through_holes(points, holes, floor=PREVIEW_FLOOR_M, ceiling=PREVIEW_CEILING_M)[0].tolist()
        world = self._geometry.course_to_world(points, anchor).tolist()
        yaws = [y+anchor['yaw_rad'] for y in tangent_yaws(points)]
        idx = _display_indices(len(world))
        return dict(points=world, times_s=None, anchor=anchor, reference=self.execution_reference(),
                    avoidance=dict(clearance_m=max(.30, clearance),
                                   planned_clearance_m=max(.30, clearance)+TRACKING_RESERVE_M,
                                   walls=[], cell_m=.08),
                    source='unchecked', marks=self.world_marks(),
                    display=dict(points=_display_points(world, idx), yaws=[yaws[i] for i in idx],
                                 version=f'{self._session}:{self._generation}:unchecked'))

    def _clear_depth(self, status, reason):
        self._adjusted_course, self._adjusted_world = [], []
        self._adjusted_yaws_world = []
        self._adjustment_status, self._adjustment_reason = status, reason
        self._adjustment_request = self._request

    def update_preview(self, depth, now=None):
        now = time.monotonic() if now is None else now
        started = time.monotonic()
        with self.lock:
            if not self.preview_enabled or self.document is None or self.anchor is None:
                return
            self._request += 1
            request, generation = self._request, self._generation
            age = depth.get('age_s') if isinstance(depth, dict) else None
            valid_age = _number(age) and age >= 0
            status = depth.get('status') if isinstance(depth, dict) else None
            if status != 'live' or not valid_age or age > MAX_DEPTH_AGE_S:
                self._depth_at = now
                self._depth_age = age if valid_age else None
                state = 'stale' if status == 'stale' or (_number(age) and age > MAX_DEPTH_AGE_S) else 'waiting'
                self._clear_depth(state, 'Waiting for wall observations no older than 1.5 seconds')
                return
            walls = depth.get('points')
            # A fresh scaled frame with zero wall cells plans the route unchanged
            # ("no wall seen"); it still does not certify unobserved space.
            if not isinstance(walls, (list, tuple)):
                self._depth_at, self._depth_age = now, age
                self._clear_depth('waiting', 'No current wall observations')
                return
            if len(walls) > MAX_DEPTH_POINTS:
                self._depth_at, self._depth_age = now, age
                self._clear_depth('blocked', 'Wall observation count exceeds the preview limit')
                return
            walls = copy.deepcopy(walls)
            cells3d = copy.deepcopy(depth.get('cells3d'))
            # Plan only around cells confirmed in 2 of the last 3 distinct frames (+-1 cell for jitter):
            # a one-frame ML speckle must not move the planned route; a real obstacle shows in ~1 s.
            cm = depth.get('cell_m', .08) if _number(depth.get('cell_m')) and depth.get('cell_m') > 0 else .08
            key = lambda p: (int(math.floor(p[0]/cm)), int(math.floor(p[1]/cm)))
            version = depth.get('geometry_version') or depth.get('stamp_ns') or depth.get('produced_wall_ns')
            if not all(isinstance(p, (list, tuple)) and len(p) == 2 and all(_number(v) for v in p) for p in walls):
                version = None       # malformed: skip confirmation; the validation below rejects it
            if version is not None and version != self._preview_version:
                self._preview_version = version
                self._preview_frames = (self._preview_frames+[{key(p) for p in walls}])[-3:]
            # ponytail: unversioned frames (unit tests, legacy sources) are planned as-is; live frames
            # always carry depth_cache's geometry_version.
            frames = self._preview_frames if version is not None else None
            if frames is not None and len(frames) < 2:
                self._depth_at, self._depth_age = now, age
                self._clear_depth('waiting', 'Confirming wall observations over two depth frames')
                return

            def confirmed(p):
                kx, ky = key(p)
                return sum(any((kx+i, ky+j) in f for i in (-1, 0, 1) for j in (-1, 0, 1)) for f in frames) >= 2
            if frames is not None:
                walls = [p for p in walls if confirmed(p)]
                if cells3d:
                    cells3d = [c for c in cells3d if all(_number(v) for v in c[:2]) and confirmed(c)]
            points = copy.deepcopy(self.document['waypoints'])
            anchor = copy.deepcopy(self.anchor)
            holes_course = control_holes(self.marks['holes'], self.marks['zones'], self.marks['polygons'])
            clearance, offset = self.clearance_m, self.max_offset_m
            cell = depth.get('cell_m', .08)
        try:
            if not _number(cell) or not .02 <= cell <= .5:
                raise ValueError('wall cell size must be 0.02–0.5 m')
            if any(not isinstance(p, (list, tuple)) or len(p) != 2
                   or not all(_number(v) for v in p) for p in walls):
                raise ValueError('wall observations must be finite XY pairs')
            openings = []
            holes_world = []
            if holes_course:
                from pass_through import through_holes
                points = through_holes(points, holes_course, floor=PREVIEW_FLOOR_M,
                                       ceiling=PREVIEW_CEILING_M)[0].tolist()
                holes_world = self._geometry.course_to_world(holes_course, anchor).tolist()
            if cells3d:
                from pass_through import centre_route
                from depth_height import keep_mask
                import numpy as _np
                world = self._geometry.course_to_world(points, anchor)
                floor = anchor['origin_enu'][2]+PREVIEW_FLOOR_M
                centred, openings = centre_route(world, cells3d, floor=floor,
                                                 ceiling=anchor['origin_enu'][2]+PREVIEW_CEILING_M,
                                                 holes=holes_world)
                if openings:
                    c3 = _np.asarray(cells3d, dtype=float).reshape(-1, 4)
                    keep = keep_mask(c3[:, :2], c3[:, 2:], centred)
                    walls = list({(round(x, 3), round(y, 3)): [x, y] for x, y in c3[keep, :2].tolist()}.values())
                    points = self._geometry.world_to_course(centred, anchor).tolist()
            wall_xyz = [[p[0], p[1], anchor['origin_enu'][2]] for p in walls]
            course_walls = (self._geometry.world_to_course(wall_xyz, anchor)[:, :2] if wall_xyz
                            else [])
            result = self._geometry.deform_preview(points, course_walls,
                                                   clearance_m=max(.30, clearance)+TRACKING_RESERVE_M,
                                                   max_offset_m=offset, cell_m=cell,
                                                   pass_clearance_m=max(.30, clearance))
            if result['status'] not in ('unchanged', 'adjusted', 'blocked'):
                raise ValueError('unknown preview result')
            if openings and result['status'] != 'blocked':
                result = dict(result, status='adjusted', reason=f'Centred through {len(openings)} opening(s); '
                              + str(result.get('reason', '')))
            adjusted = result['points'] if result['status'] != 'blocked' else []
            world = self._geometry.course_to_world(adjusted, anchor).tolist() if adjusted else []
            world_headings = [yaw + anchor['yaw_rad'] for yaw in tangent_yaws(adjusted)]
        except (ValueError, TypeError, KeyError, OverflowError) as exc:
            result = dict(status='blocked', reason='Preview unavailable: '+str(exc)[:180])
            adjusted, world, world_headings = [], [], []
        elapsed = max(0.0, time.monotonic()-started)
        with self.lock:
            if (generation != self._generation or request != self._request
                    or not self.preview_enabled):
                return
            self._depth_at, self._depth_age = now, age
            if age + elapsed > MAX_DEPTH_AGE_S:
                self._clear_depth('stale', 'Wall observations expired during preview calculation')
                return
            self._adjusted_course = copy.deepcopy(adjusted)
            self._adjusted_world = world
            self._adjusted_yaws_world = world_headings
            self._adjustment_status = result['status']
            self._plan_walls, self._plan_cell = walls, cell
            self._adjustment_reason = result['reason']
            self._adjustment_request = request

    def snapshot(self, now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            age = (None if self._depth_at is None or self._depth_age is None else
                   self._depth_age + max(0., now-self._depth_at))
            if age is not None and age > MAX_DEPTH_AGE_S and self.preview_enabled:
                self._clear_depth('stale', 'Wall observations are stale; adjusted preview hidden')
            original = [] if self.document is None else self.document['waypoints']
            original_indices = _display_indices(len(self._original_world))
            adjusted_indices = _display_indices(len(self._adjusted_world))
            return dict(
                revision=self.revision,
                original_version=f'{self._session}:{self._generation}',
                adjusted_version=(f'{self._session}:{self._generation}:'
                                  f'{self._adjustment_request}:{self._adjustment_status}'),
                name='' if self.document is None else self.document.get('name', 'Course route'),
                status='empty' if self.document is None else 'needs_anchor' if self.anchor is None else 'preview',
                reason=self._reason, anchor=copy.deepcopy(self.anchor),
                preview_enabled=self.preview_enabled,
                original_count=len(original), adjusted_count=len(self._adjusted_course),
                original_course=_display_points(original),
                adjusted_course=_display_points(self._adjusted_course),
                original_world_enu=_display_points(self._original_world, original_indices),
                adjusted_world_enu=_display_points(self._adjusted_world, adjusted_indices),
                original_yaw_enu_rad=[self._original_yaws_world[i] for i in original_indices],
                adjusted_yaw_enu_rad=[self._adjusted_yaws_world[i] for i in adjusted_indices],
                adjustment=dict(status=self._adjustment_status, reason=self._adjustment_reason,
                    age_s=None if age is None else round(age, 3), clearance_m=self.clearance_m,
                    max_offset_m=self.max_offset_m, execution_supported=True,
                    effective_clearance_m=max(.30, self.clearance_m)+TRACKING_RESERVE_M),
                marks=copy.deepcopy(self.marks), marks_world=self.world_marks())
