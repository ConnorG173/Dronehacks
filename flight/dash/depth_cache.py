"""Bounded display-only ML depth snapshots. This module has no flight controls."""
import json
import math
import threading
import time
import uuid

SOURCE = "Depth Anything V2"
FRAME = "px4_local_enu"
MAX_JSON_BYTES = 128 * 1024
MAX_POINTS = 1600
MAX_AGE_S = 1.5


def _pair(value):
    if not isinstance(value, list) or len(value) != 2:
        raise ValueError("coordinates must be numeric XY pairs")
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in value):
        raise ValueError("coordinates must be numeric XY pairs")
    result = tuple(float(v) for v in value)
    if not all(math.isfinite(v) for v in result):
        raise ValueError("coordinates must be finite")
    return result


def _empty(status, reason):
    return dict(source=SOURCE, status=status, reason=reason, cell_m=.08,
                points=(), camera_xy=None, view_polygon=())


class DepthOverlayCache:
    def __init__(self):
        self._lock = threading.Lock()
        self._session = uuid.uuid4().hex
        self._state = _empty("waiting", "Waiting for ML depth")
        self._received_at = 0.0
        self._age_at_receipt = None
        self._stamp_ns = None

    def update(self, raw_json, now_mono=None, now_wall_ns=None):
        """Accept a worker JSON message; invalid input clears the overlay.

        Image capture time sets age. Worker publication time is deliberately
        ignored, so publishing an old frame again cannot make it fresh.
        """
        now = time.monotonic() if now_mono is None else now_mono
        wall_ns = time.time_ns() if now_wall_ns is None else now_wall_ns
        try:
            if (not isinstance(raw_json, str) or len(raw_json) > MAX_JSON_BYTES
                    or len(raw_json.encode("utf-8")) > MAX_JSON_BYTES):
                raise ValueError("depth message exceeds 128 KiB or is not JSON text")
            data = json.loads(raw_json)
            if not isinstance(data, dict):
                raise ValueError("depth message must be an object")
            if data.get("source") != SOURCE or data.get("frame") != FRAME:
                raise ValueError("depth source or coordinate frame is incorrect")
            status = data.get("status")
            if status not in ("live", "waiting", "unscaled", "error"):
                raise ValueError("unknown depth status")
            reason = data.get("reason", "")
            if not isinstance(reason, str):
                raise ValueError("depth reason must be text")
            cell = data.get("cell_m", .08)
            if (isinstance(cell, bool) or not isinstance(cell, (int, float))
                    or not math.isfinite(cell) or not .02 <= cell <= .5):
                raise ValueError("depth cell size must be 0.02 to 0.5 m")
            points = data.get("points", [])
            polygon = data.get("view_polygon", [])
            if not isinstance(points, list) or len(points) > MAX_POINTS:
                raise ValueError("depth message must contain at most 1600 points")
            if not isinstance(polygon, list) or len(polygon) > 16:
                raise ValueError("depth view polygon must contain at most 16 points")
            points = tuple(_pair(p) for p in points)
            # Optional per-entry world z range [lo, hi] (height slabs). Older workers omit it.
            z_ranges = data.get("z_ranges")
            if z_ranges is not None:
                if not isinstance(z_ranges, list) or len(z_ranges) != len(points):
                    raise ValueError("z_ranges must match the points")
                z_ranges = tuple(_pair(z) for z in z_ranges)
                if any(lo > hi for lo, hi in z_ranges):
                    raise ValueError("z_ranges must be [low, high]")
            polygon = tuple(_pair(p) for p in polygon)
            camera = data.get("camera_xy")
            camera = None if camera is None else _pair(camera)
            stamp = data.get("stamp_ns")
            if stamp is None:
                if status == "live":
                    raise ValueError("live depth requires an image timestamp")
                age = None
            else:
                if isinstance(stamp, bool) or not isinstance(stamp, int) or stamp <= 0:
                    raise ValueError("image timestamp must be positive integer nanoseconds")
                age = (wall_ns - stamp) / 1e9
                if not math.isfinite(age) or age < -.05:
                    raise ValueError("image timestamp is in the future")
                age = max(0.0, age)
            state = dict(source=SOURCE, status=status, reason=reason[:1024], cell_m=float(cell),
                         points=points, camera_xy=camera, view_polygon=polygon)
            if z_ranges is not None:
                state["z_ranges"] = z_ranges
        except (ValueError, TypeError, OverflowError, UnicodeError, RecursionError) as exc:
            with self._lock:
                self._state = _empty("error", "Invalid depth message: " + str(exc)[:180])
                self._received_at, self._age_at_receipt, self._stamp_ns = now, None, None
            return False
        with self._lock:
            if stamp is not None and stamp == self._stamp_ns and self._age_at_receipt is not None:
                # Preserve age even if wall time steps backwards between repeats.
                age = max(age, self._age_at_receipt + max(0.0, now - self._received_at))
            self._state = state
            self._received_at, self._age_at_receipt, self._stamp_ns = now, age, stamp
        return True

    def snapshot(self, now_mono=None):
        """Return independent JSON-serializable display data, never readiness flags."""
        now = time.monotonic() if now_mono is None else now_mono
        with self._lock:
            state = dict(self._state)
            stamp = self._stamp_ns
            age = (None if self._age_at_receipt is None else
                   self._age_at_receipt + max(0.0, now - self._received_at))
        if state["status"] == "live" and age > MAX_AGE_S:
            state["status"] = "stale"
            state["reason"] = "Waiting for a fresh depth frame"
        state["age_s"] = None if age is None else round(age, 3)
        state["geometry_version"] = f'{self._session}:{stamp or 0}:{state["status"]}'
        if state["status"] != "live":
            state.update(points=[], camera_xy=None, view_polygon=[])
            state.pop("z_ranges", None)
        else:
            state["points"] = [list(p) for p in state["points"]]
            if "z_ranges" in state:
                state["z_ranges"] = [list(z) for z in state["z_ranges"]]
            state["view_polygon"] = [list(p) for p in state["view_polygon"]]
            camera = state["camera_xy"]
            state["camera_xy"] = None if camera is None else list(camera)
        return state
