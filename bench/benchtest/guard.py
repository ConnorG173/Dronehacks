"""bench.guard - the software safety envelope for tethered tests. PURE: no ROS, no clock reads.

There is no RC transmitter, so this guard (on the Jetson) and PX4's own failsafes are the
only automatic protection.  It answers one question every control tick:

    OK    keep going
    LAND  something is wrong but the vehicle is controllable: land where it is
    KILL  the vehicle is out of control or escaping: force-disarm (motors stop instantly)

KILL is only for clear loss of control (hard fence / ceiling, extreme tilt).  On a short
tether at low height, dropping the vehicle is the lesser harm; at real altitude it is not,
which is why these limits are sized for the tether rig and not for free flight.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

OK, LAND, KILL = "ok", "land", "kill"


@dataclass
class Envelope:
    fence_xy_m: float = 0.5          # horizontal distance from the takeoff point: land beyond this
    ceiling_m: float = 1.0           # height above the takeoff point: land above this
    hard_margin_m: float = 0.3       # beyond fence + this, or ceiling + this: kill
    tilt_land_deg: float = 30.0
    tilt_kill_deg: float = 60.0
    track_err_m: float = 0.35        # |setpoint - position| larger than this ...
    track_err_s: float = 0.4         # ... for this long: land
    telemetry_stale_s: float = 0.5   # PX4 position / status older than this: land
    health_bad_s: float = 0.5        # estimator unhealthy this long: land (None health = not required)
    battery_land_frac: float = 0.20  # remaining capacity below this: land (ignored if PX4 reports < 0)
    max_flight_s: float = 120.0      # airborne longer than this: land


@dataclass
class State:
    """Latest vehicle state, as the ROS layer last saw it (times on the guard's clock)."""
    pos: tuple | None = None         # NED (m)
    vel: tuple | None = None
    yaw: float | None = None
    tilt_deg: float | None = None
    armed: bool | None = None
    nav_state: int | None = None
    landed: bool | None = None
    pos_ok: bool = False
    health: bool | None = None       # estimator health topic; None = not received
    battery_frac: float | None = None
    t_pos: float | None = None
    t_status: float | None = None
    t_health: float | None = None


@dataclass
class Guard:
    env: Envelope = field(default_factory=Envelope)
    require_health: bool = True
    _err_since: float | None = None
    _bad_health_since: float | None = None

    def reset(self):
        self._err_since = self._bad_health_since = None

    def check(self, now, st: State, origin, setpoint_p=None, t_airborne=None):
        """-> (OK | LAND | KILL, reason).  origin = (x, y, z) of the takeoff point (NED)."""
        e = self.env
        if st.t_pos is None or now - st.t_pos > e.telemetry_stale_s:
            return LAND, "PX4 position telemetry stale"
        if st.t_status is None or now - st.t_status > e.telemetry_stale_s:
            return LAND, "PX4 status telemetry stale"
        if not st.pos_ok or st.pos is None or not all(math.isfinite(c) for c in st.pos):
            return LAND, "PX4 local position invalid"

        dx, dy = st.pos[0] - origin[0], st.pos[1] - origin[1]
        horiz = math.hypot(dx, dy)
        height = origin[2] - st.pos[2]
        tilt = st.tilt_deg if st.tilt_deg is not None else 0.0
        if horiz > e.fence_xy_m + e.hard_margin_m:
            return KILL, f"hard fence: {horiz:.2f} m from takeoff (limit {e.fence_xy_m + e.hard_margin_m:.2f})"
        if height > e.ceiling_m + e.hard_margin_m:
            return KILL, f"hard ceiling: {height:.2f} m up (limit {e.ceiling_m + e.hard_margin_m:.2f})"
        if tilt > e.tilt_kill_deg:
            return KILL, f"tilt {tilt:.0f} deg (kill limit {e.tilt_kill_deg:.0f})"

        if horiz > e.fence_xy_m:
            return LAND, f"fence: {horiz:.2f} m from takeoff (limit {e.fence_xy_m:.2f})"
        if height > e.ceiling_m:
            return LAND, f"ceiling: {height:.2f} m up (limit {e.ceiling_m:.2f})"
        if tilt > e.tilt_land_deg:
            return LAND, f"tilt {tilt:.0f} deg (limit {e.tilt_land_deg:.0f})"

        if self.require_health:
            fresh = st.t_health is not None and now - st.t_health <= e.telemetry_stale_s * 2
            bad = not fresh or st.health is not True
            if bad:
                self._bad_health_since = self._bad_health_since if self._bad_health_since is not None else now
                if now - self._bad_health_since >= e.health_bad_s:
                    return LAND, "estimator unhealthy" if fresh else "estimator health topic silent"
            else:
                self._bad_health_since = None

        if st.battery_frac is not None and 0.0 <= st.battery_frac < e.battery_land_frac:
            return LAND, f"battery {st.battery_frac * 100:.0f}% (limit {e.battery_land_frac * 100:.0f}%)"

        if setpoint_p is not None:
            err = math.dist(setpoint_p, st.pos)
            if err > e.track_err_m:
                self._err_since = self._err_since if self._err_since is not None else now
                if now - self._err_since >= e.track_err_s:
                    return LAND, f"tracking error {err:.2f} m (limit {e.track_err_m:.2f})"
            else:
                self._err_since = None
        else:
            self._err_since = None

        if t_airborne is not None and now - t_airborne > e.max_flight_s:
            return LAND, f"flight time limit {e.max_flight_s:.0f} s"
        return OK, ""


def tilt_from_q(q):
    """Tilt (deg) of the body z axis from vertical, from PX4's quaternion (w, x, y, z)."""
    w, x, y, z = q
    # R[2][2] of the body->NED rotation = cos(tilt)
    c = 1.0 - 2.0 * (x * x + y * y)
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def yaw_from_q(q):
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
