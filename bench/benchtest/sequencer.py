"""bench.sequencer - takeoff, run one profile, land, with the guard watching. PURE: no ROS.

    wait -> prime -> activate -> climb -> settle -> run -> hold -> land -> done
                                  any airborne phase --guard LAND / operator--> land
                                  any phase        --guard KILL / operator--> kill -> done
                     PX4 leaves Offboard or disarms on its own ---------------> yielded / aborted

Rules taken from the existing flight code (onboard/examples/takeoff_land.py, refnav/core.py):
* stream setpoints for a while before requesting Offboard and arming;
* never fight PX4: if it leaves Offboard (failsafe, another program) we stop streaming;
* once PX4 is in AUTO_LAND we stop streaming (a setpoint stream blocks its land detector);
* we never send a normal disarm in the air; only KILL force-disarms.

The caller feeds `step(now, state)` at a fixed rate and executes `Output.commands`:
    "offboard", "arm", "land", "disarm", "kill"
and publishes `Output.setpoint` (PX4 local NED) whenever it is not None.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from .guard import KILL, LAND, Guard, State
from .profiles import Setpoint, to_ned

NAV_DESCEND, NAV_OFFBOARD, NAV_AUTO_LAND = 12, 14, 18
LANDING_NAV = (NAV_AUTO_LAND, NAV_DESCEND)
AIRBORNE = ("climb", "settle", "run", "hold")
FINAL = ("done", "aborted", "yielded")


@dataclass
class FlightParams:
    height_m: float = 0.4
    climb_rate_mps: float = 0.3
    climb_tol_m: float = 0.08
    settle_s: float = 3.0
    hold_s: float = 2.0
    prime_s: float = 1.5
    wait_s: float = 30.0
    arm_timeout_s: float = 8.0
    land_timeout_s: float = 30.0
    arm_min_battery_frac: float = 0.5
    cmd_resend_s: float = 1.0


@dataclass
class Output:
    phase: str
    setpoint: Setpoint | None = None      # PX4 local NED; None = do not stream
    commands: list = field(default_factory=list)
    events: list = field(default_factory=list)


class FlightSequencer:
    def __init__(self, profile, params: FlightParams, guard: Guard):
        self.profile, self.p, self.guard = profile, params, guard
        self.phase, self.t_phase = "wait", None
        self.result = None
        self.why_not_ready = ""
        self.origin = self.yaw0 = self.hover = None
        self.t_airborne = self.t_run = None
        self.last_sp: Setpoint | None = None
        self.t_cmd = None
        self.auto_land_seen = False
        self.landed_since = None
        self.land_request = self.kill_request = None

    # ------------------------------------------------------------ operator
    def request_land(self, why="operator"):
        if self.land_request is None:
            self.land_request = why

    def request_kill(self, why="operator"):
        if self.kill_request is None:
            self.kill_request = why

    # ------------------------------------------------------------ helpers
    def _set(self, phase, now, out, why=""):
        out.events.append(f"{self.phase} -> {phase}" + (f": {why}" if why else ""))
        self.phase, self.t_phase, self.t_cmd = phase, now, None
        out.phase = phase

    def _resend(self, now):
        if self.t_cmd is None or now - self.t_cmd >= self.p.cmd_resend_s:
            self.t_cmd = now
            return True
        return False

    def _ready(self, now, st: State):
        g, e = self.guard, self.guard.env
        if st.t_pos is None or now - st.t_pos > e.telemetry_stale_s:
            return "no fresh PX4 local position"
        if st.t_status is None or now - st.t_status > e.telemetry_stale_s:
            return "no fresh PX4 vehicle status"
        if not st.pos_ok:
            return "PX4 local position / heading not valid"
        if st.armed:
            return "vehicle already armed"
        if st.landed is False:
            return "PX4 does not report landed"
        if g.require_health and (st.health is not True or st.t_health is None
                                 or now - st.t_health > 2 * e.telemetry_stale_s):
            return "estimator health not reported healthy"
        if st.battery_frac is not None and 0.0 <= st.battery_frac < self.p.arm_min_battery_frac:
            return f"battery {st.battery_frac * 100:.0f}% < {self.p.arm_min_battery_frac * 100:.0f}% needed to arm"
        return ""

    def _hold_here(self, st):
        """A hold setpoint at the current position (used when landing is requested)."""
        yaw = self.last_sp.yaw if self.last_sp else (st.yaw or 0.0)
        z = self.last_sp.p[2] if self.last_sp else st.pos[2]
        return Setpoint((st.pos[0], st.pos[1], z), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), yaw, 0.0)

    def _start_land(self, now, out, st, why):
        self.result = self.result or why
        self.auto_land_seen = False
        self.landed_since = None
        if st.pos is not None:
            self.last_sp = self._hold_here(st)
        self._set("land", now, out, why)

    # ------------------------------------------------------------ main
    def step(self, now, st: State) -> Output:
        out = Output(self.phase)
        if self.t_phase is None:
            self.t_phase = now
        if self.phase in FINAL:
            return out

        # operator kill and guard kill take priority over everything, in every phase
        if self.kill_request is not None and self.phase != "kill":
            if st.armed:
                self.result = f"killed: {self.kill_request}"
                self._set("kill", now, out, self.kill_request)
            else:
                self.result = f"stopped before arming: {self.kill_request}"
                self._set("aborted", now, out, self.kill_request)
                return out
        if self.phase == "kill":
            if st.armed is False:
                self._set("done", now, out, "disarmed")
            else:
                out.commands.append("kill")       # every tick until PX4 reports disarmed
            return out

        armed_phase = self.phase in AIRBORNE or (self.phase == "activate" and st.armed)
        if armed_phase or (self.phase == "land" and st.armed):
            action, why = self.guard.check(now, st, self.origin, self.last_sp.p if (
                self.last_sp and self.phase in AIRBORNE) else None, self.t_airborne)
            if action == KILL:
                self.result = f"killed: {why}"
                self._set("kill", now, out, why)
                out.commands.append("kill")
                return out
            if action == LAND and self.phase != "land":
                self._start_land(now, out, st, why)

        if self.land_request is not None and self.phase in ("wait", "prime"):
            self.result = f"stopped before arming: {self.land_request}"
            self._set("aborted", now, out, self.land_request)
            return out
        if self.land_request is not None and self.phase in AIRBORNE + ("activate",):
            self._start_land(now, out, st, f"operator: {self.land_request}")

        # PX4 took over or disarmed on its own: do not fight it
        if self.phase in AIRBORNE:
            if st.armed is False:
                self.result = "PX4 disarmed in flight"
                self._set("aborted", now, out, "PX4 disarmed the vehicle (failsafe?) - inspect it")
                return out
            if st.nav_state != NAV_OFFBOARD:
                self.result = f"PX4 left Offboard (nav_state {st.nav_state})"
                self._set("yielded", now, out, f"PX4 left Offboard (nav_state {st.nav_state}); not fighting it")
                return out

        p, ph = self.p, self.phase
        if ph == "wait":
            why = self._ready(now, st)
            self.why_not_ready = why
            if not why:
                self.origin, self.yaw0 = tuple(st.pos), float(st.yaw)
                self.last_sp = Setpoint(self.origin, yaw=self.yaw0)
                self._set("prime", now, out, f"origin {_fmt(self.origin)} yaw {math.degrees(self.yaw0):.0f} deg")
            elif now - self.t_phase > p.wait_s:
                self.result = f"not ready: {why}"
                self._set("aborted", now, out, f"not ready after {p.wait_s:.0f} s: {why}")
            return out

        if ph == "prime":
            why = self._ready(now, st)
            if why:
                self._set("wait", now, out, f"lost readiness: {why}")
                return out
            out.setpoint = self.last_sp
            if now - self.t_phase >= p.prime_s:
                out.commands += ["offboard", "arm"]
                self._set("activate", now, out, "requested Offboard + arm")
                self.t_cmd = now
            return out

        if ph == "activate":
            out.setpoint = self.last_sp
            if st.armed and st.nav_state == NAV_OFFBOARD:
                self.t_airborne = now
                self._set("climb", now, out, "armed in Offboard")
            elif now - self.t_phase > p.arm_timeout_s:
                if st.armed:
                    out.commands.append("disarm")   # still on the ground: a normal disarm is accepted
                self.result = "arm/Offboard timeout (check PX4 arming checks in the log / QGC)"
                self._set("aborted", now, out, self.result)
            elif self._resend(now):
                if st.nav_state != NAV_OFFBOARD:
                    out.commands.append("offboard")
                if not st.armed:
                    out.commands.append("arm")
            return out

        if ph == "climb":
            target = self.origin[2] - p.height_m
            dt = now - self.t_phase
            ramp_z = self.origin[2] - p.climb_rate_mps * dt
            if ramp_z > target:
                self.last_sp = Setpoint((self.origin[0], self.origin[1], ramp_z), (0.0, 0.0, -p.climb_rate_mps),
                                        yaw=self.yaw0)
            else:
                self.last_sp = Setpoint((self.origin[0], self.origin[1], target), yaw=self.yaw0)
                if abs(st.pos[2] - target) < p.climb_tol_m:
                    self.hover = (self.origin[0], self.origin[1], target)
                    self._set("settle", now, out, f"at {p.height_m:.2f} m")
            if ph == self.phase and dt > p.height_m / p.climb_rate_mps + 10.0:
                self._start_land(now, out, st, "could not reach height (tether too short? weight? thrust?)")
            out.setpoint = self.last_sp
            return out

        if ph == "settle":
            out.setpoint = self.last_sp
            if now - self.t_phase >= p.settle_s:
                self.t_run = now
                self._set("run", now, out, f"profile {type(self.profile).__name__}, {self.profile.duration:.1f} s")
            return out

        if ph == "run":
            t = now - self.t_run
            self.last_sp = to_ned(self.profile.sample(t), self.hover, self.yaw0)
            out.setpoint = self.last_sp
            if t >= self.profile.duration:
                self._set("hold", now, out, "profile complete")
            return out

        if ph == "hold":
            sp = self.last_sp
            self.last_sp = Setpoint(sp.p, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), sp.yaw, 0.0)
            out.setpoint = self.last_sp
            if now - self.t_phase >= p.hold_s:
                self._start_land(now, out, st, "test complete")
            return out

        if ph == "land":
            return self._land(now, st, out)
        return out

    def _land(self, now, st, out):
        p, e = self.p, self.guard.env
        if st.armed is False:
            self._set("done", now, out, "landed and disarmed")
            return out
        if st.nav_state in LANDING_NAV:
            self.auto_land_seen = True
        if st.landed:
            self.landed_since = self.landed_since if self.landed_since is not None else now
        else:
            self.landed_since = None
        status_fresh = st.t_status is not None and now - st.t_status <= e.telemetry_stale_s
        if (st.nav_state not in LANDING_NAV and st.nav_state != NAV_OFFBOARD
                and status_fresh and st.armed):
            if self.auto_land_seen or now - self.t_phase > 3.0:
                self.result = (self.result or "") + f"; PX4 switched to nav_state {st.nav_state} while landing"
                self._set("yielded", now, out, f"PX4 switched to nav_state {st.nav_state} while landing")
                return out
        # keep the Offboard heartbeat until PX4 has switched to its own landing, and only while
        # we can see PX4 at all (otherwise let its Offboard-loss failsafe take over)
        if not self.auto_land_seen and status_fresh and st.nav_state == NAV_OFFBOARD:
            out.setpoint = self.last_sp
        if self._resend(now):
            if self.landed_since is not None and now - self.landed_since >= 2.0:
                out.commands.append("disarm")    # on the ground but PX4 has not auto-disarmed yet
            elif not self.auto_land_seen or st.nav_state not in LANDING_NAV:
                out.commands.append("land")
        if now - self.t_phase > p.land_timeout_s and not out.events:
            out.events.append("landing is taking long: check the vehicle; press k to kill if it is stuck")
            self.t_phase = now
        return out


def _fmt(v):
    return "(" + ", ".join(f"{c:.2f}" for c in v) + ")"
