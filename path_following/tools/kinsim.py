#!/usr/bin/env python3
"""Offline stand-in for flight interface + PX4 velocity loop + VIO, to run the
mission/tracker code on a laptop in seconds.

Models (defaults from the drone's PX4 parameter files, see the proposal section 1):
  flight interface : latest twist, norm-clamped to max_speed_mps, zero-order held at
                     setpoint_rate_hz; a twist older than 1 s -> hold current position
  PX4              : velocity PID (MPC_XY_VEL_*_ACC, MPC_Z_VEL_*_ACC), tilt limit,
                     first-order attitude lag, linear drag; land detector; auto-disarm
  VIO              : pose at 30 Hz with latency and noise, in an odom frame offset from
                     map by the launch pose (+ optional drift)
The mission code sees only what the real node sees: odometry, a low-latency velocity
(EKF2), offboard/armed/landed flags.

    python3 kinsim.py COURSE.yaml PATH.bin [--speed-scale K] [--plot out.png] [--json]
"""
import argparse
import json
import math
import os
import sys
from dataclasses import dataclass, asdict, field

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'team_path_following'))

from team_path_following import frames  # noqa: E402
from team_path_following.course import load_course, read_dhmap, score_checkpoints  # noqa: E402
from team_path_following.clearance import ClearanceMap  # noqa: E402
from team_path_following.mission import Mission, MissionParams, Inputs, COURSE, LANDING, DONE, ABORT  # noqa: E402
from team_path_following.pathfile import Trajectory  # noqa: E402
from team_path_following.tracker import TrackerParams  # noqa: E402

G = 9.81


@dataclass
class SimParams:
    dt: float = 0.004                 # physics step
    control_hz: float = 50.0          # our node
    setpoint_rate_hz: float = 10.0    # flight interface
    max_speed_mps: float = 1.5
    twist_timeout_s: float = 1.0
    # PX4 velocity loop (fleet params)
    xy_p: float = 1.8
    xy_i: float = 0.4
    xy_d: float = 0.2
    z_p: float = 4.0
    z_i: float = 2.0
    tilt_max_deg: float = 45.0
    att_tau: float = 0.08             # thrust-vector response
    drag: float = 0.25                # 1/s linear drag
    yaw_tau: float = 0.15
    takeoff_ramp_s: float = 3.0       # MPC_TKO_RAMP_T: vertical setpoint ramps in after liftoff
    ekf_vel_delay_s: float = 0.0      # PX4 controls on a delayed velocity estimate (VIO-derived EKF2)
    ekf_vel_scale: float = 1.0        # EKF2 velocity = scale x true (VIO scale error); <1: flies faster than told
    link_delay_s: float = 0.02        # DDS + uXRCE to PX4
    # VIO
    odom_hz: float = 30.0
    odom_latency_s: float = 0.06
    odom_noise_m: float = 0.01
    odom_drift_mps: float = 0.0       # random-walk drift rate of odom vs truth
    vel_noise_mps: float = 0.02
    yaw_residual_deg: float = 0.0     # odom vs PX4-local heading mismatch
    # vehicle
    radius_m: float = 0.20            # body + props half-span for touch counting
    seed: int = 0
    t_max: float = 120.0
    clearance_slow_m: float = 0.0     # node's clearance slow-down (0 = off), as mission_tracker
    clearance_stop_m: float = 0.0


class Px4Model:

    def __init__(self, sp, start_map, yaw0):
        self.sp = sp
        self.p = np.array(start_map, float)
        self.v = np.zeros(3)
        self.a = np.zeros(3)
        self.yaw = float(yaw0)
        self.yaw_rate = 0.0
        self.i = np.zeros(3)
        self.armed = True
        self.landed = True
        self.t_landed = 0.0
        self.flew = False
        self.ground_z = float(start_map[2])
        self._vhist = []

    def step(self, dt, v_sp, yaw_rate_sp, t):
        sp = self.sp
        if self.landed and self.flew and t - self.t_landed > 2.0:   # COM_DISARM_LAND
            self.armed = False
        if not self.armed:
            self.v[:] = 0.0
            return
        if self.landed and v_sp[2] <= 0.05:
            self.i[:] = 0.0
            self.v[:] = 0.0
            self.t_takeoff = None
            return
        if self.landed:
            if getattr(self, 't_takeoff', None) is None:
                self.t_takeoff = t
            ramp = min(1.0, (t - self.t_takeoff) / max(sp.takeoff_ramp_s, 1e-3))
            v_sp = np.array([v_sp[0] * ramp, v_sp[1] * ramp, v_sp[2] * ramp])
        elif getattr(self, 't_takeoff', None) is not None and t - self.t_takeoff < sp.takeoff_ramp_s:
            ramp = (t - self.t_takeoff) / sp.takeoff_ramp_s
            v_sp = np.array([v_sp[0], v_sp[1], v_sp[2] * ramp if v_sp[2] > 0 else v_sp[2]])
        self._vhist.append(self.v.copy())
        nd = int(round(sp.ekf_vel_delay_s / dt))
        v_est = self._vhist[-1 - nd] if len(self._vhist) > nd else self._vhist[0]
        if len(self._vhist) > 200:
            del self._vhist[:100]
        v_est = v_est * np.array([sp.ekf_vel_scale, sp.ekf_vel_scale, 1.0])
        e = v_sp - v_est
        kp = np.array([sp.xy_p, sp.xy_p, sp.z_p])
        ki = np.array([sp.xy_i, sp.xy_i, sp.z_i])
        self.i += ki * e * dt
        self.i = np.clip(self.i, -3.0, 3.0)
        acc = kp * e + self.i
        acc[:2] -= sp.xy_d * self.a[:2]   # D on the (true) acceleration
        amax = G * math.tan(math.radians(sp.tilt_max_deg))
        n = np.linalg.norm(acc[:2])
        if n > amax:
            acc[:2] *= amax / n
        acc[2] = np.clip(acc[2], -G * 0.8, G)
        self.a += (acc - self.a) * (dt / sp.att_tau)
        self.v += (self.a - sp.drag * self.v) * dt
        self.p += self.v * dt
        self.yaw_rate += (yaw_rate_sp - self.yaw_rate) * (dt / sp.yaw_tau)
        self.yaw = float(frames.wrap_pi(self.yaw + self.yaw_rate * dt))
        if self.p[2] > self.ground_z + 0.05:
            self.landed = False
            self.flew = True
        if self.p[2] <= self.ground_z:
            self.p[2] = self.ground_z
            self.v[2] = max(self.v[2], 0.0)
            if self.flew and not self.landed and np.linalg.norm(self.v) < 0.25:
                self.landed = True
                self.t_landed = t


def path_distance(traj, p, tau, window=3.0):
    lo = np.searchsorted(traj.t, tau - window)
    hi = np.searchsorted(traj.t, tau + window) + 1
    seg = traj.p[lo:hi]
    return float(np.min(np.linalg.norm(seg - p, axis=1)))


def run(course, traj, tracker_params=None, mission_params=None, sim=None, clearance=None,
        record=False, return_traj=None, return_course=None):
    sp = sim or SimParams()
    rng = np.random.default_rng(sp.seed)
    tp = tracker_params or TrackerParams(v_max=sp.max_speed_mps - 0.1)
    mission = Mission(traj, pad_z=float(course.start_pos[2]), tracker_params=tp,
                      params=mission_params or MissionParams(), return_traj=return_traj,
                      return_pad_z=float(return_course.start_pos[2]) if return_course is not None else None)
    if clearance is None:
        clearance = ClearanceMap(read_dhmap(course.map_file), course.map_bound)

    t_map_base0 = frames.map_from_launch(course)
    # odom = the frame cuVSLAM started in: base pose at power-up == launch pose
    t_map_odom_true = t_map_base0
    px4 = Px4Model(sp, course.start_pos, course.start_heading_rad)
    yaw_res = math.radians(sp.yaw_residual_deg)

    odom_queue = []          # (t_available, stamp, p_odom, yaw_odom)
    latest_odom = None
    t_map_odom_est = None    # node's fallback map->odom, captured at go
    twist_latest = None      # (t_rx, v_odom, yaw_rate)
    px4_sp = (np.zeros(3), 0.0)
    pending_sp = []          # link delay
    drift = np.zeros(3)

    ctrl_dt = 1.0 / sp.control_hz
    next_ctrl = next_sp = next_odom = 0.0
    offboard = False
    t_offboard = 0.5          # pilot "go" shortly after start
    log = []
    cp_pts = []                # positions during the course leg, scored after the run
    min_clear = float('inf')
    touches = 0
    touching = False
    clamped = 0
    first_land_pos = None
    max_xt = 0.0
    xt_sq = []
    t = 0.0
    while t < sp.t_max:
        # VIO
        if t >= next_odom:
            next_odom += 1.0 / sp.odom_hz
            drift += rng.normal(0, sp.odom_drift_mps * math.sqrt(1.0 / sp.odom_hz), 3) * np.array([1, 1, 0.3])
            p_meas = px4.p + drift + rng.normal(0, sp.odom_noise_m, 3)
            p_odom = t_map_odom_true.inverse().apply(p_meas)
            yaw_odom = float(frames.wrap_pi(px4.yaw - t_map_odom_true.yaw))
            odom_queue.append((t + sp.odom_latency_s, t, p_odom, yaw_odom))
        while odom_queue and odom_queue[0][0] <= t:
            latest_odom = odom_queue.pop(0)

        if not offboard and t >= t_offboard and twist_latest is not None and px4.landed and px4.armed:
            offboard = True
            if latest_odom is not None:
                # node: fallback map->odom from the current odom pose and the launch pose
                _, _, po, yo = latest_odom
                t_map_odom_est = frames.map_odom_at_go(t_map_base0, frames.Pose2(*po, yo))

        # our node
        if t >= next_ctrl:
            next_ctrl += ctrl_dt
            pos = vel = yaw = None
            age = 1e9
            if latest_odom is not None and t_map_odom_est is not None:
                _, stamp, po, yo = latest_odom
                age = t - stamp
                vel_odom = t_map_odom_true.inverse().apply_vec(px4.v) + rng.normal(0, sp.vel_noise_mps, 3)
                po_pred = po + vel_odom * age              # latency compensation
                pos = t_map_odom_est.apply(po_pred)
                vel = t_map_odom_est.apply_vec(vel_odom)
                yaw = float(frames.wrap_pi(yo + t_map_odom_est.yaw))
                age = 0.0 if age < 0.2 else age
            out = mission.step(ctrl_dt, Inputs(
                now=t, offboard=offboard, armed=px4.armed, landed=px4.landed,
                pos=pos, vel=vel, yaw=yaw, pose_age=age,
                clearance_scale=(float(np.squeeze(clearance.speed_scale(pos[None], sp.clearance_slow_m, sp.clearance_stop_m)))
                                 if sp.clearance_slow_m > 0.0 and pos is not None else 1.0)))
            if t_map_odom_est is not None:
                v_odom = t_map_odom_est.inverse().apply_vec(out.v)
            else:
                v_odom = np.zeros(3)
            twist_latest = (t, v_odom, out.yaw_rate)
            if record:
                d = out.debug
                log.append([t, *px4.p, *px4.v, d.get('tau', np.nan), d.get('taudot', np.nan),
                            d.get('e_cross', np.nan), *(d.get('p_ref', [np.nan] * 3)), *out.v,
                            ['idle', 'takeoff', 'course', 'landing', 'landed', 'done', 'abort', 'return'].index(out.state)])
            if mission.state in (DONE, ABORT):
                break

        # flight interface tick
        if t >= next_sp:
            next_sp += 1.0 / sp.setpoint_rate_hz
            if twist_latest is not None and t - twist_latest[0] < sp.twist_timeout_s:
                v = np.array(twist_latest[1])
                n = np.linalg.norm(v)
                if n > sp.max_speed_mps:
                    v *= sp.max_speed_mps / n
                    clamped += 1
                # PX4-local frame = odom rotated by the residual heading error
                c, s = math.cos(yaw_res), math.sin(yaw_res)
                v_loc = np.array([c * v[0] - s * v[1], s * v[0] + c * v[1], v[2]])
                v_map = t_map_odom_true.apply_vec(v_loc)
                pending_sp.append((t + sp.link_delay_s, v_map, twist_latest[2]))
            else:
                pending_sp.append((t + sp.link_delay_s, -0.5 * px4.v, 0.0))  # hold ~ brake
        while pending_sp and pending_sp[0][0] <= t:
            _, vsp, yr = pending_sp.pop(0)
            px4_sp = (vsp, yr)

        px4.step(sp.dt, px4_sp[0] if offboard else np.zeros(3), px4_sp[1] if offboard else 0.0, t)

        # metrics
        if first_land_pos is None and mission.state == 'landed':
            first_land_pos = px4.p.copy()
        if mission.state == COURSE and mission.leg == 0:
            c = clearance.distance(px4.p)
            min_clear = min(min_clear, c)
            if c < sp.radius_m and not touching:
                touches += 1
            touching = c < sp.radius_m
            xt = path_distance(traj, px4.p, mission.tracker.tau)
            max_xt = max(max_xt, xt)
            xt_sq.append(xt * xt)
        if mission.state in (COURSE, LANDING) and mission.leg == 0:
            cp_pts.append(px4.p.copy())
        t += sp.dt

    ev = {}
    for tt, s_, _ in mission.events:   # first occurrence of each state
        ev.setdefault(s_, tt)
    landed_all = [tt for tt, s_, _ in mission.events if s_ == 'landed']
    t_course = ev.get(LANDING, np.nan) - ev.get(COURSE, np.nan)
    land_err = float(np.linalg.norm((first_land_pos if first_land_pos is not None else px4.p)[:2] - traj.p[-1][:2]))
    cp_passed, cp_miss = score_checkpoints(cp_pts, course.checkpoints)
    res = dict(
        result=mission.state,
        course_time_s=round(float(t_course), 2),
        mission_time_s=round(float(ev.get('landed', np.nan) - t_offboard), 2),
        plan_time_s=round(traj.duration, 2),
        max_xt_m=round(max_xt, 3),
        rms_xt_m=round(float(np.sqrt(np.mean(xt_sq))) if xt_sq else float('nan'), 3),
        min_clear_m=round(min_clear, 3),
        plan_min_clear_m=round(float(np.min(clearance.distance(traj.p))), 3),
        touches=touches,
        checkpoints=f'{sum(cp_passed)}/{len(course.checkpoints)}',
        cp_missed=[i + 1 for i, ok in enumerate(cp_passed) if not ok],
        cp_worst_miss_m=round(max(cp_miss), 3) if cp_miss else 0.0,
        clamped_ticks=clamped,
        land_err_m=round(land_err, 3),
        disarmed=not px4.armed,
    )
    if return_traj is not None:
        res['return_time_s'] = round(float(landed_all[-1] - ev['return']), 2) if 'return' in ev and len(landed_all) > 1 else None
        res['total_time_s'] = round(float(landed_all[-1] - t_offboard), 2) if len(landed_all) > 1 else None
        res['return_land_err_m'] = round(float(np.linalg.norm(px4.p[:2] - return_traj.p[-1][:2])), 3)
    if record:
        res['log'] = np.array(log)
    return res


def plot(course, traj, res, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    pts = read_dhmap(course.map_file)
    lg = res['log']
    fig, ax = plt.subplots(1, 2, figsize=(14, 6))
    band = pts[(pts[:, 2] > 0.6) & (pts[:, 2] < 1.4)]
    ax[0].plot(band[:, 0], band[:, 1], ',', color='0.6')
    ax[0].plot(traj.p[:, 0], traj.p[:, 1], 'b-', lw=1, label='plan')
    ax[0].plot(lg[:, 1], lg[:, 2], 'r-', lw=1, label='flown')
    for b in course.checkpoints:
        ax[0].add_patch(plt.Rectangle(b.center[:2] - b.half[:2], *(2 * b.half[:2]), fill=False, color='g'))
    ax[0].set_aspect('equal')
    ax[0].legend()
    m = lg[:, -1] == 2
    ax[1].plot(lg[m, 0], lg[m, 9], label='cross-track (m)')
    ax[1].plot(lg[m, 0], lg[m, 8], label='taudot')
    ax[1].plot(lg[m, 0], np.linalg.norm(lg[m, 4:7], axis=1), label='speed (m/s)')
    ax[1].legend()
    ax[1].grid(True)
    fig.suptitle(json.dumps({k: v for k, v in res.items() if k != 'log'}), fontsize=7)
    fig.savefig(path, dpi=110)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('course')
    ap.add_argument('path')
    ap.add_argument('--speed-scale', type=float, default=1.0, help='slow the plan by K (>1) or speed it up')
    ap.add_argument('--set', action='append', default=[], help='tracker/sim/mission param, e.g. kc=1.5 or sim.setpoint_rate_hz=50')
    ap.add_argument('--plot')
    ap.add_argument('--return-course', help='return-leg config (enables the touch-and-go return)')
    ap.add_argument('--return-path', help='return-leg DHPATH file')
    ap.add_argument('--json', action='store_true')
    ap.add_argument('--npz', help='save t, u (commanded), v (true), state for fit_lag.py')
    a = ap.parse_args()
    course = load_course(a.course)
    traj = Trajectory.load(a.path, course)
    if a.speed_scale != 1.0:
        traj = traj.time_scaled(a.speed_scale)
    sim = SimParams()
    tp = TrackerParams(v_max=sim.max_speed_mps - 0.1)
    mp = MissionParams()
    for kv in a.set:
        k, v = kv.split('=', 1)
        target = tp
        if k.startswith('sim.'):
            target, k = sim, k[4:]
        elif k.startswith('mission.'):
            target, k = mp, k[8:]
        cur = getattr(target, k)
        setattr(target, k, type(cur)(v) if not isinstance(cur, str) else v)
    if not any(kv.startswith('v_max') for kv in a.set):
        tp.v_max = sim.max_speed_mps - 0.1
    rc = load_course(a.return_course) if a.return_course else None
    rt = Trajectory.load(a.return_path, rc) if rc is not None else None
    res = run(course, traj, tp, mp, sim, record=bool(a.plot or a.npz), return_traj=rt, return_course=rc)
    if a.npz:
        lg = res['log']
        np.savez(a.npz, t=lg[:, 0], v=lg[:, 4:7], u=lg[:, 13:16], state=lg[:, -1])
    if a.plot:
        plot(course, traj, res, a.plot)
    res.pop('log', None)
    print(json.dumps(res) if a.json else '\n'.join(f'{k:16s} {v}' for k, v in res.items()))


if __name__ == '__main__':
    main()
