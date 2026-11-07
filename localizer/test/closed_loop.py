"""Offline closed loop: the Follower core, configured exactly as each adapter configures it, flying
course_v3 against a model of PX4's position controller (test/px4_model.py). Laptop only, no ROS.

  sim   sim.sim_config(approved): the core requests Offboard + arm, TrajectorySetpoint carries the
        full feedforward p, v, a at 20 Hz; local frame = PX4 local NED (north-aligned, origin off the
        pad), so T is rotated and offset; lands with NAV_LAND at the end.
  real  real.real_config(): the pilot arms + switches to Offboard; position only: the lead point q,
        re-sent by flight_interface at 10 Hz (zero-order hold); local frame = odom at the pad.

Both start landed on the pad, so WAIT -> READY -> TAKEOFF -> FOLLOW -> FINISH (-> LANDING -> DONE)
all run. Perfect state estimate, no DDS latency (see HALF_ASSED.md for the model's assumptions).

    py localizer/test/closed_loop.py [--png out.png] [--max-speed 1.0]
"""
import argparse
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(HERE, ".."), HERE]
from localizer import real, sim  # noqa: E402
from localizer.core import frames as F  # noqa: E402
from localizer.core.follower import Follower, VehicleState  # noqa: E402
from localizer.core.path import load_path, path_to_course  # noqa: E402
from px4_model import PX4PosModel  # noqa: E402

PATH = os.path.join(HERE, "..", "data", "course_v3_path.bin")
MAP = os.path.join(HERE, "..", "..", "planner", "maps", "course_v3.bin")
PAD = (-2.5146, 0.9144)
CHECKPOINTS = [   # planner/config/course_v3.yaml boxes: centre, half-extents
    (-1.9304, 2.5845, 1.0, 0.10, 0.12, 0.25), (-0.8128, 0.9715, 1.0, 0.10, 0.12, 0.25),
    (0.3048, 2.5845, 1.0, 0.10, 0.12, 0.25), (2.3114, 0.4572, 0.94, 0.25, 0.30, 0.25),
    (-0.4710, -2.2715, 0.762, 0.08, 0.08, 0.12)]
NODE_DT, FI_DT, PX4_DT = 0.05, 0.1, 0.005


def load(pad_yaw=0.0):
    return path_to_course(load_path(PATH), PAD, pad_yaw)


def run(path, mode, max_speed=1.0, model_kw=None, push=None, t_max=200.0, pilot_t=4.0):
    """Returns a dict of arrays at the 20 Hz node tick, plus the final follower."""
    if mode == "sim":
        cfg = sim.sim_config(True, max_path_speed_mps=max_speed)
        # PX4 local ENU (from north-aligned NED): spawned 0.4 m off its origin, course +x = local
        # heading 25 deg, floor at local z 0.03
        T_true = F.Transform2D(PAD[0] - 0.3, PAD[1] + 0.2, math.radians(-25))
        z_floor = 0.03
    else:
        cfg = real.real_config(max_path_speed_mps=max_speed)
        T_true, z_floor = F.Transform2D(PAD[0], PAD[1], 0.0), 0.0
    f = Follower(path, cfg)
    m = PX4PosModel(np.r_[PAD, 0.0], **(model_kw or {}))
    s = VehicleState(estimate_ok=True, landed=True, local_tilt_deg=0.4, imu_tilt_deg=0.3)
    px4_sp = None            # what PX4 is tracking: (pos, vel_ff, acc_ff) course frame
    fi_sp = None             # real: what flight_interface last received from us
    armed = offboard = auto_land = False
    arm_at = None
    log = {k: [] for k in ("t", "mode", "tau", "p", "q", "x", "v", "stall", "err")}
    t, k, follow_t0 = 0.0, 0, None
    while t < t_max:
        # ---- vehicle state as the adapter would hand it over (local frame, receipt time = t)
        xy, yaw = T_true.to_local(m.x[:2], 0.0)
        s.pos, s.yaw = np.r_[xy, m.x[2] + z_floor], yaw
        s.pose_stamp = s.status_stamp = s.landed_stamp = s.imu_stamp = t
        s.armed, s.offboard, s.auto_land = armed, offboard, auto_land
        s.landed = (not armed) or (m.x[2] < 0.02 and np.linalg.norm(m.v) < 0.05 and px4_sp is None)
        cmd = f.step(t, s)
        # ---- what the adapter sends, converted back to the course frame for the model
        to_course = lambda p: np.r_[T_true.to_course(p[:2]), p[2] - z_floor]
        rot = lambda v: np.r_[F.rot2(T_true.yaw) @ v[:2], v[2]]
        if cmd.publish:
            if mode == "sim":
                px4_sp = (to_course(cmd.p), rot(cmd.v), rot(cmd.a)) if offboard else px4_sp
            else:
                fi_sp = (to_course(cmd.q), None, None)
        if mode == "sim":
            if cmd.request_offboard_arm:
                arm_at = t + 0.3                               # PX4 accepts after a few ticks
            if arm_at is not None and t >= arm_at and not armed:
                armed = offboard = True
            if cmd.request_land and offboard:
                offboard, auto_land = False, True              # PX4 AUTO_LAND: descend at 0.7 m/s
        else:
            if f.mode == "READY" and t >= pilot_t:
                armed = offboard = True                        # the pilot
        if f.mode == "FOLLOW" and follow_t0 is None:
            follow_t0 = t
        sp_course = to_course(cmd.p) if cmd.publish else None
        q_course = to_course(cmd.q) if cmd.publish else None
        for key, val in (("t", t), ("mode", f.mode), ("tau", f.clock.tau if f.clock else 0.0),
                         ("p", sp_course if sp_course is not None else np.full(3, np.nan)),
                         ("q", q_course if q_course is not None else np.full(3, np.nan)),
                         ("x", m.x.copy()), ("v", m.v.copy()),
                         ("stall", bool(f.clock and f.clock.stalled and f.mode == "FOLLOW")),
                         ("err", f.sp.err if (f.sp is not None and f.mode == "FOLLOW") else np.nan)):
            log[key].append(val)
        # ---- PX4 until the next node tick
        for _ in range(int(round(NODE_DT / PX4_DT))):
            if mode == "real" and fi_sp is not None and (k * PX4_DT + 0.025) % FI_DT < PX4_DT:
                px4_sp = fi_sp if offboard else px4_sp
            if armed and auto_land:                            # PX4 AUTO_LAND, kinematic 0.7 m/s
                m.v[:] = (0.0, 0.0, -0.7)
                m.x[2] -= 0.7 * PX4_DT
                if m.x[2] <= 0.0:
                    m.x[2], m.v[:] = 0.0, 0.0
                    armed, px4_sp = False, None
            elif armed and px4_sp is not None:
                g = push(t - follow_t0) if (push and follow_t0 is not None) else None   # t since FOLLOW
                m.step(PX4_DT, px4_sp[0], px4_sp[1], px4_sp[2], g)
            k += 1
        t += NODE_DT
        if f.mode in ("DONE", "STOPPED"):
            break
        if mode == "real" and f.mode == "FINISH" and np.linalg.norm(m.v) < 0.05:
            break
    out = {k: np.array(v) for k, v in log.items()}
    out["follower"], out["follow_t0"] = f, follow_t0
    return out


def metrics(path, log, clear=None):
    fol = log["mode"] == "FOLLOW"
    x = log["x"][fol]
    dense = np.c_[path.xy, path.z]
    xt = np.linalg.norm(x[:, None, :] - dense[None, ::2, :], axis=2).min(axis=1)
    stall = log["stall"].astype(int)
    out = {"final_mode": str(log["mode"][-1]),
           "t_follow": float(fol.sum() * NODE_DT),
           "t_total": float(log["t"][-1]),
           "err_max": float(np.nanmax(log["err"])), "err_rms": float(np.sqrt(np.nanmean(log["err"] ** 2))),
           "xtrack_max": float(xt.max()), "xtrack_rms": float(np.sqrt(np.mean(xt ** 2))),
           "stalls": int(np.sum(np.diff(np.r_[0, stall]) == 1)), "stalled_s": float(stall.sum() * NODE_DT),
           "vmax": float(np.linalg.norm(log["v"], axis=1).max())}
    if clear is not None:
        out["wall_min_vehicle"] = float(clear(x).min())
    return out


class Clearance:
    """3-D distance (m) to the nearest CAD surface point of planner/maps/course_v3.bin, floor and
    cage net excluded. Needs scipy (optional)."""

    def __init__(self, map_file=MAP, cage_half=3.049):
        from scipy.spatial import cKDTree
        raw = open(map_file, "rb").read()
        n = int(np.frombuffer(raw, "<u8", count=1, offset=16)[0])
        pts = np.frombuffer(raw, "<f4", count=3 * n, offset=24).reshape(n, 3).astype(float)
        keep = (pts[:, 2] > 0.05) & (np.abs(pts[:, :2]).max(axis=1) < cage_half - 0.02)
        self.pts = pts[keep]
        self.tree = cKDTree(self.pts)

    def __call__(self, xyz):
        return self.tree.query(np.atleast_2d(xyz))[0]


COLS = ("final_mode", "t_follow", "t_total", "err_max", "err_rms", "xtrack_max", "xtrack_rms", "stalls",
        "stalled_s", "vmax", "wall_min_vehicle")


def row(name, mm):
    cells = []
    for c in COLS:
        v = mm.get(c, float("nan"))
        cells.append(f"{v:>10}" if isinstance(v, str) else f"{v:10d}" if isinstance(v, int) else f"{v:10.2f}")
    return f"{name:28s}" + "".join(cells)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--png", default="")
    ap.add_argument("--max-speed", type=float, default=1.0)
    a = ap.parse_args()
    path = load()
    try:
        clear = Clearance()
    except ImportError:
        clear = None
    print(f"course_v3 path {path.duration:.2f} s, peak {path.speed.max():.2f} m/s; played at <= {a.max_speed} m/s")
    print(f"{'case':28s}" + "".join(f"{c[:10]:>10}" for c in COLS))
    logs = {}
    for mode in ("sim", "real"):
        logs[mode] = run(path, mode, a.max_speed)
        print(row(f"{mode} (default gains)", metrics(path, logs[mode], clear)))
    cases = {"MPC_XY_P 0.7": {"kp_xy": 0.7}, "MPC_XY_P 1.3": {"kp_xy": 1.3}, "VEL_P 1.2": {"kv_xy": 1.2},
             "attitude lag 0.25 s": {"tau_att_s": 0.25}}
    for name, kw in cases.items():
        for mode in ("sim", "real"):
            print(row(f"{mode} {name}", metrics(path, run(path, mode, a.max_speed, kw), clear)))
    gust = lambda t: np.array([0.0, 4.0, 0.0]) if 12.0 <= t < 13.0 else None     # 4 m/s^2, 1 s, 12 s into FOLLOW
    for mode in ("sim", "real"):
        print(row(f"{mode} gust 4 m/s^2 1 s", metrics(path, run(path, mode, a.max_speed, push=gust), clear)))
    fast = run(path, "sim", 10.0)
    print(row("sim full speed (x1.0)", metrics(path, fast, clear)))
    if a.png:
        plot(path, logs, a.png, clear)
        print(f"wrote {a.png}")


def plot(path, logs, out, clear=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    colors = {"sim": "#2e8b57", "real": "#2a6fdb"}
    labels = {"sim": "sim adapter: p + v + a feedforward", "real": "real adapter: lead point q only"}
    fig = plt.figure(figsize=(15, 9))
    ax = fig.add_subplot(1, 2, 1)
    if clear is not None:
        wall = clear.pts[(clear.pts[:, 2] > 0.3) & (clear.pts[:, 2] < 1.6)]
        ax.plot(wall[:, 0], wall[:, 1], ",", color="#999")
    ax.plot(path.xy[:, 0], path.xy[:, 1], color="#222", lw=2.5, label="planned path")
    for mode, lg in logs.items():
        ax.plot(lg["x"][:, 0], lg["x"][:, 1], color=colors[mode], lw=1.3, label=labels[mode])
    for c in CHECKPOINTS:
        ax.add_patch(plt.Rectangle((c[0] - c[3], c[1] - c[4]), 2 * c[3], 2 * c[4], fill=False, ec="#d07a1f"))
    ax.set_aspect("equal"); ax.set_xlim(-3.2, 3.2); ax.set_ylim(-3.2, 3.2)
    ax.set_title("course frame (top view), PX4 v1.16 default-gain model"); ax.legend(loc="lower right", fontsize=8)
    ax2, ax3, ax4 = (fig.add_subplot(3, 2, i) for i in (2, 4, 6))
    for mode, lg in logs.items():
        t = lg["t"] - lg["follow_t0"]
        ax2.plot(t, lg["err"], color=colors[mode], label=mode, lw=1.2)
        ax3.plot(t, np.linalg.norm(lg["v"], axis=1), color=colors[mode], lw=1.2)
        ax4.plot(t, lg["tau"], color=colors[mode], lw=1.2)
    ax2.set_ylabel("|x - p(tau)| m"); ax2.legend(fontsize=8); ax2.grid(alpha=.3)
    ax3.set_ylabel("speed m/s"); ax3.grid(alpha=.3)
    ax4.set_ylabel("tau (path s)"); ax4.set_xlabel("time since FOLLOW s"); ax4.grid(alpha=.3)
    fig.tight_layout()
    fig.savefig(out, dpi=110)


if __name__ == "__main__":
    main()
