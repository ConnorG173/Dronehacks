"""Offline: play the path_streamer timing logic against a model of PX4's position controller.

Laptop only, no ROS. Our node ticks at 20 Hz; flight_interface re-sends the latest pose at 10 Hz
(zero-order hold); the PX4 model (test/px4_model.py) runs at 200 Hz. Perfect localisation (the
vehicle position the node sees is the true one) and T = the pad init with odom at zero.

    py localizer/tools/sim_streamer.py [--png out.png] [--table-dt 1.0]

Schemes:
  old    the previous Carrot: sends p(tau), pauses tau while |x - p(tau)| > 0.6 m
  lead   PathClock: on-path lead q = p(tau + rate / MPC_XY_P), position only (what path_streamer sends)
  alg    PathClock + algebraic lead p + v/Kp + a/(Kp Kv), position only (rejected: leaves the path)
  ff     PathClock + p with real velocity/acceleration feedforward (needs a flight_interface change)
"""
import argparse
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(HERE, ".."), os.path.join(HERE, "..", "test")]
from localizer import streamer as S  # noqa: E402
from px4_model import PX4PosModel  # noqa: E402

DATA = os.path.join(HERE, "..", "data")


class OldCarrot:
    """Copy of the Carrot this replaces (commit 222185c), kept only for the comparison."""

    def __init__(self, path, scale=1.0, max_err_m=0.6):
        self.path, self.scale, self.max_err = path, float(scale), float(max_err_m)
        self.tau, self.paused = 0.0, False

    @property
    def done(self):
        return self.tau >= self.path.duration

    def step(self, dt, veh):
        xy, z, _ = self.path.at(self.tau)
        err = float(np.linalg.norm(np.r_[xy, z] - np.asarray(veh, float)))
        self.paused = err > self.max_err
        if not self.paused:
            self.tau = min(self.tau + dt * self.scale, self.path.duration)
        return self.path.at(self.tau), err


def load():
    with np.load(os.path.join(DATA, "course_v3_wallgrid.npz")) as g:
        grid = {k: g[k] for k in g.files}
    path = S.path_to_course(S.load_path(os.path.join(DATA, "course_v3_path.bin")),
                            grid["start_xy"], float(grid["start_heading_rad"]))
    return path, grid


class Clearance:
    """3-D distance (m) to the nearest CAD surface point of the planner map (planner/maps/course_v3.bin,
    4 cm sampling), floor (z < 0.05) and the cage net excluded. The 2-D wall grid cannot be used:
    the window and the tube are wall columns in its 0.3-1.6 m band."""

    def __init__(self, map_file=os.path.join(HERE, "..", "..", "planner", "maps", "course_v3.bin"),
                 cage_half=3.049):
        from scipy.spatial import cKDTree
        raw = open(map_file, "rb").read()
        n = int(np.frombuffer(raw, "<u8", count=1, offset=16)[0])
        pts = np.frombuffer(raw, "<f4", count=3 * n, offset=24).reshape(n, 3).astype(float)
        keep = (pts[:, 2] > 0.05) & (np.abs(pts[:, :2]).max(axis=1) < cage_half - 0.02)
        self.tree = cKDTree(pts[keep])

    def __call__(self, xyz):
        return self.tree.query(np.atleast_2d(xyz))[0]


def simulate(path, scheme, scale, model_kw=None, push=None, t_max=120.0, cage_half=3.049,
             fence_margin=0.3, kp_xy=0.95, kp_z=1.0, kv_xy=1.8, kv_z=4.0):
    """Returns a dict of arrays sampled at the node's 20 Hz tick."""
    node_dt, fi_dt, px4_dt = 0.05, 0.1, 0.005
    xy0, z0, _ = path.at(0.0)
    m = PX4PosModel(np.r_[xy0, z0], **(model_kw or {}))     # hovering at the path start (TAKEOFF done)
    if scheme == "old":
        clock = OldCarrot(path, scale, 0.6)
    else:
        clock = S.PathClock(path, scale, lead_s=1.0 / kp_xy)
    sent = None            # what the node last published (position, vel_ff, acc_ff)
    fi = None              # what flight_interface last forwarded to PX4
    log = {k: [] for k in ("t", "tau", "q", "p", "x", "v", "stall", "err")}
    t, n_px4 = 0.0, int(round(node_dt / px4_dt))
    k = 0
    while t < t_max:
        # node tick (20 Hz)
        veh = m.x.copy()
        if scheme == "old":
            (xy, z, _), err = clock.step(node_dt, veh)
            p = np.r_[xy, z]
            q, vff, aff, stall = p, None, None, clock.paused
        else:
            sp = clock.step(node_dt, veh)
            p, stall, err = sp.p, clock.stalled, sp.err
            if scheme == "lead":
                q, vff, aff = sp.q, None, None
            elif scheme == "ff":
                q, vff, aff = sp.p, sp.v, sp.a
            else:  # alg
                kp, kv = np.array([kp_xy, kp_xy, kp_z]), np.array([kv_xy, kv_xy, kv_z])
                q, vff, aff = sp.p + sp.v / kp + sp.a / (kp * kv), None, None
        q = np.r_[S.clamp_to_fence(q[:2], cage_half, fence_margin), q[2]]
        sent = (q, vff, aff)
        for key, val in (("t", t), ("tau", clock.tau), ("q", q), ("p", p), ("x", veh),
                         ("v", m.v.copy()), ("stall", stall), ("err", err)):
            log[key].append(val)
        # PX4 steps until the next node tick; flight_interface samples at 10 Hz (offset 25 ms)
        for _ in range(n_px4):
            if fi is None or (k * px4_dt + 0.025) % fi_dt < px4_dt:
                fi = sent
            g = push(t) if push else None
            m.step(px4_dt, fi[0], fi[1], fi[2], g)
            k += 1
        t += node_dt
        if clock.done and np.linalg.norm(m.x - p) < 0.05 and np.linalg.norm(m.v) < 0.05:
            break
    return {k: np.array(v) for k, v in log.items()}


def metrics(path, log, clear):
    dense = np.c_[path.xy, path.z]
    d = np.linalg.norm(log["x"][:, None, :] - dense[None, ::2, :], axis=2).min(axis=1)   # cross-track
    stall = log["stall"].astype(int)
    stops = int(np.sum(np.diff(np.r_[0, stall]) == 1))
    done = np.nonzero(log["tau"] >= path.duration)[0]
    t_done = float(log["t"][done[0]]) if len(done) else float("nan")
    return {"t_total": float(log["t"][-1]), "t_tau_end": t_done,
            "err_max": float(log["err"].max()), "err_rms": float(np.sqrt(np.mean(log["err"] ** 2))),
            "xtrack_max": float(d.max()), "xtrack_rms": float(np.sqrt(np.mean(d ** 2))),
            "stops": stops, "stalled_s": float(stall.sum() * 0.05),
            "wall_min_vehicle": float(clear(log["x"]).min()),
            "wall_min_setpoint": float(clear(log["q"]).min()),
            "vmax": float(np.linalg.norm(log["v"], axis=1).max())}


def to_ned(grid, q):
    """What flight_interface would send (unanchored): odom = course - pad (T0 with odom at zero,
    StartHeadingDeg 0), NED = (y_odom, x_odom, -z)."""
    T = S.Transform2D(float(grid["start_xy"][0]), float(grid["start_xy"][1]), float(grid["start_heading_rad"]))
    xo = T.to_odom(q[:2])
    return np.array([xo[1], xo[0], -q[2]])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--png", default="")
    ap.add_argument("--table-dt", type=float, default=1.0)
    ap.add_argument("--max-speed", type=float, default=1.0)
    a = ap.parse_args()
    path, grid = load()
    scale = S.time_scale_for(path, a.max_speed)
    clear = Clearance()
    print(f"path {path.duration:.2f} s, peak {path.speed.max():.2f} m/s, scale {scale:.3f} -> "
          f"{path.duration / scale:.1f} s nominal; path's own min clearance to CAD surfaces "
          f"{clear(np.c_[path.xy, path.z]).min():.2f} m")
    logs = {s: simulate(path, s, scale) for s in ("old", "lead", "alg", "ff")}

    L = logs["lead"]
    print("\nlead scheme (what path_streamer sends), every %.1f s:" % a.table_dt)
    print(f"{'t':>5} {'tau':>6} {'setpoint course x,y,z':>22} {'NED sent by FI':>22} "
          f"{'vehicle course x,y,z':>22} {'err':>5} {'speed':>5}")
    every = int(round(a.table_dt / 0.05))
    for i in list(range(0, len(L["t"]), every)) + [len(L["t"]) - 1]:
        q, x = L["q"][i], L["x"][i]
        ned = to_ned(grid, q)
        print(f"{L['t'][i]:5.1f} {L['tau'][i]:6.2f} {q[0]:7.2f}{q[1]:7.2f}{q[2]:7.2f}  "
              f"{ned[0]:7.2f}{ned[1]:7.2f}{ned[2]:7.2f}  {x[0]:7.2f}{x[1]:7.2f}{x[2]:7.2f}  "
              f"{L['err'][i]:5.2f} {np.linalg.norm(L['v'][i]):5.2f}")

    print("\ncomparison on the same PX4 model (err = |x - p(tau)|, xtrack = distance to the path):")
    hdr = ("scheme", "total s", "tau end s", "err max", "err rms", "xtrk max", "xtrk rms", "stops",
           "stalled s", "wall veh", "wall sp", "vmax")
    print(" ".join(f"{h:>9}" for h in hdr))
    for s, lg in logs.items():
        mm = metrics(path, lg, clear)
        print(f"{s:>9} " + " ".join(f"{mm[k]:9.2f}" if isinstance(mm[k], float) else f"{mm[k]:9d}"
                                    for k in ("t_total", "t_tau_end", "err_max", "err_rms", "xtrack_max",
                                              "xtrack_rms", "stops", "stalled_s", "wall_min_vehicle",
                                              "wall_min_setpoint", "vmax")))

    print("\nlead scheme robustness (gains assumed = v1.16 defaults; the drone's differ):")
    cases = {"MPC_XY_P 0.7": {"kp_xy": 0.7}, "MPC_XY_P 1.3": {"kp_xy": 1.3},
             "VEL_P 1.2": {"kv_xy": 1.2}, "attitude lag 0.25 s": {"tau_att_s": 0.25}}
    for name, kw in cases.items():
        mm = metrics(path, simulate(path, "lead", scale, kw), clear)
        print(f"  {name:22s} total {mm['t_total']:5.1f} s  err max {mm['err_max']:.2f}  "
              f"xtrack max {mm['xtrack_max']:.2f} rms {mm['xtrack_rms']:.2f}  stops {mm['stops']}")
    gust = lambda t: np.array([0.0, 4.0, 0.0]) if 12.0 <= t < 13.0 else None   # 4 m/s^2 for 1 s
    for s in ("old", "lead"):
        mm = metrics(path, simulate(path, s, scale, push=gust), clear)
        print(f"  gust 4 m/s^2 +y 1 s, {s:5s} total {mm['t_total']:5.1f} s  err max {mm['err_max']:.2f}  "
              f"xtrack max {mm['xtrack_max']:.2f}  stops {mm['stops']} ({mm['stalled_s']:.1f} s)")

    if a.png:
        plot(path, grid, logs, a.png)
        print(f"\nwrote {a.png}")


def plot(path, grid, logs, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    res, ox, oy = float(grid["res_m"]), float(grid["origin_x"]), float(grid["origin_y"])
    n = grid["wall"].shape[0]
    ext = (ox, ox + n * res, oy, oy + n * res)
    fig = plt.figure(figsize=(15, 9))
    ax = fig.add_subplot(1, 2, 1)
    ax.imshow(grid["wall"].T, origin="lower", extent=ext, cmap="Greys", vmin=0, vmax=1.4)
    ax.plot(path.xy[:, 0], path.xy[:, 1], color="#222", lw=2.5, label="planned path")
    L = logs["lead"]
    ax.plot(L["q"][:, 0], L["q"][:, 1], ".", color="#d07a1f", ms=2.5, label="lead setpoint q (sent)")
    ax.plot(L["x"][:, 0], L["x"][:, 1], color="#2a6fdb", lw=1.4, label="vehicle, lead (new)")
    O = logs["old"]
    ax.plot(O["x"][:, 0], O["x"][:, 1], color="#c0392b", lw=1.0, ls="--", label="vehicle, old Carrot")
    for c in grid["checkpoints"]:
        ax.add_patch(plt.Rectangle((c[0] - c[3], c[1] - c[4]), 2 * c[3], 2 * c[4], fill=False, ec="#2e8b57"))
    ax.set_aspect("equal"); ax.set_xlim(-3.2, 3.2); ax.set_ylim(-3.2, 3.2)
    ax.set_title("course frame (top view), PX4 v1.16 default-gain model"); ax.legend(loc="lower right", fontsize=8)
    colors = {"old": "#c0392b", "lead": "#2a6fdb", "alg": "#8e44ad", "ff": "#2e8b57"}
    ax2 = fig.add_subplot(3, 2, 2)
    ax3 = fig.add_subplot(3, 2, 4)
    ax4 = fig.add_subplot(3, 2, 6)
    for s, lg in logs.items():
        ax2.plot(lg["t"], lg["err"], color=colors[s], label=s, lw=1.2)
        ax3.plot(lg["t"], np.linalg.norm(lg["v"], axis=1), color=colors[s], lw=1.2)
        ax4.plot(lg["t"], lg["tau"], color=colors[s], lw=1.2)
    ax2.set_ylabel("|x - p(tau)| m"); ax2.legend(fontsize=8, ncol=4); ax2.grid(alpha=.3)
    ax3.set_ylabel("speed m/s"); ax3.grid(alpha=.3)
    ax4.set_ylabel("tau (path s)"); ax4.set_xlabel("wall time s"); ax4.grid(alpha=.3)
    fig.tight_layout()
    fig.savefig(out, dpi=110)


if __name__ == "__main__":
    main()
