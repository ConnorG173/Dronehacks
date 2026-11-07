import numpy as np

from team_path_following.mission import Inputs, Mission, IDLE, TAKEOFF, COURSE, LANDING, LANDED, DONE, ABORT
from test_tracker import line_traj


def sim(mission, drop_offboard_at=None, t_max=60.0):
    dt = 0.02
    pos = np.array([0.0, 0.0, 0.0])
    v = np.zeros(3)
    armed, landed, offboard = True, True, False
    seen = []
    t = 0.0
    while t < t_max:
        if t > 0.5:
            offboard = True
        if drop_offboard_at is not None and t > drop_offboard_at:
            offboard = False
        out = mission.step(dt, Inputs(now=t, offboard=offboard, armed=armed, landed=landed,
                                      pos=pos.copy(), vel=v.copy(), yaw=0.0))
        assert np.all(np.isfinite(out.v))
        if not seen or seen[-1] != out.state:
            seen.append(out.state)
        if offboard:
            v += (out.v - v) * dt / 0.3
            pos += v * dt
        if pos[2] <= 0.0:
            pos[2] = 0.0
            if mission.state == LANDED or (mission.state == LANDING and v[2] < 0):
                landed = True
        else:
            landed = False
        if mission.state == LANDED:
            armed = False
        if mission.state in (DONE, ABORT):
            break
        t += dt
    return seen, pos


def test_full_sequence():
    traj = line_traj()
    m = Mission(traj, pad_z=0.0)
    seen, pos = sim(m)
    assert seen == [IDLE, TAKEOFF, COURSE, LANDING, LANDED, DONE]
    assert np.linalg.norm(pos[:2] - traj.p[-1][:2]) < 0.2


def test_idle_outputs_zero_and_waits():
    m = Mission(line_traj(), pad_z=0.0)
    out = m.step(0.02, Inputs(now=0, offboard=False, armed=True, landed=True,
                              pos=np.zeros(3), vel=np.zeros(3), yaw=0.0))
    assert out.state == IDLE and np.all(out.v == 0)


def test_offboard_loss_aborts():
    m = Mission(line_traj(), pad_z=0.0)
    seen, _ = sim(m, drop_offboard_at=4.0)
    assert seen[-1] == ABORT


def test_stale_pose_zero_velocity():
    m = Mission(line_traj(), pad_z=0.0)
    m.step(0.02, Inputs(now=0, offboard=True, armed=True, landed=True, pos=np.zeros(3), vel=np.zeros(3), yaw=0.0))
    out = m.step(0.02, Inputs(now=0.02, offboard=True, armed=True, landed=True,
                              pos=np.zeros(3), vel=np.zeros(3), yaw=0.0, pose_age=1.0))
    assert np.all(out.v == 0) and 'stale' in out.note
