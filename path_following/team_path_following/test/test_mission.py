import numpy as np

from team_path_following.mission import Inputs, Mission, IDLE, TAKEOFF, COURSE, LANDING, LANDED, DONE, ABORT, RETURN
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
        if mission.state == LANDED and (mission.return_traj is None or mission.leg == 1):
            if not hasattr(mission, '_t_l'):
                mission._t_l = t
            if t - mission._t_l > 2.0:   # PX4 COM_DISARM_LAND
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


def test_return_touch_and_go():
    out_traj = line_traj()
    back = line_traj()
    back.p = back.p[::-1].copy()
    back.v = -back.v[::-1].copy()
    back.a = back.a[::-1].copy()
    m = Mission(out_traj, pad_z=0.0, return_traj=back, return_pad_z=0.0)
    seen, pos = sim(m, t_max=90.0)
    assert seen == [IDLE, TAKEOFF, COURSE, LANDING, LANDED, RETURN, LANDING, LANDED, DONE], seen
    assert np.linalg.norm(pos[:2] - back.p[-1][:2]) < 0.2


def test_pose_jump_guard():
    import numpy as np
    from team_path_following.mission import PoseJumpGuard
    g = PoseJumpGuard(v_max=1.5)
    t, p = 0.0, np.zeros(3)
    for _ in range(100):                       # 1.4 m/s at 25 Hz: fine
        t += 0.04
        p = p + np.array([1.4 * 0.04, 0, 0])
        assert g.update(p, t) is None
    assert g.update(p + np.array([9.5, 0, 0]), t + 0.04) is not None   # vio21's liftoff jump
    assert g.update(p, t + 0.08) is not None   # latched


def test_pose_fault_aborts_and_descends():
    import numpy as np
    from team_path_following.mission import Inputs, Mission, ABORT, IDLE
    m = Mission(line_traj(), pad_z=0.0)
    s = dict(offboard=True, armed=True, landed=False, pos=np.array([0.0, 0.0, 0.5]), vel=np.zeros(3), yaw=0.0)
    m.step(0.02, Inputs(now=0.0, **s))
    out = m.step(0.02, Inputs(now=0.02, pose_fault='jump', **s))
    assert m.state == ABORT and out.v[2] < 0 and abs(out.v[0]) + abs(out.v[1]) == 0.0
    out = m.step(0.02, Inputs(now=0.04, pose_fault='jump', **{**s, 'landed': True}))
    assert np.allclose(out.v, 0.0)
