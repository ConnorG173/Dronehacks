"""Adapter message conversions with fake message objects (no ROS on the laptop).

Checked against skybeta flight_interface's conventions: ENU setpoint -> NED as (y, x, -z),
yaw_ned = pi/2 - yaw_enu; and PX4's VehicleLocalPosition / TrajectorySetpoint NED fields.
"""
import math
import os
import re
import sys
from types import SimpleNamespace as NS

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(HERE, ".."), HERE]
from localizer import real, sim  # noqa: E402
from localizer.core import frames as F  # noqa: E402
from localizer.core.follower import Command, Follower, VehicleState  # noqa: E402
from localizer.core.path import load_path, path_to_course  # noqa: E402
from test_core import q_px4  # noqa: E402

PATH = os.path.join(HERE, "..", "data", "course_v3_path.bin")
PAD = (-2.5146, 0.9144)
PKG = os.path.join(HERE, "..", "localizer")


def fake_pose():
    return NS(header=NS(stamp=None, frame_id=""),
              pose=NS(position=NS(x=0.0, y=0.0, z=0.0), orientation=NS(x=0.0, y=0.0, z=0.0, w=0.0)))


def flight_interface(pose_msg):
    """skybeta_flight_interface's conversion of /skybeta/setpoint/pose (unanchored)."""
    p, o = pose_msg.pose.position, pose_msg.pose.orientation
    ned = np.array([p.y, p.x, -p.z])
    if (o.x, o.y, o.z, o.w) == (0.0, 0.0, 0.0, 0.0):
        return ned, None
    yaw_enu = math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y ** 2 + o.z ** 2))
    return ned, F.wrap(math.pi / 2 - yaw_enu)


# ----------------------------------------------------------------------------- real adapter

def test_real_pose_message_matches_flight_interface():
    msg = real.fill_pose(fake_pose(), np.array([1.5, -0.25, 1.0]), math.radians(30), "odom", "stamp")
    assert msg.header.frame_id == "odom" and msg.header.stamp == "stamp"
    ned, yaw_ned = flight_interface(msg)
    assert np.allclose(ned, (-0.25, 1.5, -1.0))
    assert yaw_ned == pytest.approx(math.radians(60))
    exp_ned, exp_yaw = real.flight_interface_ned(np.array([1.5, -0.25, 1.0]), math.radians(30))
    assert np.allclose(exp_ned, ned) and exp_yaw == pytest.approx(yaw_ned)


def test_real_no_yaw_is_all_zero_quaternion():
    ned, yaw = flight_interface(real.fill_pose(fake_pose(), np.zeros(3), None, "odom", None))
    assert yaw is None


def test_real_odom_conversion():
    yaw, roll = math.radians(40), math.radians(2)
    # yaw about z then roll about x (ENU/FLU): q = qz * qx
    w = math.cos(yaw / 2) * math.cos(roll / 2)
    x = math.cos(yaw / 2) * math.sin(roll / 2)
    y = math.sin(yaw / 2) * math.sin(roll / 2)
    z = math.sin(yaw / 2) * math.cos(roll / 2)
    m = NS(pose=NS(pose=NS(position=NS(x=0.1, y=-0.2, z=269.0), orientation=NS(x=x, y=y, z=z, w=w))))
    xyz, yaw_o, tilt = real.odom_to_pose(m)
    assert np.allclose(xyz, (0.1, -0.2, 269.0)) and yaw_o == pytest.approx(yaw) and tilt == pytest.approx(2.0)
    pos = real.local_position(xyz, 0.08)               # height comes from PX4, never cuVSLAM z
    assert np.allclose(pos, (0.1, -0.2, 0.08))
    assert np.isnan(real.local_position(xyz, None)[2])


def test_real_state_json():
    assert real.parse_state('{"armed": true, "landed": false, "offboard_active": true, "local_enu": [1, 2, 0.5]}') \
        == (True, False, True, 0.5)
    assert real.parse_state('{"armed": false}') == (False, False, False, None)
    assert real.parse_state("not json") is None and real.parse_state("[1, 2]") is None


def test_real_attitude():
    tilt, yaw_ned = real.attitude_to_tilt_yaw(q_px4(3, 4, 70))
    assert tilt == pytest.approx(math.degrees(math.acos(math.cos(math.radians(3)) * math.cos(math.radians(4)))))
    assert yaw_ned == pytest.approx(math.radians(70))


def test_real_config_can_never_arm_or_land():
    cfg = real.real_config(self_arm=True, land_at_end=True, land_after_hold_s=1.0, on_offboard_lost="stop")
    assert not cfg.self_arm and not cfg.land_at_end and cfg.land_after_hold_s is None
    assert cfg.on_offboard_lost == "hold"
    assert cfg.max_pad_offset_m == 0.15 and cfg.max_tilt_diff_deg == 3.0     # pad checks on


def test_real_source_cannot_reach_fmu_in():
    src = open(os.path.join(PKG, "real.py"), encoding="utf-8").read()
    code = re.sub(r'""".*?"""', "", src, flags=re.S)                         # ignore docstrings
    for forbidden in ("/fmu/in", "VehicleCommand", "TrajectorySetpoint", "OffboardControlMode", "arm_disarm"):
        assert forbidden not in code, forbidden
    pubs = re.findall(r"create_publisher\((\w+), ([^,]+),", code)
    assert pubs == [("PoseStamped", "self.topic"), ("PoseStamped", '"/localizer/pose"'),
                    ("String", '"/localizer/status"')]
    assert real.PREVIEW_TOPIC == "/localizer/setpoint_preview" and real.LIVE_TOPIC == "/skybeta/setpoint/pose"
    assert re.search(r"self\.topic = LIVE_TOPIC if self\.live else PREVIEW_TOPIC", code)
    assert re.search(r'P\("live", False\)', code)                             # preview is the default


def test_real_chain_course_to_px4_ned():
    """Course point -> follower (T from the pad, odom at zero) -> PoseStamped -> flight_interface NED.
    PX4's local frame follows odom (BRIEF: local_enu ~ odom_enu), so the NED it receives must be the
    odom ENU point under (y, x, -z)."""
    cpath = path_to_course(load_path(PATH), PAD, 0.0)
    f = Follower(cpath, real.real_config())
    s = VehicleState(pose_stamp=0.0, pos=np.array([0.01, -0.02, 0.08]), yaw=0.0, estimate_ok=True,
                     status_stamp=0.0, landed=True, landed_stamp=0.0, local_tilt_deg=0.4,
                     imu_stamp=0.0, imu_tilt_deg=0.3)
    f.step(0.0, s)
    s.armed = s.offboard = True
    cmd = f.step(0.05, s)
    assert f.mode == "TAKEOFF"
    ned, yaw_ned = flight_interface(real.fill_pose(fake_pose(), cmd.q, cmd.q_yaw, "odom", None))
    xy, z, yaw = cpath.at(0.0)
    odom = f.T.to_local(xy)
    assert np.allclose(ned, (odom[1], odom[0], -z))
    assert yaw_ned == pytest.approx(math.pi / 2 - (yaw - f.T.yaw))


# ----------------------------------------------------------------------------- sim adapter

def fake_status(arming=1, nav=4):
    return NS(arming_state=arming, nav_state=nav, ARMING_STATE_ARMED=2, NAVIGATION_STATE_OFFBOARD=14,
              NAVIGATION_STATE_AUTO_LAND=18)


def fake_lpos(ned, heading, ts=10_000_000, ok=True):
    return NS(x=ned[0], y=ned[1], z=ned[2], heading=heading, timestamp=ts, xy_valid=ok, z_valid=ok,
              heading_good_for_control=ok)


def test_sim_status_flags():
    assert sim.status_flags(fake_status(1, 4)) == (False, False, False)
    assert sim.status_flags(fake_status(2, 14)) == (True, True, False)
    assert sim.status_flags(fake_status(2, 18)) == (True, False, True)


def test_sim_local_position_and_setpoint_roundtrip():
    pos, yaw = sim.local_position_to_pose(fake_lpos((1.0, 2.0, -1.5), math.radians(30)))
    assert np.allclose(pos, (2.0, 1.0, 1.5)) and yaw == pytest.approx(math.radians(60))
    cmd = Command("FOLLOW", "", publish=True, p=pos, v=np.array([0.5, -0.2, 0.1]), a=np.array([0.0, 1.0, 0.0]),
                  yaw=yaw, yaw_rate=0.3)
    sp = sim.fill_trajectory_setpoint(NS(), cmd, 123)
    assert sp.timestamp == 123
    assert np.allclose(sp.position, (1.0, 2.0, -1.5))                        # back to the same NED point
    assert np.allclose(sp.velocity, (-0.2, 0.5, -0.1)) and np.allclose(sp.acceleration, (1.0, 0.0, 0.0))
    assert sp.yaw == pytest.approx(math.radians(30)) and sp.yawspeed == pytest.approx(-0.3)
    sp = sim.fill_trajectory_setpoint(NS(), Command("READY", "", p=pos, yaw=None), 1)
    assert math.isnan(sp.yaw) and math.isnan(sp.yawspeed)


def test_sim_vehicle_command_fields():
    m = sim.fill_vehicle_command(NS(), 176, 5, 1.0, 6.0)
    assert (m.command, m.param1, m.param2, m.timestamp, m.from_external) == (176, 1.0, 6.0, 5, True)
    assert m.target_system == m.target_component == m.source_system == m.source_component == 1
    assert sim.fill_offboard_mode(NS(), 7).position is True


def test_sim_estimate_ok_and_yaw_alignment():
    lpos = fake_lpos((0, 0, 0), 0.0)
    assert sim.estimate_ok(lpos, False)
    lpos.heading_good_for_control = False
    assert not sim.estimate_ok(lpos, False)
    flags = NS(cs_yaw_align=True, timestamp=9_000_000)
    assert sim.yaw_aligned(flags, 0.5, lpos) and sim.estimate_ok(lpos, True)
    assert not sim.yaw_aligned(flags, 3.5, lpos)                              # stale by receipt
    assert not sim.yaw_aligned(NS(cs_yaw_align=True, timestamp=11_000_000), 0.1, lpos)   # from the future
    assert not sim.yaw_aligned(None, None, lpos)
    lpos.heading_good_for_control, lpos.x = True, math.nan
    assert not sim.estimate_ok(lpos, True)


def test_sim_approval_flag_and_lab_guard(monkeypatch):
    assert sim.split_approval(["--ros-args", "-p", "x:=1"]) == (False, ["--ros-args", "-p", "x:=1"])
    assert sim.split_approval(["--i-have-team-approval", "--ros-args"]) == (True, ["--ros-args"])
    assert not sim.sim_config(False).self_arm and sim.sim_config(True).self_arm
    monkeypatch.delenv("FLYX_LAB_ACTIVE", raising=False)
    assert sim.main(["--i-have-team-approval"]) == 2                          # refused before ROS starts


def test_sim_publishers_only_with_approval():
    code = re.sub(r'""".*?"""', "", open(os.path.join(PKG, "sim.py"), encoding="utf-8").read(), flags=re.S)
    block = code[code.index("if approved:"):code.index("self.create_subscription")]
    assert block.count("create_publisher") == 3 and code.count("create_publisher") == 3
    assert "if not self.approved:\n            return" in code


def test_sim_chain_px4_spawn_to_takeoff_setpoint():
    """Fake PX4 on the pad (local NED north-aligned, heading east = course +x, origin offset):
    the TAKEOFF setpoint must be HoverHeight above the spawn point, facing the same way."""
    cpath = path_to_course(load_path(PATH), PAD, 0.0)
    f = Follower(cpath, sim.sim_config(True))
    spawn_ned, heading = (0.3, -0.4, -0.02), math.radians(90)                 # facing east
    s = VehicleState()
    t, requested = 0.0, 0
    for k in range(200):
        pos_msg = fake_lpos(spawn_ned, heading)
        s.pose_stamp, (s.pos, s.yaw) = t, sim.local_position_to_pose(pos_msg)
        s.estimate_ok = sim.estimate_ok(pos_msg, False)
        s.armed, s.offboard, s.auto_land = sim.status_flags(fake_status(2 if k > 60 else 1, 14 if k > 60 else 4))
        s.status_stamp, s.landed, s.landed_stamp = t, k <= 60, t
        cmd = f.step(t, s)
        requested += cmd.request_offboard_arm                               # PX4 "accepts" at k = 61
        t += 0.05
        if f.mode == "TAKEOFF":
            break
    assert f.mode == "TAKEOFF" and requested == 1
    sp = sim.fill_trajectory_setpoint(NS(), cmd, 0)
    assert np.allclose(sp.position, (spawn_ned[0], spawn_ned[1], spawn_ned[2] - 1.0), atol=1e-9)
    assert sp.yaw == pytest.approx(heading)
