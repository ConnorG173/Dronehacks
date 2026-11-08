"""Isolated dashboard -> FlightInterface -> MAVLink yaw tests.

Requires ROS_DOMAIN_ID=221. Publishers are replaced with collectors, no executor
is spun, no network/serial flight command is sent, and no live dashboard is used.
"""
import copy
import math
import os
from pathlib import Path
import sys
import threading
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock

from builtin_interfaces.msg import Time
import rclpy
from px4_msgs.msg import VehicleStatus, VehicleLocalPosition, VehicleLandDetected
from pymavlink.dialects.v20 import common as mav
from skybeta_flight_interface.flight_interface import FlightInterface

from dash_node import DashNode

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'mavlink_usb'))
from conversion import setpoint_message


POSITION_ENU = (1.25, -.4, 1.2)
POSITION_NED = (-.4, 1.25, -1.2)


def angle_error(actual, expected):
    return math.atan2(math.sin(actual-expected), math.cos(actual-expected))


class YawWireTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if os.environ.get('ROS_DOMAIN_ID') != '221':
            raise RuntimeError('Yaw wire tests require isolated ROS_DOMAIN_ID=221')
        if rclpy.ok():
            raise RuntimeError('Run yaw wire tests in a fresh ROS context')
        rclpy.init(args=['--ros-args', '-p', 'max_altitude_m:=2.0',
                        '-p', 'max_speed_mps:=1.0', '-p', 'use_px4_height:=true',
                        '-p', 'target_system:=12', '-p', 'auto_offboard_on_arm:=false'])

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.interface = FlightInterface()
        self.addCleanup(self.interface.destroy_node)
        self.records = dict(control=[], setpoint=[], command=[])
        self.interface.offboard_pub.publish = self.records['control'].append
        self.interface.setpoint_pub.publish = self.records['setpoint'].append
        self.interface.command_pub.publish = self.records['command'].append
        status = VehicleStatus()
        status.arming_state, status.nav_state = 1, 14
        self.interface.status_cb(status)
        land = VehicleLandDetected()
        land.landed = True
        self.interface.land_detected_cb(land)
        local = VehicleLocalPosition()
        local.x = local.y = local.z = 0.
        local.xy_valid = local.z_valid = True
        self.interface.local_position_cb(local)

    def dashboard_pose(self, yaw):
        """Exercise the real node output construction with an inert core/pubs."""
        poses, twists = [], []
        core = NS(mode='engaged', heading=NS(yaw=yaw), course_run=None,
                  step=Mock(return_value=('pose', POSITION_ENU)), _trip=Mock())
        node = NS(core=core, lock=threading.Lock(), pos=None, fc=None,
                  a=NS(live=True), update_arm_request=Mock(),
                  readiness=Mock(return_value=[]), freeze_interface=Mock(),
                  pub_pose=NS(publish=poses.append), pub_twist=NS(publish=twists.append),
                  get_clock=lambda: NS(now=lambda: NS(to_msg=lambda: Time(sec=123))), n_out=0)
        DashNode._tick(node)
        self.assertEqual(len(poses), 1)
        self.assertEqual(twists, [])
        self.assertEqual(node.n_out, 1)
        node.freeze_interface.assert_not_called()
        core._trip.assert_not_called()
        self.assertEqual(poses[0].header.frame_id, 'odom')
        self.assertEqual((poses[0].pose.position.x, poses[0].pose.position.y,
                          poses[0].pose.position.z), POSITION_ENU)
        return poses[0]

    def wire_pose(self, pose):
        self.interface.pose_setpoint_cb(pose)
        self.interface.tick()
        self.assertEqual(self.interface.last_clamp_reasons, ())
        msg, mode = self.records['setpoint'][-1], self.records['control'][-1]
        wire = setpoint_message(msg, mode, msg.timestamp)
        encoded = wire.pack(mav.MAVLink(None, srcSystem=12, srcComponent=197))
        result = mav.MAVLink(None).parse_char(encoded)
        self.assertEqual(result.get_type(), 'SET_POSITION_TARGET_LOCAL_NED')
        self.assertEqual((result.target_system, result.target_component, result.coordinate_frame), (12, 1, 1))
        for actual, expected in zip((result.x, result.y, result.z), POSITION_NED):
            self.assertAlmostEqual(actual, expected, places=6)
        self.assertEqual(self.records['command'], [])
        return result

    def assert_yaw(self, wire, expected_degrees):
        self.assertTrue(math.isfinite(wire.yaw))
        self.assertAlmostEqual(angle_error(wire.yaw, math.radians(expected_degrees)), 0., places=6)
        self.assertEqual(wire.type_mask, 2552)
        self.assertFalse(wire.type_mask & 1024)  # YAW_IGNORE must be clear.
        self.assertTrue(wire.type_mask & 2048)   # YAW_RATE_IGNORE stays set.
        self.assertFalse(wire.type_mask & 512)  # FORCE_SET must remain clear.

    def test_real_dashboard_quaternions_survive_interface_and_wire_for_all_headings(self):
        for enu_degrees, ned_degrees in ((0, 90), (90, 0), (180, -90),
                                         (-90, 180), (179, -89), (-179, -91)):
            with self.subTest(enu_degrees=enu_degrees):
                pose = self.dashboard_pose(math.radians(enu_degrees))
                q = pose.pose.orientation
                self.assertAlmostEqual(sum(v*v for v in (q.x, q.y, q.z, q.w)), 1., places=12)
                if enu_degrees == 0:
                    self.assertEqual((q.x, q.y, q.z, q.w), (0., 0., 0., 1.))
                self.assert_yaw(self.wire_pose(pose), ned_degrees)
                negated = copy.deepcopy(pose)
                qn = negated.pose.orientation
                qn.x, qn.y, qn.z, qn.w = -q.x, -q.y, -q.z, -q.w
                self.assert_yaw(self.wire_pose(negated), ned_degrees)

    def test_wire_mapping_preserves_shortest_signed_wrap_transitions(self):
        headings = [self.wire_pose(self.dashboard_pose(math.radians(deg))).yaw
                    for deg in (179, -179, 179)]
        differences = [math.degrees(angle_error(b, a)) for a, b in zip(headings, headings[1:])]
        self.assertAlmostEqual(differences[0], -2., places=4)
        self.assertAlmostEqual(differences[1], 2., places=4)

    def test_missing_heading_preserves_legacy_zero_quaternion_and_yaw_ignore(self):
        pose = self.dashboard_pose(None)
        q = pose.pose.orientation
        self.assertEqual((q.x, q.y, q.z, q.w), (0., 0., 0., 0.))
        wire = self.wire_pose(pose)
        self.assertTrue(math.isnan(self.records['setpoint'][-1].yaw))
        self.assertEqual(wire.type_mask, 3576)
        self.assertTrue(wire.type_mask & 1024)
        self.assertTrue(wire.type_mask & 2048)
        self.assertFalse(wire.type_mask & 512)


if __name__ == '__main__':
    unittest.main(verbosity=2)
