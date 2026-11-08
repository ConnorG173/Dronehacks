"""Synthetic wire packets only; never connects to ROS, UDP or flight hardware."""
import math
import unittest

from pymavlink.dialects.v20 import common as mav
from px4_msgs.msg import VehicleCommand

from return_telemetry import command_message, convert


def received(message, system=12, component=1):
    return mav.MAVLink(None).parse_char(message.pack(
        mav.MAVLink(None, srcSystem=system, srcComponent=component)))


def odometry(**kw):
    fields = dict(time_usec=12345, frame_id=1, child_frame_id=1,
        x=1.25, y=-2.5, z=-.75, q=[math.sqrt(.5), 0., 0., math.sqrt(.5)],
        vx=.1, vy=.2, vz=-.3, rollspeed=0., pitchspeed=0., yawspeed=0.,
        pose_covariance=[float('nan')]*21, velocity_covariance=[float('nan')]*21,
        reset_counter=7, estimator_type=8, quality=0)
    fields.update(kw)
    return mav.MAVLink_odometry_message(**fields)


def request(command=400, source=197, p1=1., p2=0.):
    out = VehicleCommand()
    out.timestamp = 1_000_000
    out.command = command
    out.param1, out.param2 = p1, p2
    out.target_system = out.source_system = 12
    out.target_component = 1
    out.source_component = source
    out.from_external = True
    return out


class ReturnTelemetryTests(unittest.TestCase):
    def test_odometry_preserves_ned_and_heading_without_invented_ekf_data(self):
        topic, out = convert(received(odometry()), 99999)
        self.assertEqual(topic, 'vehicle_local_position')
        self.assertEqual((out.x, out.y, out.z), (1.25, -2.5, -.75))
        self.assertAlmostEqual(out.heading, math.pi/2, places=6)
        self.assertTrue(out.xy_valid and out.z_valid)
        self.assertEqual((out.timestamp, out.timestamp_sample), (99999, 12345))
        self.assertFalse(out.dist_bottom_valid)
        self.assertTrue(math.isnan(out.dist_bottom))
        # MAVLink's aggregate reset_counter cannot identify XY/yaw resets.
        self.assertEqual((out.xy_reset_counter, out.heading_reset_counter), (0, 0))

    def test_invalid_frame_source_quaternion_and_pose_are_rejected(self):
        for values in ({'frame_id':20}, {'child_frame_id':12}, {'estimator_type':3},
                       {'x':float('nan')}, {'q':[0.,0.,0.,0.]},
                       {'q':[float('nan'),0.,0.,0.]}, {'q':[2.,0.,0.,0.]}):
            with self.subTest(values=values), self.assertRaises(ValueError):
                convert(received(odometry(**values)), 123)
        self.assertIsNone(convert(received(odometry(), system=13), 123))
        self.assertIsNone(convert(received(odometry(), component=197), 123))

    def test_nonfinite_velocity_does_not_mark_it_valid(self):
        _, out = convert(received(odometry(vx=float('nan'))), 1)
        self.assertTrue(out.xy_valid)
        self.assertFalse(out.v_xy_valid)

    def test_heartbeat_maps_actual_armed_offboard_and_failsafe(self):
        for state in (mav.MAV_STATE_ACTIVE, mav.MAV_STATE_CRITICAL,
                      mav.MAV_STATE_EMERGENCY, mav.MAV_STATE_FLIGHT_TERMINATION):
            hb = mav.MAVLink_heartbeat_message(2,12,129,6<<16,state,3)
            topic, out = convert(received(hb), 100)
            self.assertEqual((topic, out.arming_state, out.nav_state),
                             ('vehicle_status_v1',2,14))
            self.assertEqual(out.failsafe, state != mav.MAV_STATE_ACTIVE)
            self.assertFalse(out.pre_flight_checks_pass)
        _, out = convert(received(mav.MAVLink_heartbeat_message(2,12,1,6<<16,3,3)), 100)
        self.assertEqual(out.arming_state,1)

    def test_other_modes_and_missing_custom_flag_cannot_look_offboard(self):
        modes = [(1,0,0),(2,0,1),(3,0,2),(3,1,21),(3,2,6),(5,0,10),
                 (7,0,15),(10,0,13),(4,2,17),(4,3,4),(4,4,3),(4,5,5),
                 (4,6,18),(4,8,19),(4,9,20),(4,10,22),(4,18,30),(0,0,31)]
        for main, sub, expected in modes:
            hb = mav.MAVLink_heartbeat_message(2,12,1,(main<<16)|(sub<<24),3,3)
            self.assertEqual(convert(received(hb),1)[1].nav_state,expected)
        hb = mav.MAVLink_heartbeat_message(2,12,128,6<<16,4,3)
        self.assertEqual(convert(received(hb),1)[1].nav_state,31)

    def test_landed_unknown_never_becomes_airborne_or_grounded(self):
        for value in (0,255):
            self.assertIsNone(convert(received(mav.MAVLink_extended_sys_state_message(0,value)),1))
        for value in (1,2,3,4):
            _, out = convert(received(mav.MAVLink_extended_sys_state_message(0,value)),1)
            self.assertEqual(out.landed,value==1)

    def test_sys_status_battery_scaling_and_unknown_sentinels(self):
        for voltage, current, remaining, expected in (
                (21994,123,65,(True,21.994,1.23,.65)),
                (65535,-1,-1,(False,0.,-1.,-1.))):
            packet = mav.MAVLink_sys_status_message(0,0,0,0,voltage,current,remaining,0,0,0,0,0,0)
            _, out = convert(received(packet),1)
            self.assertEqual((out.connected,out.voltage_v,out.current_a,out.remaining),expected)

    def test_attitude_and_gyro_are_not_axis_swapped(self):
        packet = mav.MAVLink_attitude_quaternion_message(125,1.,0.,0.,0.,.1,.2,.3)
        topic, out = convert(received(packet),200)
        self.assertEqual(topic,'vehicle_attitude')
        self.assertEqual(list(out.q),[1.,0.,0.,0.])
        self.assertEqual(out.timestamp_sample,125000)
        packet = mav.MAVLink_highres_imu_message(123,1.,2.,3.,.1,.2,.3,0.,0.,0.,0.,0.,0.,0.,63)
        topic, out = convert(received(packet),200)
        self.assertEqual(topic,'sensor_combined')
        for actual, expected in zip(out.gyro_rad,(.1,.2,.3)):
            self.assertAlmostEqual(actual,expected,places=6)
        packet.fields_updated=7
        self.assertIsNone(convert(received(packet),200))

    def test_ack_preserves_recipient_progress_and_result(self):
        packet = mav.MAVLink_command_ack_message(400,5,25,-3,12,197)
        topic, out = convert(received(packet),1)
        self.assertEqual(topic,'vehicle_command_ack')
        self.assertEqual((out.command,out.result,out.result_param1,out.result_param2,
                          out.target_system,out.target_component),(400,5,25,-3,12,197))


class CommandTests(unittest.TestCase):
    def test_exact_normal_arm_disarm_kill_and_mode_wire_contract(self):
        for command, source, p1, p2 in ((400,197,1.,0.),(400,197,0.,0.),
                (400,197,0.,21196.),(176,198,1.,6.),(176,1,1.,6.)):
            out = received(command_message(request(command,source,p1,p2),1_000_010),component=197)
            self.assertEqual((out.command,out.target_system,out.target_component,
                              out.confirmation,out.param1,out.param2),(command,12,1,0,p1,p2))

    def test_stale_future_wrong_source_target_and_force_arm_rejected(self):
        edits = [dict(timestamp=749999),dict(timestamp=1050001),dict(target_system=13),
                 dict(target_component=0),dict(source_system=255),dict(source_component=1),
                 dict(from_external=False),dict(param2=21196.),dict(param3=1.),
                 dict(param1=float('nan')),dict(command=22)]
        for fields in edits:
            msg=request()
            for key,value in fields.items(): setattr(msg,key,value)
            with self.subTest(fields=fields), self.assertRaises(ValueError):
                command_message(msg,1_000_000)
        with self.assertRaises(ValueError):
            command_message(request(176,198,1.,3.),1_000_000)


if __name__ == '__main__':
    unittest.main()
