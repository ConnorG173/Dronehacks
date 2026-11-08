"""Run inside the ROS environment; all cases use synthetic data and publish nothing."""
import math
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock
from dash_core import DashCore, Fc, Pos
from dash_node import ARM_SOURCE_COMPONENT, DashNode
from px4_msgs.msg import VehicleCommand


class ReadinessTests(unittest.TestCase):
    def state(self):
        n = SimpleNamespace(a=SimpleNamespace(live=True, usb_telemetry=True),
            core=DashCore(), pos=Pos(20, 0, 0, 0, 0), fc=Fc(20, True, False, False, True),
            batt=(20, 16, 1, True),
            vehicle=(20, {"failsafe": False}), land=(20, True), vo_t=20, vo_state=1,
            count_publishers=lambda topic: 1, course=Mock(document=None), last_course_sample_us=None)
        n.other_setpoint_publishers = lambda: DashNode.other_setpoint_publishers(n)
        return n

    def test_ready_without_detailed_ekf_and_px4_failsafe_still_blocks(self):
        n = self.state()
        self.assertEqual(DashNode.readiness(n, 20), [])
        # No fusion flags, reset history or preflight_ok field is needed.
        n.vehicle[1]["failsafe"] = True
        self.assertIn("PX4 failsafe active", DashNode.readiness(n, 20))

    def test_stale_vio_and_landed_telemetry_still_block(self):
        n = self.state()
        n.vo_t = 19
        self.assertTrue(any("VIO" in x for x in DashNode.readiness(n, 20)))
        n.vo_t = 20
        n.land = (18, True)
        self.assertIn("PX4 landed state missing or stale", DashNode.readiness(n, 20))

    def test_battery_default_usb_interval_and_inflight_height(self):
        n = self.state()
        n.batt = (17.5, 16, .8, True)
        self.assertEqual(DashNode.readiness(n, 20), [])
        n.pos.z = 2
        self.assertIn("flight interface/PX4 ground heights disagree", DashNode.readiness(n, 20))
        self.assertEqual(DashNode.readiness(n, 20, preflight=False), [])
        n.batt = (14.9, 16, .8, True)
        self.assertIn("flight battery missing or stale", DashNode.readiness(n, 20, preflight=False))

    def test_go_works_without_ekf_streams_but_rejects_stale_landed_state(self):
        n = self.arm_node()
        n.fc.armed, n.fc.landed = True, False
        n.land = (20, False)
        n.pos.z = .5
        n.readiness = lambda now, preflight=True: DashNode.readiness(n, now, preflight)
        self.assertTrue(DashNode.do_command(n, 20, "tester", {"op": "goto", "forward": 1, "left": .5})[0])
        self.assertEqual(n.core.target[:2], [1, .5])
        n.land = (18, False)
        ok, why = DashNode.do_command(n, 20, "tester", {"op": "goto", "forward": 0, "left": 0})
        self.assertFalse(ok)
        self.assertIn("landed state", why)
        n.arm_pub.publish.assert_not_called()

    def test_twist_competitor_and_dry_run_graph(self):
        n = self.state()
        n.count_publishers = lambda topic: 2 if topic.endswith("twist") else 1
        self.assertEqual(n.other_setpoint_publishers(), 1)
        n.a.live = False
        self.assertEqual(n.other_setpoint_publishers(), 3)

    def test_command_validation_and_heartbeat_latch(self):
        c = DashCore()
        p, f = Pos(0, 0, 0, 0, 0), Fc(0, True, False, False, True)
        self.assertFalse(c.command(0, "", "claim")[0])
        self.assertTrue(c.command(0, "tester", "claim")[0])
        for v in (math.nan, math.inf, "bad", None, True):
            self.assertFalse(c.command(0, "tester", "engage", pos=p, fc=f, alt=v)[0])
        f.armed = True
        self.assertFalse(c.command(0, "tester", "engage", pos=p, fc=f, alt=.5)[0])
        f.armed = False
        self.assertTrue(c.command(0, "tester", "engage", pos=p, fc=f, alt=.5)[0])
        self.assertIsNotNone(c.step(0, p, f))
        p.t = f.t = 1.1
        self.assertIsNone(c.step(1.1, p, f))
        self.assertEqual(c.mode, "failsafe")
        c.heartbeat(1.2, "tester")
        self.assertIsNone(c.step(1.2, p, f))

    def test_prepare_in_disarmed_offboard_waits_for_arming(self):
        c = DashCore()
        p, f = Pos(0, 0, 0, 0, 0), Fc(0, True, False, True, True)
        self.assertTrue(c.command(0, "tester", "claim")[0])
        self.assertTrue(c.command(0, "tester", "engage", pos=p, fc=f, alt=.5)[0])
        for tick in range(21):
            t = tick / 10
            p.t = f.t = t
            c.heartbeat(t, "tester")
            self.assertEqual(c.step(t, p, f), ("pose", (0, 0, .3)))
            self.assertFalse(c.was_offboard)
        p.t = f.t = 2.1
        c.heartbeat(2.1, "tester")
        f.armed = True
        out = c.step(2.1, p, f)
        self.assertTrue(c.was_offboard)
        self.assertAlmostEqual(out[1][2], .33)

    def arm_node(self):
        n = self.state()
        n.lock = threading.Lock()
        n.arm_request = None
        n.arm_pub = Mock()
        n.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=20_000_000_000))
        n.get_logger = lambda: Mock()
        n.update_arm_request = lambda now: DashNode.update_arm_request(n, now)
        n.readiness = lambda now: DashNode.readiness(n, now)
        n.core.command(20, "tester", "claim")
        n.fc.offboard_active = True
        n.core.command(20, "tester", "engage", pos=n.pos, fc=n.fc, alt=.5)
        return n

    def test_each_click_sends_normal_arm_then_telemetry_confirms(self):
        n = self.arm_node()
        self.assertTrue(DashNode.do_command(n, 20, "tester", {"op": "arm", "auto_course": False})[0])
        m = n.arm_pub.publish.call_args.args[0]
        self.assertEqual(m.command, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM)
        self.assertEqual((m.param1, m.param2, m.target_system, m.target_component), (1., 0., 12, 1))
        self.assertEqual((m.source_system, m.source_component, m.from_external), (12, ARM_SOURCE_COMPONENT, True))
        self.assertTrue(DashNode.do_command(n, 20, "tester", {"op": "arm", "auto_course": False})[0])
        self.assertEqual(n.arm_pub.publish.call_count, 2)
        ack = SimpleNamespace(command=m.command, target_system=12, target_component=1, result=0)
        DashNode._arm_ack(n, ack)
        self.assertIn("requested", n.arm_request["message"])
        ack.target_component = ARM_SOURCE_COMPONENT
        DashNode._arm_ack(n, ack)
        self.assertTrue(n.arm_request["pending"])
        self.assertFalse(n.fc.armed)
        n.fc.t, n.fc.armed = 20.2, True
        n.update_arm_request(20.2)
        self.assertEqual(n.arm_request["message"], "PX4 confirms ARMED")
        self.assertFalse(n.arm_request["pending"])

    def test_manual_arm_and_kill_preserve_px4_decisions(self):
        for change in (lambda n: setattr(n.core, "mode", "idle"),
                       lambda n: setattr(n.core, "mode", "failsafe"),
                       lambda n: setattr(n.fc, "landed", False),
                       lambda n: setattr(n.fc, "offboard_active", False),
                       lambda n: setattr(n.fc, "t", 18),
                       lambda n: setattr(n.fc, "px4_link", False),
                       lambda n: setattr(n.fc, "armed", True),
                       lambda n: setattr(n, "fc", None),
                       lambda n: setattr(n.pos, "valid", False),
                       lambda n: setattr(n, "batt", (20, 16, 0.0, True)),
                       lambda n: setattr(n, "batt", (20, 16, math.nan, True)),
                       lambda n: n.vehicle[1].update(preflight_ok=False)):
            n = self.arm_node()
            change(n)
            n.readiness = Mock(side_effect=AssertionError("arm/disarm must not consult readiness"))
            for op in ("arm", "disarm", "disarm"):
                self.assertTrue(DashNode.do_command(n, 20, "tester", {"op": op, "auto_course": False})[0])
                m = n.arm_pub.publish.call_args.args[0]
                self.assertEqual((m.param1, m.param2), (1., 0.) if op == "arm" else (0., 21196.))
            self.assertEqual(n.arm_pub.publish.call_count, 3)

    def test_arm_disarm_still_require_owner_and_live_mode(self):
        n = self.arm_node()
        n.a.live = False
        for op in ("arm", "disarm"):
            self.assertFalse(DashNode.do_command(n, 20, "tester", {"op": op, "auto_course": False})[0])
        n.arm_pub.publish.assert_not_called()
        n = self.arm_node()
        n.core.owner = "another"
        self.assertFalse(DashNode.do_command(n, 20, "tester", {"op": "arm", "auto_course": False})[0])
        n.arm_pub.publish.assert_not_called()
        self.assertTrue(DashNode.do_command(n, 20, "tester", {"op": "disarm"})[0])  # kill needs no control
        self.assertEqual(n.arm_pub.publish.call_args.args[0].param2, 21196.)

    def test_disarm_works_despite_failed_readiness_and_reports_denial(self):
        n = self.arm_node()
        n.fc.armed, n.fc.landed = True, False
        n.core.mode = "failsafe"
        n.pos.valid = False
        n.vehicle[1]["preflight_ok"] = False
        n.batt = (20, 16, 0.0, True)
        self.assertTrue(DashNode.do_command(n, 20, "viewer", {"op": "disarm"})[0])  # kill needs no control
        self.assertTrue(DashNode.do_command(n, 20, "tester", {"op": "disarm"})[0])
        m = n.arm_pub.publish.call_args.args[0]
        self.assertEqual((m.param1, m.param2), (0., 21196.))
        DashNode._arm_ack(n, SimpleNamespace(command=m.command, target_system=12,
            target_component=ARM_SOURCE_COMPONENT, result=2))
        self.assertEqual(n.arm_request["message"], "PX4 arm/disarm: denied")
        self.assertFalse(n.arm_request["pending"])
        self.assertTrue(n.fc.armed)

    def test_timeout_never_retries_or_infers_armed_state(self):
        n = self.arm_node()
        self.assertTrue(DashNode.do_command(n, 20, "tester", {"op": "arm", "auto_course": False})[0])
        n.update_arm_request(25)
        self.assertFalse(n.arm_request["pending"])
        self.assertIn("not confirmed", n.arm_request["message"])
        self.assertFalse(n.fc.armed)
        self.assertEqual(n.arm_pub.publish.call_count, 1)

    def test_reset_home_clears_targets_without_changing_px4_position(self):
        n = self.arm_node()
        n.freeze_interface = Mock()
        n.pos = Pos(20, 4, -3, -.8, math.pi / 2)
        self.assertTrue(DashNode.do_command(n, 20, "tester", {"op": "reset"})[0])
        self.assertEqual(n.core.home, (4, -3, -.8, math.pi / 2))
        self.assertEqual(n.core.mode, "idle")
        self.assertEqual(n.core.target, [4, -3, -.8])
        self.assertEqual(n.core.status(20)["home"], {"x":4,"y":-3,"z":-.8,"yaw_deg":90})
        self.assertIsNone(n.core.step(20, n.pos, n.fc))
        self.assertEqual((n.pos.x,n.pos.y,n.pos.z), (4,-3,-.8))
        n.freeze_interface.assert_called_once()
        n.arm_pub.publish.assert_not_called()

    def test_reset_home_refuses_in_air_armed_and_stale_data(self):
        for change in (lambda n: setattr(n.fc, "armed", True),
                       lambda n: setattr(n.fc, "landed", False),
                       lambda n: setattr(n.fc, "t", 18),
                       lambda n: setattr(n.pos, "t", 18)):
            n = self.arm_node()
            n.freeze_interface = Mock()
            change(n)
            home_before = n.core.home
            self.assertFalse(DashNode.do_command(n, 20, "tester", {"op": "reset"})[0])
            self.assertEqual(n.core.home, home_before)
            n.freeze_interface.assert_not_called()

    def test_range_height_requires_valid_range_sensor_estimate(self):
        n = self.arm_node()
        m = SimpleNamespace(x=0.,y=0.,z=-.5,heading=0.,xy_valid=True,z_valid=True,
            xy_reset_counter=0,heading_reset_counter=0,dist_bottom=.5,timestamp_sample=0,
            dist_bottom_valid=True,dist_bottom_sensor_bitfield=1)
        DashNode._lpos(n,m)
        self.assertEqual(n.range_height[1:],(.5,True))
        m.dist_bottom_valid=False
        DashNode._lpos(n,m)
        self.assertFalse(n.range_height[2])
        m.dist_bottom_valid=True; m.dist_bottom_sensor_bitfield=2
        DashNode._lpos(n,m)
        self.assertFalse(n.range_height[2])


if __name__ == "__main__":
    unittest.main()
