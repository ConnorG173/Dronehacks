"""Ground-reference recovery and reporting with inert service/publisher doubles."""
from concurrent.futures import Future
import math
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import test_autoflight_api_offline as api
from course_safety import MAX_COURSE_SPEED_MPS


class GroundReferenceTests(unittest.TestCase):
    node = api.FlightApiTests.node

    def grounded(self):
        n = self.node()
        n.fc.origin_z = -1.13
        n.fc_raw = dict(altitude_reference='px4', setpoint_mode='position',
                        origin_enu=[0.,0.,-1.13])
        n.other_setpoint_publishers = lambda: 0
        n.hold_client = Mock()
        n.hold_client.service_is_ready.return_value = True
        n.hold_client.call_async.return_value = Future()
        context = patch.dict(api.DashNode.sync_ground_reference.__globals__,
                             Trigger=SimpleNamespace(Request=object),
                             MAX_COURSE_SPEED_MPS=MAX_COURSE_SPEED_MPS)
        context.start(); self.addCleanup(context.stop)
        return n

    def test_grounded_mismatch_requests_one_hold_and_never_arms(self):
        n = self.grounded()
        n.sync_ground_reference(10.)
        n.sync_ground_reference(10.)
        n.hold_client.call_async.assert_called_once()
        n.arm_pub.publish.assert_not_called()
        n.pub_pose.publish.assert_not_called()
        self.assertEqual(n.fc.origin_z, -1.13)  # Never forge the interface origin.

    def test_airborne_stale_prepared_or_competing_control_never_refreshes(self):
        changes = [lambda n:setattr(n.fc,'armed',True),
            lambda n:setattr(n.fc,'landed',False),
            lambda n:setattr(n,'land',(10.,False)),
            lambda n:setattr(n,'land',(8.,True)),
            lambda n:setattr(n.pos,'t',8.),
            lambda n:setattr(n.fc,'t',8.),
            lambda n:setattr(n.pos,'z',math.nan),
            lambda n:setattr(n.core,'mode','engaged'),
            lambda n:setattr(n.core,'deferred_course',{}),
            lambda n:setattr(n,'arm_request',dict(pending=True)),
            lambda n:setattr(n,'other_setpoint_publishers',lambda:1)]
        for change in changes:
            with self.subTest(change=change):
                n=self.grounded();change(n);n.sync_ground_reference(10.)
                n.hold_client.call_async.assert_not_called()

    def test_pending_hold_blocks_launch_even_after_origin_appears_aligned(self):
        n=self.grounded();n.sync_ground_reference(10.)
        n.fc.origin_z=0.
        for op in ('launch','launch_setup','arm','engage','reset'):
            ok,message=n.do_command(10.,'pilot',dict(op=op,flight='course'))
            self.assertFalse(ok,op)
            self.assertIn('Refreshing ground height',message)
        n.sync_ground_reference(100.)  # A timeout cannot release a delayed Hold.
        self.assertIsNotNone(n._ground_sync)
        n.arm_pub.publish.assert_not_called()

    def test_service_success_requires_new_grounded_held_aligned_state(self):
        n=self.grounded();n.sync_ground_reference(10.)
        n.hold_client.call_async.return_value.set_result(SimpleNamespace(success=True))
        n.sync_ground_reference(10.)
        n.fc.origin_z=0.;n.fc_raw['setpoint_mode']='hold'
        n.sync_ground_reference(10.)
        self.assertIsNotNone(n._ground_sync)  # Earlier telemetry is not confirmation.
        n.fc.t=n.pos.t=10.1;n.land=(10.1,True);n.fc.armed=True
        n.sync_ground_reference(10.1)
        self.assertIsNotNone(n._ground_sync)
        n.fc.armed=False;n.fc_raw['setpoint_mode']='position'
        n.sync_ground_reference(10.1)
        self.assertIsNotNone(n._ground_sync)
        n.fc_raw['setpoint_mode']='hold';n.sync_ground_reference(10.1)
        self.assertIsNone(n._ground_sync)
        n.sync_ground_reference(10.1)
        n.hold_client.call_async.assert_called_once()

    def test_transport_error_does_not_release_uncertain_hold(self):
        n=self.grounded();n.sync_ground_reference(10.)
        n.hold_client.call_async.return_value.set_exception(RuntimeError('transport'))
        n.sync_ground_reference(10.)
        self.assertIsNotNone(n._ground_sync)
        self.assertFalse(n.do_command(10.,'pilot',dict(op='arm'))[0])

    def test_ground_readiness_waits_for_service_but_runtime_does_not(self):
        n=self.grounded();n.sync_ground_reference(10.)
        n.vo_t=10.;n.vo_state=1;n.batt=(10.,16.,.8,True)
        n.vehicle=(10.,dict(failsafe=False))
        why=api.DashNode.readiness(n,10.,preflight=True)
        self.assertIn('Refreshing ground height; waiting for the flight interface',why)
        self.assertEqual(api.DashNode.readiness(n,10.,preflight=False),[])

    def test_airborne_failsafe_snapshot_uses_runtime_not_ground_checks(self):
        n=self.grounded();n.fc.armed=True;n.fc.landed=False;n.core.mode='failsafe'
        n.pose_topic='/skybeta/setpoint/pose';n.vo_t=None;n.batt=None
        n.vehicle=None;n.range_height=None
        n.snapshot(10.,'pilot')
        self.assertTrue(n.readiness.call_args_list)
        self.assertTrue(all(call.kwargs.get('preflight') is False
                            for call in n.readiness.call_args_list))


if __name__ == '__main__':
    unittest.main()
