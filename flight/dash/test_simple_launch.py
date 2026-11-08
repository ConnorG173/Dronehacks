"""Single-click takeoff orchestration using production methods and inert I/O."""
import copy
import math
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import test_autoflight_api_offline as api


class SimpleLaunchTests(unittest.TestCase):
    node = api.FlightApiTests.node

    def assert_one_normal_arm(self, n):
        n.arm_pub.publish.assert_called_once()
        sent = n.arm_pub.publish.call_args.args[0]
        self.assertEqual((sent.command, sent.param1, sent.param2), (400, 1., 0.))
        self.assertEqual((sent.target_system, sent.target_component), (12, 1))
        self.assertTrue(n.arm_request['launch'])

    def assert_refusal_unchanged(self, n, body, client='pilot'):
        original = n.core
        before = copy.deepcopy(n.core.status(10))
        anchor = n.course.execution_reference()
        ok, message = n.do_command(10, client, body)
        self.assertFalse(ok, message)
        self.assertIs(n.core, original)
        self.assertEqual(n.core.status(10), before)
        self.assertEqual(n.course.execution_reference(), anchor)
        n.arm_pub.publish.assert_not_called()
        n.pub_pose.publish.assert_not_called()
        n.pub_twist.publish.assert_not_called()
        n.freeze_interface.assert_not_called()
        return message

    def test_hover_claims_resets_prepares_and_requests_one_arm_without_depth(self):
        n = self.node()
        n.core.owner = None
        n.core.mode, n.core.reason = 'failsafe', 'dashboard heartbeat lost'
        n.pos.x, n.pos.y, n.pos.z, n.pos.yaw = 1., -2., -6., .2
        n.fc.origin_z = -6.
        n.data = {'status': 'unscaled', 'points': []}
        ok, message = n.do_command(10, 'pilot', {'op': 'launch', 'flight': 'hover', 'alt': .7})
        self.assertTrue(ok, message)
        self.assertEqual(n.core.owner, 'pilot')
        self.assertEqual(n.core.mode, 'engaged')
        self.assertEqual(n.core.home, (1., -2., -6., .2))
        self.assertEqual(n.core.target, [1., -2., -5.3])
        self.assertIsNone(n.core.queued_course)
        self.assertIsNone(n.core.course_run)
        self.assert_one_normal_arm(n)
        n.pub_pose.publish.assert_not_called()
        n.pub_twist.publish.assert_not_called()
        # Still disarmed: no rising takeoff target is emitted. Armed telemetry
        # then starts only the vertical ramp at the selected location.
        for armed, now in ((False, 10.1), (True, 10.2)):
            n.fc.armed = armed
            n.pos.t = n.fc.t = now
            n.core.heartbeat(now, 'pilot')
            with patch.object(time, 'monotonic', return_value=now):
                n._tick()
            sent = n.pub_pose.publish.call_args.args[0].pose.position
            self.assertEqual((sent.x, sent.y), (1., -2.))
            self.assertAlmostEqual(sent.z, -5.67 if armed else -5.7)
        n.arm_pub.publish.assert_called_once()

    def test_course_launch_preserves_reference_and_defers_horizontal_route(self):
        for controller in ('smooth_position', 'velocity_tracker'):
            with self.subTest(controller=controller):
                n = self.node()
                n.core.owner = None
                n.core.mode = 'failsafe'
                n.core.course_controller = controller
                n.fc_raw = dict(velocity_timeout_hold_current=True, envelope={'max_speed_mps': 1.})
                home, reference = n.core.home, n.course.execution_reference()
                ok, message = n.do_command(10, 'pilot', {'op': 'launch', 'flight': 'course'})
                self.assertTrue(ok, message)
                self.assertEqual(n.core.home, home)
                self.assertEqual(n.course.execution_reference(), reference)
                self.assertEqual(n.core.deferred_course['reference'], reference)
                self.assertIsNone(n.core.queued_course)
                self.assertIsNone(n.core.course_run)
                self.assertEqual(n.core.launch_state, 'waiting_arm')
                self.assert_one_normal_arm(n)

    def test_setup_sets_start_without_requiring_depth_or_sending_arm(self):
        n = self.node()
        n.core.owner = None
        n.core.mode = 'failsafe'
        n.pos.x, n.pos.y, n.pos.z, n.pos.yaw = 1., 2., -6., .3
        n.fc.origin_z = -6.
        n.data = dict(status='live', age_s=.1, points=[])
        n.readiness.side_effect = AssertionError('setup does not fly')
        ok, message = n.do_command(10, 'pilot', {'op': 'launch_setup', 'flight': 'course'})
        self.assertTrue(ok, message)
        self.assertEqual(n.core.mode, 'idle')
        self.assertEqual(n.core.home, (1., 2., -6., .3))
        anchor = n.course.snapshot(10)['anchor']
        self.assertEqual(anchor['origin_enu'], [1., 2., -6.])
        self.assertEqual(anchor['yaw_rad'], .3)
        self.assertIsNone(n.core.queued_course)
        n.arm_pub.publish.assert_not_called()
        n.pub_pose.publish.assert_not_called()
        n.pub_twist.publish.assert_not_called()
        n.freeze_interface.assert_called_once()

    def test_invalid_takeoff_never_changes_owner_fault_or_preparation(self):
        changes = {
            'preview': lambda n: setattr(n.a, 'live', False),
            'stale_position': lambda n: setattr(n.pos, 't', 9.),
            'future_position': lambda n: setattr(n.pos, 't', 11.),
            'invalid_position': lambda n: setattr(n.pos, 'valid', False),
            'nan_position': lambda n: setattr(n.pos, 'x', math.nan),
            'stale_status': lambda n: setattr(n.fc, 't', 8.),
            'future_status': lambda n: setattr(n.fc, 't', 11.),
            'lost_link': lambda n: setattr(n.fc, 'px4_link', False),
            'nan_ground': lambda n: setattr(n.fc, 'origin_z', math.nan),
            'armed': lambda n: setattr(n.fc, 'armed', True),
            'airborne': lambda n: setattr(n.fc, 'landed', False),
            'stale_landed': lambda n: setattr(n, 'land', (8., True)),
            'future_landed': lambda n: setattr(n, 'land', (11., True)),
            'raw_airborne': lambda n: setattr(n, 'land', (10., False)),
            'health': lambda n: setattr(n.readiness, 'return_value', ['PX4 failsafe active']),
            'unknown_velocity': lambda n: setattr(n.pos, 'vz', None),
            'moving': lambda n: setattr(n.pos, 'vx', .2),
            'moving_vertical': lambda n: setattr(n.pos, 'vz', .11),
        }
        for flight in ('hover', 'course'):
            for name, change in changes.items():
                with self.subTest(flight=flight, case=name):
                    n = self.node()
                    n.core.mode, n.core.reason = 'failsafe', 'prior stop'
                    n.core.owner = None
                    change(n)
                    self.assert_refusal_unchanged(n, {'op': 'launch', 'flight': flight})
        for flight in ('hover', 'course'):
            n = self.node()
            self.assert_refusal_unchanged(n, {'op': 'launch', 'flight': flight}, 'other_dashboard')

    def test_stale_owner_can_launch_but_invalid_request_does_not_steal_claim(self):
        n = self.node()
        n.core.owner_hb = 6.
        self.assert_refusal_unchanged(n, {'op': 'launch', 'flight': 'hover', 'alt': 4.}, 'other')
        self.assertTrue(n.do_command(10, 'other', {'op': 'launch', 'flight': 'hover', 'alt': .5})[0])
        self.assertEqual(n.core.owner, 'other')
        self.assert_one_normal_arm(n)

    def test_blocked_course_never_falls_back_to_hover_or_reanchors(self):
        for case in ('occupied_column', 'reference', 'position', 'radius', 'velocity_interface'):
            with self.subTest(case=case):
                n = self.node()
                body = {'op': 'launch', 'flight': 'course'}
                if case == 'occupied_column': n.data['points'] = [[0., 0.]]
                elif case == 'reference': n.core.home = (1., 0., 0., 0.)
                elif case == 'position': n.pos.x = .2
                elif case == 'radius': body['radius_m'] = 1.
                else: n.core.course_controller = 'velocity_tracker'
                self.assert_refusal_unchanged(n, body)

    def test_explicit_mode_and_valid_height_are_required(self):
        for body in ({'op': 'launch'}, {'op': 'launch', 'flight': 'invalid'},
                     {'op': 'launch_setup', 'flight': 'hover'},
                     {'op': 'launch', 'flight': 'hover', 'alt': True},
                     {'op': 'launch', 'flight': 'hover', 'alt': math.nan},
                     {'op': 'launch', 'flight': 'hover', 'alt': 0.},
                     {'op': 'launch', 'flight': 'hover', 'alt': 3.}):
            self.assert_refusal_unchanged(self.node(), body)
        self.assert_refusal_unchanged(self.node(), {'op': 'launch', 'flight': 'hover'}, '')

    def test_duplicate_clicks_do_not_reset_or_repeat_pending_arm(self):
        n = self.node()
        body = {'op': 'launch', 'flight': 'hover'}
        self.assertTrue(n.do_command(10, 'pilot', body)[0])
        original = n.core
        for op in ('launch', 'launch_setup'):
            ok, message = n.do_command(10.2, 'pilot', dict(body, op=op, flight='course'))
            self.assertFalse(ok, message)
            self.assertIn('previous arming request', message)
            self.assertIs(n.core, original)
        self.assert_one_normal_arm(n)

    def test_simultaneous_clicks_send_one_arm(self):
        n = self.node()
        barrier = threading.Barrier(3)
        results = []
        def click():
            barrier.wait()
            results.append(n.do_command(10, 'pilot', {'op': 'launch', 'flight': 'hover'})[0])
        workers = [threading.Thread(target=click) for _ in range(2)]
        for worker in workers: worker.start()
        barrier.wait()
        for worker in workers: worker.join(timeout=3)
        self.assertTrue(all(not worker.is_alive() for worker in workers))
        self.assertCountEqual(results, [True, False])
        self.assert_one_normal_arm(n)

    def test_denied_or_unconfirmed_takeoff_cancels_preparation_without_retry(self):
        for flight in ('hover', 'course'):
            for failure in ('denied', 'timeout'):
                with self.subTest(flight=flight, failure=failure):
                    n = self.node()
                    self.assertTrue(n.do_command(10, 'pilot', {'op': 'launch', 'flight': flight})[0])
                    if failure == 'denied':
                        ack = SimpleNamespace(command=400, target_system=12, target_component=197, result=2)
                        with patch.object(time, 'monotonic', return_value=10.1): n._arm_ack(ack)
                    else:
                        n.update_arm_request(15.)
                    self.assertFalse(n.arm_request['pending'])
                    self.assertEqual(n.core.mode, 'failsafe')
                    self.assertIsNone(n.core.queued_course)
                    self.assertIsNone(n.core.deferred_course)
                    n.freeze_interface.assert_called_once()
                    # A later manual arm cannot resume the rejected takeoff.
                    n.fc.armed = True
                    n.pos.t = n.fc.t = 16.
                    n.core.heartbeat(16., 'pilot')
                    with patch.object(time, 'monotonic', return_value=16.): n._tick()
                    n.pub_pose.publish.assert_not_called()
                    n.pub_twist.publish.assert_not_called()
                    self.assert_one_normal_arm(n)


class TakeoffSetsOffboardTests(unittest.TestCase):
    node = SimpleLaunchTests.node
    def test_takeoff_outside_offboard_requests_offboard_instead_of_refusing(self):
        for flight in ('hover', 'course'):
            with self.subTest(flight=flight):
                n = self.node(); n.fc.offboard_active = False
                ok, message = n.do_command(10, 'pilot', {'op': 'launch', 'flight': flight})
                self.assertTrue(ok, message)
                sent = [c.args[0] for c in n.arm_pub.publish.call_args_list]
                self.assertEqual([(s.command, s.param1, s.param2) for s in sent], [(176, 1., 6.)])


if __name__ == '__main__':
    unittest.main()
