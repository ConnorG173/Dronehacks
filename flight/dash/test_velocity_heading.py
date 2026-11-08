"""Regression checks for the near-start spin; pure modules, no ROS or hardware."""
import ast
import importlib.util
import json
import math
from pathlib import Path
import threading
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock

import numpy as np

from alternate_velocity import (AlternateVelocityRun, ALIGN_TIMEOUT_S, ALIGN_YAW_RATE,
                                ALIGN_YAW_ACCEL, MAX_LINEAR_ACCEL, MAX_VERTICAL_ACCEL)
from alt_tracker.pathfile import Trajectory
from velocity_tracker import Tracker, TrackerParams
from course_follow import CourseRun
from course_state import CourseState
from dash_core import DashCore, Fc, Pos
from path_heading import wrap_angle, yaw_quaternion
import os as _os
from unittest.mock import patch as _patch
# These tests check the dashboard's in-flight holds, now off by default (DashParams.no_holds).
LEGACY_HOLDS = _patch.dict(_os.environ, {'DASH_LEGACY_HOLDS': '1'})

HERE = Path(__file__).resolve().parent


def straight(heading=0.):
    return dict(points=[[i*.1*math.cos(heading), i*.1*math.sin(heading), 1.] for i in range(21)],
                times_s=[i*.2 for i in range(21)],
                anchor=dict(origin_enu=[0., 0., 0.], yaw_rad=0.), reference=('offline', 1, 1))


def prepared(heading=0., yaw=0., path=None):
    path = path or straight(heading)
    core = DashCore()
    core.mode, core.owner, core.owner_hb, core.was_offboard = 'engaged', 'offline', 1., True
    core.home = (0., 0., 0., 0.)
    core.cmd, core.target = list(path['points'][0]), list(path['points'][0])
    core.last_step = 1.
    core.course_controller = 'velocity_tracker'
    core.heading.reset(yaw)
    pos = Pos(1., *path['points'][0], yaw, True, 0., 0.)
    fc = Fc(1., True, True, True, False)
    ok, reason = core.start_course(1., path, 6., pos, fc)
    assert ok, reason
    return core, pos, fc


def tick(core, pos, fc, now, heartbeat=True):
    pos.t = fc.t = now
    if heartbeat:
        core.heartbeat(now, 'offline')
    return core.step(now, pos, fc)


class InitialHeadingTests(unittest.TestCase):
    def test_old_caller_cannot_silently_drop_the_absolute_heading_command(self):
        validated = CourseRun(straight(), 6., 1., .3, .3, 2.)
        with self.assertRaisesRegex(ValueError, 'dashboard restart'):
            AlternateVelocityRun(validated)

    def test_initial_turn_holds_a_fixed_position_and_uses_absolute_yaw(self):
        core, pos, fc = prepared(math.pi/2)
        held = tuple(core.cmd)
        previous_yaw, previous_rate = 0., 0.
        for n in range(1, 31):
            pos.x = .01*math.sin(n)
            output = tick(core, pos, fc, 1.+n*.1)
            if n == 1:
                held = output[1]
            self.assertEqual(output, ('pose', held))
            self.assertEqual(core.course_yaw_rate, 0.)
            self.assertEqual(core.course_run.elapsed, 0.)
            self.assertEqual(core.heading.phase, 'initial_align')
            commanded = core.heading.yaw
            self.assertLessEqual(abs(wrap_angle(commanded-previous_yaw)), ALIGN_YAW_RATE*.1+1e-9)
            rate = core.course_run._alignment_turn.rate
            self.assertLessEqual(abs(rate-previous_rate), ALIGN_YAW_ACCEL*.1+1e-9)
            previous_yaw, previous_rate = commanded, rate
        self.assertAlmostEqual(core.course_run.desired_heading, math.pi/2)

    def test_crossing_target_while_still_rotating_does_not_release_translation(self):
        core, pos, fc = prepared()
        for n in range(1, 31):
            pos.yaw = math.radians(2. if n % 2 else -2.)
            output = tick(core, pos, fc, 1.+n*.1)
            self.assertEqual(output[0], 'pose')
            self.assertFalse(core.course_run._initial_aligned)
            self.assertEqual(core.course_run.elapsed, 0.)
        pos.yaw = 0.
        for n in range(31, 36):
            tick(core, pos, fc, 1.+n*.1)
        self.assertTrue(core.course_run._initial_aligned)
        self.assertGreater(core.course_run.elapsed, 0.)

    def test_shortest_absolute_turn_converges_with_yaw_lag(self):
        core, pos, fc = prepared(math.radians(170), math.radians(-170))
        start = pos.yaw
        commanded_travel = 0.
        previous = core.heading.yaw
        for n in range(1, 121):
            out = tick(core, pos, fc, 1.+n*.1)
            if out[0] == 'twist':
                break
            commanded_travel += abs(wrap_angle(core.heading.yaw-previous))
            previous = core.heading.yaw
            pos.yaw += (1-math.exp(-2.8*.1))*wrap_angle(core.heading.yaw-pos.yaw)
        self.assertTrue(core.course_run.active, core.course_run.reason)
        self.assertTrue(core.course_run._initial_aligned)
        self.assertLess(commanded_travel, math.radians(21))
        self.assertLess(abs(wrap_angle(pos.yaw-start)), math.radians(21))
        self.assertLess(abs(wrap_angle(pos.yaw-math.radians(170))), math.radians(5))

    def test_unresponsive_heading_times_out_and_never_restarts_itself(self):
        core, pos, fc = prepared(math.pi/2)
        for n in range(1, int((ALIGN_TIMEOUT_S+1)/.1)):
            self.assertEqual(tick(core, pos, fc, 1.+n*.1)[0], 'pose')
        self.assertFalse(core.course_run.active)
        self.assertEqual(core.course_run.state, 'aborted')
        self.assertIn('did not settle', core.course_run.reason)
        self.assertEqual(core.course_run.elapsed, 0.)
        pos.yaw = math.pi/2
        for n in range(140, 150):
            self.assertEqual(tick(core, pos, fc, 1.+n*.1)[0], 'pose')
            self.assertFalse(core.course_run.active)

    def test_timeout_holds_latest_actual_position(self):
        core, pos, fc = prepared(math.pi/2)
        tick(core, pos, fc, 1.1)
        pos.x = .1
        for n in range(2, 132):
            output = tick(core, pos, fc, 1.+n*.1)
        self.assertEqual(core.course_run.state, 'aborted')
        self.assertEqual(output, ('pose', (pos.x, pos.y, pos.z)))


class VelocityHeadingTests(unittest.TestCase):
    def run_for(self, path=None):
        run = AlternateVelocityRun(CourseRun(path or straight(), 6., 1., .3, .3, 2.),
                                   position_hold_supported=True)
        run._initial_aligned = True
        return run

    def test_backward_braking_and_cross_track_correction_do_not_turn_the_camera(self):
        for command in ([-.5, 0., 0.], [0., .3, 0.], [-1e-5, -1e-5, 0.]):
            run = self.run_for()
            run.tracker.step = lambda *args, **kwargs: (np.array(command), 0.)
            output, rate = run.step(.1, [0., 0., 1.], 0., 0.)
            self.assertEqual(run.phase, 'following')
            self.assertAlmostEqual(run.desired_heading, 0.)
            self.assertAlmostEqual(rate, 0.)
            self.assertLessEqual(np.linalg.norm(output), MAX_LINEAR_ACCEL*.1+1e-9)

    def test_saved_flight_progress_keeps_the_geometric_heading(self):
        state = CourseState(HERE/'course.json')
        state.capture_anchor([0.,0.,0.], 0.)
        run = self.run_for(state.original_path())  # Replay the recorded original-route incident.
        run.tracker.reset(tau=.99, taudot=.104)
        reference = run.traj.sample(.99)
        heading = reference[3]
        run.tracker.step = lambda *args, **kwargs: (np.array([0., -.15, 0.]), 0.)
        _, rate = run.step(.1, reference[0], heading, .05)
        self.assertAlmostEqual(run.desired_heading, heading)
        self.assertEqual(run.phase, 'following')
        self.assertLess(abs(rate), math.radians(5))
        self.assertGreater(math.degrees(heading), 90.)

    def test_heading_wait_freezes_path_and_keeps_its_target(self):
        run = self.run_for()
        run.tracker.reset(tau=1., taudot=.8)
        run.last_velocity = [.5, 0., 0.]
        previous = np.array(run.last_velocity)
        for n in range(10):
            out, _ = run.step(.1, [.5, .01*n, 1.], math.radians(20), .1)
            self.assertEqual(run.elapsed, 1.)
            self.assertEqual(run.phase, 'heading_wait')
            self.assertAlmostEqual(run.desired_heading, 0.)
            self.assertLessEqual(np.linalg.norm(np.array(out)-previous), MAX_LINEAR_ACCEL*.1+1e-9)
            previous = np.array(out)
        np.testing.assert_allclose(previous, np.zeros(3), atol=1e-9)
        for _ in range(20):
            run.step(.1, [.5, 0., 1.], 0., 0.)
        self.assertGreater(run.elapsed, 1.)

    def test_heading_wait_timeout_is_latched(self):
        run = self.run_for()
        for _ in range(130):
            run.step(.1, [0., 0., 1.], math.pi/2, 0.)
        self.assertFalse(run.active)
        self.assertEqual(run.state, 'aborted')
        for _ in range(10):
            self.assertEqual(run.step(.1, [0., 0., 1.], 0., 0.), ([0.,0.,0.],0.))
        self.assertEqual(run.elapsed, 0.)

    def test_ordinary_velocity_changes_have_acceleration_and_magnitude_bounds(self):
        run = self.run_for()
        previous = np.zeros(3)
        for command in ([1.,0.,.3], [-1.,0.,-.3], [0.,1.,.3], [0.,0.,0.]):
            run.tracker.step = lambda *args, **kwargs: (np.array(command), 0.)
            for _ in range(12):
                output, _ = run.step(.1, [0.,0.,1.], 0., 0.)
                output = np.array(output)
                self.assertLessEqual(np.linalg.norm(output-previous), MAX_LINEAR_ACCEL*.1+1e-9)
                self.assertLessEqual(abs(output[2]-previous[2]), MAX_VERTICAL_ACCEL*.1+1e-9)
                self.assertLessEqual(np.linalg.norm(output), 1.+1e-9)
                self.assertLessEqual(abs(output[2]), .3+1e-9)
                previous = output

    def test_pause_stop_and_invalid_input_do_not_resume(self):
        run = self.run_for()
        before = run.elapsed
        self.assertEqual(run.step(.1, [0.,0.,1.], 0., None)[0], [0.,0.,0.])
        self.assertEqual(run.elapsed, before)
        run.stop()
        self.assertEqual(run.step(.1, [0.,0.,1.], 0., 0.), ([0.,0.,0.],0.))
        run = self.run_for()
        self.assertEqual(run.step(.1, [math.nan,0.,1.], 0., 0.), ([0.,0.,0.],0.))
        self.assertFalse(run.active)

    @LEGACY_HOLDS
    def test_missing_authority_and_heartbeat_still_cancel_alignment(self):
        for change in ('offboard', 'link', 'heartbeat'):
            core, pos, fc = prepared(math.pi/2)
            tick(core, pos, fc, 1.1)
            pos.t = fc.t = 3.
            if change == 'offboard':
                fc.offboard_active = False
            if change == 'link':
                fc.px4_link = False
            if change != 'heartbeat':
                core.heartbeat(3., 'offline')
            self.assertIsNone(core.step(3., pos, fc))
            self.assertFalse(core.course_run.active)
            fc.offboard_active = fc.px4_link = True
            self.assertIsNone(tick(core, pos, fc, 3.1))

    def test_manual_hold_cancels_alignment(self):
        core, pos, fc = prepared(math.pi/2)
        tick(core, pos, fc, 1.1)
        self.assertTrue(core.command(1.1,'offline','hold',pos,fc)[0])
        self.assertFalse(core.course_run.active)
        self.assertEqual(tick(core,pos,fc,1.2)[0],'pose')


class InverseClockTests(unittest.TestCase):
    def test_inverse_feedforward_includes_speeding_up_and_braking_the_clock(self):
        t = np.linspace(0., 10., 101)
        points = np.c_[t*.5, np.zeros(len(t)), np.ones(len(t))]
        velocity = np.tile([.5,0.,0.],(len(t),1))
        traj = Trajectory(t, points, velocity, np.zeros_like(velocity), np.zeros(len(t)),np.zeros(len(t)))
        for old_rate, scale, expected in ((.4,1.,.525),(.5,0.,-.075),(.5,.5,.25)):
            tracker = Tracker(traj, TrackerParams(t_delay=0., v_max=2.))
            tracker.reset(tau=2.,taudot=old_rate)
            tracker.step(.1,[1.,0.,1.],0.,speed_scale=scale)
            self.assertAlmostEqual(tracker.last['v_ff'][0], expected)


class DashboardPublicationTests(unittest.TestCase):
    def test_real_tick_publishes_one_pose_for_alignment_then_twist_for_tracking(self):
        # Extract the actual callback without importing/initializing ROS.
        module = ast.parse((HERE/'dash_node.py').read_text())
        cls = next(n for n in module.body if isinstance(n,ast.ClassDef) and n.name=='DashNode')
        callback = next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='_tick')
        limit = next(n for n in module.body if isinstance(n,ast.FunctionDef) and n.name=='velocity_controller_limit')
        def pose():
            return NS(header=NS(),pose=NS(position=NS(),orientation=NS()))
        def twist():
            return NS(header=NS(),twist=NS(linear=NS(),angular=NS()))
        clock = [1.1]
        env = dict(time=NS(monotonic=lambda:clock[0]), math=math,json=json,
                   PoseStamped=pose,TwistStamped=twist,yaw_quaternion=yaw_quaternion)
        exec(compile(ast.Module(body=[limit,callback],type_ignores=[]),'actual_dash_tick','exec'),env)
        core,pos,fc=prepared(math.pi/2)
        pos.t=fc.t=1.1;core.heartbeat(1.1,'offline')
        poses,twists=[],[]
        node=NS(core=core,pos=pos,fc=fc,lock=threading.Lock(),a=NS(live=True),
                update_arm_request=Mock(),readiness=Mock(return_value=[]),
                fc_raw={'velocity_timeout_hold_current':True,'envelope':{'max_speed_mps':1.}},
                course=NS(execution_reference=lambda:core.course_run.reference),
                depth_overlay=NS(snapshot=lambda now:None),
                get_clock=lambda:NS(now=lambda:NS(to_msg=lambda:NS())), get_logger=lambda:Mock(),
                pub_pose=NS(publish=poses.append),pub_twist=NS(publish=twists.append),
                freeze_interface=Mock(),n_out=0)
        env['_tick'](node)
        self.assertEqual((len(poses),len(twists)),(1,0))
        q=poses[0].pose.orientation
        commanded=math.atan2(2*q.w*q.z,1-2*q.z*q.z)
        self.assertGreater(commanded,0.)
        self.assertLessEqual(commanded,ALIGN_YAW_RATE*.1)
        core.course_run._initial_aligned=True
        pos.yaw=math.pi/2
        clock[0]=pos.t=fc.t=1.2;core.heartbeat(1.2,'offline')
        env['_tick'](node)
        self.assertEqual((len(poses),len(twists)),(1,1))
        self.assertEqual(twists[0].header.frame_id,'odom')
        self.assertLessEqual(math.hypot(twists[0].twist.linear.x,twists[0].twist.linear.y),.100001)
        node.freeze_interface.assert_not_called()


if __name__ == '__main__':
    unittest.main()
