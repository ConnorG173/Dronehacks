import math
import tempfile
import unittest
from pathlib import Path

from course_state import CourseState
from dash_core import DashCore, DashParams, Pos

ROUTE = [[0., i/10, 1.] for i in range(61)]          # course frame: 6 m forward... (x fwd, y left)


def state(tmp, route=ROUTE):
    st = CourseState(Path(tmp) / 'course.json')
    assert st.import_route([[p[1], p[0], p[2]] for p in route])[0]   # forward along +x of the course
    return st


class MarksTests(unittest.TestCase):
    def test_marks_need_home_are_course_relative_and_persist(self):
        with tempfile.TemporaryDirectory() as tmp:
            st = state(tmp)
            self.assertFalse(st.edit_marks('add_hole', world_xyz=[1, 2, 1])[0])      # no anchor yet
            self.assertTrue(st.capture_anchor([10., 20., .5], math.pi/2)[0])         # home facing +y
            self.assertTrue(st.edit_marks('add_hole', world_xyz=[10., 23., 1.5])[0])  # 3 m ahead, 1 m up
            self.assertTrue(st.edit_marks('add_zone', world_xyz=[9., 22., 0.], radius_m=1.2)[0])
            hole = st.marks['holes'][0]
            self.assertAlmostEqual(hole[0], 3., places=6)       # forward
            self.assertAlmostEqual(hole[1], 0., places=6)       # left
            self.assertAlmostEqual(hole[2], 1., places=6)       # up from home
            w = st.world_marks()
            self.assertAlmostEqual(w['holes'][0][1], 23., places=6)
            self.assertAlmostEqual(w['zones'][0][2], 1.2)
            self.assertTrue(st.edit_marks('zones_only', zones_only=True)[0])
            again = CourseState(Path(tmp) / 'course.json')                         # restart
            self.assertEqual(again.marks['holes'], st.marks['holes'])
            self.assertTrue(again.marks['zones_only'])
            self.assertTrue(again.edit_marks('remove_hole', index=0)[0])
            self.assertEqual(again.marks['holes'], [])

    def test_preview_route_passes_through_a_marked_hole(self):
        with tempfile.TemporaryDirectory() as tmp:
            st = state(tmp)
            self.assertTrue(st.capture_anchor([0., 0., 0.], 0.)[0])
            # Hole 3 m forward, 0.3 m left, 1.15 m up (route flies at 1.0 m).
            self.assertTrue(st.edit_marks('add_hole', world_xyz=[3., .3, 1.15])[0])
            st.update_preview(dict(status='live', age_s=.2, cell_m=.08, points=[]), now=10)
            plain = st.execution_path(10)                       # no zone around it: hole control ignores it
            self.assertGreater(min(math.dist(p, [3., .3, 1.15]) for p in plain['points']), .2)
            self.assertEqual(plain['marks']['control_holes'], [])
            self.assertTrue(st.edit_marks('add_zone', world_xyz=[3., 0., 0.], radius_m=1.5)[0])
            st.update_preview(dict(status='live', age_s=.2, cell_m=.08, points=[]), now=10)
            path = st.execution_path(10)
            nearest = min(path['points'], key=lambda p: math.dist(p, [3., .3, 1.15]))
            self.assertLess(math.dist(nearest, [3., .3, 1.15]), .03)
            self.assertEqual(path['points'][0], [0., 0., 1.])        # start unchanged
            self.assertEqual(path['marks']['holes'][0], [3., .3, 1.15])


class PolygonZoneTests(unittest.TestCase):
    def test_polygon_zone_is_course_relative_persists_and_filters_depth(self):
        from depth_height import limit_to_zones
        with tempfile.TemporaryDirectory() as tmp:
            st = state(tmp)
            self.assertTrue(st.capture_anchor([10., 20., .5], math.pi/2)[0])
            self.assertFalse(st.edit_marks('add_polygon', world_points=[[0, 0], [1, 0]])[0])   # < 3 corners
            # L-shaped zone in world ENU
            L = [[9., 21.], [12., 21.], [12., 22.], [10., 22.], [10., 24.], [9., 24.]]
            self.assertTrue(st.edit_marks('add_polygon', world_points=L)[0])
            back = st.world_marks()['polygons'][0]
            for a, b in zip(back, L):
                self.assertAlmostEqual(a[0], b[0], places=6)
                self.assertAlmostEqual(a[1], b[1], places=6)
            self.assertEqual(CourseState(Path(tmp) / 'course.json').marks['polygons'], st.marks['polygons'])
            depth = dict(status='live', points=[[9.5, 23.], [11., 23.], [11.5, 21.5], [13., 21.5]],
                         z_ranges=[[0, 1]]*4)
            kept = limit_to_zones(depth, [], [back])['points']
            self.assertEqual(kept, [[9.5, 23.], [11.5, 21.5]])          # inside the L only
            self.assertTrue(st.edit_marks('remove_polygon', index=0)[0])
            self.assertEqual(st.marks['polygons'], [])


class PreviewConfirmationTests(unittest.TestCase):
    def test_one_frame_speckle_does_not_move_the_preview_but_a_real_obstacle_does(self):
        with tempfile.TemporaryDirectory() as tmp:
            st = state(tmp)
            self.assertTrue(st.capture_anchor([0., 0., 0.], 0.)[0])

            def frame(v, pts):
                st.update_preview(dict(status='live', age_s=.2, cell_m=.08, points=pts, geometry_version=v), now=10)
                return st.snapshot(10)['adjustment']['status']
            self.assertEqual(frame('a', []), 'waiting')                     # needs two frames
            self.assertEqual(frame('b', [[3.0, .05]]), 'unchanged')         # speckle on the route, one frame
            self.assertEqual(frame('c', []), 'unchanged')
            box = [[4.2+.08*i, -.2+.08*j] for i in range(4) for j in range(5)]
            self.assertEqual(frame('d', box), 'unchanged')                  # first sighting: not yet
            self.assertEqual(frame('e', box), 'adjusted')                   # seen twice: planned around
            self.assertEqual(frame('e', box), 'adjusted')                   # same frame again: no change


class DryRunTests(unittest.TestCase):
    def test_carrying_the_drone_along_completes_and_commands_nothing(self):
        from dry_run import DryRun
        with tempfile.TemporaryDirectory() as tmp:
            st = state(tmp)
            self.assertTrue(st.capture_anchor([0., 0., 0.], 0.)[0])
            st.update_preview(dict(status='live', age_s=.2, cell_m=.08, points=[]), now=10)
            path = st.execution_path(10)
            core = DashCore(DashParams(land_after_course=False))
            core.home = (0., 0., 0., 0.)
            pos = Pos(10, 0., 0., 1.02, 0., True, 0., 0., 0.)
            with self.assertRaises(ValueError):                      # too far from the start
                DryRun(10, core, path, 8., Pos(10, 1., 0., 1., 0., True, 0., 0., 0.))
            dry = DryRun(10, core, path, 8., pos)
            depth = dict(status='live', age_s=.2, cell_m=.08, points=[])
            for k in range(1, 400):
                now = 10+.1*k
                pos = Pos(now, *dry.core.cmd, 0., True, 0., 0., 0.)   # the carrier follows the dot
                dry.tick(now, pos, depth)
                if dry.state != 'running':
                    break
            self.assertEqual(dry.state, 'finished', dry.reason)
            self.assertIsNone(core.course_run)                      # the real core never ran anything
            self.assertEqual(core.mode, 'idle')


class RejoinTests(unittest.TestCase):
    def test_nudged_run_glides_back_when_the_plan_is_clear_and_keeps_its_holes(self):
        from course_replan import plan_remaining
        from dash_core import Fc
        with tempfile.TemporaryDirectory() as tmp:
            st = state(tmp)
            self.assertTrue(st.capture_anchor([0., 0., 0.], 0.)[0])
            self.assertTrue(st.edit_marks('add_hole', world_xyz=[5., 0., 1.])[0])
            st.update_preview(dict(status='live', age_s=.2, cell_m=.08, points=[]), now=10)
            c = DashCore(DashParams(land_after_course=False))
            c.owner, c.owner_hb, c.home = 'p', 10, (0., 0., 0., 0.)
            c.mode, c.was_offboard, c.cmd, c.target = 'engaged', True, [0., 0., 1.], [0., 0., 1.]
            c.heading.reset(0.); c.last_step = 10
            ok, why = c.start_course(10, st.execution_path(10), 8., Pos(10, 0., 0., 1., 0., True, 0., 0., 0.),
                                     Fc(10, True, True, True, False))
            self.assertTrue(ok, why)
            plan = c.course_run.plan
            self.assertIsNone(c.rejoin_job(11))                    # not nudged: nothing to rejoin
            # A nudge (as if around a one-frame speckle): a 0.3 m sideways bump of the remainder.
            bumped = dict(st.execution_path(10), points=[[p[0], p[1]+.3*math.sin(math.pi*min(1, p[0]/3)), p[2]]
                                                         for p in plan['points']], times_s=None)
            import numpy as np
            c.nudge_request = dict(run=c.course_run, at=11, path=bumped, planned_at=11, trigger=[[2.0, .2]],
                                   trigger_version='v0')
            self.assertTrue(c.apply_nudge(11, bumped)[0])
            self.assertTrue(c.course_run.nudged)
            self.assertEqual(c.course_run.marks['holes'], [[5., 0., 1.]])   # holes survive the nudge
            g = c.course_run.obstacle_guard

            def frame(t, version, cells):
                g._last, g._last_t, g._last_version = (np.asarray(cells, float).reshape(-1, 2), .08), t, version
            c._last_pos = Pos(12, 0., 0., 1., 0., True, 0., 0., 0.)        # nose along +x, toward the trigger
            frame(12., 'v1', [[2.02, .21]])                                  # the trigger is still seen
            self.assertIsNone(c.rejoin_job(12.))
            frame(11.4, 'v0', [])
            self.assertIsNone(c.rejoin_job(11.5))                  # hysteresis, and the triggering frame itself
            frame(12.9, 'v2', [])
            self.assertIsNone(c.rejoin_job(13.))                   # one clear frame is not enough
            frame(12.0, 'v3', [])
            self.assertIsNone(c.rejoin_job(13.))                   # stale (1 s old): no evidence
            c._last_pos = Pos(13, 0., 0., 1., math.pi/2, True, 0., 0., 0.)  # looking sideways: no evidence
            frame(13.4, 'v4', [])
            self.assertIsNone(c.rejoin_job(13.5))
            c._last_pos = Pos(13, 0., 0., 1., 0., True, 0., 0., 0.)
            frame(13.9, 'v5', [])
            job = c.rejoin_job(14.)
            self.assertIsNotNone(job)
            path = plan_remaining(job['points'], job['times'], job['elapsed'], job['start'], job['anchor'],
                                  job['reference'], job['radius'], dict(status='live', age_s=.2, cell_m=.08, points=[]),
                                  job['walls'], known_cell_m=job['cell'], clearance_m=job['clearance'],
                                  floor=job['floor'], ceiling=job['ceiling'])
            self.assertTrue(c.commit_rejoin(14., job, path))
            self.assertTrue(c.apply_nudge(14., c.nudge_request['path'])[0])
            self.assertFalse(c.course_run.nudged)
            self.assertIn('clear again', c.events[-1][1])
            # The rest of the flown route is the original plan again.
            tail = c.course_run.points[-1]
            self.assertLess(math.dist(tail, plan['points'][-1]), 1e-6)
            # A stop + replan (resume) is never abandoned: no trigger, no rejoin.
            c.course_run.nudged, c.course_run.detour_trigger = True, None
            self.assertIsNone(c.rejoin_job(20.))


class VelocityTrackerTests(unittest.TestCase):
    def test_set_points_rebuilds_the_followed_trajectory(self):
        try:
            import alternate_velocity  # noqa: F401  (needs alt_tracker, on the drone)
        except ImportError:
            self.skipTest('alt_tracker not installed here')
        with tempfile.TemporaryDirectory() as tmp:
            st = state(tmp)
            self.assertTrue(st.capture_anchor([0., 0., 0.], 0.)[0])
            st.update_preview(dict(status='live', age_s=.2, cell_m=.08, points=[]), now=10)
            core = DashCore(DashParams(land_after_course=False))
            core.home, core.course_controller = (0., 0., 0., 0.), 'velocity_tracker'
            run = core._make_course(st.execution_path(10), 8.)
            moved = [(p[0], p[1]+.2, p[2]) for p in run.points]
            run.set_points(moved)
            self.assertEqual(run.points, moved)
            self.assertAlmostEqual(float(run.tracker.traj.p[len(moved)//2][1]), moved[len(moved)//2][1], places=6)


if __name__ == '__main__':
    unittest.main()
