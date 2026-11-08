"""Synthetic course storage/preview tests; no ROS or flight commands."""
import copy
import json
import math
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch

import course_state
from course_state import CourseState, validate_document


def document():
    return dict(version=1, frame='course_flu', units='m', name='Example course',
                waypoints=[[0, 0, 1], [1, 0, 1], [2, 0, 1]], times_s=[0, 1.5, 3])


class CourseStateTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / 'course.json'
        self.state = CourseState(self.path)

    def anchored(self):
        self.assertTrue(self.state.import_route(document())[0])
        self.assertTrue(self.state.capture_anchor([5, -2, .2], math.pi/2)[0])
        self.assertTrue(self.state.configure_preview()[0])
        return self.state

    def test_original_persisted_exactly_and_restart_requires_anchor(self):
        source = document()
        self.assertTrue(self.state.import_route(source)[0])
        source['waypoints'][0][0] = 99
        self.assertEqual(json.loads(self.path.read_text()), document())
        self.assertTrue(self.state.capture_anchor([5, -2, .2], math.pi/2)[0])
        snap = self.state.snapshot()
        self.assertAlmostEqual(snap['original_world_enu'][1][0], 5)
        self.assertAlmostEqual(snap['original_world_enu'][1][1], -1)
        self.assertAlmostEqual(snap['original_world_enu'][1][2], 1.2)
        self.assertEqual(snap['original_yaw_enu_rad'], [math.pi/2]*3)
        restored = CourseState(self.path).snapshot()
        self.assertEqual(restored['status'], 'needs_anchor')
        self.assertIsNone(restored['anchor'])
        self.assertFalse(restored['preview_enabled'])
        self.assertEqual(restored['original_yaw_enu_rad'], [])
        self.assertEqual(json.loads(self.path.read_text()), document())

    def test_invalid_input_does_not_replace_previous_document(self):
        self.state.import_route(document())
        for key, value in (
            ('version', True), ('frame', 'enu'), ('units', 'ft'), ('name', 'x'*81),
            ('waypoints', [[0, 0, 0]]), ('waypoints', [[0, 0, True], [1, 2, 3]]),
            ('waypoints', [[0, 0, math.nan], [1, 2, 3]]),
            ('waypoints', [[1001, 0, 0], [1, 2, 3]]),
            ('times_s', [0, 0, 2]), ('times_s', [0, 1]), ('times_s', [0, True, 2]),
        ):
            bad = document()
            bad[key] = value
            self.assertFalse(self.state.import_route(bad)[0], (key, value))
            self.assertEqual(json.loads(self.path.read_text()), document())
        with self.assertRaises(ValueError):
            validate_document([[0, 0, 0]]*4001)

    def test_raw_list_and_large_route_preserve_all_points_and_times(self):
        self.assertTrue(self.state.import_route([[0, 0, 1], [1, 0, 1]])[0])
        self.assertNotIn('times_s', json.loads(self.path.read_text()))
        route = document()
        route['waypoints'] = [[i/1000, 0, 1] for i in range(1394)]
        route['times_s'] = [i/100 for i in range(1394)]
        self.assertTrue(self.state.import_route(route)[0])
        self.assertEqual(json.loads(self.path.read_text()), route)
        snapshot = self.state.snapshot()
        self.assertEqual(snapshot['original_count'], 1394)
        self.assertEqual(len(snapshot['original_course']), 600)
        self.assertEqual(snapshot['original_course'][0], route['waypoints'][0])
        self.assertEqual(snapshot['original_course'][-1], route['waypoints'][-1])

    def test_reanchoring_clears_preview_without_changing_original(self):
        state = self.anchored()
        self.assertTrue(state.capture_anchor([0, 0, 0], 0)[0])
        s = state.snapshot()
        self.assertTrue(s['preview_enabled'])
        self.assertEqual(s['adjusted_course'], [])
        self.assertEqual(s['original_world_enu'], document()['waypoints'])
        self.assertEqual(json.loads(self.path.read_text()), document())
        self.assertFalse(state.capture_anchor([0, 0, math.nan], 0)[0])
        self.assertEqual(state.snapshot()['anchor'], s['anchor'])

    def test_preview_freshness_and_original_immutability(self):
        state = self.anchored()
        original = copy.deepcopy(state.snapshot()['original_course'])
        # In course coordinates this wall is far from the route.
        state.update_preview(dict(status='live', age_s=.1, points=[[10., 10.]], cell_m=.08), now=10)
        fresh = state.snapshot(10.1)
        self.assertEqual(fresh['adjustment']['status'], 'unchanged')
        self.assertGreater(fresh['adjusted_count'], 0)
        self.assertTrue(fresh['adjustment']['execution_supported'])
        fresh['original_course'][0][0] = 999
        stale = state.snapshot(11.41)
        self.assertEqual(stale['adjustment']['status'], 'stale')
        self.assertEqual(stale['adjusted_world_enu'], [])
        self.assertEqual(stale['adjusted_yaw_enu_rad'], [])
        self.assertEqual(stale['original_course'], original)
        self.assertEqual(json.loads(self.path.read_text()), document())

    def test_missing_stale_empty_and_bad_depth_never_imply_free_space(self):
        state = self.anchored()
        for depth in (None, {}, dict(status='stale', age_s=2),
                      dict(status='live', age_s=1.51, points=[[1, 2]]),
                      dict(status='live', age_s=.1, points=[[math.nan, 2]])):
            state.update_preview(depth, now=10)
            s = state.snapshot(10)
            self.assertEqual(s['adjusted_world_enu'], [])
            self.assertIn(s['adjustment']['status'], ('waiting', 'stale', 'blocked'))
        # A fresh scaled frame with no wall cells keeps the route unchanged.
        state.update_preview(dict(status='live', age_s=.1, points=[]), now=10)
        self.assertEqual(state.snapshot(10)['adjustment']['status'], 'unchanged')

    def test_configuration_bounds_and_import_clear_anchor(self):
        self.assertFalse(self.state.configure_preview()[0])
        state = self.anchored()
        for clearance, offset in ((.14, .2), (1.01, .2), (.3, -1), (.3, 2.1), (True, .2)):
            self.assertFalse(state.configure_preview(clearance, offset)[0])
        self.assertTrue(state.configure_preview(.15, 0)[0])
        self.assertTrue(state.import_route([[0, 0, 1], [1, 1, 1]])[0])
        s = state.snapshot()
        self.assertIsNone(s['anchor'])
        self.assertFalse(s['preview_enabled'])
        self.assertEqual(s['original_world_enu'], [])
        self.assertEqual(s['original_yaw_enu_rad'], [])

    def test_invalidate_anchor_preserves_original_and_rejects_inflight_computation(self):
        state = self.anchored()
        entered, release = threading.Event(), threading.Event()

        def delayed(points, walls, **kwargs):
            entered.set()
            self.assertTrue(release.wait(2))
            return dict(status='unchanged', points=points, reason='synthetic delayed result')

        with patch('course_geometry.deform_preview', side_effect=delayed):
            thread = threading.Thread(target=state.update_preview,
                args=(dict(status='live', age_s=.1, points=[[10., 10.]], cell_m=.08), 10))
            thread.start()
            self.assertTrue(entered.wait(2))
            state.invalidate_anchor('PX4 local reference became uncertain')
            release.set()
            thread.join(2)
            self.assertFalse(thread.is_alive())
        s = state.snapshot(10.1)
        self.assertIsNone(s['anchor'])
        self.assertFalse(s['preview_enabled'])
        self.assertEqual(s['adjusted_world_enu'], [])
        self.assertEqual(s['original_world_enu'], [])
        self.assertEqual(s['original_course'], document()['waypoints'])
        self.assertEqual(json.loads(self.path.read_text()), document())

    def test_snapshot_does_not_wait_for_disk_save(self):
        state = self.anchored()
        next_route = document()
        next_route['name'] = 'Next route'
        save_entered, release_save, snapshot_done = (threading.Event() for _ in range(3))
        saved_result, snapshots = [], []
        real_save = course_state._atomic_save

        def delayed_save(path, doc):
            save_entered.set()
            if not release_save.wait(2):
                raise RuntimeError('test did not release the blocked save')
            real_save(path, doc)

        def read_snapshot():
            snapshots.append(state.snapshot())
            snapshot_done.set()

        with patch('course_state._atomic_save', side_effect=delayed_save):
            writer = threading.Thread(target=lambda: saved_result.append(state.import_route(next_route)))
            writer.start()
            self.assertTrue(save_entered.wait(2))
            reader = threading.Thread(target=read_snapshot)
            reader.start()
            try:
                self.assertTrue(snapshot_done.wait(.5), 'snapshot blocked behind disk I/O')
                self.assertEqual(snapshots[0]['name'], document()['name'])
                self.assertIsNotNone(snapshots[0]['anchor'])
            finally:
                release_save.set()
                writer.join(2)
                reader.join(2)
        self.assertFalse(writer.is_alive())
        self.assertFalse(reader.is_alive())
        self.assertTrue(saved_result[0][0])
        self.assertEqual(state.snapshot()['name'], 'Next route')
        self.assertIsNone(state.snapshot()['anchor'])
        self.assertEqual(json.loads(self.path.read_text()), next_route)

    def test_geometry_versions_are_stable_until_geometry_or_state_changes(self):
        state = self.anchored()
        depth = dict(status='live', age_s=.1, points=[[10., 10.]], cell_m=.08)
        state.update_preview(depth, now=10)
        first = state.snapshot(10)
        aged = state.snapshot(10.2)
        for key in ('original_version', 'adjusted_version'):
            self.assertEqual(first[key], aged[key])
        self.assertNotEqual(first['adjustment']['age_s'], aged['adjustment']['age_s'])
        state.update_preview(depth, now=10.3)
        second = state.snapshot(10.3)
        self.assertEqual(first['original_version'], second['original_version'])
        self.assertNotEqual(first['adjusted_version'], second['adjusted_version'])
        stale = state.snapshot(11.71)
        self.assertEqual(stale['adjusted_world_enu'], [])
        self.assertNotEqual(second['adjusted_version'], stale['adjusted_version'])
        self.assertEqual(stale['adjusted_version'], state.snapshot(12)['adjusted_version'])
        state.invalidate_anchor('reference changed')
        invalid = state.snapshot(12)
        self.assertNotEqual(stale['original_version'], invalid['original_version'])
        self.assertNotEqual(stale['adjusted_version'], invalid['adjusted_version'])
        restored = CourseState(self.path)
        self.assertNotEqual(invalid['original_version'], restored.snapshot()['original_version'])

    def test_pending_preview_does_not_version_old_arrays_as_new_result(self):
        state = self.anchored()
        depth = dict(status='live', age_s=.1, points=[[10., 10.]], cell_m=.08)
        state.update_preview(depth, now=10)
        old = state.snapshot(10)
        entered, release = threading.Event(), threading.Event()

        def delayed(points, walls, **kwargs):
            entered.set()
            if not release.wait(2):
                raise RuntimeError('test did not release preview calculation')
            return dict(status='unchanged', points=points, reason='synthetic result')

        with patch('course_geometry.deform_preview', side_effect=delayed):
            thread = threading.Thread(target=state.update_preview, args=(depth, 10.1))
            thread.start()
            self.assertTrue(entered.wait(2))
            try:
                pending = state.snapshot(10.1)
                self.assertEqual(old['adjusted_version'], pending['adjusted_version'])
                self.assertEqual(old['adjusted_world_enu'], pending['adjusted_world_enu'])
            finally:
                release.set()
                thread.join(2)
        self.assertFalse(thread.is_alive())
        self.assertNotEqual(old['adjusted_version'], state.snapshot(10.2)['adjusted_version'])

    def test_adjusted_heading_uses_adjusted_tangent_then_anchor_rotation(self):
        state = self.anchored()
        adjusted = [[0, 0, 1], [0, 1, 1], [0, 2, 1]]
        with patch('course_geometry.deform_preview', return_value=dict(
                status='adjusted', points=adjusted, reason='synthetic northbound course')):
            state.update_preview(dict(status='live', age_s=.1, points=[[10., 10.]], cell_m=.08), now=10)
        snap = state.snapshot(10.1)
        self.assertEqual(snap['original_yaw_enu_rad'], [math.pi/2]*3)
        self.assertEqual(snap['adjusted_yaw_enu_rad'], [math.pi]*3)
        self.assertEqual(snap['adjusted_course'], adjusted)
        self.assertEqual(state.snapshot(11.41)['adjusted_yaw_enu_rad'], [])

    def test_preview_display_expires_after_one_point_five_seconds_including_compute(self):
        state = self.anchored()
        depth = dict(status='live', age_s=.75, points=[[10., 10.]], cell_m=.08)
        # Out-of-phase 2 Hz input/preview timers plus 150 ms processing must not
        # immediately hide a successfully computed display-only preview.
        with patch('course_state.time.monotonic', side_effect=[100., 100.15]):
            state.update_preview(depth, now=10)
        computed = state.snapshot(10.15)
        self.assertEqual(computed['adjustment']['status'], 'unchanged')
        self.assertEqual(computed['adjustment']['age_s'], .9)
        self.assertTrue(computed['adjusted_world_enu'])
        self.assertTrue(computed['adjustment']['execution_supported'])
        at_limit = state.snapshot(10.75)
        self.assertEqual(at_limit['adjustment']['age_s'], 1.5)
        self.assertEqual(at_limit['adjustment']['status'], 'unchanged')
        expired = state.snapshot(10.751)
        self.assertEqual(expired['adjustment']['status'], 'stale')
        self.assertEqual(expired['adjusted_world_enu'], [])
        self.assertNotEqual(at_limit['adjusted_version'], expired['adjusted_version'])

        # Processing time still counts: a 1.4 s observation expiring during
        # 200 ms of work cannot be committed as a fresh preview.
        with patch('course_state.time.monotonic', side_effect=[100., 100.2]):
            state.update_preview(dict(depth, age_s=1.4), now=11)
        self.assertEqual(state.snapshot(11.2)['adjustment']['status'], 'stale')
        self.assertEqual(state.snapshot(11.2)['adjusted_world_enu'], [])

    def test_display_downsampling_uses_exact_same_indices_for_xyz_and_yaw(self):
        route = document()
        route['waypoints'] = [[i/1000, 0, 1] for i in range(1394)]
        route['times_s'] = [i/100 for i in range(1394)]
        adjusted = [[i/1000, .5, 1] for i in range(1394)]
        # Encode the source index in the synthetic helper result. This catches
        # headings derived after downsampling or sampled with different indices.
        with patch('course_state.tangent_yaws', side_effect=lambda points: [i/100 for i in range(len(points))]):
            self.assertTrue(self.state.import_route(route)[0])
            self.assertTrue(self.state.capture_anchor([0, 0, 0], .4)[0])
            self.assertTrue(self.state.configure_preview()[0])
            with patch('course_geometry.deform_preview', return_value=dict(
                    status='adjusted', points=adjusted, reason='synthetic long route')):
                self.state.update_preview(dict(status='live', age_s=.1, points=[[10., 10.]], cell_m=.08), now=10)
        snap = self.state.snapshot(10.1)
        indices = [i*1393//599 for i in range(600)]
        for prefix in ('original', 'adjusted'):
            xyz, yaw = snap[prefix+'_world_enu'], snap[prefix+'_yaw_enu_rad']
            self.assertEqual(len(xyz), 600)
            self.assertEqual(len(yaw), 600)
            y = 0 if prefix == 'original' else .5
            for point, angle, i in zip(xyz, yaw, indices):
                self.assertAlmostEqual(point[0], (i/1000)*math.cos(.4)-y*math.sin(.4))
                self.assertAlmostEqual(point[1], (i/1000)*math.sin(.4)+y*math.cos(.4))
                self.assertAlmostEqual(angle, i/100+.4)
        self.assertEqual(json.loads(self.path.read_text()), route)


if __name__ == '__main__':
    unittest.main()
