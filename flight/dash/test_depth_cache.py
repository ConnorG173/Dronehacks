"""Synthetic depth input only; no ROS, network, camera or flight commands."""
import json
import math
import unittest

from depth_cache import DepthOverlayCache, FRAME, SOURCE, MAX_JSON_BYTES

WALL_NS = 1_790_000_000_000_000_000


def message(**changes):
    data = dict(source=SOURCE, frame=FRAME, status="live", reason="", stamp_ns=WALL_NS,
                cell_m=.08, points=[[1., 2.]], camera_xy=[0., 0.],
                view_polygon=[[0., 0.], [1., -1.], [1., 1.]])
    data.update(changes)
    return json.dumps(data)


class DepthCacheTests(unittest.TestCase):
    def setUp(self):
        self.cache = DepthOverlayCache()

    def update(self, raw=None, mono=10, wall=WALL_NS):
        return self.cache.update(message() if raw is None else raw, mono, wall)

    def test_delayed_receipt_uses_capture_age_and_clears_stale_geometry(self):
        self.assertTrue(self.update(wall=WALL_NS + 1_200_000_000))
        live = self.cache.snapshot(10.2)
        self.assertEqual((live["status"], live["age_s"]), ("live", 1.4))
        self.assertEqual(live["points"], [[1., 2.]])
        stale = self.cache.snapshot(10.31)
        self.assertEqual((stale["status"], stale["points"], stale["camera_xy"], stale["view_polygon"]),
                         ("stale", [], None, []))
        self.assertTrue(self.update(wall=WALL_NS + 2_000_000_000))
        self.assertEqual(self.cache.snapshot(10)["status"], "stale")

    def test_repeated_stamp_and_publication_time_do_not_refresh_old_frame(self):
        self.assertTrue(self.update())
        self.assertTrue(self.update(message(produced_wall_ns=WALL_NS + 4_000_000_000),
                                    mono=12, wall=WALL_NS + 2_000_000_000))
        self.assertEqual(self.cache.snapshot(12)["status"], "stale")
        # A backwards wall-clock adjustment must not rejuvenate that same frame.
        self.assertTrue(self.update(mono=13, wall=WALL_NS + 100_000_000))
        state = self.cache.snapshot(13)
        self.assertEqual((state["age_s"], state["points"]), (3., []))

    def test_invalid_messages_replace_live_geometry_with_error(self):
        invalid = ["not JSON", "[]", message(frame="camera"), message(source="stereo"),
                   message(status="stale"), message(stamp_ns=None), message(stamp_ns=True),
                   message(stamp_ns=WALL_NS + 50_000_001), message(points=[[math.nan, 0]]),
                   message(points=[[0, math.inf]]), message(points=[[True, 0]]),
                   message(points=[[0, 0, 0]]), message(camera_xy=[math.inf, 0]),
                   message(view_polygon=[[0, None]]), message(points=[[0, 0]] * 1601),
                   message(cell_m=.01), message(cell_m=.51), message(cell_m=math.nan),
                   message(reason=7), message(reason="x" * MAX_JSON_BYTES)]
        for raw in invalid:
            with self.subTest(raw=raw[:90]):
                self.assertTrue(self.update())
                self.assertFalse(self.update(raw))
                state = self.cache.snapshot(10)
                self.assertEqual(state["status"], "error")
                self.assertEqual((state["points"], state["camera_xy"], state["view_polygon"]), ([], None, []))

    def test_utf8_size_limit_is_bytes_not_characters(self):
        raw = message(reason="").replace('"reason": ""', '"reason": "' + "é" * (MAX_JSON_BYTES // 2) + '"')
        self.assertLess(len(raw), MAX_JSON_BYTES)
        self.assertFalse(self.update(raw))

    def test_empty_live_frame_and_small_clock_tolerance_are_valid(self):
        self.assertTrue(self.update(message(points=[], stamp_ns=WALL_NS + 40_000_000)))
        state = self.cache.snapshot(10)
        self.assertEqual((state["status"], state["points"], state["age_s"]), ("live", [], 0))
        self.assertEqual(set(state), {"source", "status", "reason", "age_s", "cell_m", "points", "camera_xy", "view_polygon", "geometry_version"})
        self.assertNotIn("ready", state)
        self.assertNotIn("runtime_ready", state)

    def test_nonlive_input_never_exposes_geometry_and_waiting_needs_no_timestamp(self):
        self.assertEqual(self.cache.snapshot(0)["status"], "waiting")
        for status in ("waiting", "unscaled", "error"):
            self.assertTrue(self.update(message(status=status, reason="calibrating", stamp_ns=None)))
            state = self.cache.snapshot(100)
            self.assertEqual((state["status"], state["reason"], state["age_s"]), (status, "calibrating", None))
            self.assertEqual((state["points"], state["camera_xy"], state["view_polygon"]), ([], None, []))

    def test_snapshots_cannot_mutate_cached_geometry(self):
        self.assertTrue(self.update())
        state = self.cache.snapshot(10)
        state["points"][0][0] = 42
        state["camera_xy"][0] = 42
        state["view_polygon"].clear()
        fresh = self.cache.snapshot(10)
        self.assertEqual(fresh["points"], [[1., 2.]])
        self.assertEqual(fresh["camera_xy"], [0., 0.])
        self.assertEqual(len(fresh["view_polygon"]), 3)

    def test_geometry_versions_track_capture_and_effective_status_not_age(self):
        self.assertTrue(self.update())
        first = self.cache.snapshot(10)
        aged = self.cache.snapshot(10.5)
        self.assertNotEqual(first['age_s'], aged['age_s'])
        self.assertEqual(first['geometry_version'], aged['geometry_version'])
        self.assertTrue(self.update(message(stamp_ns=WALL_NS+1_000_000_000),
                                    mono=11, wall=WALL_NS+1_000_000_000))
        second = self.cache.snapshot(11)
        self.assertNotEqual(first['geometry_version'], second['geometry_version'])
        stale = self.cache.snapshot(12.6)
        self.assertEqual(stale['points'], [])
        self.assertNotEqual(second['geometry_version'], stale['geometry_version'])
        self.assertEqual(stale['geometry_version'], self.cache.snapshot(13)['geometry_version'])
        restored = DepthOverlayCache()
        restored.update(message(stamp_ns=WALL_NS+1_000_000_000), 11, WALL_NS+1_000_000_000)
        self.assertNotEqual(second['geometry_version'], restored.snapshot(11)['geometry_version'])


class HeightRangeTests(unittest.TestCase):
    def test_z_ranges_pass_through_and_bad_ones_are_rejected(self):
        c = DepthOverlayCache()
        self.assertTrue(c.update(message(points=[[1., 2.], [3., 4.]], z_ranges=[[0., .3], [.5, 1.]]),
                                 now_mono=10, now_wall_ns=WALL_NS))
        self.assertEqual(c.snapshot(10)['z_ranges'], [[0., .3], [.5, 1.]])
        for bad in ([[0., .3]], [[1., .5], [0., 1.]], 'x'):
            self.assertFalse(c.update(message(points=[[1., 2.], [3., 4.]], z_ranges=bad), now_mono=11, now_wall_ns=WALL_NS))
        self.assertTrue(c.update(message(), now_mono=12, now_wall_ns=WALL_NS))          # older workers: no z_ranges
        self.assertNotIn('z_ranges', c.snapshot(12))


if __name__ == "__main__":
    unittest.main(verbosity=2)
