"""Synthetic geometry checks only; no aircraft or flight simulation."""
import copy
import math
import unittest

from course_replan import plan_remaining


def minimum_clearance(points, walls, cell):
    best = math.inf
    for a, b in zip(points, points[1:]):
        vx, vy = b[0]-a[0], b[1]-a[1]
        length2 = vx*vx+vy*vy
        for wx, wy in walls:
            t = min(1., max(0., ((wx-a[0])*vx+(wy-a[1])*vy)/length2)) if length2 else 0.
            best = min(best, math.hypot(wx-a[0]-t*vx, wy-a[1]-t*vy)-cell/math.sqrt(2))
    return best


class RemainingCourseTests(unittest.TestCase):
    def setUp(self):
        self.kw = dict(points=[[0., 0., 1.], [2., 0., 1.], [4., 0., 1.], [6., 0., 1.]],
                       times_s=[0., 2., 4., 6.], elapsed_s=2.4,
                       current_position=[2.4, 0., 1.],
                       anchor=dict(valid=True, origin_enu=[0., 0., 0.], yaw_rad=.2),
                       reference=('session', 4, 1234), radius=8.,
                       depth=dict(status='live', age_s=.1, cell_m=.08, points=[[4., 0.]]),
                       floor=.3, ceiling=2., max_work_s=.5)

    def test_detour_starts_at_midcourse_hover_and_keeps_original_untouched(self):
        saved = copy.deepcopy(self.kw)
        result = plan_remaining(**self.kw)
        self.assertEqual(result['source'], 'adjusted')
        self.assertEqual(result['points'][0], self.kw['current_position'])
        self.assertEqual(result['points'][-1], [6., 0., 1.])
        self.assertEqual(result['next_index'], 2)
        self.assertGreaterEqual(min(p[0] for p in result['points']), 2.4-1e-9)
        self.assertGreater(max(abs(p[1]) for p in result['points']), .4)
        self.assertGreaterEqual(minimum_clearance(result['points'], [[4., 0.]], .08), .4-1e-9)
        self.assertEqual(result['anchor'], saved['anchor'])
        self.assertEqual(result['reference'], saved['reference'])
        self.assertIsNone(result['times_s'])
        self.assertEqual(len(result['display']['points']), len(result['display']['yaws']))
        self.assertLessEqual(len(result['display']['points']), 600)
        self.assertEqual(self.kw, saved)

    def test_future_loop_is_preserved_in_time_order(self):
        self.kw.update(points=[[0., 0., 1.], [1., 0., 1.], [1., 1., 1.],
                               [0., 1., 1.], [0., 0., 1.], [2., 0., 1.]],
                       times_s=[0., 1., 2., 3., 4., 5.], elapsed_s=2.2,
                       current_position=[.8, 1., 1.],
                       depth=dict(status='live', age_s=.1, cell_m=.08, points=[[10., 10.]]))
        result = plan_remaining(**self.kw)
        self.assertEqual(result['next_index'], 3)
        corners = ([0., 1., 1.], [0., 0., 1.], [2., 0., 1.])
        indices = [result['points'].index(p) for p in corners]
        self.assertEqual(indices, sorted(indices))

    def test_duplicate_current_vertex_is_not_a_backwards_restart(self):
        self.kw.update(elapsed_s=2., current_position=[4., 0., 1.],
                       depth=dict(status='live', age_s=.1, cell_m=.08, points=[[10., 10.]]))
        result = plan_remaining(**self.kw)
        self.assertEqual(result['points'][0], [4., 0., 1.])
        self.assertGreater(result['points'][1][0], 4.)
        self.assertGreaterEqual(min(p[0] for p in result['points']), 4.)

    def test_missing_stale_and_empty_depth_never_return_a_route(self):
        for depth in (None, dict(status='unscaled'),
                      dict(status='live', age_s=1.6, cell_m=.08, points=[[4., 0.]])):
            with self.subTest(depth=depth), self.assertRaises(ValueError):
                plan_remaining(**{**self.kw, 'depth': depth})

    def test_blocked_endpoint_or_zero_detour_allowance_fails_closed(self):
        for extra in (dict(known_walls=[[2.4, 0.]]), dict(max_offset_m=0.)):
            with self.subTest(extra=extra), self.assertRaises(ValueError):
                plan_remaining(**{**self.kw, **extra})

    def test_disappearing_wall_is_retained_and_avoided(self):
        self.kw['depth']['points'] = [[12., 12.]]
        remembered = [[4., 0.], [30., 30.]]
        result = plan_remaining(**self.kw, known_walls=remembered, known_cell_m=.1)
        self.assertEqual(result['source'], 'adjusted')
        self.assertEqual(result['avoidance']['cell_m'], .1)
        self.assertGreaterEqual(minimum_clearance(result['points'], remembered, .1), .4-1e-9)
        for wall in remembered + [[12., 12.]]:
            self.assertIn(wall, result['avoidance']['walls'])

    def test_irrelevant_memory_is_pruned_only_from_geometry_not_flight_guard(self):
        self.kw['depth']['points'] = [[12., 12.]]
        remembered = [[100.+i*.01, 100.] for i in range(2000)]
        result = plan_remaining(**self.kw, known_walls=remembered)
        self.assertEqual(result['source'], 'unchanged')
        self.assertEqual(len(result['avoidance']['walls']), 2001)
        self.assertIn(remembered[-1], result['avoidance']['walls'])

    def test_memory_and_nearby_geometry_limits_fail_without_discarding_cells(self):
        for remembered in ([[100.+i*.01, 100.] for i in range(6400)],
                           [[3.+i*.0001, .9] for i in range(1601)]):
            with self.subTest(count=len(remembered)), self.assertRaises(ValueError):
                plan_remaining(**self.kw, known_walls=remembered)

    def test_invalid_times_bounds_and_work_limit_fail_closed(self):
        for extra in (dict(times_s=[0., 2., 2., 6.]), dict(elapsed_s=6.),
                      dict(elapsed_s=-1.), dict(radius=3.), dict(ceiling=.9),
                      dict(floor=2., ceiling=1.), dict(current_position=[float('nan'), 0., 1.]),
                      dict(max_work_s=1e-12)):
            with self.subTest(extra=extra), self.assertRaises(ValueError):
                plan_remaining(**{**self.kw, **extra})


    def test_empty_live_frame_without_memory_keeps_the_remaining_route(self):
        result = plan_remaining(**{**self.kw, 'depth': dict(status='live', age_s=.1, cell_m=.08, points=[])})
        self.assertEqual(result['source'], 'unchanged')
        self.assertEqual(result['points'][0], self.kw['current_position'])
        self.assertEqual(result['points'][-1], [6., 0., 1.])

    def test_hover_inside_clearance_backs_off_then_detours(self):
        wall = [[2.75, 0.]]
        self.kw['depth']['points'] = wall
        result = plan_remaining(**self.kw)
        pts = result['points']
        self.assertEqual(pts[0], self.kw['current_position'])
        self.assertEqual(pts[-1], [6., 0., 1.])
        here = minimum_clearance([pts[0], pts[0]], wall, .08)
        self.assertGreaterEqual(minimum_clearance(pts[:2], wall, .08), here-.02-1e-9)
        self.assertGreaterEqual(minimum_clearance(pts[1:], wall, .08), .4-1e-9)

    def test_wide_obstacle_needs_the_larger_detour_allowance(self):
        wall = [[4., y/10] for y in range(-5, 6)]
        self.kw['depth']['points'] = wall
        with self.assertRaises(ValueError):
            plan_remaining(**{**self.kw, 'max_offset_m': .6})
        result = plan_remaining(**self.kw)
        self.assertEqual(result['source'], 'adjusted')
        self.assertGreaterEqual(minimum_clearance(result['points'], wall, .08), .4-1e-9)


if __name__ == '__main__':
    unittest.main()
