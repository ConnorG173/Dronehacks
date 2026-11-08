"""Pure synthetic controller tests; no ROS, HTTP or vehicle access."""
import math
import random
import unittest

from yaw_tracker import YawTracker


def angle_error(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


class YawTrackerTests(unittest.TestCase):
    def checked_step(self, tracker, goal, dt):
        before_yaw, before_rate = tracker.yaw, tracker.rate
        yaw = tracker.step(goal, dt)
        used = max(0., min(dt, .2))
        self.assertTrue(math.isfinite(yaw))
        self.assertTrue(math.isfinite(tracker.rate))
        self.assertEqual(yaw, tracker.yaw)
        self.assertLessEqual(abs(yaw - before_yaw), tracker.max_rate * used + 1e-10)
        self.assertLessEqual(abs(tracker.rate), tracker.max_rate + 1e-10)
        self.assertLessEqual(abs(tracker.rate - before_rate), tracker.max_accel * used + 1e-10)
        return yaw

    def settle(self, tracker, goal):
        for _ in range(2000):
            self.checked_step(tracker, goal, .01)
            if abs(angle_error(tracker.yaw, goal)) < 1e-12 and abs(tracker.rate) < 1e-12:
                break
        self.assertAlmostEqual(angle_error(tracker.yaw, goal), 0., places=10)
        self.assertAlmostEqual(tracker.rate, 0., places=10)

    def test_fixed_turns_are_monotonic_from_rest_and_hold_exact_endpoint(self):
        for degrees in (.001, .5, 10., 90., 179., -135.):
            with self.subTest(degrees=degrees):
                tracker = YawTracker(.3)
                target = .3 + math.radians(degrees)
                direction = 1. if degrees > 0 else -1.
                previous = tracker.yaw
                for _ in range(1000):
                    yaw = self.checked_step(tracker, target, .01)
                    self.assertGreaterEqual(direction * (yaw - previous), -1e-10)
                    self.assertGreaterEqual(direction * (target - yaw), -1e-10)
                    previous = yaw
                    if tracker.rate == 0. and abs(yaw - target) < 1e-12:
                        break
                self.assertAlmostEqual(tracker.yaw, target, places=11)
                self.assertEqual(tracker.rate, 0.)
                before = tracker.yaw
                for _ in range(20):
                    self.assertAlmostEqual(self.checked_step(tracker, target, .1), before, places=12)
                    self.assertEqual(tracker.rate, 0.)

    def test_shortest_wrap_crossing_preserves_unwrapped_yaw(self):
        for start_deg, target_deg, delta_deg in ((179., -179., 2.), (-179., 179., -2.)):
            with self.subTest(start=start_deg):
                tracker = YawTracker(math.radians(start_deg))
                start = tracker.yaw
                self.settle(tracker, math.radians(target_deg))
                self.assertAlmostEqual(math.degrees(tracker.yaw - start), delta_deg, places=9)

    def test_mid_turn_reversal_brakes_existing_velocity_without_a_yaw_jump(self):
        tracker = YawTracker(0.)
        for _ in range(5):
            self.checked_step(tracker, math.radians(120.), .1)
        before_yaw, before_rate = tracker.yaw, tracker.rate
        self.assertGreater(before_rate, 0.)
        self.checked_step(tracker, math.radians(-90.), .01)
        self.assertGreater(tracker.yaw, before_yaw)
        self.assertGreater(tracker.rate, 0.)
        self.assertLess(tracker.rate, before_rate)
        self.settle(tracker, math.radians(-90.))

    def test_goal_inside_braking_distance_allows_only_physical_overshoot(self):
        initial_rate = math.radians(60.)
        tracker = YawTracker(0., rate=initial_rate)
        braking_distance = initial_rate ** 2 / (2 * tracker.max_accel)
        maximum = tracker.yaw
        for _ in range(600):
            maximum = max(maximum, self.checked_step(tracker, 0., .001))
        self.assertAlmostEqual(maximum, braking_distance, delta=tracker.max_accel * .001 ** 2)
        self.settle(tracker, 0.)

    def test_moving_goal_reversal_and_stop_keep_velocity_and_acceleration_bounds(self):
        tracker = YawTracker(0.)
        previous_average_rate = 0.
        goals = [3. * index for index in range(60)]
        goals += [177. - 3. * index for index in range(60)]
        goals += [0.] * 100
        for degrees in goals:
            old_yaw = tracker.yaw
            self.checked_step(tracker, math.radians(degrees), .1)
            average_rate = (tracker.yaw - old_yaw) / .1
            self.assertLessEqual(abs(average_rate - previous_average_rate) / .1,
                                 tracker.max_accel + 1e-9)
            previous_average_rate = average_rate
        self.assertAlmostEqual(tracker.yaw, 0., places=10)
        self.assertEqual(tracker.rate, 0.)

    def test_variable_timestep_random_targets_obey_bounds_and_then_settle(self):
        generator = random.Random(731)
        tracker = YawTracker(2.9, max_rate=math.radians(70), max_accel=math.radians(140))
        for _ in range(1500):
            goal = generator.uniform(-math.pi, math.pi)
            self.checked_step(tracker, goal, generator.choice((0., .001, .01, .03, .1, .15, .2)))
        self.settle(tracker, -2.9)

    def test_zero_negative_and_large_timestep_have_defined_behavior(self):
        tracker = YawTracker(.2, rate=.4)
        before = tracker.yaw, tracker.rate
        for dt in (0., -.1):
            self.checked_step(tracker, 1., dt)
            self.assertEqual((tracker.yaw, tracker.rate), before)
        other = YawTracker(.2, rate=.4)
        self.checked_step(tracker, 1., 1.)
        self.checked_step(other, 1., .2)
        self.assertEqual((tracker.yaw, tracker.rate), (other.yaw, other.rate))

    def test_nonfinite_inputs_and_invalid_limits_are_rejected_without_mutation(self):
        for invalid in (math.nan, math.inf, -math.inf):
            for argument in ('yaw', 'rate', 'max_rate', 'max_accel'):
                values = dict(yaw=0., rate=0., max_rate=1., max_accel=2.)
                values[argument] = invalid
                with self.subTest(argument=argument, value=invalid):
                    with self.assertRaises(ValueError):
                        YawTracker(**values)
            tracker = YawTracker(.2, rate=.4)
            before = tracker.yaw, tracker.rate
            for goal, dt in ((invalid, .1), (1., invalid)):
                with self.assertRaises(ValueError):
                    tracker.step(goal, dt)
                self.assertEqual((tracker.yaw, tracker.rate), before)
        for values in (dict(max_rate=0.), dict(max_rate=-1.), dict(max_accel=0.),
                       dict(max_accel=-1.), dict(rate=1.01, max_rate=1.)):
            with self.assertRaises(ValueError):
                YawTracker(0., **values)

    def test_illustrative_yaw_lag_and_clock_easing_keep_a_curve_moving(self):
        # Synthetic near-level yaw-only P=2.8 response; not a flight prediction.
        tracker = YawTracker(0.)
        measured_yaw = path_time = 0.
        for _ in range(1200):
            goal = math.radians(40.) * path_time
            error_deg = abs(math.degrees(angle_error(goal, measured_yaw)))
            linear = max(0., min(1., (10. - error_deg) / 5.))
            factor = linear * linear * (3. - 2. * linear)
            self.assertGreater(factor, 0., 'ordinary curve should ease speed without fully stopping')
            self.assertLess(error_deg, 10.)
            self.checked_step(tracker, goal, .1)
            measured_yaw += (1. - math.exp(-2.8 * .1)) * angle_error(tracker.yaw, measured_yaw)
            path_time += .1 * factor
        self.assertGreater(path_time, 40.)


if __name__ == '__main__':
    unittest.main()
