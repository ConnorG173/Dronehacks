"""Retargetable yaw with bounded angular velocity and acceleration.

This controller keeps both yaw and angular velocity when its target changes.
It replans triangular/trapezoidal motion to the latest target and integrates
only the current timestep, including any phase boundaries inside that step.
Yaw and angular velocity are continuous (C1); acceleration is bounded but may
change at phase boundaries. This is not a jerk-limited/C2 trajectory.

No ROS, telemetry or flight commands are used here.
"""
import math


class YawTracker:
    def __init__(self, yaw, rate=0., max_rate=math.radians(90), max_accel=math.radians(180)):
        if not all(math.isfinite(value) for value in (yaw, rate, max_rate, max_accel)):
            raise ValueError('yaw state and limits must be finite')
        if max_rate <= 0 or max_accel <= 0 or abs(rate) > max_rate:
            raise ValueError('invalid yaw rate or acceleration limit')
        self.yaw, self.rate = float(yaw), float(rate)
        self.max_rate, self.max_accel = float(max_rate), float(max_accel)

    def step(self, target, dt):
        if not math.isfinite(target) or not math.isfinite(dt):
            raise ValueError('target and timestep must be finite')
        remaining = max(0., min(dt, .2))
        if remaining == 0:
            return self.yaw
        difference = target - self.yaw
        if not math.isfinite(difference):
            raise ValueError('yaw difference cannot be represented')
        error = math.atan2(math.sin(difference), math.cos(difference))
        endpoint = self.yaw + error
        a = self.max_accel
        # Select acceleration direction from the distance left after braking
        # the current rate to zero. A close target or reversal may require
        # temporary overshoot; never hide that by snapping yaw or velocity.
        stopping_distance = self.rate * abs(self.rate) / (2 * a)
        direction = 1. if error - stopping_distance >= 0 else -1.
        velocity = direction * self.rate
        distance = direction * error
        peak = min(self.max_rate, math.sqrt(max(0., a * distance + velocity * velocity / 2)))
        accelerate_s = max(0., (peak - velocity) / a)
        cruise_distance = distance - (2 * peak * peak - velocity * velocity) / (2 * a)
        cruise_s = max(0., cruise_distance / peak) if peak > 0 else 0.
        brake_s = peak / a
        total_s = accelerate_s + cruise_s + brake_s
        if remaining >= total_s:
            self.yaw, self.rate = endpoint, 0.
            return self.yaw
        for duration, acceleration in ((accelerate_s, direction * a),
                                       (cruise_s, 0.), (brake_s, -direction * a)):
            used = min(remaining, duration)
            self.yaw += self.rate * used + acceleration * used * used / 2
            self.rate += acceleration * used
            remaining -= used
            if remaining <= 0:
                break
        return self.yaw
