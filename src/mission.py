import math

import config
from pid import clamp_unit


class RangeSchedule:
    def __init__(self, phases=config.MISSION_PHASES):
        self.deadlines = []
        total = 0.0
        for duration, target_range_m in phases:
            total += duration
            self.deadlines.append((total, target_range_m))
        self.duration = total

    def target_range(self, elapsed):
        return next((target_range_m for deadline, target_range_m
                     in self.deadlines if elapsed < deadline),
                    self.deadlines[-1][1])

    def finished(self, elapsed):
        return elapsed >= self.duration


def visibility(detection, now):
    freshness = clamp_unit(1.0 - detection.age(now) / config.DETECTION_HOLD_S)
    span = max(1, config.TRUSTED_CORNER_COUNT - config.MIN_CLUSTER_CORNERS + 1)
    strength = clamp_unit(
        (detection.corner_count - config.MIN_CLUSTER_CORNERS + 1) / span)
    return freshness * strength


def approach_speed_limit(range_m):
    margin_m = max(0.0, range_m - config.MINIMUM_RANGE_M)
    decel = config.BRAKING_DECEL_M_S2
    lag = config.CONTROL_LAG_S
    stopping = math.sqrt(decel * decel * lag * lag + 2.0 * decel * margin_m)
    return max(0.0, stopping - decel * lag)


def blend(weight, tracking_value, searching_value):
    return weight * tracking_value + (1.0 - weight) * searching_value


def search_yaw_rate(detection, now):
    turn_s = config.SEARCH_STEP_DEG / config.SEARCH_YAW_RATE_DPS
    period = turn_s + config.SEARCH_DWELL_S
    turning = float(now % period < turn_s)
    return math.copysign(config.SEARCH_YAW_RATE_DPS * turning,
                         detection.bearing_deg or 1.0)
