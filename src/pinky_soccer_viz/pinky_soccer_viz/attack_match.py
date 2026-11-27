"""Referee authority for the attack-only game, independent of ROS and vision."""
import math


class RefereeGate:
    def __init__(self, center_radius=.15, timeout=.5):
        self.center_radius, self.timeout = center_radius, timeout
        self.received = -math.inf
        self.valid = False
        self.ball = None
        self.armed = False
        self.sequence = -1

    def update(self, now, valid, ball, armed):
        self.received, self.valid, self.ball, self.armed = now, valid, ball, armed

    def fresh(self, now):
        return self.valid and 0 <= now-self.received <= self.timeout

    def can_start(self, now):
        return (self.fresh(now) and self.ball is not None and
                all(math.isfinite(v) for v in self.ball) and
                math.hypot(*self.ball) <= self.center_radius)

    def event(self, sequence, event, active):
        if sequence <= self.sequence:
            return None
        self.sequence = sequence
        if not active:
            return None
        return 'RETURN' if event == 'GOAL' else 'STOP' if event in ('OUT', 'UNVERIFIED_EXIT') else None
