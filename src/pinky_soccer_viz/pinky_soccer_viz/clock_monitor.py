"""ROS-free rolling clock measurements using a monotonic wall clock."""
from collections import deque


class ClockMonitor:
    """Measure a shared /clock stream independently of ROS time."""

    def __init__(self, window=2.0, stale_after=.5, min_span=.5):
        self.window = window
        self.stale_after = stale_after
        self.min_span = min_span
        self.samples = deque()

    def add(self, sim_sec, wall_sec):
        if self.samples:
            prev_wall, prev_sim = self.samples[-1]
            if sim_sec < prev_sim or wall_sec <= prev_wall or wall_sec-prev_wall > self.stale_after:
                self.samples.clear()
        self.samples.append((wall_sec, sim_sec))
        while len(self.samples)>1 and wall_sec-self.samples[0][0]>self.window:
            self.samples.popleft()

    def snapshot(self, wall_sec):
        result = dict(rtf=None, clock_hz=None, clock_age_s=None,
                      sim_time=None, wall_time=wall_sec, clock_state='waiting')
        if not self.samples:
            return result
        last_wall, last_sim = self.samples[-1]
        age = max(0., wall_sec-last_wall)
        result.update(clock_age_s=age, sim_time=last_sim)
        if age>self.stale_after:
            result.update(clock_state='stale', rtf=0., clock_hz=0.)
            return result
        first_wall, first_sim = self.samples[0]
        span = last_wall-first_wall
        if span>=self.min_span and len(self.samples)>1:
            result.update(rtf=(last_sim-first_sim)/span,
                          clock_hz=(len(self.samples)-1)/span, clock_state='ok')
        return result
