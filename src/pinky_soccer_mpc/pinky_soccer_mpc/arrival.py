"""Single source of GoToPose completion; never sends a tracking command."""
import math


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Arrival:
    def __init__(self, pos_tol=.02, ang_tol=math.radians(3), dwell=.3):
        self.pos_tol,self.ang_tol,self.dwell = pos_tol,ang_tol,dwell
        self.since = None

    def reset(self):
        self.since = None

    def update(self, now, pose, goal, v, w, wheels, feedback_valid):
        dist = math.hypot(goal[0]-pose[0],goal[1]-pose[1])
        angle = abs(wrap(goal[2]-pose[2]))
        inside = dist <= self.pos_tol and angle <= self.ang_tol
        stopped = abs(v)<=.01 and abs(w)<=.05 and max(abs(q) for q in wheels)<=.2
        if not inside or not stopped or not feedback_valid:
            self.since = None
        elif self.since is None:
            self.since = now
        done = self.since is not None and now-self.since >= self.dwell
        return inside, bool(done), dist, angle
