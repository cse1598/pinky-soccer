"""Shared match preparation policy, without sensor estimation or a motion controller."""
import math

from pinky_soccer_mpc import collision_geometry as geometry
from pinky_soccer_sim.multi_robot import kickoff_pose


STATES = ('IDLE', 'CALIBRATING', 'MOVING_TO_START', 'READY', 'PLAYING', 'RECOVERING', 'STOPPED', 'ERROR')


def start_pose(field, team, goal_offset=-1., explicit=()):
    """Reuse kickoff pose, or a configured offset inward from the own goal line."""
    if not math.isfinite(goal_offset):
        raise ValueError('goal offset must be finite')
    if explicit:
        if len(explicit) != 3:
            raise ValueError('start_pose requires x,y,yaw')
        pose = tuple(map(float, explicit))
    else:
        pose = kickoff_pose(team, True)
        if goal_offset >= 0:
            sign = field.color_sign(team)
            pose = (sign*(field.half_length-goal_offset), 0., 0. if sign < 0 else math.pi)
    if not all(math.isfinite(v) for v in pose):
        raise ValueError('start pose must be finite')
    walls = geometry.FieldObstacles(field)
    if walls.clearance(geometry.footprint_polygons(pose)) < .02:
        raise ValueError('start pose violates existing footprint/wall clearance (0.02m)')
    return pose


class Preparation:
    """State transitions shared by real, MuJoCo and Gazebo."""

    def __init__(self, team='yellow'):
        self.team = team
        self.state, self.reason = 'IDLE', ''
        self.prepared = self.arrived = False
        self.resume_state = None

    def prepare(self):
        if self.state in ('CALIBRATING', 'MOVING_TO_START', 'PLAYING'):
            return False, 'Preparation already running or game playing'
        self.state, self.reason = 'CALIBRATING', ''
        self.resume_state = None
        self.prepared = self.arrived = False
        return True, 'Preparation started'

    def calibrated(self):
        if self.state == 'CALIBRATING':
            self.state = 'MOVING_TO_START'

    def arrival(self):
        if self.state == 'MOVING_TO_START':
            self.arrived = True

    def stopped_at_start(self):
        if self.state == 'MOVING_TO_START' and self.arrived:
            self.state, self.prepared = 'READY', True

    def start(self, valid):
        if not self.prepared or self.state not in ('READY', 'STOPPED') or not valid:
            return False, 'Start requires completed preparation and valid localization'
        self.state, self.reason = 'PLAYING', ''
        return True, 'Game started'

    def stop(self, reason='User stop'):
        self.resume_state = None
        self.state, self.reason = 'STOPPED', reason
        if not self.prepared:
            self.arrived = False

    def fail(self, reason):
        self.resume_state = None
        self.state, self.reason, self.prepared = 'ERROR', reason, False

    def localization_lost(self):
        if self.state in ('MOVING_TO_START', 'READY', 'PLAYING'):
            self.resume_state = self.state
            self.state, self.reason = 'RECOVERING', 'Waiting for valid localization'

    def localization_restored(self):
        if self.state == 'RECOVERING' and self.resume_state is not None:
            self.state, self.reason = self.resume_state, ''
            self.resume_state = None

    @property
    def command_allowed(self):
        return self.state in ('CALIBRATING', 'PLAYING') or (
            self.state == 'MOVING_TO_START' and not self.arrived)

    @property
    def led(self):
        if self.state == 'IDLE':
            return (0, 0, 0), False
        if self.state == 'ERROR':
            return (96, 0, 0), True
        if self.state in ('CALIBRATING', 'MOVING_TO_START', 'RECOVERING'):
            return (0, 96, 0), True
        return ((96, 96, 0) if self.team == 'yellow' else (0, 0, 96)), False
