"""
Inputs and outputs of one skill step.

A skill never reads ROS topics or the simulator directly. The ROS node and the MuJoCo test
bench both fill the same SkillContext, so a skill tuned in simulation runs unchanged on the
robot. Only data available on the physical robot may be put here (no ground truth).
"""

from dataclasses import dataclass, field
import math
from typing import List, Optional, Tuple

from .geometry import Pose

OBSERVED, ESTIMATED, LOST = 'OBSERVED', 'ESTIMATED', 'LOST'
RUNNING, SUCCESS, FAILURE = 'RUNNING', 'SUCCESS', 'FAILURE'


@dataclass
class BallEstimate:
    """Ball belief in the map frame (from the ball filter / world model)."""

    x: float
    y: float
    vx: float = 0.0
    vy: float = 0.0
    state: str = OBSERVED
    age: float = 0.0              # [s] since the last real camera fix
    sigma: float = 0.0            # [m] worst-axis position std
    seen_now: bool = False        # a camera fix arrived in this control cycle

    @property
    def xy(self):
        """Return (x, y)."""
        return (self.x, self.y)

    @property
    def speed(self):
        """Ball speed [m/s]."""
        return math.hypot(self.vx, self.vy)

    @property
    def known(self):
        """Return True unless the filter reports LOST."""
        return self.state != LOST


@dataclass
class FieldInfo:
    """Field data the skills need (map frame, centre origin)."""

    half_length: float
    half_width: float
    wall_x: float                 # inner wall faces
    wall_y: float
    goal_width: float
    own_goal: Tuple[float, float]
    opp_goal: Tuple[float, float]

    @classmethod
    def from_field(cls, f):
        """Build from pinky_soccer_field.Field."""
        walls = f.inner_walls
        return cls(f.half_length, f.half_width, walls.xmax, walls.ymax,
                   f.spec['goal_width'], tuple(f.own_goal), tuple(f.opp_goal))


@dataclass
class SkillContext:
    """Everything a skill may use in one control cycle."""

    t: float                                  # [s] monotonic (sim or steady clock)
    pose: Pose                                # self pose in map
    v: float = 0.0                            # measured linear speed [m/s] (odom)
    w: float = 0.0                            # measured turn rate [rad/s]
    ball: Optional[BallEstimate] = None
    us_range: float = math.nan                # ultrasonic [m]; -inf too close, inf nothing
    opponents: List[Tuple[float, float]] = field(default_factory=list)
    field: Optional[FieldInfo] = None
    last_cmd: Tuple[float, float] = (0.0, 0.0)

    def ball_local(self):
        """Ball (x, y) in base_link, or None when unknown."""
        if self.ball is None:
            return None
        return self.pose.to_local(self.ball.x, self.ball.y)


@dataclass
class SkillOutput:
    """Velocity command plus status for the caller and for logs."""

    v: float = 0.0
    w: float = 0.0
    status: str = RUNNING
    reason: str = ''
    phase: str = ''
    info: dict = field(default_factory=dict)

    @classmethod
    def done(cls, reason='', **info):
        """Success with a stop command."""
        return cls(0.0, 0.0, SUCCESS, reason, info=info)

    @classmethod
    def fail(cls, reason, **info):
        """Failure with a stop command."""
        return cls(0.0, 0.0, FAILURE, reason, info=info)
