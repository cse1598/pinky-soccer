"""RCJ soccer field geometry in the map frame, shared by every runtime node.

The map frame is the field frame: origin at the field centre, z=0 on the flat
carpet top, +X towards the BLUE goal and -X towards the YELLOW goal.

The dimension source is a single YAML file (today
``pinky_soccer_sim/config/rcj_field.yaml``). This module re-implements the
validation rules of ``pinky_soccer_sim.rcj_field.load_spec`` so it can run on
machines without the simulator package; ``test_field_geometry`` checks that
both loaders accept and reject the same files when the simulator is present.
"""

from dataclasses import dataclass
import math
import os
from pathlib import Path

import yaml

GOAL_COLORS = ('yellow', 'blue')
# Fixed by the field model: blue goal on +X, yellow goal on -X.
COLOR_SIGN = {'blue': 1.0, 'yellow': -1.0}
TEXT_KEYS = ('source', 'specification_date')


def load_spec(path):
    """Load and validate a field YAML with the same rules as the sim generator."""
    cfg = yaml.safe_load(Path(path).read_text())
    if not isinstance(cfg, dict):
        raise ValueError(f'Field spec is not a mapping: {path}')
    for key, value in cfg.items():
        if key in TEXT_KEYS:
            continue
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value) or value <= 0):
            raise ValueError(f'Invalid field setting: {key}')
    required = ('playing_length', 'playing_width', 'outer_strip', 'wall_height',
                'goal_width', 'goal_height', 'goal_depth', 'ramp_base', 'line_width',
                'penalty_depth', 'penalty_width', 'penalty_radius', 'center_diameter',
                'neutral_distance_from_end', 'wall_thickness', 'arc_segments')
    missing = [k for k in required if k not in cfg]
    if missing:
        raise ValueError(f'Missing field settings: {", ".join(missing)}')
    if (cfg['goal_width'] >= cfg['playing_width'] or cfg['goal_depth'] >= cfg['outer_strip']
            or cfg['ramp_base'] > cfg['outer_strip']
            or cfg['penalty_radius'] >= cfg['penalty_depth']
            or 2*cfg['penalty_radius'] >= cfg['penalty_width']):
        raise ValueError('Inconsistent RCJ field dimensions')
    if type(cfg['arc_segments']) is not int or cfg['arc_segments'] < 32:
        raise ValueError('arc_segments must be an integer >=32')
    return cfg


def default_spec_path():
    """Return the installed sim field YAML, the only dimension source for now.

    Raises FileNotFoundError when the simulator package is not installed (for
    example on the robot); pass ``field_spec`` explicitly in that case.
    """
    env = os.environ.get('PINKY_SOCCER_FIELD_SPEC')
    if env:
        return env
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('pinky_soccer_sim')
    except Exception as exc:  # PackageNotFoundError or missing ament index
        raise FileNotFoundError(
            'pinky_soccer_sim is not installed; set the field_spec parameter '
            'to a copy of rcj_field.yaml') from exc
    return os.path.join(share, 'config', 'rcj_field.yaml')


@dataclass(frozen=True)
class Rect:
    """Axis-aligned rectangle in the map frame."""

    xmin: float
    ymin: float
    xmax: float
    ymax: float

    @property
    def width(self):
        """Extent along X."""
        return self.xmax - self.xmin

    @property
    def height(self):
        """Extent along Y."""
        return self.ymax - self.ymin

    def contains(self, x, y, margin=0.0):
        """Return True when (x, y) lies inside the rectangle shrunk by margin."""
        return (self.xmin + margin <= x <= self.xmax - margin
                and self.ymin + margin <= y <= self.ymax - margin)

    def clamp(self, x, y, margin=0.0):
        """Project (x, y) into the rectangle shrunk by margin."""
        return (min(max(x, self.xmin + margin), self.xmax - margin),
                min(max(y, self.ymin + margin), self.ymax - margin))


@dataclass(frozen=True)
class Obstacle:
    """A solid box footprint with its top height, for map generation."""

    name: str
    rect: Rect
    top: float


class Field:
    """Geometry derived from one validated spec and the team goal colour."""

    def __init__(self, spec, our_goal_color='yellow'):
        if our_goal_color not in GOAL_COLORS:
            raise ValueError(f'our_goal_color must be one of {GOAL_COLORS}')
        self.spec = dict(spec)
        self.our_goal_color = our_goal_color
        self.opp_goal_color = 'blue' if our_goal_color == 'yellow' else 'yellow'

    @classmethod
    def from_file(cls, path=None, our_goal_color='yellow'):
        """Load the field from a YAML path (default: installed sim spec)."""
        return cls(load_spec(path or default_spec_path()), our_goal_color)

    # --- basic dimensions -------------------------------------------------
    @property
    def half_length(self):
        """Half of the playing length; the goal mouth plane is at +-this x."""
        return self.spec['playing_length'] / 2

    @property
    def half_width(self):
        """Half of the playing width."""
        return self.spec['playing_width'] / 2

    @property
    def playing_area(self):
        """Outside edge of the white boundary line."""
        return Rect(-self.half_length, -self.half_width, self.half_length, self.half_width)

    @property
    def inner_walls(self):
        """Inside faces of the outer black walls (whole reachable floor)."""
        hx = self.half_length + self.spec['outer_strip']
        hy = self.half_width + self.spec['outer_strip']
        return Rect(-hx, -hy, hx, hy)

    # --- goals --------------------------------------------------------------
    @staticmethod
    def color_sign(color):
        """+1 for blue (+X), -1 for yellow (-X)."""
        return COLOR_SIGN[color]

    def goal_center(self, color):
        """Centre of the goal mouth line (on the goal line)."""
        return (self.color_sign(color) * self.half_length, 0.0)

    def goal_mouth(self, color):
        """Goal mouth segment ((x, -w/2), (x, +w/2)) on the goal line."""
        x = self.color_sign(color) * self.half_length
        half = self.spec['goal_width'] / 2
        return ((x, -half), (x, half))

    def goal_back(self, color):
        """Inner face segment of the goal back plate."""
        x = self.color_sign(color) * (self.half_length + self.spec['goal_depth'])
        half = self.spec['goal_width'] / 2
        return ((x, -half), (x, half))

    def goal_interior(self, color):
        """Floor area inside the goal, between mouth and back plate."""
        s = self.color_sign(color)
        x0, x1 = s * self.half_length, s * (self.half_length + self.spec['goal_depth'])
        half = self.spec['goal_width'] / 2
        return Rect(min(x0, x1), -half, max(x0, x1), half)

    @property
    def own_goal(self):
        """Mouth centre of the goal we defend."""
        return self.goal_center(self.our_goal_color)

    @property
    def opp_goal(self):
        """Mouth centre of the goal we attack."""
        return self.goal_center(self.opp_goal_color)

    @property
    def attack_sign(self):
        """+1 when we attack towards +X, -1 when towards -X."""
        return self.color_sign(self.opp_goal_color)

    def ball_in_goal(self, x, y, color):
        """Return True when the ball centre has crossed the goal line inside the posts."""
        s = self.color_sign(color)
        return s * x > self.half_length and abs(y) < self.spec['goal_width'] / 2

    # --- markings -------------------------------------------------------------
    def penalty_area(self, color):
        """Penalty area rectangle (rounded outer corners ignored)."""
        s = self.color_sign(color)
        x0 = s * self.half_length
        x1 = s * (self.half_length - self.spec['penalty_depth'])
        half = self.spec['penalty_width'] / 2
        return Rect(min(x0, x1), -half, max(x0, x1), half)

    @property
    def penalty_radius(self):
        """Radius of the rounded outer penalty corners."""
        return self.spec['penalty_radius']

    @property
    def center_circle(self):
        """(x, y, radius) of the centre circle."""
        return (0.0, 0.0, self.spec['center_diameter'] / 2)

    @property
    def neutral_spots(self):
        """Return the centre spot followed by the four corner neutral spots (as in sim)."""
        dx = self.half_length - self.spec['neutral_distance_from_end']
        dy = self.spec['penalty_width'] / 2
        return [(0.0, 0.0)] + [(sx * dx, sy * dy) for sx in (-1, 1) for sy in (-1, 1)]

    # --- solids ---------------------------------------------------------------
    def obstacles(self):
        """Solid boxes with their top height (walls and goal structure)."""
        walls = self.inner_walls
        t, wall_h = self.spec['wall_thickness'], self.spec['wall_height']
        out = [
            Obstacle('end_wall_+', Rect(walls.xmax, walls.ymin - t, walls.xmax + t,
                                        walls.ymax + t), wall_h),
            Obstacle('end_wall_-', Rect(walls.xmin - t, walls.ymin - t, walls.xmin,
                                        walls.ymax + t), wall_h),
            Obstacle('side_wall_+', Rect(walls.xmin, walls.ymax, walls.xmax,
                                         walls.ymax + t), wall_h),
            Obstacle('side_wall_-', Rect(walls.xmin, walls.ymin - t, walls.xmax,
                                         walls.ymin), wall_h),
        ]
        gh, half = self.spec['goal_height'], self.spec['goal_width'] / 2
        for color in GOAL_COLORS:
            s = self.color_sign(color)
            front, back = self.half_length, self.half_length + self.spec['goal_depth']
            xs = sorted((s * back, s * (back + t)))
            out.append(Obstacle(f'{color}_goal_back', Rect(xs[0], -half - t, xs[1], half + t),
                                gh))
            xs = sorted((s * front, s * back))
            for side in (-1, 1):
                ys = sorted((side * half, side * (half + t)))
                out.append(Obstacle(f'{color}_goal_side_{side:+d}',
                                    Rect(xs[0], ys[0], xs[1], ys[1]), gh))
            xs = sorted((s * front, s * (back + t)))
            out.append(Obstacle(f'{color}_goal_roof', Rect(xs[0], -half - t, xs[1], half + t),
                                gh + t))
        return out

    def obstacles_seen_at(self, height):
        """Obstacles whose top is above a horizontal scan plane at ``height``."""
        return [o for o in self.obstacles() if o.top > height]


def occupancy_grid_msg(grid, resolution, origin_xy, frame_id='map', stamp=None):
    """Wrap an int8 numpy grid (row 0 = lowest y) into nav_msgs/OccupancyGrid."""
    from nav_msgs.msg import OccupancyGrid
    msg = OccupancyGrid()
    msg.header.frame_id = frame_id
    if stamp is not None:
        msg.header.stamp = stamp
    msg.info.resolution = float(resolution)
    msg.info.height, msg.info.width = int(grid.shape[0]), int(grid.shape[1])
    msg.info.origin.position.x, msg.info.origin.position.y = map(float, origin_xy)
    msg.info.origin.orientation.w = 1.0
    msg.data = grid.astype('int8').ravel().tolist()
    return msg
