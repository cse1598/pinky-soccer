"""Publish /world_model as RViz markers with the shared display rules.

/world_model/field_markers  MarkerArray, transient_local, sent once (field geometry)
/world_model/markers        MarkerArray, every /world_model message
/world_model/search_grid    OccupancyGrid copy of ball_search_grid (RViz Map display)
"""

import math

from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid
from pinky_soccer_field.field_geometry import Field
from pinky_soccer_msgs.msg import BehaviorGoal, WorldModel
from pinky_soccer_viz import style
from pinky_soccer_viz.style import ESTIMATED, LOST, OBSERVED
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def color(name, alpha=1.0):
    """Return ColorRGBA for a style colour."""
    r, g, b = style.COLORS[name]
    return ColorRGBA(r=r, g=g, b=b, a=alpha)


def pt(x, y, z=0.0):
    """Return geometry_msgs/Point."""
    return Point(x=float(x), y=float(y), z=float(z))


class MarkerFactory:
    """Creates markers with consistent header and ids."""

    def __init__(self, frame='map'):
        self.frame = frame
        self.next_id = 0

    def make(self, ns, kind, rgba, scale=(0.01, 0.01, 0.01)):
        """Return a new marker."""
        m = Marker()
        m.header.frame_id = self.frame
        m.ns, m.id, m.type, m.action = ns, self.next_id, kind, Marker.ADD
        self.next_id += 1
        m.color = rgba
        m.scale.x, m.scale.y, m.scale.z = (float(v) for v in scale)
        m.pose.orientation.w = 1.0
        return m


def rect_strip(f, rect, ns, rgba, width, z=0.001):
    """Return a closed LINE_STRIP for a Rect."""
    m = f.make(ns, Marker.LINE_STRIP, rgba, (width, 0, 0))
    m.points = [pt(rect.xmin, rect.ymin, z), pt(rect.xmax, rect.ymin, z),
                pt(rect.xmax, rect.ymax, z), pt(rect.xmin, rect.ymax, z),
                pt(rect.xmin, rect.ymin, z)]
    return m


def field_markers(field):
    """Return the static field MarkerArray."""
    f = MarkerFactory()
    out = MarkerArray()
    walls = field.inner_walls
    carpet = f.make('field', Marker.CUBE, color('carpet'), (walls.width, walls.height, 0.002))
    carpet.pose.position.z = -0.001
    out.markers.append(carpet)
    for o in field.obstacles():
        if not o.name.startswith(('end_wall', 'side_wall')):
            continue
        m = f.make('walls', Marker.CUBE, color('wall'),
                   (o.rect.width, o.rect.height, field.spec['wall_height']))
        m.pose.position = pt((o.rect.xmin + o.rect.xmax) / 2, (o.rect.ymin + o.rect.ymax) / 2,
                             field.spec['wall_height'] / 2)
        out.markers.append(m)
    lw = field.spec['line_width']
    out.markers.append(rect_strip(f, field.playing_area, 'lines', color('line'), lw))
    for c in ('yellow', 'blue'):
        out.markers.append(rect_strip(f, field.penalty_area(c), 'lines', color('line'), lw))
        g = field.goal_interior(c)
        m = f.make('goals', Marker.CUBE, color(c, 0.8),
                   (g.width, g.height, field.spec['goal_height']))
        m.pose.position = pt((g.xmin + g.xmax) / 2, 0.0, field.spec['goal_height'] / 2)
        out.markers.append(m)
    cx, cy, r = field.center_circle
    circle = f.make('lines', Marker.LINE_STRIP, color('center'), (0.005, 0, 0))
    circle.points = [pt(cx + r * math.cos(a), cy + r * math.sin(a), 0.001)
                     for a in [2 * math.pi * k / 64 for k in range(65)]]
    out.markers.append(circle)
    spots = f.make('lines', Marker.SPHERE_LIST, color('center'), (0.015, 0.015, 0.002))
    spots.points = [pt(x, y, 0.001) for x, y in field.neutral_spots]
    out.markers.append(spots)
    return out


def ellipse_marker(f, ns, x, y, cov, rgba):
    """Return a flat cylinder at the 2-sigma covariance ellipse, or None."""
    major, minor, angle = style.covariance_ellipse(cov)
    if major <= 0:
        return None
    m = f.make(ns, Marker.CYLINDER, rgba, (2 * major, max(2 * minor, 0.002), 0.002))
    m.pose.position = pt(x, y, 0.003)
    m.pose.orientation.z, m.pose.orientation.w = math.sin(angle / 2), math.cos(angle / 2)
    return m


class WorldModelViz(Node):
    """Marker publisher."""

    def __init__(self):
        super().__init__('world_model_viz')
        p = self.declare_parameter
        self.field = Field.from_file(p('field_spec', '').value or None,
                                     p('our_goal_color', 'yellow').value)
        self.hfov = math.radians(p('camera_hfov_deg', 60.0).value)
        self.fov_range = p('fov_range', 1.5).value
        self.show_truth = p('show_ground_truth', True).value
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.field_pub = self.create_publisher(MarkerArray, 'world_model/field_markers',
                                               latched)
        self.field_pub.publish(field_markers(self.field))
        self.pub = self.create_publisher(MarkerArray, 'world_model/markers', 10)
        self.grid_pub = self.create_publisher(OccupancyGrid, 'world_model/search_grid', 2)
        self.goal = None
        self.truth = {}
        self.create_subscription(WorldModel, 'world_model', self.on_wm, 10)
        self.create_subscription(BehaviorGoal, 'behavior_goal',
                                 lambda m: setattr(self, 'goal', m), 10)
        self.create_subscription(CameraInfo, 'camera/camera_info', self.on_info,
                                 qos_profile_sensor_data)
        if self.show_truth:
            self.create_subscription(PoseStamped, 'ground_truth',
                                     lambda m: self.on_truth('robot', m), 10)
            self.create_subscription(PoseStamped, 'sim/ball_ground_truth',
                                     lambda m: self.on_truth('ball', m), 10)

    def on_info(self, msg):
        """Use the calibrated horizontal FOV."""
        if msg.k[0] > 0 and msg.width > 0:
            self.hfov = 2 * math.atan(msg.width / (2 * msg.k[0]))

    def on_truth(self, name, msg):
        """Store ground truth for the grey overlay (evaluation only)."""
        self.truth[name] = (msg.pose.position.x, msg.pose.position.y)

    def on_wm(self, wm):
        """Convert one world model into markers."""
        f = MarkerFactory()
        out = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        out.markers.append(clear)
        now = wm.header.stamp.sec + wm.header.stamp.nanosec * 1e-9

        me = wm.self
        arrow = f.make('self', Marker.ARROW, color('self'), (0.12, 0.02, 0.02))
        arrow.pose.position = pt(me.pose.x, me.pose.y, 0.05)
        th = me.pose.theta
        arrow.pose.orientation.z, arrow.pose.orientation.w = math.sin(th / 2), math.cos(th / 2)
        body = f.make('self', Marker.CYLINDER, color('self', 0.8),
                      (2 * style.ROBOT_RADIUS, 2 * style.ROBOT_RADIUS, 0.04))
        body.pose.position = pt(me.pose.x, me.pose.y, 0.02)
        out.markers += [arrow, body]
        if me.state == ESTIMATED:
            e = ellipse_marker(f, 'self', me.pose.x, me.pose.y, me.covariance, color('self', 0.3))
            if e:
                out.markers.append(e)
        fov = f.make('fov', Marker.LINE_STRIP, color('fov', 0.6), (0.005, 0, 0))
        poly = style.fov_polygon((me.pose.x, me.pose.y, th), self.hfov, self.fov_range)
        fov.points = [pt(x, y, 0.004) for x, y in poly + poly[:1]]
        out.markers.append(fov)

        b = wm.ball
        if b.state == LOST:
            x = f.make('ball', Marker.LINE_LIST, color('ball'), (0.01, 0, 0))
            d = 0.04
            x.points = [pt(b.pose.x - d, b.pose.y - d, 0.01), pt(b.pose.x + d, b.pose.y + d, 0.01),
                        pt(b.pose.x - d, b.pose.y + d, 0.01), pt(b.pose.x + d, b.pose.y - d, 0.01)]
            out.markers.append(x)
        else:
            alpha = style.SOLID_ALPHA if b.state == OBSERVED else style.FAINT_ALPHA
            s = f.make('ball', Marker.SPHERE, color('ball', alpha), (0.0427, 0.0427, 0.0427))
            s.pose.position = pt(b.pose.x, b.pose.y, 0.0213)
            out.markers.append(s)
            if b.state == ESTIMATED:
                e = ellipse_marker(f, 'ball', b.pose.x, b.pose.y, b.covariance,
                                   color('ball', 0.3))
                if e:
                    out.markers.append(e)
            if math.hypot(b.velocity.x, b.velocity.y) > 0.02:
                pred = f.make('ball_prediction', Marker.LINE_LIST, color('ball', 0.8),
                              (0.006, 0, 0))
                pts = style.ball_prediction((b.pose.x, b.pose.y), (b.velocity.x, b.velocity.y),
                                            steps=12)
                pred.points = [pt(x, y, 0.01) for x, y in pts[:len(pts) // 2 * 2]]
                out.markers.append(pred)

        for o in wm.opponents:
            if o.state == LOST:
                continue
            alpha = style.SOLID_ALPHA if o.state == OBSERVED else style.FAINT_ALPHA
            box = f.make('opponents', Marker.CUBE, color('opponent', alpha),
                         (style.OPPONENT_SIZE, style.OPPONENT_SIZE, 0.08))
            box.pose.position = pt(o.pose.x, o.pose.y, 0.04)
            out.markers.append(box)
            if o.state == ESTIMATED:
                age = max(0.0, now - (o.last_seen.sec + o.last_seen.nanosec * 1e-9))
                text = f.make('opponents', Marker.TEXT_VIEW_FACING, color('opponent'),
                              (0, 0, 0.05))
                text.pose.position = pt(o.pose.x, o.pose.y, 0.14)
                text.text = f'{age:.1f}s'
                out.markers.append(text)

        if self.goal is not None and self.goal.mode != BehaviorGoal.STOP:
            t = self.goal.target
            g = f.make('goal', Marker.ARROW, color('target'), (0.08, 0.015, 0.015))
            g.pose.position = pt(t.x, t.y, 0.06)
            g.pose.orientation.z, g.pose.orientation.w = math.sin(t.theta / 2), \
                math.cos(t.theta / 2)
            out.markers.append(g)
            if len(self.goal.planned_path.poses) > 1:
                path = f.make('goal', Marker.LINE_STRIP, color('path'), (0.006, 0, 0))
                path.points = [pt(p.pose.position.x, p.pose.position.y, 0.005)
                               for p in self.goal.planned_path.poses]
                out.markers.append(path)
        label = f.make('status', Marker.TEXT_VIEW_FACING, color('line'), (0, 0, 0.07))
        label.pose.position = pt(0.0, self.field.inner_walls.ymax + 0.15, 0.1)
        label.text = (f'{self.goal.branch} / {self.goal.action}' if self.goal else 'no goal') + \
            f'  ball {style.STATE_NAMES.get(b.state, "?")}'
        out.markers.append(label)

        if self.show_truth:
            for name, (x, y) in self.truth.items():
                m = f.make('ground_truth', Marker.SPHERE, color('truth', 0.9),
                           (0.03, 0.03, 0.03))
                m.pose.position = pt(x, y, 0.06 if name == 'robot' else 0.025)
                out.markers.append(m)
        for m in out.markers:
            m.header.stamp = wm.header.stamp
        self.pub.publish(out)
        if wm.ball_search_grid.info.width:
            self.grid_pub.publish(wm.ball_search_grid)


def main(args=None):
    """Run the marker publisher."""
    rclpy.init(args=args)
    node = WorldModelViz()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
