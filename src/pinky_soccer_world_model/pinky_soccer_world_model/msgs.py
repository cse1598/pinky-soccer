"""Builders for pinky_soccer_msgs shared by world_model_node and fake_world_model."""

from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Point
from pinky_soccer_field.field_geometry import occupancy_grid_msg
from pinky_soccer_msgs.msg import TrackedObject, WorldModel


def time_msg(seconds):
    """Return builtin_interfaces/Time for float seconds (>= 0)."""
    seconds = max(0.0, float(seconds))
    sec = int(seconds)
    return TimeMsg(sec=sec, nanosec=int(round((seconds - sec) * 1e9)) % 1000000000)


def tracked(obj_id, pose, velocity=(0.0, 0.0), cov=None, state=TrackedObject.OBSERVED,
            last_seen=0.0, confidence=1.0):
    """Return a TrackedObject; pose is (x, y, theta) in map, cov row-major 3x3."""
    msg = TrackedObject()
    msg.id = obj_id
    msg.pose.x, msg.pose.y, msg.pose.theta = map(float, pose)
    msg.velocity.x, msg.velocity.y = map(float, velocity)
    msg.covariance = [float(v) for v in (cov if cov is not None else [0.0] * 9)]
    msg.state = int(state)
    msg.last_seen = time_msg(last_seen)
    msg.confidence = float(max(0.0, min(1.0, confidence)))
    return msg


def world_model(stamp, field, self_obj, ball, opponents, playing, grid=None):
    """Assemble a WorldModel in the map frame. grid: SearchGrid or None."""
    msg = WorldModel()
    msg.header.stamp = stamp
    msg.header.frame_id = 'map'
    msg.self = self_obj
    msg.ball = ball
    msg.opponents = list(opponents)
    msg.own_goal = Point(x=field.own_goal[0], y=field.own_goal[1], z=0.0)
    msg.opp_goal = Point(x=field.opp_goal[0], y=field.opp_goal[1], z=0.0)
    msg.our_goal_color = field.our_goal_color
    msg.playing = bool(playing)
    if grid is not None:
        msg.ball_search_grid = occupancy_grid_msg(grid.as_int8(), grid.res, grid.origin,
                                                  'map', stamp)
    return msg
