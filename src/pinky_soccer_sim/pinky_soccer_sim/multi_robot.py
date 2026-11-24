"""
Robot namespaces for running several Pinky robots in one ROS graph.

Every robot's nodes live under /<name>/ (relative topics) and use their own TF topics
/<name>/tf and /<name>/tf_static, so frame names (odom, base_footprint, ...) stay the same as
for a single robot and no node needs a frame prefix. An empty name keeps the old single-robot
layout (no namespace, global /tf). Game-wide topics stay global: /game/*, /clock, /sim/*.
"""

import math
import os

from launch.actions import GroupAction, SetEnvironmentVariable
from launch_ros.actions import PushRosNamespace, SetRemap

TEAMS = {'Y': 'yellow', 'B': 'blue', 'yellow': 'yellow', 'blue': 'blue'}


def check_name(name):
    """Validate a robot name usable as a ROS namespace ('' = none)."""
    if name and not (name[0].isalpha() and all(c.isalnum() or c == '_' for c in name)):
        raise ValueError(f'robot name {name!r}: letters, digits and _ only, starting with a '
                         'letter')
    return name


def team_color(team, fallback='yellow'):
    """Map the team option Y|B (or yellow|blue; '' = fallback) to our goal colour."""
    if not team:
        return fallback
    if team not in TEAMS:
        raise ValueError('team must be Y or B')
    return TEAMS[team]


def other(color):
    """Return the other team colour."""
    return 'blue' if color == 'yellow' else 'yellow'


def topic(name, relative):
    """Absolute topic of a robot ('/name/relative', or '/relative' without a name)."""
    return f'/{name}/{relative}' if name else f'/{relative}'


def group(name, actions):
    """Put actions in the robot namespace with per-robot TF topics (no-op for '')."""
    if not name:
        return list(actions)
    return [GroupAction([PushRosNamespace(name), SetRemap('/tf', 'tf'),
                         SetRemap('/tf_static', 'tf_static'), *actions])]


def kickoff_pose(color, two_robots, distance=0.4):
    """Return the default start pose: our half, facing the opponent goal."""
    sign = -1.0 if color == 'yellow' else 1.0         # yellow defends -X
    if not two_robots:
        return (0.0, 0.0, 0.0 if color == 'yellow' else math.pi)
    return (sign * distance, 0.0, 0.0 if color == 'yellow' else math.pi)


def obstacle_pose(color, mode, distance=0.4):
    """Return the default obstacle pose in the half that `color` attacks."""
    x = distance if color == 'yellow' else -distance
    if mode == 'lateral':
        return (x, 0.0, math.pi / 2)                    # sideways: drives across the field
    return (x, 0.0, math.pi if color == 'yellow' else 0.0)   # faces the robot under test


GZ_TOPIC_TAGS = ('topic', 'odom_topic', 'tf_topic')


def namespace_gz_topics(urdf, name):
    """Move every Gazebo plugin/sensor topic of an expanded URDF under /<name>/."""
    import xml.etree.ElementTree as ET
    if not name:
        return urdf
    robot = ET.fromstring(urdf)
    for gazebo in robot.iter('gazebo'):
        for tag in GZ_TOPIC_TAGS:
            for el in gazebo.iter(tag):
                el.text = f'/{name}/{(el.text or "").strip().lstrip("/")}'
    return ET.tostring(robot, encoding='unicode')


def namespace_bridge_config(entries, name):
    """
    Return (robot entries, global entries) of a ros_gz_bridge config for robot `name`.

    Robot entries keep relative ROS names (the bridge node runs in the namespace) and get
    /<name>/ gz names; /clock stays global and is returned separately.
    """
    robot, shared = [], []
    for entry in entries:
        if entry['ros_topic_name'].lstrip('/') == 'clock':
            shared.append(dict(entry))
            continue
        e = dict(entry)
        e['ros_topic_name'] = entry['ros_topic_name'].lstrip('/')
        e['gz_topic_name'] = f'/{name}/{entry["gz_topic_name"].lstrip("/")}'
        robot.append(e)
    return robot, shared


# Every ROS process is one DDS participant. CycloneDDS allocates a per-host participant index
# with a small default ceiling; two full robot stacks (+ simulator) exceed it and further
# nodes fail with "Failed to find a free participant index" (measured with Jazzy).
DDS_PARTICIPANTS = ('<CycloneDDS><Domain id="any"><Discovery>'
                    '<MaxAutoParticipantIndex>120</MaxAutoParticipantIndex>'
                    '</Discovery></Domain></CycloneDDS>')


def dds_environment():
    """Raise the CycloneDDS participant limit for launched processes (user config kept)."""
    if os.environ.get('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp') != 'rmw_cyclonedds_cpp':
        return []
    current = os.environ.get('CYCLONEDDS_URI', '')
    if 'MaxAutoParticipantIndex' in current:
        return []
    value = f'{current},{DDS_PARTICIPANTS}' if current else DDS_PARTICIPANTS
    return [SetEnvironmentVariable('CYCLONEDDS_URI', value)]
