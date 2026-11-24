"""Robot namespaces: gz topic/bridge rewriting and the two-robot MuJoCo scene."""

import math
import xml.etree.ElementTree as ET

from pinky_soccer_sim import multi_robot
from pinky_soccer_sim.mujoco_multi import place_robots, PREFIX
import pytest

URDF = """<robot name="pinky">
  <gazebo><plugin name="diff"><topic>cmd_vel</topic><odom_topic>odom</odom_topic>
    <tf_topic>/tf</tf_topic><frame_id>odom</frame_id></plugin></gazebo>
  <gazebo reference="rplidar_link"><sensor name="lidar"><topic>scan</topic>
    <gz_frame_id>rplidar_link</gz_frame_id></sensor></gazebo>
</robot>"""

SCENE = """<mujoco><worldbody>
  <body name="base_link" pos="0 0 0.03"><freejoint name="floating_base"/>
    <geom name="base_collision" group="1" rgba="0.4 0.4 0.4 0" type="sphere" size="0.05"/>
    <geom name="base_visual" group="2" type="sphere" size="0.05"/>
    <body name="l_wheel"><joint name="l_wheel_joint" type="hinge"/>
      <geom name="l_wheel_collision_0" group="1" type="sphere" size="0.02"/></body>
    <site name="soccer_imu_link"/>
  </body>
  <body name="soccer_ball"><freejoint/><geom type="sphere" size="0.02"/></body>
</worldbody>
<actuator><velocity name="l_motor" joint="l_wheel_joint"/></actuator>
<sensor><gyro name="soccer_gyro" site="soccer_imu_link"/>
  <framequat name="soccer_quat" objtype="site" objname="soccer_imu_link"/></sensor>
</mujoco>"""


def test_gz_topics_move_under_the_namespace_frames_stay():
    out = ET.fromstring(multi_robot.namespace_gz_topics(URDF, 'blue'))
    assert out.find('.//plugin/topic').text == '/blue/cmd_vel'
    assert out.find('.//plugin/odom_topic').text == '/blue/odom'
    assert out.find('.//plugin/tf_topic').text == '/blue/tf'
    assert out.find('.//sensor/topic').text == '/blue/scan'
    assert out.find('.//plugin/frame_id').text == 'odom'            # frames unchanged
    assert out.find('.//sensor/gz_frame_id').text == 'rplidar_link'
    assert multi_robot.namespace_gz_topics(URDF, '') == URDF


def test_bridge_config_keeps_clock_global():
    entries = [{'ros_topic_name': 'clock', 'gz_topic_name': 'clock'},
               {'ros_topic_name': 'tf', 'gz_topic_name': 'tf'},
               {'ros_topic_name': 'sim/lidar/scan', 'gz_topic_name': 'scan'}]
    robot, shared = multi_robot.namespace_bridge_config(entries, 'yellow')
    assert shared == [entries[0]]
    assert [(e['ros_topic_name'], e['gz_topic_name']) for e in robot] == [
        ('tf', '/yellow/tf'), ('sim/lidar/scan', '/yellow/scan')]


def test_names_teams_and_default_poses():
    assert multi_robot.team_color('Y') == 'yellow' and multi_robot.team_color('B') == 'blue'
    assert multi_robot.team_color('', 'blue') == 'blue'
    with pytest.raises(ValueError):
        multi_robot.team_color('R')
    with pytest.raises(ValueError):
        multi_robot.check_name('2robot')
    assert multi_robot.topic('', 'odom') == '/odom'
    assert multi_robot.topic('blue', 'odom') == '/blue/odom'
    assert multi_robot.kickoff_pose('yellow', True) == (-0.4, 0.0, 0.0)
    assert multi_robot.kickoff_pose('blue', True) == (0.4, 0.0, math.pi)
    assert multi_robot.obstacle_pose('yellow', 'lateral')[2] == pytest.approx(math.pi / 2)
    assert multi_robot.obstacle_pose('yellow', 'longitudinal') == (0.4, 0.0, math.pi)


def test_second_robot_is_a_prefixed_copy(tmp_path):
    path = tmp_path / 'scene.xml'
    path.write_text(SCENE)
    place_robots(str(path), ours=(-0.4, 0.1, 0.0), opponent=(0.4, 0.0, math.pi))
    root = ET.parse(path).getroot()
    world = root.find('worldbody')
    ours = world.find("body[@name='base_link']")
    twin = world.find(f"body[@name='{PREFIX}base_link']")
    assert ours.get('pos').startswith('-0.4 0.1')
    assert twin is not None and twin.get('pos').startswith('0.4 0')
    assert twin.find(f"freejoint[@name='{PREFIX}floating_base']") is not None
    # collision geoms: own groups, hittable by rays (alpha > 0), visuals untouched
    assert ours.find("geom[@name='base_collision']").get('group') == '5'
    assert twin.find(f"geom[@name='{PREFIX}base_collision']").get('group') == '3'
    assert float(twin.find(f"geom[@name='{PREFIX}base_collision']").get('rgba').split()[3]) > 0
    assert twin.find(f"geom[@name='{PREFIX}base_visual']").get('group') == '2'
    act = root.find(f"actuator/velocity[@name='{PREFIX}l_motor']")
    assert act is not None and act.get('joint') == f'{PREFIX}l_wheel_joint'
    assert root.find(f"sensor/gyro[@name='{PREFIX}soccer_gyro']").get('site') == \
        f'{PREFIX}soccer_imu_link'
    assert root.find(f"sensor/framequat[@name='{PREFIX}soccer_quat']").get('objname') == \
        f'{PREFIX}soccer_imu_link'
    assert world.find("body[@name='soccer_ball']") is not None     # ball not duplicated
