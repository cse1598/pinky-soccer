"""Dimension, engine parity and real contact tests for the generated RCJ field."""

from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import numpy as np
from pinky_soccer_sim.rcj_field import generate, install_mujoco, load_spec, scene
import pytest

ROOT = Path(__file__).parents[1]
SPEC = ROOT/'config/rcj_field.yaml'
WORLD = ROOT/'worlds/rcj_soccer.world'


def test_shared_dimensions_and_deterministic_outputs(tmp_path):
    cfg = load_spec(SPEC)
    assert cfg['playing_length'] + 2*cfg['outer_strip'] == pytest.approx(2.43)
    assert cfg['playing_width'] + 2*cfg['outer_strip'] == pytest.approx(1.82)
    objects = {o['name']: o for o in scene(cfg)}
    assert objects['blue_goal_back']['pos'][0] > 0
    assert objects['yellow_goal_back']['pos'][0] < 0
    assert objects['blue_goal_roof']['pos'][2] - objects['blue_goal_roof']['size'][2]/2 == .1
    assert len([o for o in objects if o.startswith('neutral_')]) == 5
    assert all(not o['collision'] for name, o in objects.items()
               if name.startswith(('boundary_', 'penalty_', 'center_circle', 'neutral_')))
    generate(SPEC, tmp_path)
    for name in ('rcj_soccer.world', 'rcj_soccer.mjcf', 'rcj_field_manifest.yaml'):
        assert (tmp_path/name).read_bytes() == (ROOT/'worlds'/name).read_bytes()
    sdf = ET.parse(tmp_path/'rcj_soccer.world').getroot()
    mj = ET.parse(tmp_path/'rcj_soccer.mjcf').getroot()
    assert len(sdf.findall('.//visual')) == len(mj.findall('worldbody/geom')) == len(objects)
    assert len(sdf.findall('.//collision')) == sum(o['collision'] for o in objects.values())


def ball_model(tmp_path, pos, velocity):
    mj = pytest.importorskip('mujoco')
    root = ET.parse(WORLD.with_suffix('.mjcf')).getroot()
    ET.SubElement(root, 'option', timestep='.002', gravity='0 0 -9.81')
    body = ET.SubElement(root.find('worldbody'), 'body', name='test_ball',
                         pos=' '.join(map(str, pos)))
    ET.SubElement(body, 'freejoint')
    ET.SubElement(body, 'geom', type='sphere', size='.021335', mass='.0459',
                  contype='3', conaffinity='3', friction='.5 .005 .0001')
    for mesh in root.findall('asset/mesh'):
        mesh.set('file', str((WORLD.parent/mesh.get('file')).resolve()))
    model = mj.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
    data = mj.MjData(model)
    data.qvel[:3] = velocity
    mj.mj_forward(model, data)
    return mj, model, data


def test_ball_enters_goal_and_hits_back_wall(tmp_path):
    mj, model, data = ball_model(tmp_path, [1.06, 0, .022], [.35, 0, 0])
    data.qvel[4] = .35/.021335
    positions = []
    for _ in range(600):
        mj.mj_step(model, data)
        positions.append(data.qpos[0])
    assert max(positions) == pytest.approx(1.169-.021335, abs=.003)
    assert abs(data.qpos[1]) < .01
    assert .015 < data.qpos[2] < .03


def test_ball_rolls_down_ramp_without_corner_overlap(tmp_path):
    mj, model, data = ball_model(tmp_path, [0, .875, .037], [0, 0, 0])
    for _ in range(1500):
        mj.mj_step(model, data)
    assert data.qpos[1] < .81
    assert .015 < data.qpos[2] < .03


def test_lidar_detects_wall_and_line_is_not_obstacle(tmp_path):
    mj, model, data = ball_model(tmp_path, [0, 0, .022], [0, 0, 0])
    groups = np.array([1, 0, 0, 0, 1, 0], dtype=np.uint8)
    geom = np.zeros(1, dtype=np.int32)
    distance = mj.mj_ray(model, data, np.array([0., 0., .15]),
                         np.array([0., 1., 0.]), groups, True, -1, geom)
    assert distance == pytest.approx(.91, abs=1e-5)
    distance = mj.mj_ray(model, data, np.array([.3, 0., .1]),
                         np.array([0., 0., -1.]), groups, True, -1, geom)
    assert distance == pytest.approx(.1, abs=1e-5)  # black circle is not a bump


def test_wrapper_preserves_robot_and_injects_field(tmp_path):
    mj = pytest.importorskip('mujoco')
    from ament_index_python.packages import get_package_share_directory
    from pinky_mujoco.description import expand_urdf
    from pinky_mujoco.model import build_model
    bridge = SimpleNamespace(build_model=build_model)
    install_mujoco(bridge)
    urdf = expand_urdf(package='pinky_soccer_description',
                       filename='urdf/soccer_robot.urdf.xacro')
    path = bridge.build_model(get_package_share_directory('pinky_mujoco'), tmp_path,
                              urdf=urdf, world=WORLD, simple_render=True)
    model = mj.MjModel.from_xml_path(str(path))
    assert model.joint('floating_base').qposadr[0] == 0
    assert model.geom('rcj_blue_goal_back').id >= 0
    assert model.nmesh >= 6
