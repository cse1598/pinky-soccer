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


def ball_model(tmp_path, pos, velocity, world=WORLD):
    mj = pytest.importorskip('mujoco')
    root = ET.parse(world.with_suffix('.mjcf')).getroot()
    ET.SubElement(root, 'option', timestep='.002', gravity='0 0 -9.81')
    body = ET.SubElement(root.find('worldbody'), 'body', name='test_ball',
                         pos=' '.join(map(str, pos)))
    ET.SubElement(body, 'freejoint')
    ET.SubElement(body, 'geom', type='sphere', size='.021335', mass='.0459',
                  contype='3', conaffinity='3', friction='.5 .005 .0001')
    for mesh in root.findall('asset/mesh'):
        mesh.set('file', str((world.parent/mesh.get('file')).resolve()))
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


def test_hardboard_exact_interior_and_independent_assets(tmp_path):
    """Four boards define the complete interior, including space behind both goals."""
    spec = ROOT/'config/hardboard_field.yaml'
    cfg = load_spec(spec)
    assert cfg['playing_length']+2*cfg['outer_strip'] == pytest.approx(1.820)
    assert cfg['playing_width']+2*cfg['outer_strip'] == pytest.approx(1.220)
    assert cfg['wall_height'] == .200
    objects = {o['name']: o for o in scene(cfg)}
    assert not any(o['kind'] == 'mesh' for o in objects.values())
    assert objects['carpet']['size'][:2] == pytest.approx([1.82, 1.22])
    for name, item in objects.items():
        if name.startswith(('side_wall_', 'end_wall_')):
            assert item['pos'][2]+item['size'][2]/2 == pytest.approx(.2)
        if '_goal_' in name:
            assert abs(item['pos'][0])+item['size'][0]/2 < .91
    assert cfg['goal_height'] > .04267
    assert cfg['goal_depth'] > .04267
    assert objects['blue_goal_back']['size'] == pytest.approx([.005, .4, .07])
    assert objects['blue_goal_side_1']['size'] == pytest.approx([.055, .005, .07])
    assert objects['blue_goal_roof']['size'] == pytest.approx([.055, .41, .005])
    generate(spec, tmp_path)
    assert not (tmp_path/'rcj_soccer.world').exists()
    for name in ('hardboard_soccer.world', 'hardboard_soccer.mjcf',
                 'hardboard_field_manifest.yaml'):
        assert (tmp_path/name).read_bytes() == (ROOT/'worlds'/name).read_bytes()
    sdf = ET.parse(tmp_path/'hardboard_soccer.world').getroot()
    mjcf = ET.parse(tmp_path/'hardboard_soccer.mjcf').getroot()
    assert sdf.find("world[@name='hardboard_soccer']/model[@name='hardboard_field']") is not None
    assert len(sdf.findall('.//visual')) == len(mjcf.findall('worldbody/geom')) == len(objects)


def test_hardboard_ball_enters_scaled_goal_and_lidar_sees_lowest_wall(tmp_path):
    """The unchanged golf ball fits the reduced goal and the 20cm walls reflect LiDAR."""
    world = ROOT/'worlds/hardboard_soccer.world'
    mj, model, data = ball_model(tmp_path, [.79, 0, .022], [.35, 0, 0], world)
    data.qvel[4] = .35/.021335
    positions = []
    for _ in range(600):
        mj.mj_step(model, data)
        positions.append(data.qpos[0])
    assert max(positions) == pytest.approx(.88-.021335, abs=.003)
    geom = np.zeros(1, dtype=np.int32)
    groups = np.array([1, 0, 0, 0, 1, 0], dtype=np.uint8)
    for direction, expected in [([1., 0., 0.], .91), ([0., 1., 0.], .61)]:
        assert mj.mj_ray(model, data, np.array([0., 0., .125]),
                         np.array(direction), groups, True, -1, geom) == pytest.approx(expected)


def test_switching_field_presets_replaces_mujoco_geometry(tmp_path):
    """Reusing a model directory must not retain the previous field or alter robot joints."""
    mj = pytest.importorskip('mujoco')
    from ament_index_python.packages import get_package_share_directory
    from pinky_mujoco.description import expand_urdf
    from pinky_mujoco.model import build_model
    bridge = SimpleNamespace(build_model=build_model)
    install_mujoco(bridge)
    urdf = expand_urdf(package='pinky_soccer_description',
                       filename='urdf/soccer_robot.urdf.xacro')
    for name, prefix, other in [('rcj_soccer', 'rcj', 'hardboard'),
                                ('hardboard_soccer', 'hardboard', 'rcj')]:
        path = bridge.build_model(get_package_share_directory('pinky_mujoco'), tmp_path,
                                  urdf=urdf, world=ROOT/'worlds'/f'{name}.world',
                                  simple_render=True)
        model = mj.MjModel.from_xml_path(str(path))
        assert model.joint('floating_base').qposadr[0] == 0
        assert model.geom(f'{prefix}_blue_goal_back').id >= 0
        with pytest.raises(KeyError):
            model.geom(f'{other}_blue_goal_back')


def test_field_selector_keeps_legacy_worlds_and_rejects_conflicting_choices(monkeypatch):
    """A preset selects matched YAML/world assets; it never silently replaces an explicit world."""
    from pinky_soccer_sim import field_selection
    monkeypatch.setattr(field_selection, 'get_package_share_directory', lambda _: str(ROOT))
    world, spec = field_selection.field_files('hardboard')
    assert world == ROOT/'worlds/hardboard_soccer.world'
    assert spec == ROOT/'config/hardboard_field.yaml'
    assert field_selection.selected_world('hardboard', '', 'empty.world') == world
    assert field_selection.selected_world('', '/tmp/custom.world', 'empty.world') is None
    for args in [('bad', '', 'empty.world'), ('rcj', '', 'hardboard_soccer.world'),
                 ('hardboard', '/tmp/custom.world', 'empty.world')]:
        with pytest.raises(ValueError):
            field_selection.selected_world(*args)


def test_custom_non_generated_soccer_world_keeps_original_loader(tmp_path):
    """A user's legacy SDF filename ending in _soccer is not sufficient to intercept it."""
    world = tmp_path/'custom_soccer.world'
    world.write_text('<sdf version="1.9"><world name="custom"/></sdf>')
    calls = []

    def original(*args):
        calls.append(args)
        return 'legacy-result'

    bridge = SimpleNamespace(build_model=original)
    install_mujoco(bridge)
    assert bridge.build_model('/share', tmp_path, world=world) == 'legacy-result'
    assert calls[0][4] == world
