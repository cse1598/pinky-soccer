"""Tests for the MuJoCo test-ball injection (physics tests need the mujoco pip package)."""
import math
import xml.etree.ElementTree as ET

from pinky_soccer_sim.mujoco_soccer_bridge import add_ball, parse_args
import pytest

SCENE = """<mujoco model="t">
  <worldbody>
    <geom name="floor" type="plane" size="1 1 0.1" contype="1" conaffinity="2"/>
    <body name="base_link"><freejoint name="floating_base"/></body>
  </worldbody>
</mujoco>"""


def write_scene(tmp_path):
    path = tmp_path / 'scene.xml'
    path.write_text(SCENE)
    return path


def test_ball_is_appended_last_with_free_joint(tmp_path):
    path = add_ball(write_scene(tmp_path), x=0.4, y=0.1, radius=0.02, mass=0.03)
    bodies = ET.parse(path).getroot().find('worldbody').findall('body')
    assert [b.get('name') for b in bodies] == ['base_link', 'soccer_ball']
    ball = bodies[-1]
    assert ball.find('freejoint') is not None
    assert [float(v) for v in ball.get('pos').split()] == pytest.approx([0.4, 0.1, 0.021])


def test_ball_collides_with_floor_and_robot_masks(tmp_path):
    path = add_ball(write_scene(tmp_path))
    geom = ET.parse(path).getroot().find(".//geom[@name='soccer_ball_geom']")
    contype, conaffinity = int(geom.get('contype')), int(geom.get('conaffinity'))

    def collide(a, b):
        return bool((a[0] & b[1]) or (b[0] & a[1]))

    ball = (contype, conaffinity)
    assert collide(ball, (1, 2)), 'floor / world geoms'
    assert collide(ball, (2, 1)), 'robot collision geoms'
    assert geom.get('group') == '0', 'lidar casts against group 0'


def test_ball_inertia_is_solid_sphere(tmp_path):
    path = add_ball(write_scene(tmp_path), radius=0.02135, mass=0.0459)
    inertial = ET.parse(path).getroot().find(".//body[@name='soccer_ball']/inertial")
    expected = 2 * 0.0459 * 0.02135 ** 2 / 5
    assert [float(v) for v in inertial.get('diaginertia').split()] == pytest.approx([expected] * 3)


def test_measured_diameter_matches_gazebo_and_perception(tmp_path):
    """Keep the user-measured 42.7 mm physical/visual and camera plane consistent."""
    import importlib.util
    from pathlib import Path

    from ament_index_python.packages import get_package_share_directory
    import yaml

    # Perception is optional when only the simulator package is installed.
    BallFilter = pytest.importorskip('pinky_soccer_world_model.ball_filter').BallFilter
    radius = .0427/2
    scene = ET.parse(add_ball(write_scene(tmp_path))).getroot()
    geom = scene.find(".//geom[@name='soccer_ball_geom']")
    assert float(geom.get('size')) == pytest.approx(radius)
    assert parse_args([])[0].ball_radius == pytest.approx(radius)
    material = scene.find("asset/material[@name='soccer_ball_material']")
    rgba = [float(v) for v in material.get('rgba').split()]
    assert rgba == pytest.approx([1, .30, 0, 1])
    path = Path(__file__).parents[1]/'launch/gazebo_soccer.launch.py'
    spec = importlib.util.spec_from_file_location('ball_sdf_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    sdf = ET.fromstring(module.BALL_SDF.format(name='ball', mass=.0459, radius=radius,
                                              inertia=2*.0459*radius**2/5))
    assert all(float(e.text) == pytest.approx(radius) for e in sdf.findall('.//sphere/radius'))
    assert [float(v) for v in sdf.find('.//diffuse').text.split()] == pytest.approx(rgba)
    cfg = Path(get_package_share_directory('pinky_soccer_world_model'))/'config'
    for name, node in [('ball_detector.yaml', 'ball_detector'), ('world_model.yaml', 'world_model_node')]:
        data = yaml.safe_load((cfg/name).read_text())[node]['ros__parameters']
        assert data['ball_radius'] == pytest.approx(radius)
    assert BallFilter().radius == pytest.approx(radius)


@pytest.mark.parametrize('radius, mass', [(0, 0.03), (0.02, 0), (-1, 1)])
def test_invalid_ball_rejected(tmp_path, radius, mass):
    with pytest.raises(ValueError):
        add_ball(write_scene(tmp_path), radius=radius, mass=mass)


def test_duplicate_ball_rejected(tmp_path):
    path = add_ball(write_scene(tmp_path))
    with pytest.raises(ValueError):
        add_ball(path)


def test_own_args_are_split_from_bridge_args():
    options, rest = parse_args(['--ball-x', '0.5', '--viewer', '--urdf-file', 'r.urdf'])
    assert options.ball and options.ball_x == 0.5
    assert rest == ['--viewer', '--urdf-file', 'r.urdf']
    assert parse_args(['--no-ball'])[0].ball is False


def build_soccer_scene(tmp_path, **bumper):
    """Build the real scene with the official builder + bumper URDF + ball."""
    pytest.importorskip('mujoco')
    pytest.importorskip('trimesh')
    from ament_index_python.packages import get_package_share_directory
    from pinky_mujoco.description import expand_urdf
    from pinky_mujoco.model import build_model

    urdf = expand_urdf(package='pinky_soccer_description',
                       filename='urdf/soccer_robot.urdf.xacro', mappings=bumper)
    world = f"{get_package_share_directory('pinky_gz_sim')}/worlds/empty.world"
    scene = build_model(get_package_share_directory('pinky_mujoco'), tmp_path, urdf, world=world,
                        simple_render=True)
    return add_ball(scene)


def pitch_deg(quat):
    w, x, y, z = quat
    return math.degrees(math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))))


def test_physics_robot_pushes_ball_and_stays_level(tmp_path):
    scene = build_soccer_scene(tmp_path)
    from pinky_mujoco.model import Physics
    physics = Physics(scene)
    model, data = physics.model, physics.data
    assert model.body('bumper_link') is not None
    ball_q = model.joint('soccer_ball_free').qposadr[0]
    assert model.joint('floating_base').qposadr[0] == 0, 'robot must stay first in qpos'

    physics.step(0.0, 0.0, steps=250)  # settle
    start_ball = data.qpos[ball_q:ball_q + 3].copy()
    max_pitch = 0.0
    for _ in range(300):  # 3 s at bridge rate, max_linear 0.25 m/s
        physics.step(0.25, 0.0)
        max_pitch = max(max_pitch, abs(pitch_deg(data.qpos[3:7])))
    for _ in range(100):  # hard stop
        physics.step(0.0, 0.0)
        max_pitch = max(max_pitch, abs(pitch_deg(data.qpos[3:7])))
    ball = data.qpos[ball_q:ball_q + 3]
    robot_x = data.qpos[0]
    assert ball[0] - start_ball[0] > 0.2, 'ball was not pushed forward'
    assert ball[0] > robot_x, 'ball must stay in front of the robot'
    assert 0.0 < ball[2] < 0.03, 'ball must stay on the floor (not over the bumper)'
    assert max_pitch < 3.0
