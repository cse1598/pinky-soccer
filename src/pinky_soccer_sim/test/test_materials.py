"""Physical outcomes and collision filtering of the shared material implementation."""

import math
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import numpy as np
from pinky_soccer_sim.materials import (
    configure_mujoco, ContactKernel, install_mujoco, load_materials, parameters)
import pytest


def test_no_force_in_air_and_rolling_opposes_motion():
    cfg, kernel = load_materials(), ContactKernel()
    params = parameters(cfg, 'ball_floor')
    assert not kernel.force([0, 0, .1], [1, 0, 0], [0, 40, 0], params, .02135, .0459, .001).any()
    force = kernel.force([0, 0, .021], [1, 0, 0], [0, 1/.02135, 0],
                         params, .02135, .0459, .001)
    assert force[2] > 0
    assert force[4] < 0
    assert abs(force[0]) < 1e-10  # pure rolling has no tangential slip


def test_corner_box_force_uses_outward_normal():
    cfg, kernel = load_materials(), ContactKernel()
    f = kernel.box_force([1.015, 0, 0], [-1, 0, 0], [0, 0, 0],
                         parameters(cfg, 'ball_wall'), .02135, .0459, .001,
                         [0, 0, 0], [1, 1, 1], np.eye(3))
    assert f[0] > 0
    assert np.allclose(f[1:], 0)


def test_sphere_box_and_floor_have_no_double_native_contact(tmp_path):
    p = tmp_path/'scene.xml'
    p.write_text("""<mujoco><option/><worldbody>
      <geom name="floor" type="plane"/>
      <geom name="wall" type="box"/>
      <body name="l_wheel"><geom name="wheel"/></body>
      <body name="soccer_ball"><geom name="ball"/></body>
      <body name="base_link"><geom name="body"/></body>
      </worldbody></mujoco>""")
    root = ET.parse(configure_mujoco(p, load_materials())).getroot()
    geoms = {e.get('name'): (int(e.get('contype')), int(e.get('conaffinity')))
             for e in root.findall('.//geom')}

    def collides(a, b):
        a, b = geoms[a], geoms[b]
        return bool(a[0] & b[1] or b[0] & a[1])

    assert not collides('ball', 'floor')
    assert not collides('wheel', 'floor')
    assert not collides('ball', 'wall')
    assert collides('body', 'floor')  # tipped chassis still has native floor contact
    assert collides('wheel', 'wall')
    assert collides('ball', 'body') and collides('ball', 'wheel')


def test_real_robot_stands_and_drives_with_shared_foam_contacts(tmp_path):
    from ament_index_python.packages import get_package_share_directory
    from pinky_mujoco.description import expand_urdf
    from pinky_mujoco.model import build_model, Physics
    from pinky_soccer_sim.mujoco_soccer_bridge import add_ball

    cfg = load_materials()
    urdf = expand_urdf(package='pinky_soccer_description',
                       filename='urdf/soccer_robot.urdf.xacro')
    world = get_package_share_directory('pinky_gz_sim')+'/worlds/empty.world'
    path = add_ball(build_model(
        get_package_share_directory('pinky_mujoco'), tmp_path,
        urdf, world=world, simple_render=True), x=2)
    bridge = SimpleNamespace(build_model=lambda: path, Physics=Physics)
    install_mujoco(bridge, cfg)
    physics = bridge.Physics(bridge.build_model())
    physics.step(steps=500)
    height = physics.data.qpos[2]
    assert .02 < height < .04
    x = physics.data.qpos[0]
    for _ in range(200):
        physics.step(.1, 0)
    assert .10 < physics.data.qpos[0]-x < .30
    assert .02 < physics.data.qpos[2] < .04
    assert np.isfinite(physics.data.qpos).all()
    assert np.allclose(physics.data.xfrc_applied, 0), 'external forces must not accumulate'
    # Exercise the still-native bumper/ball collision with the new foam support.
    ball = physics.model.joint('soccer_ball_free')
    q, v = ball.qposadr[0], ball.dofadr[0]
    start = physics.data.qpos[0]+.15
    physics.data.qpos[q:q+3] = [start, 0, cfg['ball']['diameter_m']/2]
    physics.data.qvel[v:v+6] = 0
    for _ in range(150):
        physics.step(.2, 0)
    assert physics.data.qpos[q] > start+.08
    assert .015 < physics.data.qpos[q+2] < .04


@pytest.mark.parametrize('scenario', ['drop', 'roll'])
def test_mujoco_foam_drop_and_roll_outcomes(scenario):
    import mujoco

    cfg, kernel = load_materials(), ContactKernel()
    radius, mass = cfg['ball']['diameter_m']/2, cfg['ball']['mass_kg']
    inertia = .4*mass*radius**2
    height = radius+.2 if scenario == 'drop' else radius-.0003
    model = mujoco.MjModel.from_xml_string(f"""<mujoco>
      <option timestep=".001" integrator="implicitfast"/>
      <worldbody><body pos="0 0 {height}"><freejoint/>
      <inertial pos="0 0 0" mass="{mass}" diaginertia="{inertia} {inertia} {inertia}"/>
      <geom type="sphere" size="{radius}" contype="0" conaffinity="0"/>
      </body></worldbody></mujoco>""")
    data = mujoco.MjData(model)
    if scenario == 'roll':
        data.qvel[0], data.qvel[4] = 1, 1/radius
    heights = []
    touched = False
    for _ in range(4000):
        mujoco.mj_forward(model, data)
        data.xfrc_applied[1] = kernel.force(
            data.qpos[:3], data.qvel[:3], data.qvel[3:], parameters(cfg, 'ball_floor', mass),
            radius, mass, .001)
        mujoco.mj_step(model, data)
        touched |= data.qpos[2] < radius
        if touched:
            heights.append(data.qpos[2])
    if scenario == 'drop':
        cor = math.sqrt(max(0, max(heights)-radius)/.2)
        assert abs(cor-cfg['contacts']['ball_floor']['restitution']) < .06
    else:
        expected = 1/(2*cfg['contacts']['ball_floor']['rolling_deceleration_m_s2'])
        assert data.qpos[0] == pytest.approx(expected, abs=.02)
        assert abs(data.qvel[0]) < .01
