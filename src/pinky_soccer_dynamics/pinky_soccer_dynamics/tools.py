"""
Command-line tools of pinky_soccer_dynamics.

  show_params        print the merged parameters with their sources
  check_wheel_signs  passive /joint_states sign check while a person pushes the robot
                     (publishes nothing; never drives the robot)
  identify_sim       wheel step responses in headless MuJoCo -> identification file for the
                     `mujoco` target (validates the identification pipeline; not real data)
"""

import argparse
import json
import math
from pathlib import Path
import sys
import time

import numpy as np
import yaml

from . import params as params_mod


def _common(ap):
    ap.add_argument('--target', default='real', choices=['real', 'mujoco', 'gazebo'])
    ap.add_argument('--config', help='dynamics.yaml (default: installed)')
    ap.add_argument('--profile', help='pinky_soccer_calibration profile.yaml')
    ap.add_argument('--identification', help='identification.yaml from identify tools')


def show_params(argv=None):
    """Print parameters and provenance."""
    ap = argparse.ArgumentParser(description='Show Pinky dynamics parameters and sources')
    _common(ap)
    a = ap.parse_args(argv)
    p = params_mod.load(a.target, a.config, a.profile, a.identification)
    yaml.safe_dump(p.as_dict(), sys.stdout, sort_keys=False, allow_unicode=True)
    if p.unmeasured():
        print(f'# NOT MEASURED for {a.target}: {", ".join(p.unmeasured())}', file=sys.stderr)
    return 0


# --------------------------------------------------------------------------- sign check
def sign_verdict(joint_delta_l, joint_delta_r, odom_distance, odom_yaw, min_distance=0.05):
    """
    Return expected signs from a hand push.

    A forward push (odom distance > min_distance, little yaw) must turn both wheels in the
    model's forward direction, so the sign of each raw joint position change is the
    joint_sign for that wheel.
    """
    if odom_distance < min_distance:
        raise ValueError(f'push the robot forward at least {min_distance} m (got '
                         f'{odom_distance:.3f})')
    if abs(odom_yaw) > 0.3:
        raise ValueError('push straight forward (yaw change too large)')
    return {'left': 1 if joint_delta_l > 0 else -1, 'right': 1 if joint_delta_r > 0 else -1}


def check_wheel_signs(argv=None):
    """Record /joint_states and /odom while the robot is pushed forward by hand."""
    ap = argparse.ArgumentParser(description=check_wheel_signs.__doc__)
    ap.add_argument('--seconds', type=float, default=10.0)
    _common(ap)
    a = ap.parse_args(argv)
    import rclpy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import JointState
    rclpy.init()
    node = rclpy.create_node('check_wheel_signs')
    first, last = {}, {}

    def on_js(m):
        pos = dict(zip(m.name, m.position))
        if 'l_wheel_joint' in pos and 'r_wheel_joint' in pos:
            first.setdefault('js', (pos['l_wheel_joint'], pos['r_wheel_joint']))
            last['js'] = (pos['l_wheel_joint'], pos['r_wheel_joint'])

    def on_odom(m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        sample = (m.pose.pose.position.x, m.pose.pose.position.y, yaw)
        first.setdefault('odom', sample)
        last['odom'] = sample
    node.create_subscription(JointState, '/joint_states', on_js, 10)
    node.create_subscription(Odometry, '/odom', on_odom, 10)
    print(f'Push the robot straight FORWARD by hand about 20 cm within {a.seconds:.0f} s '
          '(motors may be torqued; push gently or lift-free if the driver holds).')
    t0 = time.monotonic()
    while time.monotonic() - t0 < a.seconds:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    rclpy.shutdown()
    if 'js' not in last or 'odom' not in last:
        print('No /joint_states or /odom received.')
        return 2
    (l0, r0), (l1, r1) = first['js'], last['js']
    (x0, y0, a0), (x1, y1, a1) = first['odom'], last['odom']
    heading = a0
    dist = (x1 - x0) * math.cos(heading) + (y1 - y0) * math.sin(heading)
    yaw = math.atan2(math.sin(a1 - a0), math.cos(a1 - a0))
    verdict = sign_verdict(l1 - l0, r1 - r0, dist, yaw)
    expected = params_mod.load(a.target, a.config, a.profile, a.identification).joint_sign
    report = {'joint_delta_rad': [l1 - l0, r1 - r0], 'odom_forward_m': dist, 'odom_yaw_rad': yaw,
              'measured_sign': verdict,
              'configured_sign': {k: int(v.value) for k, v in expected.items()}}
    report['matches_config'] = report['measured_sign'] == report['configured_sign']
    print(json.dumps(report, indent=1))
    return 0 if report['matches_config'] else 1


# --------------------------------------------------------------------------- MuJoCo
def _mujoco_physics(cache, params, profile=None):
    """Official pinky_mujoco Physics with the soccer URDF, field and shared materials."""
    import types
    from ament_index_python.packages import get_package_share_directory as share
    from pinky_mujoco import model as pm
    from pinky_mujoco.description import expand_urdf
    from pinky_soccer_sim import materials, rcj_field
    from pinky_soccer_bringup.calibration_profile import configure_urdf
    from pinky_soccer_sim.mujoco_soccer_bridge import (
        install_drive_constants, install_real_driver)
    out = Path(cache)
    out.mkdir(parents=True, exist_ok=True)
    urdf = expand_urdf(package='pinky_soccer_description',
                       filename='urdf/soccer_robot.urdf.xacro',
                       mappings={'cam_tilt_deg': '8', 'screen_tilt_deg': '-25',
                                 'bumper_enabled': 'true'})
    if profile:
        urdf = configure_urdf(urdf, profile, True)
    sim_share = Path(share('pinky_soccer_sim'))
    mat = materials.load_materials(str(sim_share / 'config/materials.yaml'))
    phys = yaml.safe_load((Path(share('pinky_mujoco')) / 'config/mujoco.yaml').read_text())
    phys['physics']['timestep'] = mat['timestep']
    phys['camera']['enabled'] = False
    # Match the soccer launch: no independent v/w clamp; scale wheels at 100 RPM.
    phys['control']['max_linear'] = 10.0
    phys['control']['max_angular'] = 100.0
    path = out / 'physics.yaml'
    path.write_text(yaml.safe_dump(phys))
    ns = types.SimpleNamespace(build_model=pm.build_model, Physics=pm.Physics)
    rcj_field.install_mujoco(ns)
    materials.install_mujoco(ns, mat)
    install_drive_constants(ns, params.d('wheel_radius'), params.d('wheel_separation'))
    install_real_driver(ns, params.d('max_rpm'))
    model = ns.build_model(share('pinky_mujoco'), str(out), urdf, str(path),
                           str(sim_share / 'worlds/hardboard_soccer.world'), simple_render=True)
    return ns.Physics(model, str(path))


def mujoco_step_log(physics, adapter, wheel_targets, hold=0.6, dt=None):
    """Sample state at t, then apply the corresponding command for one bridge period."""
    dt = physics.bridge_period if dt is None else dt
    if not math.isclose(dt, physics.bridge_period, rel_tol=1e-9):
        raise ValueError('dt must match the physics bridge period')
    t, us, ws = [], [], []
    k = 0
    for target in wheel_targets:
        v, w_cmd = adapter.to_twist(*target)
        left, right, _ = adapter.driver_applied(v, w_cmd)
        for _ in range(int(round(hold / dt))):
            qv = physics.data.qvel[physics.wheel_v]
            t.append(k * dt)
            us.append((left, right))
            ws.append((float(qv[0]), float(qv[1])))
            physics.step(v, w_cmd)
            k += 1
    return np.array(t), np.array(us), np.array(ws)


def identify_sim(argv=None):
    """Identify the MuJoCo wheel response (simulation only)."""
    ap = argparse.ArgumentParser(description=identify_sim.__doc__)
    ap.add_argument('--out', default='identification_mujoco.yaml')
    ap.add_argument('--cache', default=str(Path.home() / '.cache' / 'pinky_soccer_dynamics'))
    ap.add_argument('--profile', default='auto', help='auto, none, or calibration profile path')
    ap.add_argument('--zero-delay', action='store_true',
                    help='fit a zero-delay model compatible with the current NMPC')
    a = ap.parse_args(argv)
    from ament_index_python.packages import get_package_share_directory as share
    from .command_adapter import CommandAdapter
    from .identification import (fit_response, identification_document, wheel_validation,
                                 write_identification)
    profile = a.profile
    if profile == 'auto':
        profile = str(Path(share('pinky_soccer_sim')) / 'config/calibration/mujoco_profile.yaml')
    elif profile == 'none':
        profile = None
    params = params_mod.load('mujoco', profile=profile)
    adapter = CommandAdapter(params)
    physics = _mujoco_physics(a.cache, params, profile)
    train = [(0, 0), (6, 6), (0, 0), (-6, -6), (0, 0), (4, -4), (0, 0), (8, 3), (0, 0)]
    test = [(0, 0), (5, 5), (-3, 3), (0, 0), (7, 7), (2, -2), (0, 0)]
    t, u, w = mujoco_step_log(physics, adapter, train)
    accel = (None, params.m('wheel_accel_max'))
    fit = fit_response(t, u, w, accel_limits=accel,
                       delays=[0.0] if a.zero_delay else None)
    t2, u2, w2 = mujoco_step_log(physics, adapter, test)
    val = wheel_validation(fit, t2, u2, w2)
    doc = identification_document('mujoco', 'mujoco-sim', fit, val, {'rms_relative': 0.1},
                                  notes=f'headless soccer MuJoCo; profile={profile}; zero_delay={a.zero_delay}')
    doc['calibration_profile'] = profile
    write_identification(a.out, doc)
    print(json.dumps({'fit': fit, 'validation': val, 'accepted': doc['accepted'],
                      'file': a.out}, indent=1))
    return 0 if doc['accepted'] else 1
