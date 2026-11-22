#!/usr/bin/env python3
"""Compare isolated Gazebo/MuJoCo drop, rolling and wall-impact experiments."""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET

import numpy as np
from pinky_soccer_sim import materials


def initial(cfg, scenario):
    radius, mass = cfg['ball']['diameter_m']/2, cfg['ball']['mass_kg']
    height = radius+.2 if scenario == 'drop' else (1. if scenario == 'wall' else radius-.0003)
    velocity = [0., 0., 0.] if scenario == 'drop' else [1., 0., 0.]
    angular = [0., 1/radius, 0.] if scenario == 'roll' else [0., 0., 0.]
    return radius, mass, height, velocity, angular


def measure_mujoco(cfg, scenario):
    import mujoco

    radius, mass, height, velocity, angular = initial(cfg, scenario)
    inertia = .4*mass*radius**2
    gravity = 0 if scenario == 'wall' else -cfg['gravity']
    model = mujoco.MjModel.from_xml_string(f"""<mujoco>
      <option timestep="{cfg['timestep']}" integrator="implicitfast" gravity="0 0 {gravity}"/>
      <worldbody><body pos="0 0 {height}"><freejoint/>
      <inertial pos="0 0 0" mass="{mass}" diaginertia="{inertia} {inertia} {inertia}"/>
      <geom type="sphere" size="{radius}" contype="0" conaffinity="0"/>
      </body></worldbody></mujoco>""")
    data = mujoco.MjData(model)
    data.qvel[:3], data.qvel[3:] = velocity, angular
    kernel, rows = materials.ContactKernel(), []
    for _ in range(round(4/cfg['timestep'])):
        mujoco.mj_forward(model, data)
        force = kernel.force(data.qpos[:3], data.qvel[:3], data.qvel[3:],
                             materials.parameters(cfg, 'ball_floor', mass),
                             radius, mass, cfg['timestep'])
        if scenario == 'wall':
            force += kernel.box_force(
                data.qpos[:3], data.qvel[:3], data.qvel[3:],
                materials.parameters(cfg, 'ball_wall', mass), radius, mass, cfg['timestep'],
                [.5, 0, 1], [.01, 1, 1], np.eye(3))
        data.xfrc_applied[1] = force
        mujoco.mj_step(model, data)
        rows.append([data.time, *data.qpos[:3]])
    return rows


def measure_gazebo(cfg, scenario, directory):
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.transport13 import Node

    radius, mass, height, velocity, angular = initial(cfg, scenario)
    base = directory/'base.sdf'
    base.write_text("""<sdf version="1.9"><world name="material_test">
      <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
      <model name="ground"><static>true</static><link name="floor">
      <collision name="floor"><geometry><plane><normal>0 0 1</normal>
      <size>20 20</size></plane></geometry></collision></link></model>
      </world></sdf>""")
    if scenario == 'wall':
        tree = ET.parse(base)
        world = tree.getroot().find('world')
        wall = ET.SubElement(world, 'model', name='wall')
        materials.text(wall, 'static', 'true')
        materials.text(wall, 'pose', '.5 0 1 0 0 0')
        link = ET.SubElement(wall, 'link', name='wall')
        collision = ET.SubElement(link, 'collision', name='wall')
        materials.text(collision, 'geometry/box/size', '.02 2 2')
        tree.write(base)
    path = materials.gazebo_world(base, directory/'world.sdf', cfg, 'unused', [], (radius, mass))
    tree = ET.parse(path)
    world = tree.getroot().find('world')
    if scenario == 'wall':
        materials.text(world, 'gravity', '0 0 0')
    contact = world.find("plugin[@name='pinky_soccer_sim::FoamContact']/contact")
    materials.text(contact, 'initial_velocity', ' '.join(str(v) for v in velocity))
    materials.text(contact, 'initial_angular', ' '.join(str(v) for v in angular))
    inertia = .4*mass*radius**2
    model = ET.fromstring(f"""<model name="soccer_ball"><pose>0 0 {height} 0 0 0</pose>
      <link name="ball"><inertial><mass>{mass}</mass><inertia>
      <ixx>{inertia}</ixx><iyy>{inertia}</iyy><izz>{inertia}</izz></inertia></inertial>
      <collision name="ball"><geometry><sphere><radius>{radius}</radius></sphere></geometry>
      </collision></link><plugin filename="gz-sim-pose-publisher-system"
      name="gz::sim::systems::PosePublisher"><publish_model_pose>true</publish_model_pose>
      <publish_link_pose>false</publish_link_pose><use_pose_vector_msg>true</use_pose_vector_msg>
      <update_frequency>{1/cfg['timestep']}</update_frequency></plugin></model>""")
    materials.sdf_surface(model.find('.//collision'), 4, .35, .6)
    world.append(model)
    tree.write(path)
    rows, node = [], Node()
    topic = '/model/soccer_ball/pose'

    def callback(message):
        for pose in message.pose:
            if pose.name == 'soccer_ball':
                stamp = pose.header.stamp.sec+pose.header.stamp.nsec*1e-9
                rows.append([stamp, pose.position.x, pose.position.y, pose.position.z])

    node.subscribe(Pose_V, topic, callback)
    with (directory/'gazebo.log').open('w') as log:
        process = subprocess.Popen(['gz', 'sim', '-r', '-s', str(path)],
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            started = time.monotonic()
            while not rows or rows[-1][0] < 4:
                if process.poll() is not None or time.monotonic()-started > 20:
                    raise RuntimeError(f'Gazebo probe failed; see {directory}/gazebo.log')
                time.sleep(.02)
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            node.unsubscribe(topic)
    return rows


def metrics(cfg, scenario, rows):
    data = np.array(rows)
    radius = cfg['ball']['diameter_m']/2
    if not np.isfinite(data).all() or len(rows) < 100:
        raise ValueError('Insufficient or invalid physics observations')
    if scenario == 'drop':
        impact = np.flatnonzero(data[:, 3] < radius)
        if not len(impact):
            raise ValueError('Ball never reached the foam surface')
        rebound = max(0., data[impact[0]+1:, 3].max()-radius)
        return {'rebound_height_m': float(rebound), 'cor': float(np.sqrt(rebound/.2))}
    if scenario == 'roll':
        return {'stop_distance_m': float(data[-1, 1]-data[0, 1])}
    recent = data[data[:, 0] > 2]
    velocity = (recent[-1, 1]-recent[0, 1])/(recent[-1, 0]-recent[0, 0])
    return {'outgoing_velocity_m_s': float(velocity)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--material-config')
    parser.add_argument('--output')
    args = parser.parse_args()
    cfg = materials.load_materials(args.material_config)
    output = (Path(args.output) if args.output else
              Path(tempfile.mkdtemp(prefix='pinky-materials-')))
    if args.output:
        output.mkdir(parents=True, exist_ok=False)
    os.environ['GZ_PARTITION'] = f'pinky-material-check-{os.getpid()}'
    report = {'preset': cfg, 'experiments': {}, 'passed': True}
    for scenario in ('drop', 'roll', 'wall'):
        directory = output/scenario
        directory.mkdir()
        values = {}
        for engine, function in [('mujoco', measure_mujoco), ('gazebo', measure_gazebo)]:
            rows = (function(cfg, scenario, directory) if engine == 'gazebo'
                    else function(cfg, scenario))
            np.savetxt(directory/(engine+'.csv'), rows, delimiter=',',
                       header='simulation_time_s,x_m,y_m,z_m', comments='')
            values[engine] = metrics(cfg, scenario, rows)
        key, tolerance = {'drop': ('cor', .03), 'roll': ('stop_distance_m', .03),
                          'wall': ('outgoing_velocity_m_s', .06)}[scenario]
        difference = abs(values['mujoco'][key]-values['gazebo'][key])
        if scenario == 'drop':
            target, target_tolerance = cfg['contacts']['ball_floor']['restitution'], .06
        elif scenario == 'roll':
            deceleration = cfg['contacts']['ball_floor']['rolling_deceleration_m_s2']
            target, target_tolerance = (1/(2*deceleration) if deceleration else 4), .05
        else:
            target, target_tolerance = -cfg['contacts']['ball_wall']['restitution'], .10
        target_pass = all(abs(values[e][key]-target) <= target_tolerance
                          for e in ('mujoco', 'gazebo'))
        values.update(difference=difference, tolerance=tolerance, target=target,
                      target_tolerance=target_tolerance,
                      passed=difference <= tolerance and target_pass)
        report['passed'] &= values['passed']
        report['experiments'][scenario] = values
        print(scenario, json.dumps(values), flush=True)
    (output/'report.json').write_text(json.dumps(report, indent=2))
    print('saved', output)
    if not report['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
