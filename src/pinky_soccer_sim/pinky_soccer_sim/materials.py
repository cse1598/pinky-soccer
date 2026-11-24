"""Common material preset, contact kernel and engine adapters; no official edits."""

import ctypes
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix, get_package_share_directory
import numpy as np
from scipy.spatial.transform import Rotation
import yaml


def load_materials(path=None):
    path = Path(path) if path else Path(get_package_share_directory('pinky_soccer_sim')) / \
        'config/materials.yaml'
    cfg = yaml.safe_load(path.read_text())
    if cfg.get('schema_version') != 1:
        raise ValueError('Unsupported material schema')
    for key in ('gravity', 'timestep'):
        if not math.isfinite(cfg[key]) or cfg[key] <= 0:
            raise ValueError(f'Invalid {key}')
    for pair in cfg['contacts'].values():
        if any(not math.isfinite(v) or v < 0 for v in pair.values()):
            raise ValueError('Contact parameters must be finite and nonnegative')
        if pair['restitution'] >= 1 or min(pair['stiffness_n_m'], pair['effective_mass_kg']) <= 0:
            raise ValueError('Invalid contact stiffness/mass/restitution')
        if cfg['timestep']*math.sqrt(pair['stiffness_n_m']/pair['effective_mass_kg']) > .8:
            raise ValueError('Material spring requires a smaller physics timestep')
    return cfg


def parameters(cfg, name, mass=None):
    c = cfg['contacts'][name]
    return np.array([c['sliding_friction'], c['restitution'], c['stiffness_n_m'],
                     mass or c['effective_mass_kg'], c['rolling_deceleration_m_s2'],
                     cfg['gravity']], dtype=np.float64)


class ContactKernel:
    """Call the very same C++ force calculation loaded by the Gazebo system."""

    def __init__(self):
        library = Path(get_package_prefix('pinky_soccer_sim')) / 'lib/libpinky_contact_kernel.so'
        self.library = ctypes.CDLL(str(library))
        self.function = self.library.pinky_contact
        pointer = np.ctypeslib.ndpointer(dtype=np.float64, flags='C_CONTIGUOUS')
        self.function.argtypes = [pointer]*4 + [ctypes.c_double]*3 + [pointer]
        self.function.restype = None
        self.box_function = self.library.pinky_box_contact
        self.box_function.argtypes = [pointer]*4 + [ctypes.c_double]*3 + [pointer]*4
        self.box_function.restype = None

    def force(self, position, velocity, angular, params, radius, mass, dt):
        result = np.zeros(6)
        self.function(*(np.ascontiguousarray(a, dtype=np.float64)
                        for a in (position, velocity, angular, params)), radius, mass, dt, result)
        return result

    def box_force(self, position, velocity, angular, params, radius, mass, dt,
                  center, half, rotation):
        result = np.zeros(6)
        args = [np.ascontiguousarray(a, dtype=np.float64)
                for a in (position, velocity, angular, params)]
        geometry = [np.ascontiguousarray(a, dtype=np.float64)
                    for a in (center, half, np.asarray(rotation).reshape(-1))]
        self.box_function(*args, radius, mass, dt, *geometry, result)
        return result


def text(parent, path, value):
    node = parent
    for tag in path.split('/'):
        child = node.find(tag)
        node = child if child is not None else ET.SubElement(node, tag)
    node.text = str(value)


def sdf_surface(collision, mask, mu, restitution=0):
    text(collision, 'surface/contact/collide_bitmask', mask)
    for tag in ('mu', 'mu2'):
        text(collision, 'surface/friction/ode/'+tag, mu)
    text(collision, 'surface/bounce/restitution_coefficient', restitution)


def contact_entry(plugin, model, link, offset, radius, mass, params):
    entry = ET.SubElement(plugin, 'contact')
    for name, value in [('model', model), ('link', link),
                        ('offset', ' '.join(str(x) for x in offset)),
                        ('radius', radius), ('mass', mass),
                        *zip(('mu', 'restitution', 'stiffness', 'effective_mass',
                              'rolling_decel', 'gravity'), params)]:
        text(entry, name, value)


def sphere_contacts(urdf, cfg):
    """Read actual wheel collision centres/radii, retaining calibrated geometry."""
    root = ET.fromstring(urdf)
    contacts = []
    for name in ('l_wheel', 'r_wheel', 'caster_wheel'):
        collision = root.find(f"link[@name='{name}']/collision")
        sphere = collision.find('geometry/sphere') if collision is not None else None
        if sphere is None:
            raise ValueError(f'{name}: foam contact requires the current spherical collision')
        origin = collision.find('origin')
        offset = [float(v) for v in (origin.get('xyz', '0 0 0')
                                     if origin is not None else '0 0 0').split()]
        pair = 'caster_floor' if name == 'caster_wheel' else 'wheel_floor'
        mass = float(root.find(f"link[@name='{name}']/inertial/mass").get('value'))
        contacts.append((name, offset, float(sphere.get('radius')), parameters(cfg, pair), mass))
    return contacts


def configure_gazebo_robot(sdf, urdf, cfg):
    root = ET.fromstring(sdf)
    # floor mask=1; all native robot/wall/ball collisions share bit 2.
    # Wheel/floor pairs alone are replaced by the common compliant contact law.
    for link in root.findall('.//model/link'):
        is_wheel = link.get('name') in ('l_wheel', 'r_wheel', 'caster_wheel')
        for collision in link.findall('collision'):
            sdf_surface(collision, 6 if is_wheel else 7,
                        cfg['contacts']['wheel_floor']['sliding_friction'])
    return ET.tostring(root, encoding='unicode')


def configure_gazebo_ball(sdf, cfg):
    root = ET.fromstring(sdf)
    c = cfg['contacts']['ball_wall']
    for collision in root.findall('.//collision'):
        sdf_surface(collision, 4, c['sliding_friction'], c['restitution'])
    return ET.tostring(root, encoding='unicode')


def gazebo_world(path, output, cfg, robot_name, contacts, ball=None):
    """Copy the selected world, preserving source assets and geometric dimensions."""
    path, output = Path(path).resolve(), Path(output)
    tree = ET.parse(path)
    world = tree.getroot().find('world')
    text(world, 'gravity', f"0 0 {-cfg['gravity']}")
    # Official flat ground model includes are replaced locally, not edited on disk.
    for include in list(world.findall('include')):
        if 'ground_plane' in include.findtext('uri', ''):
            world.remove(include)
            model = ET.SubElement(world, 'model', name='material_ground_plane')
            text(model, 'static', 'true')
            link = ET.SubElement(model, 'link', name='floor')
            for kind in ('collision', 'visual'):
                shape = ET.SubElement(link, kind, name='floor')
                text(shape, 'geometry/plane/normal', '0 0 1')
                text(shape, 'geometry/plane/size', '100 100')
    for collision in world.findall('.//collision'):
        name = collision.get('name', '').lower()
        floor = ('carpet' in name or collision.find('geometry/plane') is not None
                 or name in ('floor', 'ground'))
        mesh = collision.find('geometry/mesh') is not None
        mu = cfg['contacts']['wheel_floor' if floor else 'ball_wall']['sliding_friction']
        sdf_surface(collision, 1 if floor else (6 if mesh else 2), mu,
                    0 if floor else cfg['contacts']['ball_wall']['restitution'])
    # Relocation must not break relative field mesh URIs.
    for uri in world.findall('.//uri'):
        value = uri.text or ''
        if value and '://' not in value:
            uri.text = str((path.parent/value).resolve())
    text(world, 'physics/max_step_size', cfg['timestep'])
    text(world, 'physics/real_time_factor', 1)
    world.find('physics').set('name', 'shared_material_physics')
    world.find('physics').set('type', 'ignored')
    library = Path(get_package_prefix('pinky_soccer_sim')) / 'lib/libpinky_foam_contact.so'
    plugin = ET.SubElement(world, 'plugin', filename=str(library),
                           name='pinky_soccer_sim::FoamContact')
    names = ('mu', 'restitution', 'stiffness', 'effective_mass', 'rolling_decel', 'gravity')
    for name, value in zip(names, parameters(cfg, 'ball_wall')):
        text(plugin, 'wall_parameters/'+name, value)
    for pose, size in static_boxes(world):
        box = ET.SubElement(plugin, 'box')
        text(box, 'pose', ' '.join(str(v) for v in pose))
        text(box, 'size', ' '.join(str(v) for v in size))
    # One robot name or several (two-robot simulation): same wheel contacts for each model.
    for model in ([robot_name] if isinstance(robot_name, str) else robot_name):
        for name, offset, radius, params, mass in contacts:
            contact_entry(plugin, model, name, offset, radius, mass, params)
    if ball:
        radius, mass = ball
        contact_entry(plugin, 'soccer_ball', 'ball', [0, 0, 0], radius, mass,
                      parameters(cfg, 'ball_floor', mass))
    ET.indent(tree)
    tree.write(output, encoding='unicode')
    return output


def static_boxes(world):
    """Resolve nested static box poses; meshes retain native engine contact."""
    result = []

    def visit(element, parent):
        transform = parent.copy()
        if element.tag in ('model', 'link', 'collision'):
            values = [float(v) for v in element.findtext('pose', '0 0 0 0 0 0').split()]
            local = np.eye(4)
            local[:3, :3] = Rotation.from_euler('xyz', values[3:]).as_matrix()
            local[:3, 3] = values[:3]
            transform = parent@local
        if element.tag == 'model' and element.findtext('static', 'true') == 'false':
            return
        if element.tag == 'collision':
            box = element.find('geometry/box/size')
            if box is not None and 'carpet' not in element.get('name', ''):
                angles = Rotation.from_matrix(transform[:3, :3]).as_euler('xyz')
                pose = [*transform[:3, 3], *angles]
                result.append((pose, [float(v) for v in box.text.split()]))
        for child in element:
            if child.tag in ('model', 'link', 'collision'):
                visit(child, transform)

    visit(world, np.eye(4))
    return result


def configure_mujoco(path, cfg):
    """Preserve object collisions; replace only wheel/ball-to-planar-floor contacts."""
    tree = ET.parse(path)
    root = tree.getroot()
    option = root.find('option')
    option.set('timestep', str(cfg['timestep']))
    option.set('gravity', f"0 0 {-cfg['gravity']}")
    world = root.find('worldbody')
    for geom in world.findall('geom'):
        if geom.get('contype', '1') == '0':
            continue
        floor = geom.get('name', '') == 'floor' or 'carpet' in geom.get('name', '')
        geom.set('contype', '1' if floor else '2')
        geom.set('conaffinity', '4' if floor else
                 ('28' if geom.get('type') == 'mesh' else '20'))
        # MuJoCo otherwise takes max(mu), while DART's native pairs use the
        # lower surface value. Priority makes these remaining pairs agree.
        geom.set('priority', '1')
        mu = cfg['contacts']['wheel_floor' if floor else 'ball_wall']['sliding_friction']
        geom.set('friction', f'{mu} 0 0')
    for body in world.iter('body'):
        name = body.get('name', '')
        ball = name == 'soccer_ball'
        wheel = name in ('l_wheel', 'r_wheel', 'caster_wheel')
        for geom in body.findall('geom'):
            if geom.get('contype', '1') == '0':
                continue
            geom.set('contype', '8' if ball else ('16' if wheel else '4'))
            geom.set('conaffinity', '20' if ball else ('10' if wheel else '11'))
            mu = cfg['contacts']['ball_wall' if ball else 'wheel_floor']['sliding_friction']
            geom.set('friction', f'{mu} 0 0')
            geom.set('priority', '2' if ball else '0')
    tree.write(path, encoding='unicode')
    return path


def install_mujoco(bridge, cfg):
    """Wrap the official runtime without global MuJoCo callbacks or submodule edits."""
    original_build, original_physics = bridge.build_model, bridge.Physics

    def build(*args, **kwargs):
        return configure_mujoco(original_build(*args, **kwargs), cfg)

    class MaterialPhysics(original_physics):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            self.contact_kernel = ContactKernel()
            self.material_contacts = []
            self.material_boxes = [i for i in range(self.model.ngeom)
                                   if self.model.geom_bodyid[i] == 0
                                   and self.model.geom_type[i] == self.mj.mjtGeom.mjGEOM_BOX
                                   and self.model.geom_contype[i] == 2]
            for name, pair in [('l_wheel', 'wheel_floor'), ('r_wheel', 'wheel_floor'),
                               ('caster_wheel', 'caster_floor'), ('soccer_ball', 'ball_floor')]:
                try:
                    body = self.model.body(name)
                    geom = self.model.geom('soccer_ball_geom' if name == 'soccer_ball'
                                           else name+'_collision_0')
                except KeyError:
                    continue
                mass = self.model.body_mass[body.id] if name == 'soccer_ball' else None
                params = parameters(cfg, pair, mass)
                self.material_contacts.append((body.id, geom.id, params))
            self.jp = np.zeros((3, self.model.nv))
            self.jr = np.zeros((3, self.model.nv))

        def step(self, linear=0., angular=0., steps=None):
            count = steps or round(self.bridge_period/self.model.opt.timestep)
            for _ in range(count):
                # mj_step integrates qpos last; refresh geometry/Jacobians before
                # evaluating an explicit contact at the new state.
                self.mj.mj_forward(self.model, self.data)
                forces = []
                for body, geom, params in self.material_contacts:
                    p = self.data.geom_xpos[geom].copy()
                    self.mj.mj_jac(self.model, self.data, self.jp, self.jr, p, body)
                    f = self.contact_kernel.force(
                        p, self.jp@self.data.qvel, self.jr@self.data.qvel, params,
                        self.model.geom_size[geom, 0], self.model.body_mass[body],
                        self.model.opt.timestep)
                    if self.model.body(body).name == 'soccer_ball':
                        for box in self.material_boxes:
                            f += self.contact_kernel.box_force(
                                p, self.jp@self.data.qvel, self.jr@self.data.qvel,
                                parameters(cfg, 'ball_wall'), self.model.geom_size[geom, 0],
                                self.model.body_mass[body], self.model.opt.timestep,
                                self.data.geom_xpos[box], self.model.geom_size[box],
                                self.data.geom_xmat[box].reshape(3, 3))
                    f[3:] += np.cross(p-self.data.xipos[body], f[:3])
                    self.data.xfrc_applied[body] += f
                    forces.append((body, f))
                try:
                    super().step(linear, angular, steps=1)
                finally:
                    for body, force in forces:
                        self.data.xfrc_applied[body] -= force

    bridge.build_model, bridge.Physics = build, MaterialPhysics
