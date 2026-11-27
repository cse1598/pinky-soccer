#!/usr/bin/env python3
"""Generate an RCJ field in SDF and MJCF from a single dimensioned scene."""

import argparse
import copy
import math
from pathlib import Path
import re
import struct
import xml.etree.ElementTree as ET

import numpy as np
import yaml

GREEN = (0.035, 0.35, 0.075, 1)
BLACK = (0.025, 0.025, 0.025, 1)
WHITE = (0.95, 0.95, 0.95, 1)
# Mean onboard-camera RGB supplied by the user (2026-11-27).
BLUE = (5/255, 75/255, 134/255, 1)
YELLOW = (237/255, 229/255, 4/255, 1)


def vector(values):
    """Write deterministic SI vectors with enough precision for both engines."""
    return ' '.join(f'{float(v):.10g}' for v in values)


def load_spec(path):
    """Reject impossible geometry instead of silently generating overlapping goals."""
    cfg = yaml.safe_load(Path(path).read_text())
    if not isinstance(cfg, dict):
        raise ValueError('Field specification must be a mapping')
    if (not isinstance(cfg.get('field_id', 'rcj'), str)
            or not re.fullmatch(r'[a-z][a-z0-9_]*', cfg.get('field_id', 'rcj'))):
        raise ValueError('Invalid field_id')
    for key, value in cfg.items():
        if key in ('source', 'specification_date', 'field_id'):
            continue
        zero_allowed = key in ('ramp_base', 'ramp_rise')
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value) or value < 0 or (value == 0 and not zero_allowed)):
            raise ValueError(f'Invalid field setting: {key}')
    if (cfg['ramp_base'] == 0) != (cfg['ramp_rise'] == 0):
        raise ValueError('Disable ramps with both ramp_base=0 and ramp_rise=0')
    if (cfg['goal_width'] >= cfg['playing_width'] or cfg['goal_depth'] >= cfg['outer_strip']
            or cfg['ramp_base'] > cfg['outer_strip']
            or cfg['penalty_radius'] >= cfg['penalty_depth']
            or 2*cfg['penalty_radius'] >= cfg['penalty_width']):
        raise ValueError('Inconsistent RCJ field dimensions')
    if type(cfg['arc_segments']) is not int or cfg['arc_segments'] < 32:
        raise ValueError('arc_segments must be an integer >=32')
    return cfg


def scene(cfg):
    """Return shared visual and collision primitives, including mitered wedge meshes."""
    items = []

    def box(name, pos, size, color, collision=True, yaw=0):
        items.append({'name': name, 'kind': 'box', 'pos': list(pos), 'size': list(size),
                      'color': color, 'collision': collision, 'yaw': yaw})

    def line(name, start, end, width, color):
        dx, dy = end[0]-start[0], end[1]-start[1]
        box(name, [(start[0]+end[0])/2, (start[1]+end[1])/2, .0003],
            [math.hypot(dx, dy), width, .0002], color, False, math.atan2(dy, dx))

    def arc(name, cx, cy, radius, begin, end, width, color, sign=1):
        count = max(4, round(cfg['arc_segments'] * abs(end-begin)/(2*math.pi)))
        angles = np.linspace(begin, end, count+1)
        points = [(sign*(cx+radius*math.cos(a)), cy+radius*math.sin(a)) for a in angles]
        for i, (p, q) in enumerate(zip(points, points[1:])):
            line(f'{name}_{i:03d}', p, q, width, color)

    length, width = cfg['playing_length'], cfg['playing_width']
    hx, hy = length/2+cfg['outer_strip'], width/2+cfg['outer_strip']
    wall, height = cfg['wall_thickness'], cfg['wall_height']
    box('carpet', [0, 0, -cfg['floor_thickness']/2],
        [2*hx, 2*hy, cfg['floor_thickness']], GREEN)
    for sign in (-1, 1):
        box(f'end_wall_{sign}', [sign*(hx+wall/2), 0, height/2],
            [wall, 2*hy+2*wall, height], BLACK)
        box(f'side_wall_{sign}', [0, sign*(hy+wall/2), height/2],
            [2*hx, wall, height], BLACK)
    goal_half = cfg['goal_width']/2
    for sign, color, name in [(1, BLUE, 'blue'), (-1, YELLOW, 'yellow')]:
        front, depth, gh = length/2, cfg['goal_depth'], cfg['goal_height']
        if 'goal_thickness' in cfg:
            t = cfg['goal_thickness']
            # Back fits between full-depth side panels; roof sits on their top edges.
            box(f'{name}_goal_back', [sign*(front+depth+t/2), 0, gh/2],
                [t, cfg['goal_width'], gh], color)
            for side in (-1, 1):
                box(f'{name}_goal_side_{side}',
                    [sign*(front+(depth+t)/2), side*(goal_half+t/2), gh/2],
                    [depth+t, t, gh], color)
            box(f'{name}_goal_roof', [sign*(front+(depth+t)/2), 0, gh+t/2],
                [depth+t, cfg['goal_width']+2*t, t], color)
            box(f'{name}_goal_floor', [sign*(front+depth/2), 0, .00015],
                [depth, cfg['goal_width'], .0001], color, False)
            continue
        box(f'{name}_goal_back', [sign*(front+depth+wall/2), 0, gh/2],
            [wall, cfg['goal_width']+2*wall, gh], color)
        for side in (-1, 1):
            box(f'{name}_goal_side_{side}',
                [sign*(front+depth/2-cfg['line_width']/2), side*(goal_half+wall/2), gh/2],
                [depth+cfg['line_width'], wall, gh], color)
        box(f'{name}_goal_roof',
            [sign*(front+(depth+wall-cfg['line_width'])/2), 0, gh+wall/2],
            [depth+wall+cfg['line_width'], cfg['goal_width']+2*wall, wall], color)
        box(f'{name}_goal_floor', [sign*(front+depth/2), 0, .00015],
            [depth, cfg['goal_width'], .0001], color, False)

    # Convex wedges meet on 45 degree corner seams; no overlaps or corner steps.
    ramp, rise = cfg['ramp_base'], cfg['ramp_rise']

    def wedge(name, points, elevations):
        vertices = [[x, y, -.001] for x, y in points]
        vertices += [[x, y, z] for (x, y), z in zip(points, elevations)]
        items.append({'name': name, 'kind': 'mesh', 'vertices': vertices, 'pos': [0, 0, 0],
                      'color': GREEN, 'collision': True, 'yaw': 0})

    for sign in ((-1, 1) if ramp > 0 else ()):
        wedge(f'side_ramp_{sign}',
              [(-hx, sign*hy), (hx, sign*hy),
               (hx-ramp, sign*(hy-ramp)), (-hx+ramp, sign*(hy-ramp))],
              [rise, rise, 0, 0])
        for side in (-1, 1):
            gap = goal_half+wall
            wedge(f'end_ramp_{sign}_{side}',
                  [(sign*hx, side*gap), (sign*hx, side*hy),
                   (sign*(hx-ramp), side*(hy-ramp)), (sign*(hx-ramp), side*gap)],
                  [rise, rise, 0, 0])

    lw = cfg['line_width']
    # The OUTSIDE edge of the white rectangle defines the playing field.
    for sign in (-1, 1):
        line(f'boundary_side_{sign}', (-length/2, sign*(width/2-lw/2)),
             (length/2, sign*(width/2-lw/2)), lw, WHITE)
        line(f'boundary_end_{sign}', (sign*(length/2-lw/2), -width/2),
             (sign*(length/2-lw/2), width/2), lw, WHITE)
        radius = cfg['penalty_radius']
        cy = cfg['penalty_width']/2-radius
        cx = length/2-cfg['penalty_depth']+radius
        for side in (-1, 1):
            line(f'penalty_side_{sign}_{side}',
                 (sign*length/2, side*(cfg['penalty_width']/2-lw/2)),
                 (sign*cx, side*(cfg['penalty_width']/2-lw/2)), lw, WHITE)
            arc(f'penalty_arc_{sign}_{side}', cx, side*cy, radius-lw/2,
                math.pi/2 if side == 1 else math.pi,
                math.pi if side == 1 else 3*math.pi/2, lw, WHITE, sign)
        line(f'penalty_front_{sign}', (sign*(length/2-cfg['penalty_depth']+lw/2), -cy),
             (sign*(length/2-cfg['penalty_depth']+lw/2), cy), lw, WHITE)
    arc('center_circle', 0, 0, cfg['center_diameter']/2, 0, 2*math.pi,
        cfg['thin_black_line'], BLACK)
    for i, (x, y) in enumerate([(0, 0)] + [
            (sx*(length/2-cfg['neutral_distance_from_end']), sy*cfg['penalty_width']/2)
            for sx in (-1, 1) for sy in (-1, 1)]):
        items.append({'name': f'neutral_{i}', 'kind': 'cylinder', 'pos': [x, y, .0005],
                      'radius': cfg['neutral_diameter']/2, 'length': .0002,
                      'color': BLACK, 'collision': False, 'yaw': 0})
    return items


def stl(path, vertices):
    """Write outward-wound triangles of a convex eight-vertex wedge."""
    from scipy.spatial import ConvexHull
    vertices = np.asarray(vertices)
    hull = ConvexHull(vertices)
    data = bytearray(b'Generated RCJ convex wedge'.ljust(80, b'\0'))
    data.extend(struct.pack('<I', len(hull.simplices)))
    for indices, equation in zip(hull.simplices, hull.equations):
        a, b, c = vertices[indices]
        if np.dot(np.cross(b-a, c-a), equation[:3]) < 0:
            b, c = c, b
        data.extend(struct.pack('<12fH', *equation[:3], *a, *b, *c, 0))
    path.write_bytes(data)


def generate(spec, output):
    """Write a self-contained SDF world and matching MJCF field plus provenance."""
    cfg = load_spec(spec)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    field_id = cfg.get('field_id', 'rcj')
    world_name, model_name = f'{field_id}_soccer', f'{field_id}_field'
    mesh_dir = f'{field_id}_meshes'
    items = scene(cfg)
    if any(i['kind'] == 'mesh' for i in items):
        (output/mesh_dir).mkdir(exist_ok=True)
    sdf = ET.Element('sdf', version='1.9')
    world = ET.SubElement(sdf, 'world', name=world_name)
    ET.SubElement(world, 'gravity').text = '0 0 -9.81'
    for filename, name in [('physics', 'Physics'), ('user-commands', 'UserCommands'),
                           ('scene-broadcaster', 'SceneBroadcaster'), ('imu', 'Imu')]:
        ET.SubElement(world, 'plugin', filename=f'gz-sim-{filename}-system',
                      name=f'gz::sim::systems::{name}')
    plugin = ET.SubElement(world, 'plugin', filename='gz-sim-sensors-system',
                           name='gz::sim::systems::Sensors')
    ET.SubElement(plugin, 'render_engine').text = 'ogre2'
    light = ET.SubElement(world, 'light', name='sun', type='directional')
    ET.SubElement(light, 'pose').text = '0 0 3 0 0 0'
    ET.SubElement(light, 'direction').text = '-0.3 0.2 -1'
    ET.SubElement(light, 'diffuse').text = '.85 .85 .85 1'
    ET.SubElement(light, 'cast_shadows').text = 'false'
    model = ET.SubElement(world, 'model', name=model_name)
    ET.SubElement(model, 'static').text = 'true'
    link = ET.SubElement(model, 'link', name='field')
    mjcf = ET.Element('mujoco', model=model_name)
    ET.SubElement(mjcf, 'compiler', angle='radian')
    asset = ET.SubElement(mjcf, 'asset')
    body = ET.SubElement(mjcf, 'worldbody')
    for item in items:
        mesh_file = None
        if item['kind'] == 'mesh':
            mesh_file = f'{mesh_dir}/{item["name"]}.stl'
            stl(output/mesh_file, item['vertices'])
            ET.SubElement(asset, 'mesh', name=f'{field_id}_{item["name"]}', file=mesh_file)
        for kind in (['collision', 'visual'] if item['collision'] else ['visual']):
            element = ET.SubElement(link, kind, name=item['name'])
            ET.SubElement(element, 'pose').text = vector([*item['pos'], 0, 0, item['yaw']])
            geometry = ET.SubElement(element, 'geometry')
            shape = ET.SubElement(geometry, item['kind'])
            if item['kind'] == 'box':
                ET.SubElement(shape, 'size').text = vector(item['size'])
            elif item['kind'] == 'cylinder':
                ET.SubElement(shape, 'radius').text = str(item['radius'])
                ET.SubElement(shape, 'length').text = str(item['length'])
            else:
                ET.SubElement(shape, 'uri').text = mesh_file
            if kind == 'visual':
                material = ET.SubElement(element, 'material')
                for tag in ('ambient', 'diffuse'):
                    ET.SubElement(material, tag).text = vector(item['color'])
                ET.SubElement(material, 'specular').text = '0 0 0 1'
            else:
                surface = ET.SubElement(element, 'surface')
                ode = ET.SubElement(ET.SubElement(surface, 'friction'), 'ode')
                ET.SubElement(ode, 'mu').text = str(cfg['friction'])
                ET.SubElement(ode, 'mu2').text = str(cfg['friction'])
        attrs = {'name': f'{field_id}_{item["name"]}', 'pos': vector(item['pos']),
                 'euler': vector([0, 0, item['yaw']]), 'rgba': vector(item['color']),
                 'group': '0' if item['collision'] else '2',
                 'contype': '1' if item['collision'] else '0',
                 'conaffinity': '2' if item['collision'] else '0',
                 'friction': f'{cfg["friction"]} 0.005 0.0001'}
        if item['kind'] == 'box':
            attrs.update(type='box', size=vector(np.asarray(item['size'])/2))
        elif item['kind'] == 'cylinder':
            attrs.update(type='cylinder', size=vector([item['radius'], item['length']/2]))
        else:
            attrs.update(type='mesh', mesh=f'{field_id}_{item["name"]}')
        ET.SubElement(body, 'geom', **attrs)
    for root, name in [(sdf, world_name+'.world'), (mjcf, world_name+'.mjcf')]:
        ET.indent(root)
        ET.ElementTree(root).write(output/name, encoding='unicode', xml_declaration=True)
    manifest = dict(cfg, origin='field center, floor top z=0; +X blue, -X yellow',
                    total_length=round(cfg['playing_length']+2*cfg['outer_strip'], 10),
                    total_width=round(cfg['playing_width']+2*cfg['outer_strip'], 10),
                    construction_assumptions=['wall/floor thickness', 'thin black line width',
                                              'colors/lighting/friction are not measured'],
                    generated_objects=len(items))
    manifest_path = output/f'{field_id}_field_manifest.yaml'
    manifest_path.write_text(yaml.safe_dump(manifest, sort_keys=False))
    return output/(world_name+'.world')


def install_mujoco(bridge):
    """Use our matching MJCF for this field; keep official world conversion unchanged."""
    original = bridge.build_model

    def build(share, output_dir, urdf=None, config_file=None, world=None, simple_render=False):
        is_field = world and Path(world).name.endswith('_soccer.world')
        if not is_field:
            return original(share, output_dir, urdf, config_file, world, simple_render)
        world = Path(world).resolve()
        root_world = ET.parse(world).getroot()
        field_id = world.stem.removesuffix('_soccer')
        if root_world.find(f"world[@name='{world.stem}']/model[@name='{field_id}_field']") is None:
            if field_id in ('rcj', 'hardboard'):
                raise ValueError('Expected generated soccer world; filename alone is insufficient')
            return original(share, output_dir, urdf, config_file, world, simple_render)
        output = Path(output_dir)
        output.mkdir(parents=True, exist_ok=True)
        empty = output/'rcj_empty_base.world'
        empty.write_text('<sdf version="1.9"><world name="rcj_base"/></sdf>')
        path = original(share, output_dir, urdf, config_file, empty, simple_render)
        tree = ET.parse(path)
        root = tree.getroot()
        target = root.find('worldbody')
        target.remove(target.find("geom[@name='floor']"))
        field = ET.parse(world.with_suffix('.mjcf')).getroot()
        for element in field.find('asset'):
            node = copy.deepcopy(element)
            node.set('file', str((world.parent/node.get('file')).resolve()))
            root.find('asset').append(node)
        for element in field.find('worldbody'):
            target.append(copy.deepcopy(element))
        tree.write(path, encoding='unicode')
        (output/'world.txt').write_text(str(world)+'\n')
        return path
    bridge.build_model = build


def main():
    """Regenerate field assets using a YAML specification and output directory."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--spec', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    print(generate(args.spec, args.output))


if __name__ == '__main__':
    main()
