#!/usr/bin/env python3
"""
Generate a bumper from a parameter file.

    python3 generate_bumper.py bumper_v1.yaml

Steps
  1. Turn the parameters into boxes/cones to add and subtract (base_link frame, mm).
  2. Build the solid in Blender (blender_csg.py) and export it.
  3. Check it against the official robot meshes (no overlap except the intended pinch).
  4. Compute mass, centre of mass and inertia.
  5. Write
       print/<name>_print.stl                       mm, front face down, ready to slice
       ../../meshes/visual/<name>.stl               m, bumper_link frame (URDF visual)
       ../../meshes/collision/<name>_collision.stl  m, bumper_link frame (URDF collision)
       ../../config/<name>_params.yaml              URDF parameters for urdf/bumper.urdf.xacro
       print/<name>_report.txt

Needs: Blender (>= 4.2, `blender` on PATH or --blender), Python packages numpy, pyyaml,
trimesh (pip, same as pinky_mujoco) and a sourced workspace (for xacro and the meshes).
"""
import argparse
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

import numpy as np
import trimesh
import yaml

HERE = Path(__file__).resolve().parent
PKG = HERE.parent.parent                    # src/pinky_soccer_description
BASE_LINK_HEIGHT = 28.0                     # mm, base_footprint -> base_link


# ----------------------------------------------------------------------------- geometry
def box(x, y, z):
    return {'type': 'box', 'x': sorted(x), 'y': sorted(y), 'z': sorted(z)}


def cone_x(x, r0, r1, y, z):
    return {'type': 'cone', 'x': list(x), 'r0': r0, 'r1': r1, 'y': y, 'z': z}


def build_spec(p):
    fp, pr, notch, us = p['front_plate'], p['prongs'], p['ir_board_notch'], p['ultrasonic_holes']
    front_x = fp['back_x'] + fp['thickness']
    half_i = pr['pinch_interference'] / 2.0
    adds = [box((fp['back_x'], front_x), (-fp['half_width'], fp['half_width']),
                (fp['bottom_z'], fp['top_z']))]
    pre = []
    for s in (1, -1):
        ys = (s * pr['y_inner'], s * pr['y_outer'])
        # side wall in front of the plates: bears on both plate edges at plate_edge_x
        adds.append(box((pr['plate_edge_x'], fp['back_x'] + 0.01), ys,
                        (pr['wall_bottom_z'], pr['wall_top_z'])))
        # prong between the plates (press fit in z)
        adds.append(box((pr['back_x'], pr['plate_edge_x'] + 0.01), ys,
                        (pr['pinch_bottom_z'] - half_i, pr['pinch_top_z'] + half_i)))
        # groove for the IR board edge, open at the back so the bumper slides on
        pre.append(box((pr['back_x'] - 1.0, fp['back_x'] + 0.01),
                       (s * (pr['y_inner'] - 1.0), s * notch['y_max']),
                       (pr['pinch_bottom_z'] - 0.5, notch['z_top'])))
    holes = []
    r0 = us['can_radius'] + us['clearance']
    x_end = front_x + 1.0
    r1 = r0 + (x_end - us['can_front_x']) * math.tan(math.radians(us['cone_half_angle_deg']))
    for yc in us['centers_y']:
        holes.append(cone_x((fp['back_x'] - 1.0, us['can_front_x']), r0, r0, yc, us['center_z']))
        holes.append(cone_x((us['can_front_x'], x_end), r0, r1, yc, us['center_z']))
    return {'add': adds, 'subtract_before_bevel': pre, 'subtract': holes,
            'chamfer': fp.get('chamfer', 0.0)}


def inside(points, spec, margin=0.0):
    """Exact CSG membership (ignores the chamfer). margin > 0 grows the solid."""
    def in_box(b, m):
        return np.all([(points[:, i] >= b[k][0] - m) & (points[:, i] <= b[k][1] + m)
                       for i, k in enumerate('xyz')], axis=0)

    def in_cone(c, m):
        x0, x1 = c['x']
        t = np.clip((points[:, 0] - x0) / (x1 - x0), 0, 1)
        r = c['r0'] + (c['r1'] - c['r0']) * t
        d = np.hypot(points[:, 1] - c['y'], points[:, 2] - c['z'])
        return (points[:, 0] >= x0 - m) & (points[:, 0] <= x1 + m) & (d <= r + m)

    solid = np.any([in_box(b, margin) for b in spec['add']], axis=0)
    for cut in spec['subtract_before_bevel'] + spec['subtract']:
        solid &= ~(in_box(cut, -margin) if cut['type'] == 'box' else in_cone(cut, -margin))
    return solid


# ----------------------------------------------------------------------------- robot
def robot_surface_points(per_mm2=4.0):
    """Sample the official visual meshes (bumper disabled), base_link frame, mm."""
    from ament_index_python.packages import get_package_share_directory as share
    from trimesh.transformations import euler_matrix, translation_matrix
    urdf = subprocess.run(
        ['xacro', str(Path(share('pinky_soccer_description')) / 'urdf/soccer_robot.urdf.xacro'),
         'bumper_enabled:=false'], capture_output=True, text=True, check=True).stdout
    root = ET.fromstring(urdf)
    parent = {j.find('child').get('link'): j for j in root.findall('joint')}

    def origin(el):
        o = el.find('origin')
        xyz = [float(v) for v in (o.get('xyz', '0 0 0') if o is not None else '0 0 0').split()]
        rpy = [float(v) for v in (o.get('rpy', '0 0 0') if o is not None else '0 0 0').split()]
        return translation_matrix(xyz) @ euler_matrix(*rpy, 'sxyz')

    def to_base_link(name):
        T = np.eye(4)
        while name != 'base_link' and name in parent:
            j = parent[name]
            T = origin(j) @ T
            name = j.find('parent').get('link')
        return T

    points, labels = [], []
    for link in root.findall('link'):
        if link.get('name') == 'base_footprint':
            continue
        for vis in link.findall('visual'):
            g = vis.find('geometry')[0]
            if g.tag != 'mesh':
                continue
            pkg, rel = g.get('filename')[len('package://'):].split('/', 1)
            scene = trimesh.load(str(Path(share(pkg)) / rel), force='scene')
            T = to_base_link(link.get('name')) @ origin(vis)
            for node in scene.graph.nodes_geometry:
                Tn, gname = scene.graph[node]
                m = scene.geometry[gname].copy()
                m.apply_transform(T @ Tn)
                n = int(np.clip(m.area * 1e6 * per_mm2, 500, 3_000_000))
                p, _ = trimesh.sample.sample_surface(m, n, seed=0)
                points.append(p * 1000.0)
                labels += [f"{link.get('name')}/{node}"] * len(p)
    return np.vstack(points), np.array(labels)


def check_clearance(spec, p):
    """Penetrations at nominal fit (interference 0) and near contacts."""
    nominal_params = json.loads(json.dumps(p))
    nominal_params['prongs']['pinch_interference'] = 0.0
    nominal = build_spec(nominal_params)
    pts, labels = robot_surface_points()
    lines = []
    pen = inside(pts, nominal, margin=-0.05)
    lines.append(f'robot surface points sampled: {len(pts)}')
    lines.append(f'penetrating the bumper by > 0.05 mm (nominal fit): {int(pen.sum())}')
    for lab in np.unique(labels[pen]):
        q = pts[pen & (labels == lab)]
        lines.append(f'  ! {lab}: {len(q)} pts x[{q[:, 0].min():.1f},{q[:, 0].max():.1f}] '
                     f'y[{q[:, 1].min():.1f},{q[:, 1].max():.1f}] '
                     f'z[{q[:, 2].min():.1f},{q[:, 2].max():.1f}]')
    for m in (0.3, 1.0):
        near = inside(pts, nominal, margin=m) & ~pen
        lines.append(f'within {m} mm of the bumper:')
        for lab in np.unique(labels[near]):
            q = pts[near & (labels == lab)]
            lines.append(f'    {lab}: {len(q)} pts x[{q[:, 0].min():.1f},{q[:, 0].max():.1f}] '
                         f'|y|[{np.abs(q[:, 1]).min():.1f},{np.abs(q[:, 1]).max():.1f}] '
                         f'z[{q[:, 2].min():.1f},{q[:, 2].max():.1f}]')
    return int(pen.sum()), lines


# ----------------------------------------------------------------------------- outputs
def write_params(path, p, name, com_mm, mass, inertia, extents_mm):
    rel = 'package://pinky_soccer_description/meshes'
    data = {'bumper': {
        'width': round(float(extents_mm[1]) / 1000, 5),
        'depth': round(float(extents_mm[0]) / 1000, 5),
        'height': round(float(extents_mm[2]) / 1000, 5),
        'offset_x': round(float(com_mm[0]) / 1000, 5),
        'offset_y': round(float(com_mm[1]) / 1000, 5),
        'offset_z': round(float(com_mm[2]) / 1000, 5),
        'mass': round(float(mass), 5),
        'inertia': {k: float(f'{v:.6g}') for k, v in inertia.items()},
        'visual_mesh': f'{rel}/visual/{name}.stl',
        'collision_mesh': f'{rel}/collision/{name}_collision.stl',
        'mesh_scale': [1.0, 1.0, 1.0],
        'color': [0.10, 0.45, 0.90, 1.0],
    }}
    header = (f'# GENERATED by cad/bumper/generate_bumper.py from cad/bumper/{name}.yaml.\n'
              '# Do not edit by hand; change the CAD parameters and regenerate.\n'
              '# width/depth/height: bounding box. offset: centre of mass (bumper_link origin)\n'
              '# in base_link. mass/inertia: from the mesh volume, PLA density x fill fraction.\n')
    path.write_text(header + yaml.safe_dump(data, sort_keys=False))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('params', type=Path)
    ap.add_argument('--blender', default=shutil.which('blender') or '/snap/bin/blender')
    ap.add_argument('--skip-check', action='store_true', help='skip the robot clearance check')
    args = ap.parse_args()
    p = yaml.safe_load(args.params.read_text())
    name = p['name']
    spec = build_spec(p)

    with tempfile.TemporaryDirectory() as tmp:
        spec_path, raw = Path(tmp) / 'spec.json', Path(tmp) / 'raw.stl'
        spec_path.write_text(json.dumps(spec))
        out = subprocess.run([args.blender, '-b', '--factory-startup', '--python',
                              str(HERE / 'blender_csg.py'), '--', str(spec_path), str(raw)],
                             capture_output=True, text=True)
        if 'BLENDER_CSG_OK' not in out.stdout:
            sys.exit('Blender failed:\n' + out.stdout[-3000:] + out.stderr[-3000:])
        mesh = trimesh.load(raw, force='mesh')

    if not mesh.is_watertight:
        sys.exit('generated mesh is not watertight')
    material = p['material']
    density = material['density_g_cm3'] * material['fill_fraction'] * 1e-6  # kg/mm^3
    mesh.density = density
    mass = mesh.volume * density
    com = mesh.center_mass
    tensor = mesh.moment_inertia * 1e-6                         # kg m^2 about the COM
    inertia = {'ixx': tensor[0, 0], 'iyy': tensor[1, 1], 'izz': tensor[2, 2],
               'ixy': tensor[0, 1], 'ixz': tensor[0, 2], 'iyz': tensor[1, 2]}
    lo, hi = mesh.bounds

    report = [f'{name}', f'faces {len(mesh.faces)}, watertight {mesh.is_watertight}',
              f'volume {mesh.volume / 1000:.2f} cm^3 -> mass {mass * 1000:.1f} g '
              f'(density {material["density_g_cm3"]} x fill {material["fill_fraction"]})',
              f'bounds base_link mm x[{lo[0]:.2f},{hi[0]:.2f}] y[{lo[1]:.2f},{hi[1]:.2f}] '
              f'z[{lo[2]:.2f},{hi[2]:.2f}]',
              f'ground clearance {lo[2] + BASE_LINK_HEIGHT:.2f} mm, '
              f'top {hi[2] + BASE_LINK_HEIGHT:.2f} mm above ground',
              f'centre of mass base_link mm {np.round(com, 2).tolist()}',
              'inertia about COM kg m^2 ' + ', '.join(f'{k} {v:.3e}' for k, v in inertia.items())]
    penetrations = 0
    if not args.skip_check:
        penetrations, lines = check_clearance(spec, p)
        report += ['', 'clearance check against pinky_description visual meshes'] + lines

    # URDF meshes: bumper_link frame (origin = COM), metres
    visual = mesh.copy()
    visual.apply_translation(-com)
    visual.apply_scale(0.001)
    fp = p['front_plate']
    plate = trimesh.creation.box(
        extents=[fp['thickness'], 2 * fp['half_width'], fp['top_z'] - fp['bottom_z']])
    plate.apply_translation([fp['back_x'] + fp['thickness'] / 2, 0,
                             (fp['top_z'] + fp['bottom_z']) / 2])
    plate.apply_translation(-com)
    plate.apply_scale(0.001)
    (PKG / 'meshes/visual').mkdir(parents=True, exist_ok=True)
    (PKG / 'meshes/collision').mkdir(parents=True, exist_ok=True)
    visual.export(PKG / f'meshes/visual/{name}.stl')
    plate.export(PKG / f'meshes/collision/{name}_collision.stl')

    # print file: mm, front face on the bed (cones open downwards, prongs point up)
    printable = mesh.copy()
    printable.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [0, 1, 0]))
    plo, phi = printable.bounds
    printable.apply_translation([-(plo[0] + phi[0]) / 2, -(plo[1] + phi[1]) / 2, -plo[2]])
    (HERE / 'print').mkdir(exist_ok=True)
    printable.export(HERE / f'print/{name}_print.stl')
    report.append(f'\nprint size mm: {np.round(printable.extents, 1).tolist()} (front face down)')

    write_params(PKG / f'config/{name}_params.yaml', p, name, com, mass, inertia, hi - lo)
    (HERE / f'print/{name}_report.txt').write_text('\n'.join(report) + '\n')
    print('\n'.join(report))
    if penetrations:
        sys.exit(f'{penetrations} robot surface points penetrate the bumper')


if __name__ == '__main__':
    os.environ.setdefault('PYTHONHASHSEED', '0')
    main()
