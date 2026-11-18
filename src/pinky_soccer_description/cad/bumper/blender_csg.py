"""
Blender (bpy) side of generate_bumper.py: build a solid from boxes/cones and export STL.

Run by generate_bumper.py as
    blender -b --factory-startup --python blender_csg.py -- <spec.json> <out.stl>
One Blender unit = 1 mm. The spec lists primitives to add (union) and to subtract.
"""
import json
import math
import sys

import bpy


def make_box(p):
    (x0, x1), (y0, y1), (z0, z1) = p['x'], p['y'], p['z']
    bpy.ops.mesh.primitive_cube_add(size=1, location=((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))
    obj = bpy.context.object
    obj.scale = (x1 - x0, y1 - y0, z1 - z0)
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    return obj


def make_cone_x(p):
    """Make a truncated cone along +x: radius r0 at x0, r1 at x1."""
    x0, x1 = p['x']
    bpy.ops.mesh.primitive_cone_add(vertices=p.get('segments', 96), radius1=p['r0'],
                                    radius2=p['r1'], depth=x1 - x0,
                                    location=((x0 + x1) / 2, p['y'], p['z']),
                                    rotation=(0.0, math.pi / 2, 0.0))  # local +z -> +x
    bpy.ops.object.transform_apply(location=False, rotation=True, scale=False)
    return bpy.context.object


def make(p):
    return make_box(p) if p['type'] == 'box' else make_cone_x(p)


def boolean(target, tool, operation):
    mod = target.modifiers.new('csg', 'BOOLEAN')
    mod.operation = operation
    mod.object = tool
    mod.solver = 'EXACT'
    bpy.context.view_layer.objects.active = target
    bpy.ops.object.modifier_apply(modifier=mod.name)
    bpy.data.objects.remove(tool, do_unlink=True)


def main():
    spec_path, out_path = sys.argv[sys.argv.index('--') + 1:][:2]
    with open(spec_path) as f:
        spec = json.load(f)
    bpy.ops.wm.read_factory_settings(use_empty=True)

    adds = [make(p) for p in spec['add']]
    body = adds[0]

    # Chamfer only the first primitive (the front plate) before the unions: bevelling the
    # merged solid breaks the mesh where coplanar faces meet.
    chamfer = spec.get('chamfer', 0.0)
    if chamfer > 0:
        mod = body.modifiers.new('chamfer', 'BEVEL')
        mod.width = chamfer
        mod.segments = 1
        bpy.context.view_layer.objects.active = body
        bpy.ops.object.modifier_apply(modifier=mod.name)

    for tool in adds[1:]:
        boolean(body, tool, 'UNION')

    for p in spec.get('subtract_before_bevel', []):
        boolean(body, make(p), 'DIFFERENCE')

    for p in spec.get('subtract', []):
        boolean(body, make(p), 'DIFFERENCE')

    bpy.ops.object.select_all(action='DESELECT')
    body.select_set(True)
    bpy.context.view_layer.objects.active = body
    bpy.ops.wm.stl_export(filepath=out_path, export_selected_objects=True,
                          ascii_format=False, apply_modifiers=True, global_scale=1.0)
    print('BLENDER_CSG_OK', out_path, len(body.data.polygons), 'faces')


main()
