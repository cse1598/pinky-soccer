"""Lamp changes touch only materials; inferred/inertial and collision XML are preserved."""
from copy import deepcopy
import xml.etree.ElementTree as ET

from pinky_soccer_sim.led_visual import led_materials


def test_lamp_scene_material_only(tmp_path):
    original = ET.fromstring('''<mujoco><asset/><worldbody><body name="robot_lamp">
      <inertial mass="0.01" pos="0 0 .078" diaginertia=".001 .001 .001"/>
      <geom name="robot_lamp_collision_0" type="box" size=".01 .03 .01"
            contype="4" conaffinity="11" rgba=".4 .4 .4 .01"/>
      <geom name="robot_lamp_visual_0_0" type="box" size=".01 .03 .01"
            contype="0" conaffinity="0" rgba=".9 .9 .9 1"/>
    </body></worldbody></mujoco>''')
    scene = tmp_path/'scene.xml'
    ET.ElementTree(deepcopy(original)).write(scene)
    led_materials(scene)
    updated = ET.parse(scene).getroot()
    before = [g for g in original.iter('geom') if 'robot_lamp_visual' in g.get('name', '')]
    after = [g for g in updated.iter('geom') if 'robot_lamp_visual' in g.get('name', '')]
    assert before and len(before) == len(after)
    # Reverse only permitted attributes/material insertions, then entire trees must match.
    for a, b in zip(before, after):
        b.attrib.clear()
        b.attrib.update(a.attrib)
    asset = updated.find('asset')
    for m in list(asset):
        if m.get('name', '').endswith('_soccer_led'):
            asset.remove(m)
    assert ET.tostring(original) == ET.tostring(updated)


def test_gazebo_lamp_material_keeps_existing_geometry_and_dynamics():
    from pinky_soccer_sim.led_visual import gazebo_lamp_materials
    xml='''<sdf version="1.9"><model name="robot"><link name="robot_lamp">
      <inertial><mass>0.01</mass></inertial>
      <collision name="lamp_collision"><geometry><mesh><uri>lamp.stl</uri></mesh></geometry></collision>
      <visual name="robot_lamp_visual"><geometry><mesh><uri>lamp.dae</uri></mesh></geometry></visual>
    </link></model></sdf>'''
    before=ET.fromstring(xml)
    after=ET.fromstring(gazebo_lamp_materials(xml))
    assert [v.get('name') for v in before.iter('visual')] == [v.get('name') for v in after.iter('visual')]
    for tag in ('geometry','collision','inertial'):
        assert [ET.tostring(e) for e in before.iter(tag)] == [ET.tostring(e) for e in after.iter(tag)]
    assert after.find('.//visual/material/emissive').text == '0 0 0 1'
