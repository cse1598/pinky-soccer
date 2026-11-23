"""Wheel calibration changes only drive constants, never the simulated wheel geometry."""

import xml.etree.ElementTree as ET

from pinky_soccer_bringup.calibration_profile import configure_urdf, drive_constants
import pytest
import yaml

WHEEL = ('<link name="{}_wheel"><collision><geometry>'
         '<cylinder radius="0.028" length="0.02"/></geometry></collision></link>')
URDF = """<robot>""" + WHEEL.format('l') + WHEEL.format('r') + """
  <gazebo><plugin name="gz::sim::systems::DiffDrive" filename="gz-sim-diff-drive-system">
    <wheel_separation>0.0961</wheel_separation><wheel_radius>0.028</wheel_radius>
  </plugin></gazebo>
</robot>"""


def profile(tmp_path, environment):
    data = {'schema_version': 1, 'environment': environment, 'human_verified': True,
            'robot_id': 'gazebo-sim', 'camera_pose': None,
            'results': {'wheels': {'left_radius_m': 0.02797, 'right_radius_m': 0.02797,
                                   'separation_m': 0.09807}},
            'launch': {'wheel_radius': 0.02797, 'wheel_separation': 0.09807}}
    path = tmp_path / 'profile.yaml'
    path.write_text(yaml.safe_dump(data))
    return path


def test_simulation_profile_sets_drive_constants_only(tmp_path):
    root = ET.fromstring(configure_urdf(URDF, profile(tmp_path, 'rehearsal'), True))
    assert root.find('.//wheel_separation').text == '0.09807'
    assert root.find('.//wheel_radius').text == '0.02797'
    for side in 'lr':
        cylinder = root.find(f"link[@name='{side}_wheel']/collision/geometry/cylinder")
        assert cylinder.get('radius') == '0.028'          # physics untouched


def test_drive_constants_and_real_guard(tmp_path):
    path = profile(tmp_path, 'rehearsal')
    assert drive_constants(path, simulation=True) == (0.02797, 0.09807)
    assert drive_constants('', simulation=True) == (None, None)
    assert drive_constants('none', simulation=True) == (None, None)
    assert configure_urdf(URDF, 'none', True) == URDF
    with pytest.raises(ValueError):
        drive_constants(path, simulation=False)           # simulator profile on hardware
