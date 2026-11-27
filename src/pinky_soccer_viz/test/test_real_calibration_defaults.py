from pathlib import Path

import pytest

from pinky_soccer_viz.real_calibration_defaults import resolve_real_calibration


def test_default_and_explicit_override(tmp_path, monkeypatch):
    monkeypatch.setenv('PINKY_SOCCER_REAL_CALROOT', str(tmp_path))
    for name in ('profile.yaml', 'camera.yaml'):
        (tmp_path / name).touch()
    assert resolve_real_calibration(required=True) == (
        str(tmp_path / 'profile.yaml'), str(tmp_path / 'camera.yaml'))
    other = tmp_path / 'other'
    other.mkdir()
    for name in ('profile.yaml', 'camera.yaml'):
        (other / name).touch()
    assert resolve_real_calibration(str(other / 'profile.yaml')) == (
        str(other / 'profile.yaml'), str(other / 'camera.yaml'))
    assert resolve_real_calibration(str(other / 'profile.yaml'),
                                    str(tmp_path / 'camera.yaml'))[1] == str(tmp_path / 'camera.yaml')


def test_missing_never_falls_back(tmp_path, monkeypatch):
    monkeypatch.setenv('PINKY_SOCCER_REAL_CALROOT', str(tmp_path))
    with pytest.raises(FileNotFoundError):
        resolve_real_calibration()
    assert resolve_real_calibration('none') == ('', '')
    with pytest.raises(ValueError):
        resolve_real_calibration('none', required=True)
