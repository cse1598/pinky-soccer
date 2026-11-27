"""Resolve the reviewed real-robot calibration; never used by simulation."""
import os
from pathlib import Path

DEFAULT_EXPORT = ('pinky-soccer/ros_20261123_195718_b5b380/exports/'
                  '6fa01d7d19d64855b878e2613e1d5824')


def resolve_real_calibration(profile='auto', camera='', *, required=False):
    """Explicit profile/camera paths win; omitted camera follows the selected profile.

    An explicit profile=none retains the non-attack opt-out. Never search for a
    newer export or substitute unmeasured values when the selected files are absent.
    """
    root = Path(os.environ.get('PINKY_SOCCER_REAL_CALROOT') or
                (Path.home() / DEFAULT_EXPORT)).expanduser()
    profile = '' if profile == 'none' else str(
        root / 'profile.yaml' if profile in ('', 'auto') else Path(profile).expanduser())
    if camera in ('', 'auto'):
        camera = str(Path(profile).parent / 'camera.yaml') if profile else ''
    elif camera == 'none':
        camera = ''
    else:
        camera = str(Path(camera).expanduser())
    if required and not (profile and camera):
        raise ValueError('Real attack mode requires measured profile.yaml and camera.yaml')
    for label, path in (('calibration_profile', profile), ('camera_yaml', camera)):
        if path and not Path(path).is_file():
            raise FileNotFoundError(f'Real {label} not found: {path}. '
                                    'Specify the measured file or set PINKY_SOCCER_REAL_CALROOT.')
    return (str(Path(profile).resolve()) if profile else '',
            str(Path(camera).resolve()) if camera else '')
