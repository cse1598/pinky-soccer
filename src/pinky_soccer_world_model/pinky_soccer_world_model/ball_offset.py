"""Read an explicitly selected reviewed artifact; preserve raw projection and files."""
from pathlib import Path
from urllib.parse import unquote, urlparse

import numpy as np
import yaml


def load_ball_offset(source='', simulation=False):
    """Return base_link XY metres and provenance, never a radius/pixel correction.

    A camera YAML may select its sibling reviewed profile. No project is chosen
    by modification date and no value is copied from a different environment.
    """
    if not source or str(source).lower() == 'none':
        return np.zeros(2), 'none'
    if str(source).startswith('file:'):
        uri = urlparse(str(source))
        if uri.netloc not in ('', 'localhost'):
            raise ValueError('Ball offset requires a local file')
        source = unquote(uri.path)
    path = Path(source).expanduser().resolve()
    data = yaml.safe_load(path.read_text())
    if not isinstance(data, dict):
        raise ValueError('Ball offset source must be a YAML mapping')
    if 'camera_matrix' in data:
        sibling = path.parent/'profile.yaml'
        if not sibling.is_file():
            return np.zeros(2), 'camera_file_has_no_reviewed_offset'
        path = sibling
        data = yaml.safe_load(path.read_text())
        if not isinstance(data, dict):
            raise ValueError('Reviewed offset profile must be a YAML mapping')
    review = data.get('ball_review')
    if not review or 'offset_xy_m' not in review:
        return np.zeros(2), str(path)+':no_ball_offset'
    if data.get('schema_version') != 1 or data.get('human_verified') is not True:
        raise ValueError('Select an exported human_verified profile for runtime ball offset')
    expected = 'rehearsal' if simulation else 'real'
    if data.get('environment') != expected:
        raise ValueError('Ball offset profile environment does not match the camera')
    offset = np.asarray(review['offset_xy_m'], float)
    if offset.shape != (2,) or not np.isfinite(offset).all():
        raise ValueError('Ball XY offset must be two finite metre values')
    return offset, str(path)


def apply_ball_offset(point, offset_xy, output_from_base=None):
    """Translate once after projection; rotate the base-axis vector for other outputs."""
    if point is None:
        return None
    correction = np.r_[np.asarray(offset_xy, float), 0.]
    if correction.shape != (3,) or not np.isfinite(correction).all():
        raise ValueError('Invalid base_link XY metre correction')
    if output_from_base is not None:
        correction = np.asarray(output_from_base, float) @ correction
    return np.asarray(point, float)+correction
