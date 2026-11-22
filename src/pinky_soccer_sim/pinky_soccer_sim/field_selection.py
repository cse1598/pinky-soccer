"""Select a matched simulator world and runtime geometry YAML by field preset."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory


PRESETS = {'rcj': ('rcj_soccer.world', 'rcj_field.yaml'),
           'hardboard': ('hardboard_soccer.world', 'hardboard_field.yaml')}


def field_files(name):
    """Resolve both assets together so geometry and simulation cannot select different fields."""
    if name not in PRESETS:
        raise ValueError(f'Unknown field {name!r}; choose rcj or hardboard')
    share = Path(get_package_share_directory('pinky_soccer_sim'))
    world, spec = PRESETS[name]
    paths = (share/'worlds'/world, share/'config'/spec)
    for path in paths:
        if not path.is_file():
            raise FileNotFoundError(f'Field asset missing: {path}; regenerate and build sim')
    return paths


def selected_world(field, explicit, world_name):
    """Keep legacy world selection when no preset is requested; reject conflicting choices."""
    if not field:
        return None
    world, _ = field_files(field)
    if explicit or world_name not in ('empty.world', world.name):
        raise ValueError('field:=rcj|hardboard conflicts with world/world_name override')
    return world
