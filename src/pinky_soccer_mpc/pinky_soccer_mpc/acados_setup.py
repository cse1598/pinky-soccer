"""
Project entry point to acados: import acados_template only through `acados()`.

It makes the vendored acados (git submodule `external/acados`) usable with the system
NumPy of Ubuntu 24.04 / ROS 2 Jazzy without editing the submodule or exporting variables:

1. Locate acados: $ACADOS_SOURCE_DIR, else `external/acados` above this file (works for the
   source tree and for colcon --symlink-install), else ~/pinky-soccer/external/acados.
   Sets ACADOS_SOURCE_DIR / ACADOS_INSTALL_DIR for acados_template (t_renderer, headers).
2. NumPy compatibility: acados_template (acados_ocp.py, upstream main c15f50a9a,
   2026-11-22) uses `np.bool`, which exists only in NumPy >= 2.0 (alias of `np.bool_`).
   Jazzy ships NumPy 1.26, where the alias was removed. When - and only when - the attribute
   is missing we add `np.bool = np.bool_`, exactly the NumPy 2 definition. Nothing else in
   NumPy is touched. Remove this once acados uses `np.bool_` or NumPy >= 2 is the baseline.
3. Preload libblasfeo/libhpipm/libqpOASES_e/libacados with RTLD_GLOBAL so generated solver
   libraries resolve them without LD_LIBRARY_PATH.

Generated C code goes to a cache directory (never into the source tree).
"""

import ctypes
import os
from pathlib import Path

_LIBS = ('libblasfeo.so', 'libhpipm.so', 'libqpOASES_e.so', 'libacados.so')
_STATE = {}


def find_acados_root():
    """Return the acados source/install root (contains lib/ and interfaces/)."""
    candidates = []
    env = os.environ.get('ACADOS_SOURCE_DIR')
    if env:
        candidates.append(Path(env))
    here = Path(__file__).resolve()
    candidates += [parent / 'external' / 'acados' for parent in here.parents]
    candidates.append(Path.home() / 'pinky-soccer' / 'external' / 'acados')
    for root in candidates:
        if (root / 'lib' / 'libacados.so').is_file() and (root / 'interfaces').is_dir():
            return root
    raise RuntimeError('acados not found: build external/acados or set ACADOS_SOURCE_DIR')


def numpy_compat():
    """Add np.bool for NumPy < 2 (see module docstring). Return True when applied."""
    import numpy as np
    if 'bool' in np.__dict__:
        return False
    np.bool = np.bool_
    return True


def preload(root):
    """Load acados shared libraries globally; return the list loaded."""
    loaded = []
    for name in _LIBS:
        path = root / 'lib' / name
        if path.is_file():
            ctypes.CDLL(str(path), mode=ctypes.RTLD_GLOBAL)
            loaded.append(name)
    return loaded


def cache_dir(sub=''):
    """Directory for generated solvers (outside the repository)."""
    base = Path(os.environ.get('PINKY_MPC_CACHE', Path.home() / '.cache' / 'pinky_soccer_mpc'))
    path = base / sub if sub else base
    path.mkdir(parents=True, exist_ok=True)
    return path


def acados():
    """Prepare the environment once and return the acados_template module."""
    if 'module' not in _STATE:
        root = find_acados_root()
        os.environ.setdefault('ACADOS_SOURCE_DIR', str(root))
        os.environ.setdefault('ACADOS_INSTALL_DIR', str(root))
        _STATE['numpy_compat'] = numpy_compat()
        _STATE['preloaded'] = preload(root)
        _STATE['root'] = root
        import acados_template
        _STATE['module'] = acados_template
    return _STATE['module']


def info():
    """Return what acados() set up (for logs and tests)."""
    acados()
    import numpy as np
    return {'root': str(_STATE['root']), 'numpy': np.__version__,
            'numpy_compat_applied': _STATE['numpy_compat'], 'preloaded': _STATE['preloaded']}
