"""Select MuJoCo's OpenGL renderer without changing physics."""
import os
import shutil
import subprocess

MODES = ['auto', 'true', 'false', 'nvidia', 'system']
NVIDIA = {'__NV_PRIME_RENDER_OFFLOAD': '1',
          '__GLX_VENDOR_LIBRARY_NAME': 'nvidia',
          'LIBGL_ALWAYS_SOFTWARE': '0', 'MUJOCO_GL': 'glfw'}
SOFTWARE = {'__NV_PRIME_RENDER_OFFLOAD': '0',
            '__GLX_VENDOR_LIBRARY_NAME': 'mesa',
            'LIBGL_ALWAYS_SOFTWARE': '1', 'MUJOCO_GL': 'glfw'}
SYSTEM = {'LIBGL_ALWAYS_SOFTWARE': '0', 'MUJOCO_GL': 'glfw'}


def nvidia_available():
    """Check the actual GLX offload path, not just presence of a CUDA driver."""
    if not shutil.which('glxinfo') or not os.environ.get('DISPLAY'):
        return False
    try:
        result = subprocess.run(['glxinfo', '-B'], env={**os.environ, **NVIDIA},
                                capture_output=True, text=True, timeout=5, check=False)
        return (result.returncode == 0 and
                any('nvidia' in line.lower() for line in result.stdout.splitlines()
                    if line.startswith('OpenGL renderer string:')))
    except (OSError, subprocess.TimeoutExpired):
        return False


def renderer_environment(mode, probe=nvidia_available):
    """Return (label, child environment, simple_render). No global env mutation."""
    if mode not in MODES:
        raise ValueError(f'Unknown renderer mode: {mode}')
    if mode == 'false':
        return 'software (CPU)', dict(SOFTWARE), True
    if mode == 'nvidia' or (mode in ('auto', 'true') and probe()):
        return 'NVIDIA PRIME', dict(NVIDIA), False
    # System default may be Intel/AMD/NVIDIA or Mesa software.
    return 'system OpenGL', dict(SYSTEM), False
