from pinky_soccer_sim.render_backend import renderer_environment


def test_auto_nvidia():
    label, env, simple = renderer_environment('auto', lambda: True)
    assert label == 'NVIDIA PRIME'
    assert env['__NV_PRIME_RENDER_OFFLOAD'] == '1'
    assert env['__GLX_VENDOR_LIBRARY_NAME'] == 'nvidia'
    assert env['LIBGL_ALWAYS_SOFTWARE'] == '0'
    assert not simple


def test_auto_system():
    label, env, simple = renderer_environment('auto', lambda: False)
    assert label == 'system OpenGL'
    assert env['LIBGL_ALWAYS_SOFTWARE'] == '0'
    assert not simple


def test_explicit_cpu_and_nvidia():
    label, env, simple = renderer_environment('false', lambda: True)
    assert simple and env['LIBGL_ALWAYS_SOFTWARE'] == '1'
    assert renderer_environment('nvidia', lambda: False)[0] == 'NVIDIA PRIME'
