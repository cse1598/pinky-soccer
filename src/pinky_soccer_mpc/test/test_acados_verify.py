"""acados toolchain check: compat layer, code generation, compile and solve (status 0)."""

import pytest

pytest.importorskip('casadi')


def test_numpy_compat_is_minimal():
    import numpy as np
    from pinky_soccer_mpc.acados_setup import numpy_compat
    numpy_compat()
    assert np.bool is np.bool_            # NumPy 2 definition, nothing else changed


def test_acados_found_without_env(monkeypatch):
    from pinky_soccer_mpc import acados_setup
    monkeypatch.delenv('ACADOS_SOURCE_DIR', raising=False)
    root = acados_setup.find_acados_root()
    assert (root / 'lib' / 'libacados.so').is_file()


def test_wheel_model_ocp_solves_with_status_zero():
    try:
        from pinky_soccer_mpc.acados_setup import acados
        acados()
    except (RuntimeError, ImportError) as exc:
        pytest.skip(f'acados unavailable: {exc}')
    from pinky_soccer_mpc.verify import run
    result = run()
    assert result['status'] == 0, result
    assert result['passed'], result['checks']
