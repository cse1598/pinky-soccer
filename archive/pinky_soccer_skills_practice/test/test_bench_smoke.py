"""Run two short scenarios through the headless MuJoCo bench (skipped without MuJoCo)."""

import pytest

pytest.importorskip('mujoco')
pytest.importorskip('pinky_mujoco')


@pytest.mark.parametrize('name', ['recover_contact', 'approach_direct'])
def test_bench_scenario_passes(name):
    from pinky_soccer_skills.sim import bench
    (_, sc), = bench.load_scenarios([name])
    result, trace = bench.run(sc)
    assert result['passed'], bench.summary_line(result)
    assert trace and all('robot' in tr for tr in trace)
