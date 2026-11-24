import pytest
from pinky_soccer_viz.clock_monitor import ClockMonitor


@pytest.mark.parametrize('factor', [.4, 1., 1.5])
def test_measured_factor(factor):
    m = ClockMonitor()
    for i in range(301):
        m.add(i*.01*factor, 100+i*.01)
    s = m.snapshot(103.)
    assert s['rtf'] == pytest.approx(factor)
    assert s['clock_hz'] == pytest.approx(100.)
    assert s['clock_state'] == 'ok'


def test_reset_stale_and_recovery():
    m = ClockMonitor()
    assert m.snapshot(100.)['clock_state'] == 'waiting'
    for i in range(100):
        m.add(i*.01,100+i*.01)
    assert m.snapshot(102.)['clock_state'] == 'stale'
    m.add(0.,102.)
    assert m.snapshot(102.)['rtf'] is None
    for i in range(1,101):
        m.add(i*.005,102+i*.01)
    assert m.snapshot(103.)['rtf'] == pytest.approx(.5)


def test_repeated_sim_time():
    m = ClockMonitor()
    for i in range(101):
        m.add(1.,100+i*.01)
    assert m.snapshot(101.)['rtf'] == 0.
