from pinky_soccer_viz.sim_ball_reset import ResetGate


def feed(gate, now, states=('READY', 'READY'), speed=0.):
    for name, state in zip(gate.robots, states):
        gate.status[name] = (now, state, True)
        gate.odom[name] = (now, speed, 0.)


def test_goal_waits_for_both_return_and_stop():
    gate = ResetGate(['yellow', 'blue'])
    gate.event(1, 'GOAL', 1.)
    feed(gate, 1.1, ('READY', 'MOVING_TO_START'))
    assert not gate.ready(1.1)
    feed(gate, 2.)
    assert not gate.ready(2.)
    feed(gate, 2.41)
    assert gate.ready(2.41)


def test_out_waits_for_stop_and_rejects_stale_or_invalid_inputs():
    gate = ResetGate([''])
    gate.event(1, 'OUT', 1.)
    feed(gate, 1.1, ('STOPPED',), speed=.1)
    assert not gate.ready(1.1)
    feed(gate, 1.2, ('STOPPED',))
    assert not gate.ready(1.2)
    assert not gate.ready(2.)  # no fresh odometry/status
    feed(gate, 2., ('STOPPED',))
    assert not gate.ready(2.)
    feed(gate, 2.41, ('STOPPED',))
    assert gate.ready(2.41)
    gate.event(1, 'GOAL', 2.42)  # latched duplicate must not replace pending OUT
    assert gate.pending == ('OUT', 1.)
    gate.odom[''] = (2.42, float('nan'), 0.)
    assert not gate.ready(2.42)
