"""GoToPose optional collision constraints: geometry, OFF-path identity, closed loop."""
import math

import numpy as np
import pytest

from pinky_soccer_mpc import collision_geometry as cg
from pinky_soccer_mpc import obstacle_snapshot as osn

pytest.importorskip('casadi')


@pytest.fixture(scope='module')
def walls():
    from pinky_soccer_field.field_geometry import Field
    from pinky_soccer_sim.field_selection import field_files
    return cg.FieldObstacles(Field.from_file(str(field_files('hardboard')[1])))


def test_discs_cover_exact_footprint():
    pts = []
    for x0, y0, x1, y1 in cg.FOOTPRINT_RECTS:
        for t in np.linspace(0, 1, 41):
            pts += [(x0 + t * (x1 - x0), y0), (x0 + t * (x1 - x0), y1),
                    (x0, y0 + t * (y1 - y0)), (x1, y0 + t * (y1 - y0))]
    d = np.asarray(cg.FOOTPRINT_DISCS)
    for p in pts:
        assert np.min(np.hypot(d[:, 0] - p[0], d[:, 1] - p[1]) - d[:, 2]) <= 1e-9, p


def test_vectorized_clearance_matches_reference(walls):
    rng = np.random.default_rng(3)
    poses = np.c_[rng.uniform(-0.95, 0.95, 1500), rng.uniform(-0.65, 0.65, 1500),
                  rng.uniform(-3.2, 3.2, 1500)]
    ref = np.array([walls.clearance(cg.footprint_polygons(p)) for p in poses])
    fast = osn.wall_clearance(poses, walls)
    near = ref < 0.15
    assert np.max(np.abs(fast - ref)[near]) < 1e-9
    assert np.all(fast <= ref + 1e-12)          # beyond 15 cm a lower bound only
    assert np.sum(ref < 0) > 100                # the sample contains collisions
    c = (0.1, -0.05)
    exact = np.array([cg.circle_clearance(cg.footprint_polygons(p), c, 0.) for p in poses])
    assert np.max(np.abs(osn.circle_clearance(poses, c) - exact)) < 1e-9


def test_goal_block_contact_is_detected(walls):
    # Footprint at heading -90 deg reaches x = 0.76 + 0.0766 > goal line 0.83.
    _, clear = osn.ObstacleSnapshot(osn.AvoidanceOptions(walls=True), osn.AvoidanceConfig(),
                                    walls).clearances((0.76, 0.0, -math.pi / 2))
    assert clear[0, 0] < 0
    _, clear = osn.ObstacleSnapshot(osn.AvoidanceOptions(walls=True), osn.AvoidanceConfig(),
                                    walls).clearances((0.70, 0.0, -math.pi / 2))
    assert clear[0, 0] > 0.02


def test_snapshot_uses_only_enabled_options():
    class Obj:
        def __init__(self, x, y, state=0):
            self.pose = type('P', (), dict(x=x, y=y, theta=0.))()
            self.state, self.covariance = state, [0.] * 9

    world = type('W', (), dict(self=Obj(0, 0), ball=Obj(.3, 0),
                               opponents=[Obj(.5, 0), Obj(.2, 0), Obj(.1, 0, osn.LOST)]))()
    cfg = osn.AvoidanceConfig()
    snap = osn.take(osn.AvoidanceOptions(opponents=True), cfg, None, world)
    assert [c.kind for c in snap.circles] == ['opponent', 'opponent']
    assert snap.circles[0].x == pytest.approx(.2)        # nearest first, LOST ignored
    assert snap.walls is None
    snap = osn.take(osn.AvoidanceOptions(ball=True), cfg, None, world)
    assert [c.kind for c in snap.circles] == ['ball']
    assert not osn.take(osn.AvoidanceOptions(), cfg, None, world).circles


@pytest.fixture(scope='module')
def solvers():
    pytest.importorskip('scipy')
    from pinky_soccer_dynamics import params as P
    from pinky_soccer_mpc.acados_time_optimal import AcadosTimeOptimalMpc
    from pinky_soccer_mpc.avoidance_mpc import AvoidanceTimeOptimalMpc
    p = P.load('mujoco')
    return p, AcadosTimeOptimalMpc(p, damping_scale=2.), AvoidanceTimeOptimalMpc(
        p, damping_scale=2.)


def test_base_hooks_are_identity(solvers):
    _, base, _ = solvers
    p, lo, hi = np.arange(9.), -np.ones(4), np.ones(4)
    assert base.stage_parameters(3, p) is p
    assert base.stage_bounds(3, lo, hi) == (lo, hi)
    sl, su = np.zeros(11), np.ones(11)
    assert base.terminal_slack(sl, su) == (sl, su)
    assert base.ocp.model.name.startswith('pinky_time_')


def closed_loop(p, m, start, goal, snap):
    from pinky_soccer_dynamics import profile_model as plant
    from pinky_soccer_mpc.arrival import Arrival
    m.reset()
    x = np.zeros(9)
    x[:3] = start
    arrival = Arrival()
    worst = math.inf
    for i in range(120):
        v = (m.rl * x[3] + m.rr * x[4]) / 2
        w = (m.rr * x[4] - m.rl * x[3]) / m.b
        inside, done, _, _ = arrival.update(i * m.dt, x[:3], goal, v, w, x[3:5], True)
        if done:
            return True, worst
        u = np.zeros(2)
        if not inside:
            r = m.solve(np.r_[x[:7], x[7:9]], goal, None, 2., obstacles=snap)
            if r['ok']:
                u = np.asarray(r['u'])
        nxt = plant.step(p, x, u, m.dt)
        _, c = snap.clearances(cg.interpolate_poses(np.vstack([x[:3], nxt[:3]])))
        worst = min(worst, float(np.min(c)))
        x = nxt
    return False, worst


@pytest.mark.parametrize('kind,start,goal,circle', [
    ('opponents', (-.4, 0., 0.), (.4, 0., 0.), (0., 0., .10, 'opponent')),
    ('ball', (-.4, 0., 0.), (.4, 0., 0.), (0., 0., .02135, 'ball')),
    ('walls', (.70, .40, 0.), (.70, -.40, 0.), None)])
def test_closed_loop_keeps_margin(solvers, walls, kind, start, goal, circle):
    p, _, m = solvers
    opts = osn.AvoidanceOptions(**{kind: True})
    snap = osn.ObstacleSnapshot(opts, osn.AvoidanceConfig(), walls if opts.walls else None)
    if circle:
        snap.circles = [osn.Circle(*circle)]
    done, worst = closed_loop(p, m, start, goal, snap)
    assert done
    assert worst >= snap.config.margin - 2e-3      # plant replay, 5 sub-samples per step


def test_unconfirmed_obstacles_have_independent_input_and_legacy_fallback():
    from pinky_soccer_mpc.mpc_controller import MpcController
    import time
    # Constructor-free input selection: no solver execution or safety parameter changes.
    node = MpcController.__new__(MpcController)
    node.world_msg = object()
    node.obstacle_world = None
    node.obstacle_world_received = None
    assert node.avoidance_world() is node.world_msg
    raw = object()
    node.on_obstacle_world(raw)
    assert node.avoidance_world() is raw
    node.obstacle_world_received = time.monotonic()-1
    assert node.avoidance_world() is node.world_msg
