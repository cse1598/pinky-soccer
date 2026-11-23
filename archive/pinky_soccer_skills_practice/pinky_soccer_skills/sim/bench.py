"""
Skill test bench: run YAML scenarios in headless MuJoCo, score them, plot top views.

    ros2 run pinky_soccer_skills skill_bench                       # all bundled scenarios
    ros2 run pinky_soccer_skills skill_bench scenarios/dribble_straight.yaml --plot out/
    ros2 run pinky_soccer_skills skill_bench dribble_straight --set dribble.tap_speed=0.15,0.2

Ground truth is used only for scoring and plots; skills see Perception output only.
"""

import argparse
from copy import deepcopy
import itertools
import json
import math
from pathlib import Path
import sys
import time

import yaml

from .. import params as params_mod
from ..context import FieldInfo, RUNNING
from ..geometry import dist, Polyline, Pose
from ..motion import ball_clearance, Limiter
from ..skills import SKILLS
from .perception import NoiseModel, Perception  # noqa: I100
from .world import OPPONENT_SIZE, SoccerWorld

CONTROL_STEPS = 3            # 3 x 10 ms bridge periods -> 33 Hz control


def scenario_dir():
    """Bundled scenarios (source tree or installed share)."""
    here = Path(__file__).resolve().parents[2] / 'scenarios'
    if here.is_dir():
        return here
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('pinky_soccer_skills')) / 'scenarios'


def load_scenarios(names):
    """Resolve names/paths to (path, dict) list; empty = all bundled."""
    base = scenario_dir()
    paths = []
    for n in names or sorted(p.stem for p in base.glob('*.yaml')):
        p = Path(n)
        if not p.suffix:
            p = base / f'{n}.yaml'
        paths.append(p)
    return [(p, yaml.safe_load(p.read_text())) for p in paths]


class Field:
    """Field geometry for a scenario (pinky_soccer_field + SkillContext FieldInfo)."""

    def __init__(self, name, our_goal_color='yellow'):
        from pinky_soccer_field.field_geometry import Field as GeoField
        from .world import _share
        self.geo = GeoField.from_file(str(_share('pinky_soccer_sim') / 'config'
                                          / f'{name}_field.yaml'), our_goal_color)
        self.info = FieldInfo.from_field(self.geo)


def resolve(value, field):
    """Turn symbolic points (opp_goal, own_goal, center) into coordinates."""
    if isinstance(value, str):
        return {'opp_goal': field.info.opp_goal, 'own_goal': field.info.own_goal,
                'center': (0.0, 0.0)}[value]
    if isinstance(value, dict):
        return {k: resolve(v, field) for k, v in value.items()}
    if isinstance(value, list) and value and isinstance(value[0], (list, str)):
        return [resolve(v, field) for v in value]
    if isinstance(value, list):
        return tuple(value)
    return value


def make_task(spec, P, field):
    """Build the skill or behaviour named in the scenario."""
    args = resolve(deepcopy(spec.get('args', {})), field)
    if 'heading_deg' in args:
        args['heading'] = math.radians(args.pop('heading_deg'))
    if 'behavior' in spec:
        from ..playbook import BEHAVIORS
        return BEHAVIORS[spec['behavior']](P, **args)
    return SKILLS[spec['skill']](P, **args)


class Opponent:
    """Scripted kinematic opponent."""

    def __init__(self, spec, field):
        self.s = resolve(deepcopy(spec), field)
        self.pos = tuple(self.s.get('pos', self.s.get('path', [(0, 0)])[0]))
        self.theta = math.radians(self.s.get('theta_deg', 0.0))
        self.path = Polyline(self.s['path']) if 'path' in self.s else None
        self.travel = 0.0

    def update(self, world, i, t, dt):
        """Move for one control period."""
        s = self.s
        delay = s.get('delay', 0.0)
        speed = s.get('speed', 0.1)
        if t >= delay and self.path is not None:
            self.travel = min(self.path.length, self.travel + speed * dt)
            p = self.path.point_at(self.travel)
            self.theta = self.path.heading_at(self.travel)
            self.pos = p
        elif t >= delay and 'dribble_to' in s:
            bx, by, _, _ = world.ball_state()
            tgt = s['dribble_to']
            h = math.atan2(tgt[1] - by, tgt[0] - bx)
            back = OPPONENT_SIZE[0] + world.ball_radius - 0.004
            want = (bx - back * math.cos(h), by - back * math.sin(h))
            d = dist(want, self.pos)
            step = min(d, speed * dt)
            if d > 1e-6:
                self.pos = (self.pos[0] + (want[0] - self.pos[0]) * step / d,
                            self.pos[1] + (want[1] - self.pos[1]) * step / d)
            self.theta = h
        world.set_opponent(i, self.pos[0], self.pos[1], self.theta)


def run(scenario, P=None, plot_dir=None, seed=0, world_cache=None, quiet=True):
    """Run one scenario; return the result dict (and write a plot if plot_dir)."""
    sc = scenario
    field = Field(sc.get('field', 'hardboard'), sc.get('our_goal_color', 'yellow'))
    P = params_mod.load(overrides=sc.get('params')) if P is None else P
    if sc.get('params') and P is not None:
        P = params_mod.load(overrides=_merge_ns(P, sc['params']))
    opps = [Opponent(o, field) for o in sc.get('opponents', [])]
    key = (sc.get('field', 'hardboard'), len(opps))
    world = (world_cache or {}).get(key)
    if world is None:
        world = SoccerWorld(key[0], opponents=key[1])
        if world_cache is not None:
            world_cache[key] = world
    world.mj.mj_resetData(world.model, world.data)
    world.physics.odom[:] = 0.0
    rx, ry, rth = sc['robot']
    world.set_robot(rx, ry, math.radians(rth))
    ball = sc['ball'] if isinstance(sc['ball'], dict) else {'pos': sc['ball']}
    bvel = ball.get('vel', [0.0, 0.0])
    world.set_ball(*resolve(ball['pos'], field), *bvel)
    for i, o in enumerate(opps):
        o.update(world, i, 0.0, 0.0)
    noise = NoiseModel(**sc.get('noise', {}))
    perc = Perception(world, field.info, noise, seed=seed)
    if sc.get('prime_ball', sc['task'].get('skill') not in ('SearchBall', 'RecoverBall')):
        # The robot saw the ball before the scenario starts (e.g. before it reached the bumper).
        bx, by, bvx, bvy = world.ball_state()
        perc.filter.reset((bx, by), world.time, 0.01)
        perc.filter.x[2:] = [bvx, bvy]
        perc.filter.P[2, 2] = perc.filter.P[3, 3] = 0.05 ** 2
    task = make_task(sc['task'], P, field)
    ctrl_dt = CONTROL_STEPS * world.physics.bridge_period
    lim = Limiter(P.robot, ctrl_dt)
    duration = sc.get('duration', 15.0)
    trace = []
    result = {'name': sc.get('name'), 'status': RUNNING, 'reason': '', 'time': None}
    min_clear, contact_time = math.inf, 0.0
    t_wall = time.time()
    t0 = world.time
    expect = sc.get('expect', {})
    settle_until = None
    while world.time - t0 < duration:
        t = world.time - t0
        for i, o in enumerate(opps):
            o.update(world, i, t, ctrl_dt)
        ctx = perc.tick()
        ctx.t = t
        ctx.last_cmd = (lim.v, lim.w)
        out = task.step(ctx)
        v, w = lim(out.v, out.w)
        rp = world.robot_pose()
        bs = world.ball_state()
        clear = ball_clearance(Pose(*rp), bs[:2], P.robot)
        min_clear = min(min_clear, clear)
        if clear < 0.002:
            contact_time += ctrl_dt
        est = ctx.ball
        trace.append({'t': round(t, 3), 'robot': [round(x, 4) for x in rp],
                      'ball': [round(x, 4) for x in bs],
                      'est': None if est is None else [round(est.x, 4), round(est.y, 4),
                                                       est.state],
                      'cmd': [round(v, 3), round(w, 3)], 'status': out.status,
                      'phase': out.phase, 'skill': out.info.get('skill', task.name),
                      'visible': perc.visible_now, 'frame': perc.frame_now,
                      'us': ctx.us_range,
                      'opponents': [world.opponent_xy(i) for i in range(len(opps))]})
        if field.geo.ball_in_goal(bs[0], bs[1], field.geo.opp_goal_color):
            result.update(goal='scored', goal_time=round(t, 2))
            if result['time'] is None:
                result['time'] = t
            if 'behavior' in sc['task'] or expect.get('stop_on_goal', True):
                result['status'] = 'GOAL'
                break
        if field.geo.ball_in_goal(bs[0], bs[1], field.geo.our_goal_color):
            result.update(goal='conceded', goal_time=round(t, 2), status='CONCEDED')
            if result['time'] is None:
                result['time'] = t
            break
        if out.status != RUNNING and 'behavior' not in sc['task'] and settle_until is None:
            result.update(status=out.status, reason=out.reason, time=t, info=_clean(out.info))
            if not expect.get('settle'):
                break
            settle_until = t + expect['settle']   # watch the ball after the skill ended
            task = _Stop()
        if settle_until is not None and t >= settle_until:
            break
        for _ in range(CONTROL_STEPS):
            world.step(v, w)
    if result['time'] is None:
        result['time'] = world.time - t0
    rp, bs = world.robot_pose(), world.ball_state()
    speeds = [abs(tr['cmd'][0]) for tr in trace]
    result['speed_ratio'] = round(sum(speeds) / max(1, len(speeds)) / P.robot.max_lin, 2)
    result['fast_ratio'] = round(sum(v >= 0.8 * P.robot.max_lin - 1e-6 for v in speeds)
                                 / max(1, len(speeds)), 2)
    frames = [tr for tr in trace if tr['frame']]
    result['visible_ratio'] = round(sum(tr['visible'] for tr in frames) / max(1, len(frames)), 2)
    result.update(final_robot=[round(x, 4) for x in rp], final_ball=[round(x, 4) for x in bs],
                  min_clearance=round(min_clear, 4), contact_time=round(contact_time, 2),
                  wall_time=round(time.time() - t_wall, 2))
    result['checks'] = check(expect, result, trace, sc, field)
    result['passed'] = all(c['ok'] for c in result['checks'])
    if plot_dir:
        result['plot'] = str(plot(trace, sc, field, result, Path(plot_dir)))
    if not quiet:
        print(summary_line(result))
    return result, trace


class _Stop:
    """Zero command after a skill finished (used while the ball settles)."""

    name = 'Stop'

    def step(self, ctx):
        from ..context import SkillOutput
        return SkillOutput(0.0, 0.0, 'RUNNING', phase='settle')


def _merge_ns(P, extra):
    base = {k: dict(vars(v)) for k, v in vars(P).items()}
    base['robot'].pop('contact_dist', None)
    return params_mod._merge(base, extra)


def _clean(info):
    out = {}
    for k, v in info.items():
        if isinstance(v, float):
            out[k] = round(v, 4)
        elif isinstance(v, (int, str, bool)) or v is None:
            out[k] = v
    return out


def check(expect, result, trace, sc, field):
    """Evaluate the scenario's expectations against the truth trace."""
    checks = []

    def add(name, ok, value):
        checks.append({'name': name, 'ok': bool(ok), 'value': value})
    if 'status' in expect:
        add('status', result['status'] == expect['status'], result['status'])
    if 'max_time' in expect:
        add('max_time', result['time'] <= expect['max_time'], round(result['time'], 2))
    if 'max_ball_moved' in expect:
        moved = max(dist(tr['ball'][:2], trace[0]['ball'][:2]) for tr in trace)
        add('max_ball_moved', moved <= expect['max_ball_moved'], round(moved, 3))
    if 'min_clearance' in expect:
        add('min_clearance', result['min_clearance'] >= expect['min_clearance'],
            result['min_clearance'])
    if 'max_pos_err' in expect or 'max_heading_err_deg' in expect:
        info = result.get('info', {})
        if 'max_pos_err' in expect:
            add('pos_err', info.get('pos_err', 9) <= expect['max_pos_err'], info.get('pos_err'))
        if 'max_heading_err_deg' in expect:
            e = math.degrees(info.get('heading_err', 9))
            add('heading_err_deg', e <= expect['max_heading_err_deg'], round(e, 2))
    if 'max_ball_offset' in expect or 'max_aim_err_deg' in expect:
        rp, bxy = trace[-1]['robot'], trace[-1]['ball'][:2]
        lx, ly = Pose(*rp).to_local(*bxy)
        if 'max_ball_offset' in expect:
            add('ball_offset', abs(ly) <= expect['max_ball_offset'], round(ly, 4))
        if 'max_aim_err_deg' in expect:
            spec = sc['task']['args']
            aim = resolve(spec['aim'], field) if 'aim' in spec else None
            want = math.atan2(aim[1] - bxy[1], aim[0] - bxy[0]) if aim else \
                math.radians(spec['heading_deg'])
            err = abs(math.degrees(math.atan2(math.sin(want - rp[2]), math.cos(want - rp[2]))))
            add('aim_err_deg', err <= expect['max_aim_err_deg'], round(err, 2))
    if 'ball_near' in expect:
        x, y, tol = resolve(expect['ball_near'], field)
        d = dist(result['final_ball'][:2], (x, y))
        add('ball_near', d <= tol, round(d, 3))
    if 'max_ball_path_dev' in expect:
        path = Polyline(resolve(expect.get('path', sc['task']['args']['path']), field))
        dev = max(abs(path.project(tr['ball'][:2])[1]) for tr in trace)
        add('max_ball_path_dev', dev <= expect['max_ball_path_dev'], round(dev, 3))
    if 'goal' in expect:
        add('goal', result.get('goal') == expect['goal'], result.get('goal'))
    if 'no_concede' in expect:
        add('no_concede', result.get('goal') != 'conceded', result.get('goal'))
    if 'min_ball_speed' in expect:
        peak = 0.0
        for tr in trace:
            peak = max(peak, math.hypot(tr['ball'][2], tr['ball'][3]))
        add('min_ball_speed', peak >= expect['min_ball_speed'], round(peak, 3))
    if 'max_robot_ball_dist' in expect:
        far = max(dist(tr['robot'][:2], tr['ball'][:2]) for tr in trace)
        add('max_robot_ball_dist', far <= expect['max_robot_ball_dist'], round(far, 3))
    return checks


def summary_line(r):
    """One-line human summary."""
    mark = 'PASS' if r['passed'] else 'FAIL'
    fails = [c['name'] for c in r['checks'] if not c['ok']]
    return (f"{mark} {r['name']:<28} {r['status']:<9} t={r['time']:.2f}s "
            f"{r.get('reason', '')} minclr={r['min_clearance']:.3f} vis={r['visible_ratio']:.2f} "
            f"spd={r['speed_ratio']:.2f} fast={r['fast_ratio']:.2f}"
            + (f'  failed={fails}' if fails else ''))


# ---------------------------------------------------------------------------- plotting
def plot(trace, sc, field, result, out_dir):
    """Top-view PNG: field, robot footprints, ball truth/estimate, opponents."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon, Rectangle
    out_dir.mkdir(parents=True, exist_ok=True)
    g = field.geo
    fig, ax = plt.subplots(figsize=(8.4, 5.8), dpi=110)
    walls = g.inner_walls
    ax.add_patch(Rectangle((walls.xmin, walls.ymin), walls.width, walls.height,
                           fc='#2e6b3a', ec='black', lw=2, zorder=0))
    ax.add_patch(Rectangle((-g.half_length, -g.half_width), 2 * g.half_length,
                           2 * g.half_width, fill=False, ec='white', lw=1, zorder=1))
    gw, gd = g.spec['goal_width'], g.spec['goal_depth']
    for color, c in (('yellow', '#e8c31a'), ('blue', '#2f6fdb')):
        s = g.color_sign(color)
        x0 = s * g.half_length
        ax.add_patch(Rectangle((min(x0, x0 + s * gd), -gw / 2), gd, gw, fc=c, ec='white',
                               zorder=1))
    rob = [tr['robot'] for tr in trace]
    ball = [tr['ball'] for tr in trace]
    xs, ys = zip(*[(r[0], r[1]) for r in rob])
    ax.plot(xs, ys, color='#00e5ff', lw=1.6, zorder=3, label='robot path')
    est = [tr['est'] for tr in trace if tr['est']]
    if est:
        ax.plot([e[0] for e in est], [e[1] for e in est], ':', color='#ffd0a0', lw=1.0,
                zorder=4, label='ball estimate')
    ax.plot([b[0] for b in ball], [b[1] for b in ball], color='#ff6a00', lw=2.0, zorder=4,
            label='ball')
    for k, tr in enumerate(trace):
        if k % 15 and k != len(trace) - 1:
            continue
        x, y, th = tr['robot']
        alpha = 0.25 + 0.75 * k / max(1, len(trace) - 1)
        corners = []
        x0, x1, y0, y1 = (-0.075, 0.0415, -0.077, 0.077)
        for lx, ly in ((x0, y0), (x1, y0), (x1, y1), (x0, y1)):
            corners.append((x + lx * math.cos(th) - ly * math.sin(th),
                            y + lx * math.sin(th) + ly * math.cos(th)))
        rev = tr['cmd'][0] < -0.01
        ax.add_patch(Polygon(corners, closed=True, fill=False,
                             ec='#ff4da6' if rev else 'white', lw=0.8, alpha=alpha, zorder=5))
    for i in range(len(trace[0]['opponents'])):
        ox = [tr['opponents'][i][0] for tr in trace]
        oy = [tr['opponents'][i][1] for tr in trace]
        ax.plot(ox, oy, color='#555555', lw=1, zorder=2)
        ax.add_patch(Rectangle((ox[-1] - OPPONENT_SIZE[0], oy[-1] - OPPONENT_SIZE[1]),
                               2 * OPPONENT_SIZE[0], 2 * OPPONENT_SIZE[1], fc='#222222',
                               zorder=3))
    ax.plot(*ball[0][:2], 'o', color='#ff6a00', mec='white', ms=7, zorder=6)
    ax.plot(xs[0], ys[0], 's', color='#00e5ff', mec='white', ms=6, zorder=6)
    args = sc['task'].get('args', {})
    if 'path' in args:
        path = resolve(args['path'], field)
        ax.plot(*zip(*path), '--', color='white', lw=1, zorder=2, label='requested path')
    pad = 0.12
    lo_x = min(min(xs), min(b[0] for b in ball)) - pad
    hi_x = max(max(xs), max(b[0] for b in ball)) + pad
    lo_y = min(min(ys), min(b[1] for b in ball)) - pad
    hi_y = max(max(ys), max(b[1] for b in ball)) + pad
    if sc.get('plot_full', False) or hi_x - lo_x > 1.0:
        lo_x, hi_x, lo_y, hi_y = walls.xmin - 0.03, walls.xmax + 0.12, walls.ymin - 0.03, \
            walls.ymax + 0.03
    ax.set_xlim(lo_x, hi_x)
    ax.set_ylim(lo_y, hi_y)
    ax.set_aspect('equal')
    ax.set_facecolor('#1d3b24')
    ax.legend(loc='upper right', fontsize=7, framealpha=0.6)
    status = 'PASS' if result['passed'] else 'FAIL'
    ax.set_title(f"{sc.get('name')}  [{status}] {result['status']} {result['time']:.2f}s  "
                 '(white=forward, pink=reverse)', fontsize=9)
    path = out_dir / f"{sc.get('name')}.png"
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)
    return path


# ---------------------------------------------------------------------------- CLI
def parse_sets(items):
    """--set a.b=1,2 -> list of override dicts (cartesian product)."""
    axes = []
    for item in items or []:
        key, values = item.split('=', 1)
        sec, name = key.split('.', 1)
        axes.append([(sec, name, yaml.safe_load(v)) for v in values.split(',')])
    combos = []
    for combo in itertools.product(*axes) if axes else [()]:
        ov = {}
        for sec, name, val in combo:
            ov.setdefault(sec, {})[name] = val
        combos.append(ov)
    return combos


def main(argv=None):
    """CLI entry point."""
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('scenarios', nargs='*', help='names or YAML paths (default: all)')
    ap.add_argument('--params', help='skills.yaml to start from')
    ap.add_argument('--set', action='append', help='section.key=v1,v2 (sweep)')
    ap.add_argument('--plot', help='write PNG top views to this directory')
    ap.add_argument('--json', help='write results JSON here')
    ap.add_argument('--seeds', type=int, default=1, help='noise seeds per scenario')
    a = ap.parse_args(argv)
    scenarios = load_scenarios(a.scenarios)
    cache = {}
    results = []
    for ov in parse_sets(a.set):
        P = params_mod.load(a.params, ov)
        if ov:
            print(f'--- params {ov}')
        for path, sc in scenarios:
            for seed in range(a.seeds):
                r, _ = run(sc, P, a.plot if seed == 0 else None, seed, cache)
                r['params'] = ov
                r['seed'] = seed
                print(summary_line(r))
                sys.stdout.flush()
                results.append(r)
    passed = sum(r['passed'] for r in results)
    print(f'{passed}/{len(results)} passed')
    if a.json:
        Path(a.json).write_text(json.dumps(results, indent=1))
    return 0 if passed == len(results) else 1


if __name__ == '__main__':
    sys.exit(main())
