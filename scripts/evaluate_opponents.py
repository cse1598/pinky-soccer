#!/usr/bin/env python3
"""Read-only synthetic scan evaluation. No truth is supplied to the tracker.

Source ROS/workspace, then .venv/bin/python scripts/evaluate_opponents.py --output DIR.
Synthetic ground truth is used only to generate scans and measure error. Surface means
and robot axle centres differ: both errors are explicitly reported.
"""
import argparse
import importlib.util
import json
import math
from pathlib import Path
import time

import numpy as np
from pinky_soccer_field.field_geometry import Field
from pinky_soccer_world_model.opponents import (
    SingleOpponentTracker, indexed_candidates, scan_points)

ROOT = Path(__file__).resolve().parents[1]


def box_ranges(origin, directions, box, yaw=0., centre=(0., 0.)):
    c, s = math.cos(yaw), math.sin(yaw)
    rotation = np.array([[c, -s], [s, c]])
    o = (origin-np.asarray(centre)) @ rotation
    d = directions @ rotation
    lo, hi = np.asarray(box[:2]), np.asarray(box[2:])
    safe = np.where(abs(d) > 1e-12, d, 1e-12)
    a, b = (lo-o)/safe, (hi-o)/safe
    near = np.maximum(np.minimum(a, b).max(axis=1), 0.)
    far = np.maximum(a, b).min(axis=1)
    return np.where(far >= near, np.where(near > 0, near, far), np.inf)


def scan(field, sensor, opponent, n=720, rng=None, noise=.002):
    angles = -np.pi+np.arange(n)*2*np.pi/n
    world_angles = angles+sensor[2]
    d = np.column_stack((np.cos(world_angles), np.sin(world_angles)))
    origin = np.asarray(sensor[:2])
    r = np.full(n, np.inf)
    for obj in field.obstacles_seen_at(.125):
        if 'roof' not in obj.name:
            b = obj.rect
            r = np.minimum(r, box_ranges(origin, d, (b.xmin, b.ymin, b.xmax, b.ymax)))
    if opponent is not None:
        r = np.minimum(r, box_ranges(origin, d, (-.076, -.0766, .0421, .0766),
                                     opponent[2], opponent[:2]))
    if rng is not None:
        r += rng.normal(0, noise, n)
    return np.where((r >= .05) & (r < 10), r, np.inf)


def render_examples(output):
    """Render the actual shared UI with fixed radius and contrasting velocity arrows."""
    import cv2
    from pinky_soccer_msgs.msg import WorldModel, TrackedObject
    from pinky_soccer_viz.topview import TopView
    field = Field.from_file(ROOT/'src/pinky_soccer_sim/config/hardboard_field.yaml')
    view = TopView(field, 180.)
    images = []
    for label, vx, state in [('Stopped: no arrow', 0., 0), ('Forward: 0.10 m/s', .1, 0),
                             ('Reverse / estimated: 0.20 m/s', -.2, 1)]:
        w = WorldModel()
        w.header.stamp.sec = 10
        w.self.pose.x, w.self.pose.theta, w.self.confidence = -.4, .4, 1.
        w.ball.state = 2
        o = TrackedObject()
        o.id, o.pose.x, o.pose.y = 'opp_0', .25, .1
        o.velocity.x, o.confidence, o.state, o.last_seen.sec = vx, 1., state, 10
        w.opponents = [o]
        image = view.render(w)[:view.h, :view.w].copy()
        cv2.putText(image, label, (8, 23), cv2.FONT_HERSHEY_SIMPLEX, .43,
                    (255, 255, 255), 1, cv2.LINE_AA)
        images.append(image)
    cv2.imwrite(str(output), np.concatenate(images, axis=1))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    spec = importlib.util.spec_from_file_location('old_opponents', ROOT/
        'docs/validation/opponents_20261127/baseline_opponents.py')
    old = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(old)
    field = Field.from_file(ROOT/'src/pinky_soccer_sim/config/hardboard_field.yaml')
    structures = [o.rect for o in field.obstacles_seen_at(.125) if 'roof' not in o.name]
    scenarios = ['empty', 'empty_noise', 'stationary', 'forward', 'reverse',
                 'self_translate', 'self_rotate', 'opponent_rotate', 'near_wall',
                 'near_goal', 'sparse_360', 'temporary_loss', 'empty_rcj']
    result = {}
    for name in scenarios:
        field = Field.from_file(ROOT/('src/pinky_soccer_sim/config/rcj_field.yaml'
                                     if name == 'empty_rcj' else
                                     'src/pinky_soccer_sim/config/hardboard_field.yaml'))
        structures = [o.rect for o in field.obstacles_seen_at(.125) if 'roof' not in o.name]
        rng = np.random.default_rng(20261127)
        tracker = SingleOpponentTracker()
        rows, elapsed_old, elapsed_new, cpu_old, cpu_new = [], [], [], [], []
        cpu_start, wall_start = time.process_time(), time.perf_counter()
        for i in range(100):
            t = 1+i*.1
            sensor, opponent = [-.4, -.1, .0], [.25, .05, 0.]
            if name.startswith('empty'):
                opponent = None
            elif name == 'forward':
                opponent[1] = -.3+.06*(t-1)
            elif name == 'reverse':
                opponent[1] = .3-.06*(t-1)
            elif name == 'self_translate':
                sensor[1] += .04*(t-1)
            elif name == 'self_rotate':
                sensor[2] = .25*(t-1)
            elif name == 'opponent_rotate':
                opponent[2] = .5*(t-1)
            elif name == 'near_wall':
                opponent[1] = .48
            elif name == 'near_goal':
                opponent[:2] = [.74, .3]
            n = 360 if name == 'sparse_360' else 720
            r = scan(field, sensor, opponent, n, rng)
            if name == 'temporary_loss' and 35 <= i < 40:
                r[:] = np.inf
            if name == 'empty_noise' and i % 7 == 0:
                # Isolated 1/2-beam transients, not an invented solid opponent.
                r[300:302] = .4
            c, s = math.cos(sensor[2]), math.sin(sensor[2])
            R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
            translation = np.array([*sensor[:2], .125])
            start = time.perf_counter_ns()
            cpu_mark = time.process_time_ns()
            points = scan_points(r, -np.pi, 2*np.pi/n, .05, 10.)
            world = points @ R[:2, :2].T+translation[:2]
            legacy = old.cluster_opponents(world, field.inner_walls)
            elapsed_old.append((time.perf_counter_ns()-start)/1e6)
            cpu_old.append((time.process_time_ns()-cpu_mark)/1e6)
            start = time.perf_counter_ns()
            cpu_mark = time.process_time_ns()
            candidates = indexed_candidates(r, -np.pi, 2*np.pi/n, .05, 10.,
                R, translation, field.inner_walls, structures)
            tracker.update(candidates, t)
            sample = tracker.sample(t)
            elapsed_new.append((time.perf_counter_ns()-start)/1e6)
            cpu_new.append((time.process_time_ns()-cpu_mark)/1e6)
            rows.append(dict(t=t, old_count=len(legacy), candidate_count=len(candidates),
                opponent=opponent, position=sample['position'].tolist() if sample else None,
                velocity=sample['velocity'].tolist() if sample else None,
                state=sample['state'] if sample else 2,
                surface=candidates[0].xy.tolist() if len(candidates) == 1 else None))
        def timing(values):
            return dict(mean_ms=float(np.mean(values)), p95_ms=float(np.percentile(values, 95)),
                        max_ms=float(np.max(values)))
        valid = [x for x in rows[15:] if x['position'] is not None and x['opponent'] is not None]
        moving = [0., .06] if name == 'forward' else [0., -.06] if name == 'reverse' else [0., 0.]
        result[name] = dict(rows=rows, old_timing=timing(elapsed_old), new_timing=timing(elapsed_new),
            old_cpu_mean_ms=float(np.mean(cpu_old)), new_cpu_mean_ms=float(np.mean(cpu_new)),
            harness_cpu_s=time.process_time()-cpu_start, harness_wall_s=time.perf_counter()-wall_start,
            old_false_positive_scan_fraction=float(np.mean([x['old_count'] > 0 for x in rows]))
                if name.startswith('empty') else None,
            new_false_positive_scan_fraction=float(np.mean([x['position'] is not None for x in rows]))
                if name.startswith('empty') else None,
            old_multiple_scan_count=sum(x['old_count'] > 1 for x in rows),
            confirmed_fraction=float(np.mean([x['position'] is not None for x in rows])),
            first_confirmation_s=next((x['t']-1 for x in rows if x['position'] is not None), None),
            centre_rmse_m=float(np.sqrt(np.mean([np.sum((np.asarray(x['position'])-
                np.asarray(x['opponent'][:2]))**2) for x in valid]))) if valid else None,
            surface_rmse_m=float(np.sqrt(np.mean([np.sum((np.asarray(x['position'])-
                x['surface'])**2) for x in valid if x['surface'] is not None]))) if valid else None,
            velocity_rmse_mps=float(np.sqrt(np.mean([np.sum((np.asarray(x['velocity'])-
                moving)**2) for x in valid]))) if valid else None,
            speed_p95_mps=float(np.percentile([np.linalg.norm(x['velocity']) for x in valid], 95))
                if valid else None)
    (args.output/'synthetic.json').write_text(json.dumps(result, indent=2))
    print(json.dumps({k: {a: b for a, b in v.items() if a != 'rows'} for k, v in result.items()}, indent=2))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), constrained_layout=True)
    names = list(result)
    x = np.arange(len(names))
    axes[0].bar(x-.18, [result[n]['old_timing']['mean_ms'] for n in names], .36, label='before')
    axes[0].bar(x+.18, [result[n]['new_timing']['mean_ms'] for n in names], .36, label='after + tracking')
    axes[0].set(ylabel='Mean processing time [ms]', xticks=x, xticklabels=names)
    axes[0].tick_params(axis='x', rotation=35)
    axes[0].legend()
    for n in ('stationary', 'forward', 'reverse', 'self_translate', 'opponent_rotate'):
        rows = [r for r in result[n]['rows'] if r['velocity'] is not None]
        axes[1].plot([r['t'] for r in rows], [np.linalg.norm(r['velocity']) for r in rows], label=n)
    axes[1].axhline(.02, color='k', linestyle='--', label='UI arrow threshold')
    axes[1].set(xlabel='Measurement time [s]', ylabel='Estimated speed [m/s]')
    axes[1].legend(ncol=3)
    fig.savefig(args.output/'timing_speed.png', dpi=140)
    render_examples(args.output/'ui.png')


if __name__ == '__main__':
    main()
