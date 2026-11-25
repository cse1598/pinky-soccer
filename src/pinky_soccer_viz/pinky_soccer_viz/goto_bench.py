"""
Same-condition GoToPose bench: drives a fixed tour through bt_node verify_mode.

Each leg starts where the previous leg settled (2 cm / 3 deg), so every configuration sees
the same start and target poses. A warm-up leg to the first tour pose is not recorded.
Needs game.launch.py ... verify:=true (bt_node verify_mode). Records the simulator truth
(evaluation only) next to the estimate and the NMPC diagnostics, and writes
<out_dir>/<label>_<time>/summary.csv, legs/leg_<n>.csv and config.json.

  ros2 run pinky_soccer_viz goto_bench --ros-args -p use_sim_time:=true -p label:=B2 \
      -p repeats:=3 -p avoid_walls:=true
"""

import csv
import json
import math
import os
import threading
import time

from geometry_msgs.msg import PoseStamped
from pinky_soccer_msgs.msg import WorldModel
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

TERMINAL = ('success', 'failure', 'stopped', 'timeout', 'world_model_stale', 'error')
POS_TOL, ANG_TOL = 0.02, math.radians(3)
# Default tour (map frame, hardboard field): forward, in-place 180, combined, reverse-ish,
# diagonal, 5 cm short move, near the -X/-Y corner, back to the kickoff pose.
DEFAULT_TOUR = [-0.4, 0.0, 0.0,
                0.2, 0.0, 0.0,
                0.2, 0.0, math.pi,
                -0.2, 0.3, math.pi / 2,
                -0.2, -0.2, math.pi / 2,
                0.4, -0.3, -0.5,
                0.45, -0.3, -0.5,
                -0.70, -0.45, math.pi]
SAMPLE_KEYS = ('state', 'reason', 'ms', 'solve_id', 'active_solve_id', 'horizon_steps',
               'terminal_feasible', 'activation_missed', 'replay_violation', 'damping_cost',
               'wheel_l', 'wheel_r', 'wheel_cmd_l', 'wheel_cmd_r', 'avoidance',
               'min_clearance_m', 'acados_ms', 'worker_wait_ms', 'result_pickup_ms',
               'solve_input_age_wall_s', 'activation_jitter_sim_s', 'period_s',
               'wheel_source_age_s', 'pose_source_age_s', 'pose_receipt_age_sim_s',
               'feedback_age', 'activation_position_error_m',
               'activation_aligned_position_error_m', 'activation_heading_error_rad',
               'activation_aligned_heading_error_rad', 'observer_d_l', 'observer_d_r',
               'tracking', 'init_missed', 'activation_first_s', 'attempt_statuses',
               'lower_steps', 'status', 'message')


def wrap(a):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def yaw(q):
    """Yaw of a geometry_msgs quaternion."""
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def percentile(values, q):
    """Nearest-rank percentile (None for an empty list)."""
    if not values:
        return None
    v = sorted(values)
    return v[min(len(v) - 1, max(0, math.ceil(q / 100 * len(v)) - 1))]


def leg_metrics(samples, goal, state, elapsed):
    """Truth-based arrival metrics of one leg (ignores samples without truth)."""
    m = dict(result=state, elapsed_s=elapsed)
    rows = [s for s in samples if s.get('true_x') is not None]
    if not rows:
        return m
    t0 = rows[0]['t']
    first = None
    max_head = max_pos = 0.0
    for s in rows:
        d = math.hypot(s['true_x'] - goal[0], s['true_y'] - goal[1])
        a = abs(wrap(s['true_th'] - goal[2]))
        if first is None and d <= POS_TOL and a <= ANG_TOL:
            first = s['t'] - t0
        if first is not None:
            max_head, max_pos = max(max_head, a), max(max_pos, d)
    last = rows[-1]
    m.update(final_pos_cm=100 * math.hypot(last['true_x'] - goal[0], last['true_y'] - goal[1]),
             final_ang_deg=math.degrees(abs(wrap(last['true_th'] - goal[2]))),
             first_entry_s=first, settle_s=(elapsed - first) if first is not None else None,
             reexit_head_deg=math.degrees(max_head) if first is not None else None,
             reexit_pos_cm=100 * max_pos if first is not None else None,
             path_m=sum(math.hypot(b['true_x'] - a['true_x'], b['true_y'] - a['true_y'])
                        for a, b in zip(rows, rows[1:])))
    solves, acados, waits, pickups, act, act_al = {}, {}, {}, {}, {}, {}
    for s in samples:
        sid = s.get('solve_id')
        if sid is not None and s.get('ms') is not None:
            solves[sid] = s['ms']
            for store, key in ((acados, 'acados_ms'), (waits, 'worker_wait_ms'),
                               (pickups, 'result_pickup_ms')):
                if s.get(key) is not None:
                    store[sid] = s[key]
        aid = s.get('active_solve_id')
        if aid is not None and s.get('activation_position_error_m') is not None:
            act[aid] = s['activation_position_error_m']
            if s.get('activation_aligned_position_error_m') is not None:
                act_al[aid] = s['activation_aligned_position_error_m']
    ms = list(solves.values())
    track = [s['t'] for s in samples if s.get('state') == 'track']
    est_err = [math.hypot(s['est_x'] - s['true_x'], s['est_y'] - s['true_y'])
               for s in rows if s.get('est_x') is not None]
    est_head = [abs(wrap(s['est_th'] - s['true_th'])) for s in rows
                if s.get('est_th') is not None]
    periods = [s['period_s'] for s in samples if s.get('period_s') is not None]
    walls = [s['wall'] for s in samples]
    sims = [s['t'] for s in samples]
    firsts = [s['activation_first_s'] for s in samples if s.get('activation_first_s') is not None]
    inits = [s['init_missed'] for s in samples if s.get('init_missed') is not None]
    m.update(first_command_s=(track[0] - samples[0]['t']) if track else None,
             init_missed=max(inits) if inits else None,
             first_activation_s=firsts[0] if firsts else None,
             est_err_mean_cm=100 * sum(est_err) / len(est_err) if est_err else None,
             est_err_max_cm=100 * max(est_err) if est_err else None,
             est_head_max_deg=math.degrees(max(est_head)) if est_head else None,
             ms_p99=percentile(ms, 99),
             acados_p50=percentile(list(acados.values()), 50),
             acados_p95=percentile(list(acados.values()), 95),
             acados_max=max(acados.values()) if acados else None,
             wait_p95=percentile(list(waits.values()), 95),
             pickup_p95=percentile(list(pickups.values()), 95),
             act_err_mm=1000 * sum(act.values()) / len(act) if act else None,
             act_aligned_err_mm=1000 * sum(act_al.values()) / len(act_al) if act_al else None,
             tick_p95_ms=1000 * percentile(periods, 95) if periods else None,
             tick_max_ms=1000 * max(periods) if periods else None,
             rtf=(sims[-1] - sims[0]) / (walls[-1] - walls[0])
             if len(walls) > 1 and walls[-1] > walls[0] else None)
    missed = sum(1 for a, b in zip(samples, samples[1:])
                 if b.get('activation_missed') and not a.get('activation_missed'))
    clear = [s['min_clearance_m'] for s in samples if s.get('min_clearance_m') is not None]
    m.update(solves=len(ms), ms_p50=percentile(ms, 50), ms_p95=percentile(ms, 95),
             ms_max=max(ms) if ms else None, activation_missed=missed,
             min_clearance_m=min(clear) if clear else None)
    return m


class GotoBench(Node):
    """Run the tour and record each leg."""

    def __init__(self):
        super().__init__('goto_bench')
        p = self.declare_parameter
        flat = list(p('tour', DEFAULT_TOUR).value)
        if len(flat) < 6 or len(flat) % 3:
            raise ValueError('tour needs at least two x, y, theta triples')
        self.tour = [tuple(flat[i:i + 3]) for i in range(0, len(flat), 3)]
        self.repeats = int(p('repeats', 3).value)
        self.ports = json.loads(p('ports', '{}').value or '{}')
        for name in ('avoid_walls', 'avoid_opponents', 'avoid_ball'):
            if p(name, False).value:
                self.ports[name] = 'true'
        self.label = p('label', 'bench').value
        self.timeout = float(p('timeout_sec', 30.0).value)
        self.pause = float(p('pause_sec', 1.0).value)
        root = os.path.expanduser(p('out_dir', '~/.ros/pinky_soccer/bench').value)
        self.dir = os.path.join(root, f'{self.label}_{time.strftime("%Y%m%d_%H%M%S")}')
        self.lock = threading.Lock()
        self.status = None
        self.truth = None
        self.est = None
        self.samples = None
        self.req = self.create_publisher(String, 'behavior_test/request', 10)
        self.create_subscription(String, 'behavior_test/status', self.on_status, 10)
        self.create_subscription(PoseStamped, 'ground_truth', self.on_truth, 10)
        self.create_subscription(WorldModel, 'world_model', self.on_world, 10)
        self.create_subscription(String, 'mpc/status', self.on_mpc, 10)
        self.create_timer(0.5, lambda: self.send({'command': 'ping'}))
        self.request_id = 9000

    def send(self, data):
        """Publish one JSON request."""
        self.req.publish(String(data=json.dumps(data)))

    def on_status(self, msg):
        with self.lock:
            self.status = json.loads(msg.data)

    def on_truth(self, msg):
        with self.lock:
            self.truth = (msg.pose.position.x, msg.pose.position.y, yaw(msg.pose.orientation))

    def on_world(self, msg):
        with self.lock:
            s = msg.self.pose
            self.est = (s.x, s.y, s.theta)

    def on_mpc(self, msg):
        data = json.loads(msg.data)
        with self.lock:
            if self.samples is None:
                return
            row = {'t': self.get_clock().now().nanoseconds * 1e-9, 'wall': time.monotonic()}
            for k, v in zip(('true_x', 'true_y', 'true_th'), self.truth or (None,) * 3):
                row[k] = v
            for k, v in zip(('est_x', 'est_y', 'est_th'), self.est or (None,) * 3):
                row[k] = v
            for k in SAMPLE_KEYS:
                v = data.get(k)
                row[k] = json.dumps(v) if isinstance(v, (dict, list)) else v
            self.samples.append(row)

    def wait(self, predicate, limit):
        end = time.monotonic() + limit
        while time.monotonic() < end and rclpy.ok():
            with self.lock:
                if predicate():
                    return True
            time.sleep(0.02)
        return False

    def leg(self, goal):
        """Run GoToPose to `goal`; return (state, elapsed, samples)."""
        self.request_id += 1
        rid = self.request_id
        ports = {'x': f'{goal[0]:.6f}', 'y': f'{goal[1]:.6f}', 'theta': f'{goal[2]:.6f}'}
        ports.update({k: str(v) for k, v in self.ports.items()})
        with self.lock:
            self.samples = []
        self.send({'command': 'run', 'id': rid, 'target': 'GoToPose', 'ports': ports,
                   'timeout_sec': self.timeout})
        self.wait(lambda: self.status is not None and self.status.get('request_id') == rid and
                  self.status.get('state') in TERMINAL, self.timeout * 3 + 10)
        with self.lock:
            st = dict(self.status or {})
            samples, self.samples = self.samples, None
        if st.get('request_id') != rid:
            st = {'state': 'no_response', 'elapsed': None, 'message': ''}
        self.send({'command': 'stop'})
        return st.get('state'), st.get('elapsed'), st.get('message', ''), samples

    def run(self):
        """Warm up, then repeat the tour."""
        if not self.wait(lambda: self.status is not None, 60):
            self.get_logger().error('no behavior_test/status: launch with verify:=true')
            return
        os.makedirs(os.path.join(self.dir, 'legs'), exist_ok=True)
        json.dump(dict(tour=self.tour, repeats=self.repeats, ports=self.ports,
                       label=self.label, started=time.strftime('%Y-%m-%d %H:%M:%S')),
                  open(os.path.join(self.dir, 'config.json'), 'w'), indent=1)
        state, *_ = self.leg(self.tour[0])
        self.get_logger().info(f'warm-up to {self.tour[0]}: {state}')
        time.sleep(self.pause)
        legs = []
        order = self.tour[1:] + self.tour[:1]
        number = 0
        for rep in range(self.repeats):
            for i, goal in enumerate(order):
                number += 1
                start = self.truth
                state, elapsed, message, samples = self.leg(goal)
                m = leg_metrics(samples, goal, state, elapsed)
                m.update(leg=number, repeat=rep, index=i, goal=goal, start=start,
                         message=message)
                legs.append(m)
                self.write(number, samples)
                self.get_logger().info(
                    f'leg {number} -> {tuple(round(v, 3) for v in goal)}: {state} '
                    f'{elapsed if elapsed is None else round(elapsed, 2)} s, final '
                    f'{m.get("final_pos_cm", float("nan")):.2f} cm '
                    f'{m.get("final_ang_deg", float("nan")):.2f} deg, re-exit '
                    f'{m.get("reexit_head_deg")}')
                time.sleep(self.pause)
        keys = sorted({k for m in legs for k in m})
        with open(os.path.join(self.dir, 'summary.csv'), 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            for m in legs:
                w.writerow({k: json.dumps(v) if isinstance(v, (list, tuple)) else v
                            for k, v in m.items()})
        self.get_logger().info(f'bench done: {self.dir}')

    def write(self, number, samples):
        if not samples:
            return
        keys = list(samples[0].keys())
        with open(os.path.join(self.dir, 'legs', f'leg_{number:03d}.csv'), 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            w.writerows(samples)


def main(args=None):
    """Run the bench once and exit."""
    rclpy.init(args=args)
    node = GotoBench()
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()
    try:
        node.run()
    except KeyboardInterrupt:
        node.send({'command': 'stop'})
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
