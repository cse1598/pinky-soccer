"""Braking analysis from a motor experiment's raw files (referenced, never copied).

Distances here are encoder/odom distances. A sliding body is not observed by odom, so the
real chassis braking distance is only the user's manual measurement (stored separately).
"""
import csv
import json
import math
from pathlib import Path

STOP_SPEED_RADPS = 0.2      # wheel speed regarded as stopped
STOP_HOLD_S = 0.3           # ... for at least this long
WHEELS = (('l_wheel_joint', 'left_wheel_joint'), ('r_wheel_joint', 'right_wheel_joint'))


def wheel_speeds(obs):
    names, vel = obs.get('name') or [], obs.get('velocity') or []
    out = []
    for aliases in WHEELS:
        idx = next((names.index(a) for a in aliases if a in names), None)
        if idx is None or idx >= len(vel):
            return None
        out.append(vel[idx])
    return out


def analyze(folder, meta=None):
    """Stop timing and odom distance after the stop command of one experiment."""
    folder = Path(folder)
    commands = list(csv.DictReader((folder / 'commands.csv').open()))
    moving = [c for c in commands if float(c['v_mps']) or float(c['w_radps'])]
    if not moving:
        return {'status': 'not_applicable: no motion command'}
    last = moving[-1]
    after = [c for c in commands if float(c['publish_monotonic']) > float(last['publish_monotonic'])]
    if not after:
        return {'status': 'not_applicable: no stop command recorded (stopped by stop())'}
    t_stop = float(after[0]['publish_monotonic'])
    joints, odom, battery = [], [], []
    for row in csv.DictReader((folder / 'observations.csv').open()):
        obs = json.loads(row['observation_json'])
        t = float(row['receive_monotonic'])
        if row['topic'] == '/joint_states':
            w = wheel_speeds(obs)
            if w:
                joints.append((t, w))
        elif row['topic'] == '/odom' and 'x' in obs:
            odom.append((t, obs['x'], obs['y'], obs['yaw'], obs.get('v'), obs.get('w')))
        elif row['topic'] in ('/batt_state', '/battery/voltage'):
            battery.append(obs.get('voltage', obs.get('value')))
    before = [w for t, w in joints if t <= t_stop]
    if not before or not odom:
        return {'status': 'insufficient_data'}
    stopped_at, since = None, None
    for t, w in joints:
        if t <= t_stop:
            continue
        if max(abs(x) for x in w) < STOP_SPEED_RADPS:
            since = since if since is not None else t
            if t - since >= STOP_HOLD_S:
                stopped_at = since
                break
        else:
            since = None
    o0 = min(odom, key=lambda o: abs(o[0] - t_stop))
    out = {'status': 'measured' if stopped_at else 'not_stopped_within_record',
           'stop_command_monotonic': t_stop,
           'command_before_stop': {'v_mps': float(last['v_mps']), 'w_radps': float(last['w_radps'])},
           'wheel_speed_at_stop_radps': before[-1],
           'odom_speed_at_stop': {'v_mps': o0[4], 'w_radps': o0[5]},
           'stop_threshold': {'wheel_radps': STOP_SPEED_RADPS, 'hold_s': STOP_HOLD_S},
           'encoder_samples_after_stop': sum(1 for t, _ in joints if t > t_stop),
           'battery_voltage': battery[-1] if battery else None,
           'distance_kind': 'odom/encoder (excludes wheel slip)'}
    if stopped_at:
        o1 = min(odom, key=lambda o: abs(o[0] - stopped_at))
        out.update(stop_time_s=stopped_at - t_stop,
                   odom_braking_distance_m=math.hypot(o1[1] - o0[1], o1[2] - o0[2]),
                   heading_change_rad=math.atan2(math.sin(o1[3] - o0[3]), math.cos(o1[3] - o0[3])))
    if meta:
        out['steady_before_stop'] = meta.get('braking_steady')
    return out
