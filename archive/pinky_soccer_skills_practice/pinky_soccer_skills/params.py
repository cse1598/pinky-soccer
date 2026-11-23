"""
Skill parameters: robot geometry/limits and per-skill tuning, loaded from YAML.

Every value has a default in code so a missing YAML key never changes behaviour silently;
config/skills.yaml documents where each value comes from (measured, model, or assumption).
"""

from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace

import yaml

DEFAULTS = {
    'robot': {
        # From the soccer URDF/bumper v1 collision geometry in the ball height band
        # (ground 0..45 mm), base_link frame. Wheels stick out beyond the bumper.
        'bumper_front_x': 0.0415,
        'bumper_half_width': 0.0385,
        'footprint': [-0.075, 0.0415, -0.077, 0.077],   # xmin xmax ymin ymax
        'ball_radius': 0.02135,                          # user-measured 42.7 mm golf ball
        'track': 0.0971,
        'wheel_radius': 0.028,
        'wheel_max_speed': 14.0,                         # [rad/s] official MuJoCo motor cap
        # Command limits. MuJoCo official control caps are 0.25 m/s / 1.5 rad/s; on the
        # physical robot lower these until the motors are characterised.
        'max_lin': 0.25,
        'max_ang': 1.5,
        'accel_lin': 1.0,                                # [m/s^2] command ramp (assumption)
        'accel_ang': 8.0,                                # [rad/s^2] command ramp (assumption)
    },
    'ball_model': {
        # Blind-zone ball prediction. roll_decel from the MuJoCo EVA-mat material model
        # (0.25 m/s release -> ~9 cm roll); measure on the real mat. us_contact_range is the
        # simulated ultrasonic reading with the ball touching the bumper (measure on robot).
        'roll_decel': 0.35, 'us_contact_range': 0.022, 'us_weight': 0.3,
        'visible_gap': 0.06,          # camera sees the ball beyond this gap (bench model)
        'missing_time': 0.6,          # [s] predicted in view but not detected -> lost
    },
    'drive': {
        'k_rho': 1.6, 'k_alpha': 5.0, 'k_beta': -2.0,     # swept on the bench (2026-11-23)
        'pos_tol': 0.02, 'heading_tol': 0.08,
        'reverse': True, 'reverse_hysteresis': 0.25,      # [s] time saving needed to switch
        'decel': 1.0,                                     # [m/s^2] planned stopping decel
        'turn_decel': 6.0,                                # [rad/s^2] planned turn stop
        'turn_slowdown': 1.2,                             # v <= max_lin / (1 + k*|w|)
    },
    'search': {
        'confirm_frames': 2, 'turn_to_last_max_age': 4.0, 'spin_speed': 2.0,
        'spin_turns': 1.1, 'vantage': [[0.0, 0.0]],
    },
    'approach': {
        'pos_tol': 0.02, 'heading_tol': 0.10, 'clearance': 0.03,
        'replan_period': 0.5, 'switch_gain': 0.4, 'predict': True,
        'final_slow_dist': 0.08, 'final_speed': 0.20, 'max_time': 12.0,
        'final_dist': 0.05, 'final_hold': 0.02, 'final_lateral': 0.025,
        'final_lateral_abort': 0.045, 'final_min_gap': 0.015,
        'blind_time': 8.0,           # [s] a static ball may stay unseen while we go around it
        'opponent_radius': 0.17,     # robot-centre keep-out around an opponent centre
    },
    'dribble': {
        'push_depth': 0.012, 'tap_speed': 0.25, 'coast_speed': 0.06, 'coast_max': 1.0,
        'gap_open': 0.05, 'gap_close': 0.006, 'push_time': 0.12, 'fresh_time': 0.15,
        'k_path': 3.0, 'max_correction': 0.6, 'k_stanley': 3.0, 'k_heading': 4.0,
        'align_heading': 0.35, 'align_lateral': 0.025, 'lost_gap': 0.20,
        'recenter_lateral': 0.028, 'lost_time': 3.0, 'goal_tol': 0.06, 'mode': 'hybrid',
        'recheck_period': 0.9, 'corner_radius': 0.0, 'max_curvature': 1.0, 'min_turn': 0.15,
        'reorient_angle': 0.35, 'near_gap': 0.05,
        'wiggle_amp': 0.0, 'wiggle_freq': 2.0, 'lookahead': 0.10, 'max_time': 25.0,
    },
    'shot': {
        'backoff': 0.08, 'strike_accel': 0.4, 'follow_dist': 0.10,   # strike_accel: assumption
        'align_heading': 0.08, 'align_lateral': 0.02, 'quick_heading': 0.15,
        'quick_lateral': 0.02, 'k_stanley': 4.0, 'k_heading': 5.0,
        'max_time': 12.0,
    },
    'recover': {
        'near_dist': 0.16, 'back_dist': 0.14, 'back_speed': 0.15, 'confirm_frames': 1,
    },
    'block': {
        'line_offset': 0.12, 'post_margin': 0.03, 'pos_tol': 0.03, 'horizon': 2.5,
        'intercept_margin': 0.3,
    },
    'playbook': {
        # Shot reach: with 0.25 m/s and the 0.35 m/s^2 rolling model a struck ball rolls
        # ~9 cm, plus the 10 cm follow-through push -> shoot only within ~20 cm of the line.
        'shot_range': 0.20, 'shot_backoff': 0.08, 'possess_gap': 0.035,
        'possess_lateral': 0.025, 'defend_margin': 0.10, 'opponent_avoid_radius': 0.20,
        'aim_inset': 0.08, 'approach_gap': 0.02, 'steal_possess': 0.13, 'steal_carry': 0.3,
        'shadow_distance': 0.25, 'poke_dist': 0.32,
    },
}


def _merge(base, extra):
    out = deepcopy(base)
    for key, value in (extra or {}).items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = _merge(out[key], value)
        else:
            out[key] = value
    return out


def load(path=None, overrides=None):
    """Return a SimpleNamespace of sections; YAML and overrides are merged onto DEFAULTS."""
    cfg = deepcopy(DEFAULTS)
    if path:
        data = yaml.safe_load(Path(path).read_text()) or {}
        unknown = set(data) - set(DEFAULTS)
        if unknown:
            raise ValueError(f'unknown sections in {path}: {sorted(unknown)}')
        cfg = _merge(cfg, data)
    cfg = _merge(cfg, overrides)
    ns = SimpleNamespace(**{k: SimpleNamespace(**v) for k, v in cfg.items()})
    r = ns.robot
    r.contact_dist = r.bumper_front_x + r.ball_radius   # base_link -> ball centre at contact
    return ns
