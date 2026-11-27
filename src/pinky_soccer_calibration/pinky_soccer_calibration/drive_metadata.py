"""Drive configuration used by a motor experiment, every value tagged with its source.

Sources (never upgraded: a value keeps the weakest source it came from):
  running_driver       queried from the running driver node (ROS parameter service)
  driver_report        published by the driver itself (motor/info)
  applied_config_file  read from a file the launch is known to have applied
  project_file         a file of this calibration project (not proof the robot used it)
  user_input           typed by the user (floor, ...) or an approved supplementary file
  ui_setting           a setting of this UI (command rate, limits)
  code_analysis        a constant of the PC copy of the code (robot copy not verified)
  measured             measured in the experiment (battery voltage, clock quality, rates)
  inferred_from_data   derived from recorded register values (an interval, not a setting)
  assumed_default      an official default used because nothing else is known
  unavailable          could not be determined (reason given)

A project profile.yaml containing wheel values does NOT mean the driver used them: the
driver uses whatever the robot launch passed. Only running_driver / driver_report /
inferred_from_data say something about the robot.
"""
import hashlib
import math
from pathlib import Path

RPM_UNIT = .229                       # Dynamixel velocity unit [rpm]
RPM2RAD = 2 * math.pi / 60
UNIT_RAD = RPM_UNIT * RPM2RAD         # one Goal/Present Velocity unit in rad/s
OFFICIAL = {'wheel_radius': .027, 'wheel_separation': .0961, 'max_rpm': 100.,
            'profile_acceleration': 200, 'feedback_rate_hz': 30.}
# Robot-side code constants (pinky_soccer_bringup command_watchdog.py, PC copy).
WATCHDOG = {'forward_period_s': .05, 'timeout_s': .5}
# Official driver (pinky_bringup bringup.py): left wheel + forward, right wheel raw - forward
# in Present Velocity/Position and joint_states; odom negates the right wheel.
ENCODER_SIGNS = {'left': 1, 'right': -1}
STRONG = ('running_driver', 'driver_report', 'applied_config_file')


def item(value, source, detail=''):
    return {'value': value, 'source': source, 'detail': detail}


def file_sha256(path):
    try:
        return hashlib.sha256(Path(path).expanduser().read_bytes()).hexdigest()
    except OSError:
        return None


def at_start(motor_info=None, driver_parameters=None, options=None, plan=None,
             project_profile=None, clock_sync=None, battery=None):
    """Metadata to store with a new experiment (nothing invented; gaps are 'unavailable')."""
    options, plan = options or {}, plan or {}
    out = {'schema': 1}
    params = driver_parameters or {}
    info = motor_info if isinstance(motor_info, dict) else {}
    for key in ('wheel_radius', 'wheel_separation'):
        if params.get(key) is not None:
            out[key] = item(float(params[key]), 'running_driver',
                            params.get('_node', 'pinky_bringup parameter'))
        elif info.get(key) is not None:
            out[key] = item(float(info[key]), 'driver_report', 'motor/info')
        else:
            out[key] = item(None, 'unavailable', params.get('_error')
                            or 'driver not queried and motor/info has no value')
    if params.get('feedback_rate_hz') is not None:
        out['feedback_rate_hz'] = item(float(params['feedback_rate_hz']), 'running_driver')
    elif info.get('feedback_rate_hz') is not None:
        out['feedback_rate_hz'] = item(float(info['feedback_rate_hz']), 'driver_report')
    else:
        out['feedback_rate_hz'] = item(None, 'unavailable', 'measure from the received data')
    motors = info.get('motors') or []
    out['motor_model'] = item([m.get('model') for m in motors] or None,
                              'driver_report' if motors else 'unavailable')
    out['profile_acceleration_raw'] = item(
        [m.get('profile_acceleration') for m in motors] or None,
        'driver_report' if motors else 'unavailable',
        'official driver writes 200 at start (code_analysis); register unit 214.577 rpm/min')
    out['driver_kind'] = item(info.get('driver') or ('drive_diagnostics' if info else None),
                              'driver_report' if info else 'unavailable')
    out['watchdog'] = item(dict(WATCHDOG), 'code_analysis',
                           'pinky_soccer_bringup command_watchdog (PC copy)')
    out['encoder_signs'] = item(dict(ENCODER_SIGNS), 'code_analysis',
                                'official pinky_bringup: right wheel raw sign is negative')
    out['command_rate_hz'] = item(plan.get('rate_hz'), 'ui_setting')
    out['limits'] = item(plan.get('limits'), 'ui_setting')
    out['floor'] = item(options.get('floor'), 'user_input' if options.get('floor') else 'unavailable')
    out['clock_sync'] = item((clock_sync or {}).get('relative_bound_s'),
                             'measured' if clock_sync else 'unavailable',
                             (clock_sync or {}).get('quality', ''))
    out['battery_voltage_v'] = item(battery, 'measured' if battery is not None else 'unavailable')
    if project_profile:
        out['project_profile'] = item({'path': str(project_profile),
                                       'sha256': file_sha256(project_profile)},
                                      'project_file',
                                      'the project profile; NOT proof the robot launch used it')
    return out


def velocity_unit_interval(rad_s):
    """Wheel speed interval [lo, hi) that truncates to the same register unit."""
    units = math.trunc(abs(rad_s) / UNIT_RAD + 1e-9)
    return units, units * UNIT_RAD, (units + 1) * UNIT_RAD


def infer_geometry(segments):
    """Driver radius / separation intervals from steady Velocity Trajectory units.

    segments: [(v, w, units_left, units_right)] where the trajectory stayed constant after a
    command (so it equals the Goal Velocity register). The driver truncates
    rpm/0.229 toward zero, so |wheel| in [u, u+1) units. Straight runs bound r; spins bound
    b/r. Returns intervals (inclusive-exclusive) or None when no segment constrains them.
    Ratio-scaled commands (> max rpm) are skipped.
    """
    r_lo, r_hi, q_lo, q_hi = 0., math.inf, 0., math.inf
    used = 0
    for v, w, ul, ur in segments:
        for u, s in ((abs(ul), -1), (abs(ur), 1)):
            if u == 0 or u * RPM_UNIT >= OFFICIAL['max_rpm'] - RPM_UNIT:
                continue
            lo, hi = u * UNIT_RAD, (u + 1) * UNIT_RAD
            if abs(w) < 1e-9 and abs(v) > 1e-9:        # |v| / r in [lo, hi)
                r_lo, r_hi = max(r_lo, abs(v) / hi), min(r_hi, abs(v) / lo)
                used += 1
            elif abs(v) < 1e-9 and abs(w) > 1e-9:      # |w| (b/r) / 2 in [lo, hi)
                q_lo, q_hi = max(q_lo, 2 * lo / abs(w)), min(q_hi, 2 * hi / abs(w))
                used += 1
    out = {'segments_used': used}
    if r_hi < math.inf:
        out['wheel_radius_interval_m'] = [r_lo, r_hi]
        out['radius_consistent'] = r_lo < r_hi
    if q_hi < math.inf:
        out['separation_over_radius_interval'] = [q_lo, q_hi]
        if r_hi < math.inf:
            out['wheel_separation_interval_m'] = [q_lo * r_lo, q_hi * r_hi]
    return out


def contains(interval, value):
    return interval is not None and value is not None and interval[0] <= value <= interval[1]


def resolve(stored=None, override=None, inferred=None):
    """Driver values to analyse one experiment with, and why.

    Order: running_driver / driver_report stored with the experiment, then an approved
    supplementary value (override, user_input), then the centre of a narrow interval inferred
    from the registers, then the official default. The inferred interval is always reported
    as a consistency check of the value used.
    """
    stored, override, inferred = stored or {}, override or {}, inferred or {}
    out, notes = {}, []
    for key, ikey in (('wheel_radius', 'wheel_radius_interval_m'),
                      ('wheel_separation', 'wheel_separation_interval_m')):
        rec = stored.get(key) or {}
        interval = inferred.get(ikey)
        if rec.get('value') is not None and rec.get('source') in STRONG:
            chosen = item(rec['value'], rec['source'], rec.get('detail', ''))
        elif override.get(key) is not None:
            chosen = item(float(override[key]), 'user_input',
                          override.get('_note', 'approved supplementary metadata'))
        elif interval and interval[1] - interval[0] < .02 * interval[1]:
            chosen = item((interval[0] + interval[1]) / 2, 'inferred_from_data',
                          f'interval {interval[0]:.6f}..{interval[1]:.6f} from steady '
                          'Velocity Trajectory units')
        else:
            chosen = item(OFFICIAL[key], 'assumed_default',
                          'official pinky_params.yaml default; launch override unknown')
        if interval:
            ok = contains(interval, chosen['value'])
            chosen['data_check'] = 'consistent' if ok else 'CONTRADICTED by register data'
            if not ok:
                notes.append(f'{key} {chosen["value"]} lies outside the register-derived '
                             f'interval {interval}')
        out[key] = chosen
    out['notes'] = notes
    return out
