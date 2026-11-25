"""Motor identification on synthetic experiments written in the real on-disk format."""
import csv
import json
import math

import numpy as np
import pytest

from pinky_soccer_calibration import motor_identification as mi

pytest.importorskip('scipy')
TRUE = {'tau_left': .045, 'tau_right': .060, 'motor_accel': 60., 'motor_delay': .0}
UNIT = mi.RPM_UNIT * mi.RPM2RAD


def truth(goal_t, goal, t_out, tau, accel, delay):
    w, ref = mi.simulate(goal_t, goal, t_out, tau, accel, delay)
    return w, ref


def write_experiment(folder, v, rng, *, diag=True, feedback_hz=30., net=.008, noise=.05,
                     invalid=0, dropout=0., w=0., pc_offset=1000., seconds=(1., 2., 2.)):
    """One forward/spin step: PC commands 50 Hz -> watchdog 20 Hz -> driver -> motor model."""
    folder.mkdir(parents=True)
    pre, hold, post = seconds
    total = pre + hold + post
    t_pc = np.arange(0., total, .02)                     # PC publish times (robot unix base)
    cmd = np.where((t_pc >= pre) & (t_pc < pre + hold), 1., 0.)
    with (folder / 'commands.csv').open('w', newline='') as f:
        out = csv.writer(f)
        out.writerow(['publish_monotonic', 'publish_ros_ns', 'publish_unix_ns', 'v_mps', 'w_radps',
                      'command_tag'])
        for k, (t, c) in enumerate(zip(t_pc, cmd)):
            out.writerow([t + pc_offset, 0, int(t * 1e9), v * c, w * c, k + 1])
    arrive = t_pc + net                                   # at the robot watchdog
    t_wd = np.arange(0., total + .2, .05)                 # watchdog republish (20 Hz)
    receipts = []
    for t in t_wd:
        j = np.searchsorted(arrive, t, side='right') - 1
        if j >= 0 and t - arrive[j] < .5:
            receipts.append((float(t), int(j + 1), float(v * cmd[j]), float(w * cmd[j])))
    rt = np.array([r[0] + .001 for r in receipts])        # written after 1 ms serial
    goal = np.array([mi.driver_goal(r[2], r[3]) for r in receipts])
    tf = np.cumsum(rng.uniform(.8, 1.2, int(total * feedback_hz)) / feedback_hz)
    rows = []
    present, traj = [], []
    for i, side in ((0, 'left'), (1, 'right')):
        wv, ref = truth(rt, goal[:, i], tf, TRUE[f'tau_{side}'], TRUE['motor_accel'], TRUE['motor_delay'])
        present.append(np.round((wv + rng.normal(0, noise, wv.shape)) / UNIT))
        traj.append(np.round(ref / UNIT))
    keep = rng.uniform(size=len(tf)) >= dropout
    for n, t in enumerate(tf):
        if not keep[n]:
            continue
        vl, vr = present[0][n] * UNIT, -present[1][n] * UNIT          # right raw sign
        if invalid and n % 37 == 0:
            vl = 99.
        rows.append(('/joint_states', t + net + pc_offset,
                     {'name': ['l_wheel_joint', 'r_wheel_joint'], 'velocity': [vl, vr]}))
        if diag:
            sent = [{'tag': r[1], 'v': r[2], 'w': r[3], 'recv_unix_ns': int(r[0] * 1e9),
                     'written_unix_ns': int((r[0] + .001) * 1e9)}
                    for r in receipts if tf[n - 1 if n else 0] < r[0] <= t]
            motors = [{'present_velocity': int(present[0][n]), 'velocity_trajectory': int(traj[0][n]),
                       'present_current': 10, 'present_pwm': 100, 'present_input_voltage': 74},
                      {'present_velocity': int(-present[1][n]), 'velocity_trajectory': int(-traj[1][n]),
                       'present_current': 10, 'present_pwm': -100, 'present_input_voltage': 74}]
            rows.append(('/motor/diagnostics', t + net + pc_offset, {'payload': {
                'commands': sent, 'diag_read_failures': 0,
                'feedback': {'read_start_unix_ns': int(t * 1e9), 'read_done_unix_ns': int(t * 1e9),
                             'motors': motors}}}))
    with (folder / 'observations.csv').open('w', newline='') as f:
        out = csv.writer(f)
        out.writerow(['topic', 'receive_monotonic', 'header_seconds', 'observation_json'])
        for topic, t, data in rows:
            out.writerow([topic, t, t, json.dumps(data)])
    return folder


def runs(tmp_path, specs, rng, **kw):
    out = []
    for n, (v, purpose) in enumerate(specs):
        folder = write_experiment(tmp_path / f'e{n}', v, rng, **kw)
        out.append(mi.load_run(folder, {'id': f'e{n}', 'purpose': purpose,
                                        'plan': {'kind': 'forward'}}))
    return out


def test_driver_goal_reproduces_official_quantisation():
    g = mi.driver_goal(.10, 0.)
    rpm = .10 / .027 / mi.RPM2RAD
    assert g[0] == pytest.approx(math.trunc(rpm / .229) * .229 * mi.RPM2RAD)
    assert np.all(np.abs(mi.driver_goal(1., 0.)) <= 100 * mi.RPM2RAD + 1e-9)   # scaled to 100 rpm


def test_robot_clock_stages_recover_true_parameters(tmp_path):
    rng = np.random.default_rng(1)
    data = runs(tmp_path, [(.10, 'identification'), (.15, 'identification'),
                           (.05, 'identification'), (.12, 'validation')], rng)
    assert all(r.timeline == 'robot_clock' and r.trajectory is not None for r in data)
    assert data[0].quality['present_sign']['right'] == -1.         # raw right sign normalised
    result = mi.identify(data)
    ramp = result['stages']['goal_to_trajectory']['parameters']
    assert ramp['motor_accel'] == pytest.approx(TRUE['motor_accel'], rel=.1)
    lag = result['stages']['trajectory_to_present']['parameters']
    assert lag['tau_left'] == pytest.approx(TRUE['tau_left'], rel=.25)
    assert lag['tau_right'] == pytest.approx(TRUE['tau_right'], rel=.25)
    full = result['models']['ramp_first_order']['parameters']
    assert full['motor_accel'] == pytest.approx(TRUE['motor_accel'], rel=.15)
    assert result['validated'] and result['best_on_validation'] in mi.MODELS
    val = result['models']['ramp_first_order']['validation']
    assert max(r['relative_rmse'] for r in val) < .1


def test_pc_timeline_without_ramp_data_reports_combined_delay(tmp_path):
    rng = np.random.default_rng(2)
    data = runs(tmp_path, [(.10, 'identification'), (.15, 'identification'),
                           (.20, 'identification')], rng, diag=False, net=.012)
    assert all(r.timeline == 'pc_receive' and r.trajectory is None for r in data)
    assert 'Wi-Fi' in data[0].quality['delay_includes']
    fitted = mi.fit('ramp_first_order_delay', data)
    # true: 12 ms net + up to 50 ms watchdog hold (mean ~25 ms) + 1 ms write -> 20..60 ms
    assert .015 < fitted['parameters']['delay'] < .07
    assert fitted['parameters']['tau_left'] == pytest.approx(TRUE['tau_left'], abs=.03)
    assert not mi.identify(data)['validated']                       # no validation runs


def test_noise_irregular_dropouts_invalid_values(tmp_path):
    rng = np.random.default_rng(3)
    data = runs(tmp_path, [(.10, 'identification'), (.15, 'identification')], rng,
                feedback_hz=22., noise=.15, dropout=.15, invalid=1, diag=False)
    q = data[0].quality
    assert q['present_invalid_samples'] > 0 and q['present_dropouts'] >= 0
    assert 15 < q['present_rate_hz'] < 30
    fitted = mi.fit('ramp_first_order_delay', data)
    assert fitted['status'] == 'fitted'
    assert fitted['parameters']['tau_left'] == pytest.approx(TRUE['tau_left'], abs=.04)


def test_small_steps_flag_ramp_as_not_identifiable(tmp_path):
    rng = np.random.default_rng(4)
    # 0.01 m/s -> 0.37 rad/s goal: the ramp lasts ~6 ms, invisible at 30 Hz behind tau
    data = runs(tmp_path, [(.01, 'identification'), (.012, 'identification')], rng, noise=.02)
    fitted = mi.fit('ramp_first_order', data)
    assert fitted['parameter_status']['motor_accel'].startswith('identification_insufficient')


def test_missing_files_values_are_not_invented(tmp_path):
    rng = np.random.default_rng(5)
    folder = write_experiment(tmp_path / 'x', .1, rng, diag=False)
    (folder / 'observations.csv').write_text(
        'topic,receive_monotonic,header_seconds,observation_json\n')
    run = mi.load_run(folder, {'id': 'x'})
    assert run.present is None
    assert mi.fit('first_order', [run])['status'] == 'insufficient_data'


# ---- individual wheel radii -----------------------------------------------------------
from pinky_soccer_calibration import wheel_individual as wi    # noqa: E402


def wheel_trials(rl, rr, b, rng, heading=True, n=3):
    trials = []
    for k in range(n):
        for kind, sign in (('forward', 1), ('backward', -1)):
            th = sign * 1.0 / ((rl + rr) / 2)
            s = (rl + rr) * th / 2 + rng.normal(0, .001)
            dpsi = (rr - rl) * th / b + math.radians(rng.normal(0, .3))
            t = {'id': f'{kind}{k}', 'kind': kind, 'left_rad': th, 'right_rad': -th,   # raw R sign
                 'distance_m': s, 'measurement_type': 'straight_distance'}
            if heading:
                t['extra_observations'] = {'final_heading_change_deg': math.degrees(dpsi)}
            trials.append(t)
        for kind, sign in (('left', 1), ('right', -1)):
            phi = sign * 2 * math.pi * b / (rl + rr)
            trials.append({'id': f'{kind}{k}', 'kind': kind, 'left_rad': -phi, 'right_rad': -phi,
                           'angle_rad': sign * 2 * math.pi + math.radians(rng.normal(0, .5)),
                           'measurement_type': 'rotation_angle'})
    return trials


def test_individual_radii_recovered_only_with_heading_observations():
    rng = np.random.default_rng(7)
    rl, rr, b = .0275, .0278, .0981               # 0.3 mm difference
    result = wi.fit_individual(wheel_trials(rl, rr, b, rng), shared={'effective_radius_m': .02767})
    assert result['applied'] is False and result['shared_result_kept'] is True
    assert result['left_radius_m'] == pytest.approx(rl, abs=1e-4)
    assert result['right_radius_m'] == pytest.approx(rr, abs=1e-4)
    assert result['status'].startswith('identified_candidate')
    same = wi.fit_individual(wheel_trials(.0276, .0276, b, rng))
    assert same['status'].startswith('not_significant')
    none = wi.fit_individual(wheel_trials(rl, rr, b, rng, heading=False))   # like the 12 real trials
    assert none['status'].startswith('insufficient_observations') and 'left_radius_m' not in none


# ---- project integration ----------------------------------------------------------------
import yaml   # noqa: E402

from pinky_soccer_calibration.suite_model import Session   # noqa: E402


def test_project_identification_files_and_staleness(tmp_path):
    session = Session(tmp_path, 'real', 'pinky-pro')
    session.add_measurement('lidar', {'samples': [{'distance': .51}] * 12,
                                      'reference': {'sensor_reference_m': .5}})
    session.recalculate('lidar')
    profile = session.export(True)
    before = profile.read_text()
    rng = np.random.default_rng(11)
    for n, (v, purpose) in enumerate([(.10, 'identification'), (.15, 'identification'),
                                      (.12, 'validation')]):
        folder = f'motor_response/e{n}'
        write_experiment(session.root / folder, v, rng)
        session.add_measurement('motor_response', {'status': 'complete', 'purpose': purpose,
                                                   'plan': {'kind': 'forward'}, 'files': {'folder': folder}})
    assert 'motor_identification' in session.stale or 'motor_identification' not in session.results
    result = session.recalculate('motor_identification')
    assert result['validated'] and 'motor_identification' not in session.stale
    doc = yaml.safe_load((session.root / 'identification.yaml').read_text())
    assert doc['motor_identification_schema'] == 1 and doc['applied_to_mpc'] is False
    assert 'identification_schema' not in doc                          # loader cannot pick it up
    assert len(doc['validation_experiments']) == 1 and doc['raw_data']
    assert profile.read_text() == before                                # profile untouched
    exported = yaml.safe_load(session.export(True).read_text())
    assert 'motor_identification' not in exported['results']
    from pinky_soccer_dynamics import params
    with pytest.raises(ValueError):
        params.apply_identification(params.load('real'), session.root / 'identification.yaml')
    session.add_measurement('motor_response', {'status': 'complete', 'purpose': 'identification',
                                               'plan': {'kind': 'forward'}, 'files': {'folder': 'x'}})
    assert 'motor_identification' in session.stale and 'lidar' not in session.stale
    individual = session.recalculate('wheel_individual')
    assert individual['status'].startswith('insufficient_observations')
    root = session.root
    session.release()
    reopened = Session.open(root)
    assert reopened.results['wheel_individual']['applied'] is False
    reopened.release()
