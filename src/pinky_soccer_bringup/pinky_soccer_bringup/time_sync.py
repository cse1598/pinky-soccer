#!/usr/bin/env python3
"""Monitor OS chrony synchronization; never step a clock during a ROS experiment."""

import datetime
import ipaddress
import json
import math
import os
import re
import subprocess
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def parse_tracking(text):
    """Parse stable C-locale chronyc monitoring output, rejecting untrusted states."""
    fields = dict(line.split(':', 1) for line in text.splitlines() if ':' in line)
    fields = {k.strip(): v.strip() for k, v in fields.items()}

    def number(key):
        value = float(fields[key].split()[0])
        if not math.isfinite(value):
            raise ValueError('Nonfinite chrony tracking value')
        return value
    status = {'leap': fields['Leap status'], 'stratum': number('Stratum'),
              'system_offset_s': abs(number('System time')),
              'rms_offset_s': abs(number('RMS offset')),
              'root_delay_s': abs(number('Root delay')),
              'root_dispersion_s': abs(number('Root dispersion')),
              'reference_id': fields['Reference ID']}
    stamp = datetime.datetime.strptime(fields['Ref time (UTC)'], '%a %b %d %H:%M:%S %Y')
    status['sample_age_s'] = time.time() - stamp.replace(tzinfo=datetime.timezone.utc).timestamp()
    status['synchronized'] = (status['leap'] == 'Normal' and 0 < status['stratum'] < 16
                              and -2 <= status['sample_age_s'] <= 120)
    return status


def tracking():
    """Read monitoring commands without requiring root or changing OS state."""
    try:
        output = subprocess.run(['chronyc', '-n', 'tracking'], capture_output=True,
                                text=True, check=True, timeout=2, env=dict(os.environ, LC_ALL='C'))
        result = parse_tracking(output.stdout)
        result['error'] = ''
        return result
    except (OSError, ValueError, KeyError, subprocess.SubprocessError) as error:
        return {'synchronized': False, 'error': str(error)}


def relative_ready(pc, robot, server='192.168.4.1', limit_s=.005):
    """Require the PC to track this robot, not merely an unrelated time server."""
    if not pc.get('synchronized') or not robot or robot.get('error'):
        return False
    reference = pc.get('reference_id', '')
    address = ipaddress.ip_address(server)
    identity = f'{int(address):08X}'
    if server not in re.findall(r'\(([^)]+)\)', reference) and not reference.startswith(identity):
        return False
    # Conservative observed quality gate, not a guarantee of exposure timing accuracy.
    bound = (pc['system_offset_s'] + pc['rms_offset_s']
             + pc['root_delay_s']/2 + pc['root_dispersion_s'])
    return bound <= limit_s


# ---------------------------------------------------------------- PC-centred clock checks
# Architecture (2026-11-25): internet NTP -> laptop PC (NTP client + LAN server) -> every
# Pinky (chrony client of the PC). The checks below read non-root chronyc monitoring data
# only (`ntpdata` needs root and is not used); nothing here steps or changes a clock.
TRACKING_FIELDS = ('reference_id', 'reference', 'stratum', 'ref_time_unix', 'system_offset_s',
                   'last_offset_s', 'rms_offset_s', 'frequency_ppm', 'residual_freq_ppm',
                   'skew_ppm', 'root_delay_s', 'root_dispersion_s', 'update_interval_s', 'leap')


def parse_tracking_csv(line, now=None):
    """`chronyc -n -c tracking` -> dict (signed offsets, seconds)."""
    values = line.strip().split(',')
    if len(values) != len(TRACKING_FIELDS):
        raise ValueError('unexpected chronyc tracking CSV')
    out = dict(zip(TRACKING_FIELDS, values))
    for key in TRACKING_FIELDS[2:-1]:
        out[key] = float(out[key])
        if not math.isfinite(out[key]):
            raise ValueError('Nonfinite chrony tracking value')
    now = time.time() if now is None else now
    out['sample_age_s'] = now - out['ref_time_unix']
    out['synchronized'] = (out['leap'] == 'Normal' and 0 < out['stratum'] < 16
                           and out['reference'] not in ('', '0.0.0.0', '127.127.1.1'))
    return out


def parse_sources_csv(text):
    """`chronyc -n -c sources` -> list of sources; state '*' is the selected one."""
    rows = []
    for line in text.strip().splitlines():
        v = line.split(',')
        if len(v) < 10:
            continue
        rows.append({'mode': v[0], 'state': v[1], 'address': v[2], 'stratum': int(v[3]),
                     'poll': int(v[4]), 'reach': int(v[5], 8), 'last_rx_s': float(v[6]),
                     'adjusted_offset_s': float(v[7]), 'measured_offset_s': float(v[8]),
                     'error_s': float(v[9])})
    return rows


def parse_sourcestats_csv(text):
    """`chronyc -n -c sourcestats` -> {address: statistics}."""
    out = {}
    for line in text.strip().splitlines():
        v = line.split(',')
        if len(v) < 8:
            continue
        out[v[0]] = {'samples': int(v[1]), 'runs': int(v[2]), 'span_s': float(v[3]),
                     'frequency_ppm': float(v[4]), 'freq_skew_ppm': float(v[5]),
                     'offset_s': float(v[6]), 'std_dev_s': float(v[7])}
    return out


def chrony_status():
    """Tracking + sources + sourcestats without root; errors are reported, not hidden."""
    env = dict(os.environ, LC_ALL='C')
    out = {'collected_unix': time.time(), 'error': ''}

    def run(*args):
        return subprocess.run(['chronyc', '-n', '-c', *args], capture_output=True, text=True,
                              check=True, timeout=2, env=env).stdout
    try:
        out.update(parse_tracking_csv(run('tracking')))
    except (OSError, ValueError, subprocess.SubprocessError) as error:
        out.update(synchronized=False, error=f'tracking: {error}')
        return out
    for key, parser, cmd in (('sources', parse_sources_csv, 'sources'),
                             ('sourcestats', parse_sourcestats_csv, 'sourcestats')):
        try:
            out[key] = parser(run(cmd))
        except (OSError, ValueError, subprocess.SubprocessError) as error:
            out[key] = None
            out['error'] += f'{cmd}: {error}; '
    return out


def selected_server(status):
    """Address of the source the clock currently follows (None if unknown)."""
    for row in (status or {}).get('sources') or []:
        if row['state'] == '*':
            return row['address']
    ref = (status or {}).get('reference')
    if ref:
        return ref
    found = re.findall(r'\(([^)]+)\)', (status or {}).get('reference_id', ''))
    return found[0] if found else None


def pc_reference_check(pc, robot, pc_ip, limit_s=.005, max_status_age_s=15., now=None):
    """Evaluate PC-centred relative synchronization; every state is reported separately.

    Relative error ESTIMATE (robot vs PC clock), seconds, from the robot's own chrony data:
      |robot system time residual| + robot RMS offset + link_delay/2
        + std dev of the robot's PC-source measurements (robot sourcestats for pc_ip)
    link_delay = robot root delay - PC root delay is only an estimate of the LAN round trip
    (root delays are path sums of different moments), so the total is a heuristic estimate,
    not a strict bound. The PC's own upstream (internet) std dev is reported separately and
    is not part of the relative estimate. Quality classes: insufficient_info,
    insufficient_samples, uncertain (estimate > limit), verified. Status freshness uses the
    robot publish time (every 5 s), not chrony's update interval (16 s at maxpoll 4), so a
    normal poll interval does not make the state fail. The 5 ms limit is unchanged.
    """
    now = time.time() if now is None else now
    pc, robot = pc or {}, robot or {}
    out = {'pc_ip': pc_ip, 'limit_s': limit_s}
    out['pc_absolute'] = bool(pc.get('synchronized')) and not pc.get('error', '').startswith('tracking')
    out['robot_status_received'] = bool(robot)
    age = now - robot['collected_unix'] if robot.get('collected_unix') else None
    out['robot_status_age_s'] = age
    out['robot_status_fresh'] = age is not None and -2 <= age <= max_status_age_s
    out['robot_absolute'] = bool(robot.get('synchronized'))
    server = selected_server(robot)
    out['robot_selected_server'] = server
    try:
        ipaddress.ip_address(pc_ip)
        out['robot_uses_pc'] = server == pc_ip
    except ValueError:
        out['robot_uses_pc'] = False
        out['reason'] = f'PC IP가 올바르지 않음: {pc_ip!r}'
    stats = (robot.get('sourcestats') or {}).get(pc_ip)
    out['pc_source_stats'] = stats
    terms = None
    if (out['robot_uses_pc'] and stats is not None and pc.get('root_delay_s') is not None
            and robot.get('root_delay_s') is not None):
        link = robot['root_delay_s'] - pc['root_delay_s']
        terms = {'robot_system_offset_s': abs(robot['system_offset_s']),
                 'robot_rms_offset_s': abs(robot['rms_offset_s']),
                 'half_link_delay_s': max(link, 0.) / 2, 'pc_source_std_dev_s': stats['std_dev_s']}
        out['link_delay_estimate_s'] = link
        out['relative_bound_s'] = sum(terms.values())
        out['bound_kind'] = 'heuristic_estimate'
        out['relative_terms'] = terms
        out['samples'] = stats['samples']
    upstream = (pc.get('sourcestats') or {}).get(selected_server(pc) or '')
    out['pc_upstream_std_dev_s'] = upstream['std_dev_s'] if upstream else None
    out['robot_pc_source_std_dev_s'] = stats['std_dev_s'] if stats else None
    ok = (out['pc_absolute'] and out['robot_status_fresh'] and out['robot_absolute']
          and out['robot_uses_pc'] and terms is not None and out['samples'] >= 4
          and robot.get('leap') == 'Normal')
    out['relative_verified'] = bool(ok and out['relative_bound_s'] <= limit_s)
    out['quality'] = ('verified' if out['relative_verified'] else
                      'insufficient_info' if terms is None or not out['robot_status_fresh']
                      or not out['robot_uses_pc'] else
                      'insufficient_samples' if out.get('samples', 0) < 4 else 'uncertain')
    if 'reason' not in out:
        if not out['robot_status_received']:
            out['reason'] = '로봇 시간 상태 토픽 미수신'
        elif not out['robot_status_fresh']:
            out['reason'] = '로봇 시간 상태가 오래됨'
        elif not out['robot_uses_pc']:
            out['reason'] = f'로봇이 PC({pc_ip})가 아닌 {server}를 기준으로 사용'
        elif terms is None:
            out['reason'] = '상대 품질 정보 부족 (로봇 bringup 갱신 필요: sources/sourcestats)'
        elif not out['relative_verified']:
            out['reason'] = (f"표본 부족 ({out.get('samples')}개 < 4)" if out['quality'] ==
                             'insufficient_samples' else
                             f"상대 오차 추정 {out['relative_bound_s']*1e3:.2f} ms > "
                             f'{limit_s*1e3:.1f} ms (불확실)')
        else:
            out['reason'] = f"PC 기준 동기화 확인 (추정 {out['relative_bound_s']*1e3:.2f} ms)"
    return out


class TimeSyncStatus(Node):
    """Expose robot clock health over ROS while chronyd automatically disciplines clocks."""

    def __init__(self):
        """Keep this node on system time, independently of sensor drivers."""
        super().__init__('pinky_time_sync_status')
        self.publisher = self.create_publisher(String, 'calibration/time_sync_status', 10)
        self.create_timer(5., self.publish_status)
        self.publish_status()

    def publish_status(self):
        """Report missing chrony without treating it as synchronized.

        The legacy tracking() fields stay (older PC tools); chrony_status() adds signed CSV
        values with sources/sourcestats for the PC-centred check.
        """
        status = tracking()
        status['chrony'] = chrony_status()
        status['collected_unix'] = status['chrony']['collected_unix']
        status['hostname'] = os.uname().nodename
        self.publisher.publish(String(data=json.dumps(status)))


def main(args=None):
    """Run on the robot alongside the existing hardware launch."""
    rclpy.init(args=args)
    node = TimeSyncStatus()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
