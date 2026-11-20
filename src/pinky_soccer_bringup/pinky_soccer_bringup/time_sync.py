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


class TimeSyncStatus(Node):
    """Expose robot clock health over ROS while chronyd automatically disciplines clocks."""

    def __init__(self):
        """Keep this node on system time, independently of sensor drivers."""
        super().__init__('pinky_time_sync_status')
        self.publisher = self.create_publisher(String, 'calibration/time_sync_status', 10)
        self.create_timer(5., self.publish_status)
        self.publish_status()

    def publish_status(self):
        """Report missing chrony without treating it as synchronized."""
        self.publisher.publish(String(data=json.dumps(tracking())))


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
