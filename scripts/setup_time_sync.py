#!/usr/bin/env python3
"""One-time Ubuntu chrony provisioning. Run on each intended machine, never through ROS."""

import argparse
import ipaddress
import os
from pathlib import Path
import shutil
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('role', choices=['robot', 'pc'])
    parser.add_argument('--robot-address', default='192.168.4.1')
    parser.add_argument('--robot-network', default='192.168.4.0/24')
    parser.add_argument('--apply', action='store_true', help='Install/configure chrony as root')
    args = parser.parse_args()
    address = str(ipaddress.IPv4Address(args.robot_address))
    network = str(ipaddress.IPv4Network(args.robot_network))
    config = ('# Pinky isolated robot network: relative synchronization, not UTC certification\n'
              + (f'allow {network}\nlocal stratum 10\n' if args.role == 'robot' else
                 f'server {address} iburst prefer minpoll 2 maxpoll 4\n'))
    # No makestep directive: normal operation slews clocks, avoiding recording discontinuities.
    print(config)
    if not args.apply:
        print('Preview only. Run with sudo and --apply on the intended host once.')
        return
    if os.geteuid() != 0:
        parser.error('--apply requires sudo on the intended host')
    subprocess.run(['apt-get', 'install', '-y', 'chrony'], check=True)
    main_config = Path('/etc/chrony/chrony.conf')
    backup = main_config.with_suffix('.conf.before-pinky')
    if not backup.exists():
        shutil.copy2(main_config, backup)
    text = main_config.read_text()
    if 'confdir /etc/chrony/conf.d' not in text:
        with main_config.open('a') as stream:
            stream.write('\nconfdir /etc/chrony/conf.d\n')
    folder = Path('/etc/chrony/conf.d')
    folder.mkdir(exist_ok=True)
    path = folder/'pinky.conf'
    if path.exists() and path.read_text() != config:
        shutil.copy2(path, folder/'pinky.conf.before-pinky')
    path.write_text(config)
    subprocess.run(['systemctl', 'enable', '--now', 'chrony'], check=True)
    subprocess.run(['systemctl', 'restart', 'chrony'], check=True)
    print('Automatic synchronization enabled. UDP/123 must be reachable; wait for convergence.')


if __name__ == '__main__':
    main()
