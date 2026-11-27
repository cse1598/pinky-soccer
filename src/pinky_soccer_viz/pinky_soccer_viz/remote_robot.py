"""Optional SSH lifecycle for existing robot bringup; no code deployment or credentials."""
import argparse
import os
import shlex
import signal
import subprocess
import sys


def remote_command(workspace, name='', profile=''):
    """Quote all user values; only this process group is stopped on stdin EOF."""
    if not workspace or '\n' in workspace or '\n' in name or '\n' in profile:
        raise ValueError('invalid remote launch argument')
    args = ['ros2', 'launch', 'pinky_soccer_bringup', 'robot.launch.py',
            'robot_name:='+name, 'imu_enabled:=true', 'led_enabled:=true']
    if profile:
        args.append('calibration_profile:='+profile)
    # $HOME is read, never reassigned. Workspace is a quoted path under it unless absolute.
    location = shlex.quote(workspace) if workspace.startswith('/') else '"$HOME"/'+shlex.quote(workspace)
    domain = int(os.environ.get('ROS_DOMAIN_ID', '0'))
    if not 0 <= domain <= 232:
        raise ValueError('invalid ROS_DOMAIN_ID')
    return ('set -e\nexport ROS_DOMAIN_ID='+str(domain)+'\nsource /opt/ros/jazzy/setup.bash\ncd '+location+
        '\nsource install/setup.bash\nsetsid '+shlex.join(args)+' &\nchild=$!\n'
        'cleanup() { kill -INT -- -"$child" 2>/dev/null || true;\n'
        'for i in {1..30}; do kill -0 "$child" 2>/dev/null || return; sleep .1; done;\n'
        'kill -TERM -- -"$child" 2>/dev/null || true; wait "$child" || true; }\ntrap cleanup EXIT\n'
        'while kill -0 "$child" 2>/dev/null; do\n'
        'if IFS= read -r -t 1 line; then [ "$line" != stop ] || break;\n'
        'else code=$?; [ "$code" -gt 128 ] || break; fi; done\n')


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--host', required=True)
    p.add_argument('--workspace', default='pinky-soccer')
    p.add_argument('--name', default='')
    p.add_argument('--profile', default='')
    args, _ = p.parse_known_args()
    if args.host.startswith('-') or any(c.isspace() for c in args.host):
        p.error('host must be user@host or an SSH alias')
    script = remote_command(args.workspace, args.name, args.profile)
    command = ['ssh', '-T', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8',
               args.host, 'bash -lc '+shlex.quote(script)]
    child = subprocess.Popen(command, stdin=subprocess.PIPE, start_new_session=True)
    def stop(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop)
    try:
        code = child.wait()
    except KeyboardInterrupt:
        if child.stdin:
            child.stdin.close()  # remote lifecycle ends on EOF
        try:
            code = child.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(child.pid, signal.SIGTERM)
            code = child.wait(timeout=3)
    finally:
        if child.stdin and not child.stdin.closed:
            child.stdin.close()
    if code:
        print('Robot SSH bringup exited; PC stays stopped until preparation succeeds', file=sys.stderr)
    return code
