import shlex
import subprocess
from pinky_soccer_viz.remote_robot import remote_command


def test_shell_values_are_quoted_and_only_owned_process_group_is_stopped(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '37')
    script = remote_command('folder with spaces/$(echo unsafe)', 'yellow', '/tmp/profile with space.yaml')
    assert 'export ROS_DOMAIN_ID=37' in script
    assert shlex.quote('folder with spaces/$(echo unsafe)') in script
    assert 'robot_name:=yellow' in script
    assert 'kill -INT -- -"$child"' in script
    assert 'pkill' not in script and 'killall' not in script
    assert subprocess.run(['bash','-n'],input=script,text=True,capture_output=True).returncode == 0
