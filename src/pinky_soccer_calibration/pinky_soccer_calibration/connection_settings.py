"""Connection settings of the calibration desktop (separate from calibration results).

PC-centred time architecture (2026-11-25): internet NTP -> this PC (chrony client + LAN
server) -> every Pinky (chrony client of the PC). Settings are user input; local interface
discovery is shown as a reference and never overwrites what the user typed. Nothing here
runs privileged commands or rewrites system/DDS files: it only generates text to copy.
"""
from dataclasses import asdict, dataclass, field
import ipaddress
import os
from pathlib import Path
import re
import socket

import yaml

DEFAULT_PATH = Path(os.environ.get('XDG_CONFIG_HOME', Path.home() / '.config')) / \
    'pinky_soccer' / 'calibration_connection.yaml'
NAME = re.compile(r'^[A-Za-z0-9_][A-Za-z0-9_\-]{0,63}$')


@dataclass
class RobotConnection:
    robot_id: str = 'pinky-pro'
    ip: str = ''
    namespace: str = ''
    ssh_user: str = 'pinky'
    interface: str = 'wlan0'


@dataclass
class ConnectionSettings:
    pc_ip: str = ''
    pc_interface: str = ''
    ros_domain_id: str = os.environ.get('ROS_DOMAIN_ID', '0')
    rmw_implementation: str = os.environ.get('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
    robots: list = field(default_factory=lambda: [RobotConnection()])
    selected: int = 0

    @property
    def robot(self):
        return self.robots[self.selected]

    def validate(self):
        """Return a list of problems (empty: usable); never raises for partial input."""
        problems = []
        for label, value in (('PC IP', self.pc_ip), ('Pinky IP', self.robot.ip)):
            try:
                if ipaddress.ip_address(value).version != 4:
                    problems.append(f'{label}: IPv4 주소를 입력하세요')
            except ValueError:
                problems.append(f'{label}: 올바른 IPv4 주소가 아닙니다 ({value!r})')
        if self.pc_ip and self.pc_ip == self.robot.ip:
            problems.append('PC IP와 Pinky IP가 같습니다')
        if not self.ros_domain_id.isdigit() or not 0 <= int(self.ros_domain_id) <= 232:
            problems.append('ROS_DOMAIN_ID는 0~232 정수입니다')
        if not NAME.match(self.robot.robot_id or ''):
            problems.append('로봇 식별명: 영문/숫자/_/- 만 사용하세요')
        ns = self.robot.namespace.strip('/')
        if ns and not NAME.match(ns):
            problems.append('namespace: 영문/숫자/_/- 만 사용하세요')
        ids = [r.robot_id for r in self.robots]
        if len(ids) != len(set(ids)):
            problems.append('같은 로봇 식별명이 여러 번 등록되어 있습니다')
        return problems


def load(path=DEFAULT_PATH):
    path = Path(path)
    if not path.is_file():
        return ConnectionSettings()
    data = yaml.safe_load(path.read_text()) or {}
    robots = [RobotConnection(**{k: str(v) for k, v in r.items()
                                 if k in RobotConnection.__dataclass_fields__})
              for r in data.get('robots') or []] or [RobotConnection()]
    out = ConnectionSettings(**{k: str(data[k]) for k in
                                ('pc_ip', 'pc_interface', 'ros_domain_id', 'rmw_implementation')
                                if k in data}, robots=robots)
    out.selected = min(max(int(data.get('selected', 0)), 0), len(robots) - 1)
    return out


def save(settings, path=DEFAULT_PATH):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    data = asdict(settings)
    tmp = path.with_suffix('.tmp')
    tmp.write_text(yaml.safe_dump(data, allow_unicode=True, sort_keys=False))
    tmp.replace(path)
    return path


def local_interfaces():
    """[(interface, IPv4)] of this PC, for reference only."""
    try:
        import psutil
        return sorted((name, a.address) for name, addrs in psutil.net_if_addrs().items()
                      for a in addrs if a.family == socket.AF_INET and name != 'lo')
    except ImportError:
        return []


def runtime_mismatches(settings, node_namespace, node_pc_ip=None):
    """Differences between saved settings and what the running ROS process uses.

    DDS/ROS settings are read when the process starts; changing them here needs a restart.
    """
    out = []
    env_domain = os.environ.get('ROS_DOMAIN_ID', '0')
    if settings.ros_domain_id != env_domain:
        out.append(f'ROS_DOMAIN_ID 설정 {settings.ros_domain_id} ≠ 실행 중 {env_domain}')
    env_rmw = os.environ.get('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
    if settings.rmw_implementation != env_rmw:
        out.append(f'RMW 설정 {settings.rmw_implementation} ≠ 실행 중 {env_rmw}')
    ns = settings.robot.namespace.strip('/')
    if ('/' + ns if ns else '') != node_namespace:
        out.append(f"namespace 설정 '{ns}' ≠ 실행 중 '{node_namespace.strip('/')}'")
    return out


def commands(settings):
    """Copyable setup/check commands for the current input (no command is executed here)."""
    pc, robot = settings.pc_ip or '<PC_IP>', settings.robot
    rip, user = robot.ip or '<PINKY_IP>', robot.ssh_user or 'pinky'
    ns = robot.namespace.strip('/')
    iface = settings.pc_interface or '<PC_인터페이스>'
    return f"""# ===== 1) PC (시간 기준 서버): /etc/chrony/chrony.conf =====
# 인터넷 NTP(pool/server) 줄은 그대로 둡니다. Pinky를 server로 참조하는 줄이 있으면 지웁니다(순환 금지).
allow {rip}
# 적용·확인 (관리자 권한은 사용자가 직접 실행)
sudo systemctl restart chrony
chronyc -n tracking ; chronyc -n clients

# ===== 2) Pinky ({robot.robot_id}, {rip}): /etc/chrony/chrony.conf =====
ssh {user}@{rip}
server {pc} iburst prefer minpoll 2 maxpoll 4
# 인터넷 pool은 예비로 남겨도 됩니다. 단 예비 서버로 전환되면 UI는 'PC 기준 아님'(NOT READY)으로 표시합니다.
sudo systemctl restart chrony
chronyc -n sources ; chronyc -n tracking     # '^*' 가 {pc} 이어야 합니다
# 실험 중 makestep/date 로 시계를 강제로 바꾸지 마세요.

# ===== 3) 방화벽 (사용 시): PC에서 UDP 123 허용 =====
sudo ufw allow from {rip} to any port 123 proto udp

# ===== 4) CycloneDDS peer (기존 파일을 덮어쓰지 말고 해당 항목만 추가) =====
# PC   cyclonedds.xml: <NetworkInterface name="{iface}"/>  <Peer address="{rip}"/>
# Pinky cyclonedds.xml: <NetworkInterface name="{robot.interface or 'wlan0'}"/>  <Peer address="{pc}"/>
export ROS_DOMAIN_ID={settings.ros_domain_id}
export RMW_IMPLEMENTATION={settings.rmw_implementation}

# ===== 5) 로봇 bringup (Pinky) =====
ros2 launch pinky_soccer_bringup robot.launch.py{' robot_name:=' + ns if ns else ''}

# ===== 6) 이 UI를 같은 설정으로 다시 시작 (namespace/DDS는 시작 시에만 적용) =====
ROS_DOMAIN_ID={settings.ros_domain_id} RMW_IMPLEMENTATION={settings.rmw_implementation} \\
  ros2 launch pinky_soccer_calibration calibration_ui.launch.py \\
  robot_namespace:={ns or "''"} pc_ip:={pc}
"""
