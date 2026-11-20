"""Clock readiness must not be inferred from ROS receipt timestamps alone."""

from datetime import datetime, timezone

from pinky_soccer_bringup.time_sync import parse_tracking, relative_ready


def fixture_text():
    now = datetime.now(timezone.utc).strftime('%a %b %d %H:%M:%S %Y')
    return f"""Reference ID : C0A80401 (192.168.4.1)
Stratum : 11
Ref time (UTC) : {now}
System time : 0.0001 seconds slow of NTP time
Last offset : 0.0001 seconds
RMS offset : 0.0002 seconds
Root delay : 0.001 seconds
Root dispersion : 0.0003 seconds
Leap status : Normal"""


def test_clock_quality_and_source_checked():
    pc = parse_tracking(fixture_text())
    robot = {'error': ''}
    assert relative_ready(pc, robot)
    assert not relative_ready(pc, None)
    pc['reference_id'] = '01020304 (1.2.3.4)'
    assert not relative_ready(pc, robot)
    pc = parse_tracking(fixture_text().replace('Normal', 'Not synchronised'))
    assert not relative_ready(pc, robot)
    pc = parse_tracking(fixture_text().replace('0.0003', '0.1'))
    assert not relative_ready(pc, robot)
