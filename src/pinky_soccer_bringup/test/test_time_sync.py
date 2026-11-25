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


# ---- PC-centred architecture (2026-11-25) ----------------------------------------------
from pinky_soccer_bringup.time_sync import (parse_sources_csv, parse_sourcestats_csv,  # noqa: E402
                                            parse_tracking_csv, pc_reference_check,
                                            selected_server)

NOW = 1795607300.0
PC_TRACK = ('7A2D3218,122.45.50.24,2,1795607295.5,-0.000010430,-0.000042189,0.000422134,'
            '32.389,0.040,0.619,0.004183774,0.000950075,129.3,Normal')
ROBOT_TRACK = ('C0A8CC1B,192.168.204.27,3,1795607290.0,0.000120000,0.000050000,0.000300000,'
               '1.0,0.01,0.2,0.009775,0.0012,16.0,Normal')
ROBOT_SOURCES = ('^,*,192.168.204.27,2,4,377,3,0.000108,0.000108,0.0035\n'
                 '^,-,185.125.190.56,2,8,377,158,0.0012,0.0012,0.12')
ROBOT_STATS = ('192.168.204.27,46,20,180,0.01,0.05,0.000108,0.001153\n'
               '185.125.190.56,23,12,1626,-2.1,7.2,0.0053,0.0047')


def robot_status(**changes):
    status = parse_tracking_csv(ROBOT_TRACK, now=NOW)
    status.update(sources=parse_sources_csv(ROBOT_SOURCES),
                  sourcestats=parse_sourcestats_csv(ROBOT_STATS), collected_unix=NOW - 1)
    status.update(changes)
    return status


def test_csv_parsers():
    pc = parse_tracking_csv(PC_TRACK, now=NOW)
    assert pc['reference'] == '122.45.50.24' and pc['synchronized']
    assert pc['system_offset_s'] < 0                       # signed, unlike parse_tracking
    rows = parse_sources_csv(ROBOT_SOURCES)
    assert rows[0]['reach'] == 255 and rows[0]['state'] == '*'
    assert selected_server({'sources': rows}) == '192.168.204.27'
    assert parse_sourcestats_csv(ROBOT_STATS)['192.168.204.27']['samples'] == 46


def test_pc_centred_ready_and_failure_modes():
    pc = parse_tracking_csv(PC_TRACK, now=NOW)
    ok = pc_reference_check(pc, robot_status(), '192.168.204.27', now=NOW)
    assert ok['robot_uses_pc'] and ok['relative_verified']
    # link 9.775 - 4.184 ms -> half 2.80 ms; + 0.12 + 0.30 + 1.153 ms
    assert abs(ok['relative_bound_s'] - 0.004368) < 2e-5
    # robot follows an internet fallback server -> not relative-ready
    other = robot_status(sources=parse_sources_csv(ROBOT_SOURCES.replace('^,*', '^,-')
                                                   .replace('^,-,185', '^,*,185')))
    bad = pc_reference_check(pc, other, '192.168.204.27', now=NOW)
    assert not bad['robot_uses_pc'] and not bad['relative_verified']
    # wrong IP typed by the user
    assert not pc_reference_check(pc, robot_status(), '192.168.204.99', now=NOW)['relative_verified']
    assert not pc_reference_check(pc, robot_status(), 'not-an-ip', now=NOW)['relative_verified']
    # stale robot status, missing status, legacy robot without sourcestats
    assert not pc_reference_check(pc, robot_status(collected_unix=NOW - 60), '192.168.204.27',
                                  now=NOW)['robot_status_fresh']
    assert not pc_reference_check(pc, None, '192.168.204.27', now=NOW)['relative_verified']
    legacy = pc_reference_check(pc, robot_status(sourcestats=None), '192.168.204.27', now=NOW)
    assert legacy['robot_uses_pc'] and not legacy['relative_verified']
    # quality above the unchanged 5 ms gate
    noisy = robot_status(sourcestats=parse_sourcestats_csv(ROBOT_STATS.replace('0.001153', '0.004')))
    assert not pc_reference_check(pc, noisy, '192.168.204.27', now=NOW)['relative_verified']
    # PC without chrony
    assert not pc_reference_check({'synchronized': False, 'error': 'tracking: missing'},
                                  robot_status(), '192.168.204.27', now=NOW)['relative_verified']
