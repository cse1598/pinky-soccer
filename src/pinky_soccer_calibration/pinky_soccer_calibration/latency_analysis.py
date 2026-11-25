"""Latency statistics for ping-pong and the real drive-command path (raw data kept).

Rules: round trips use one device's monotonic clock only; one-way values use unix times of
two devices and are reported only when the clock check was verified for the whole run;
negative one-way values are kept (they indicate clock error) and never clipped to zero.
Values that could not be observed are absent, never zero.
"""
import csv
import json
import math
from pathlib import Path


def summary(values):
    v = sorted(x for x in values if x is not None and math.isfinite(x))
    if not v:
        return None
    n = len(v)
    mean = sum(v) / n

    def q(p):
        return v[min(n - 1, max(0, math.ceil(p * n) - 1))]
    std = math.sqrt(sum((x - mean) ** 2 for x in v) / (n - 1)) if n > 1 else 0.
    return {'n': n, 'mean': mean, 'std': std, 'min': v[0], 'p50': q(.5), 'p95': q(.95),
            'p99': q(.99), 'max': v[-1]}


def pingpong(sent, replies, sync_verified):
    """sent: {seq: {pc_send_mono_ns, pc_send_unix_ns}}; replies: [(pc_recv_mono_ns,
    pc_recv_unix_ns, reply_dict)] in arrival order. Returns statistics in seconds."""
    seen, duplicates, order, rows = set(), 0, [], []
    for recv_mono, recv_unix, reply in replies:
        seq = reply.get('seq')
        if seq not in sent:
            continue
        if seq in seen:
            duplicates += 1
            continue
        seen.add(seq)
        order.append(seq)
        s = sent[seq]
        rtt = (recv_mono - s['pc_send_mono_ns']) * 1e-9
        proc = (reply['robot_send_mono_ns'] - reply['robot_recv_mono_ns']) * 1e-9
        row = {'seq': seq, 'rtt_s': rtt, 'robot_processing_s': proc, 'net_rtt_s': rtt - proc}
        if sync_verified:
            row['pc_to_robot_s'] = (reply['robot_recv_unix_ns'] - s['pc_send_unix_ns']) * 1e-9
            row['robot_to_pc_s'] = (recv_unix - reply['robot_send_unix_ns']) * 1e-9
        rows.append(row)
    reordered = sum(1 for a, b in zip(order, order[1:]) if b < a)
    out = {'sent': len(sent), 'received': len(seen), 'lost': len(sent) - len(seen),
           'loss_ratio': (len(sent) - len(seen)) / len(sent) if sent else None,
           'duplicates': duplicates, 'reordered': reordered,
           'one_way_status': 'measured (clock verified)' if sync_verified else
           'not_measured: clock sync not verified for the whole run'}
    for key in ('rtt_s', 'robot_processing_s', 'net_rtt_s', 'pc_to_robot_s', 'robot_to_pc_s'):
        stats = summary([r[key] for r in rows if key in r])
        if stats:
            out[key] = stats
    if sync_verified:
        out['negative_one_way'] = sum(1 for r in rows for k in ('pc_to_robot_s', 'robot_to_pc_s')
                                      if r.get(k, 0) < 0)
    return out, rows


def command_path(folder, sync_verified):
    """Match tagged PC commands (commands.csv) to driver receipts in motor/diagnostics.

    Measured on the robot clock only: watchdog republish + driver handling are inside the
    PC->driver one-way value; serial write duration is measured on the robot. PC->driver
    one-way needs a verified clock. Motor application and wheel response are not observed
    directly here (estimated elsewhere from the encoder), so they are not reported.
    """
    folder = Path(folder)
    sent = {}
    with (folder / 'commands.csv').open() as f:
        for row in csv.DictReader(f):
            tag = float(row.get('command_tag') or 0)
            if tag > 0:
                sent[tag] = row
    first, repeats = {}, {}
    with (folder / 'observations.csv').open() as f:
        for row in csv.DictReader(f):
            if row['topic'] != '/motor/diagnostics':
                continue
            data = json.loads(row['observation_json']).get('payload') or {}
            for cmd in data.get('commands', []):
                tag = float(cmd.get('tag') or 0)
                if tag in sent:
                    repeats[tag] = repeats.get(tag, 0) + 1
                    first.setdefault(tag, cmd)
    if not first:
        return {'status': 'not_measured: no tagged driver receipts (motor_diagnostics:=true on '
                          'the robot and a tagged experiment are required)',
                'commands_sent': len(sent)}
    write = [(c['written_unix_ns'] - c['recv_unix_ns']) * 1e-9 for c in first.values()]
    out = {'commands_sent': len(sent), 'commands_matched': len(first),
           'driver_receipts_per_command': summary(list(repeats.values())),
           'driver_serial_write_s': summary(write),
           'matching': 'exact (Twist.linear.z tag)', 'status': 'measured'}
    if sync_verified:
        out['pc_publish_to_driver_receipt_s'] = summary(
            [(c['recv_unix_ns'] - int(sent[t]['publish_unix_ns'])) * 1e-9 for t, c in first.items()])
    else:
        out['pc_publish_to_driver_receipt_s'] = 'not_measured: clock sync not verified'
    return out
