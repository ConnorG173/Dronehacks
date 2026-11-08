#!/usr/bin/env python3
"""Summarize a settled USB run; host writes alone are not PX4 receipt proof."""
import argparse
from collections import Counter, defaultdict
import json
from pathlib import Path


def rows(path):
    with Path(path).open() as source:
        for line in source:
            try:
                yield json.loads(line)
            except json.JSONDecodeError:
                break  # A live recorder may be writing its final line.


def distribution(values):
    if not values:
        return None
    values = sorted(values)
    return dict(min=round(values[0], 3),
                median=round(values[len(values)//2], 3),
                p99=round(values[min(len(values)-1, int(len(values)*.99))], 3),
                max=round(values[-1], 3))


def timing(samples):
    stamps = [row['wall_ns'] for row in samples]
    if len(stamps) < 2:
        return dict(count=len(stamps))
    gaps = [(b-a)/1e6 for a, b in zip(stamps, stamps[1:])]
    ages = [(row['wall_ns']-row['sample_ns'])/1e6
            for row in samples if 'sample_ns' in row]
    return dict(count=len(stamps),
                hz=round((len(stamps)-1)/((stamps[-1]-stamps[0])/1e9), 3),
                gap_ms=distribution(gaps), sample_age_ms=distribution(ages))


def changes(samples, key):
    previous = None
    result = []
    for row in samples:
        value = row.get(key)
        if value != previous:
            result.append(dict(wall_ns=row['wall_ns'], value=value))
            previous = value
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--trace', required=True)
    parser.add_argument('--sent', required=True)
    parser.add_argument('--settle-seconds', type=float, default=60)
    parser.add_argument('--out', required=True)
    args = parser.parse_args()
    sent = list(rows(args.sent))
    start = sent[0]['wall_ns'] + int(args.settle_seconds*1e9)
    end = sent[-1]['wall_ns']
    streams = defaultdict(list)
    latest_trace = 0
    for row in rows(args.trace):
        latest_trace = max(latest_trace, row['wall_ns'])
        if start <= row['wall_ns'] <= end:
            streams[row['stream']].append(row)
    end = min(end, latest_trace)
    writes = defaultdict(list)
    for row in sent:
        if start <= row['wall_ns'] <= end:
            if 'sample_us' in row:
                row['sample_ns'] = row['sample_us']*1000
            writes[row['kind']].append(row)
    report = dict(start_wall_ns=start, end_wall_ns=end,
                  duration_s=round((end-start)/1e9, 3),
                  excluded_initial_seconds=args.settle_seconds,
                  host_streams={name: timing(samples) for name, samples in streams.items()},
                  usb_completed_writes={name: timing(samples) for name, samples in writes.items()},
                  state_changes={})
    for stream, keys in {
        'local': ['xy_valid', 'z_valid', 'xy_reset_counter', 'heading_reset_counter'],
        'attitude': ['quat_reset_counter'],
        'fusion': ['cs_ev_pos', 'cs_ev_yaw', 'cs_mag_hdg', 'reject_hor_pos', 'reject_yaw'],
        'failsafe': ['offboard_control_signal_lost', 'local_position_invalid'],
        'status': ['arming_state', 'nav_state'],
        'landed': ['landed'],
        'tracker': ['vo_state'],
        'vision': ['reset_counter']
    }.items():
        report['state_changes'][stream] = {key: changes(streams[stream], key) for key in keys}
    report['old_dds_input_counts'] = {name: len(streams[name]) for name in
                                    ('dds_vision', 'dds_heartbeat', 'dds_setpoint')}
    for stream, write_kind, trace_key, write_key in (
        ('vision', 'vision', 'sample_ns', 'sample_us'),
        ('setpoint', 'setpoint', 'message_ns', 'publish_us')):
        source = {row[trace_key]//1000 for row in streams[stream] if trace_key in row}
        output = {row[write_key] for row in writes[write_kind]}
        report[stream+'_matching'] = dict(source_samples=len(source), written_samples=len(output),
            matched=len(source & output),
            source_only=len(source-output), written_only=len(output-source))
    Path(args.out).write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
