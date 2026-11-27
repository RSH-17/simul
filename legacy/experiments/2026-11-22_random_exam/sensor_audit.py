"""Read finalized MCAP files to measure camera coverage and terminal Nav2 goal states."""
import argparse
from collections import Counter
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from surface_audit import evaluate


CLOCK = '/clock'
CAMERA = '/mission/processed_camera'
STATUS = '/mission/detector_status'
NAV = '/navigate_to_pose/_action/status'
SPIN = '/spin/_action/status'


def read(path):
    return json.loads(path.read_text())


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1.e-9


def epochs(messages):
    current = None
    last_clock = 0.
    for topic, when, payload in messages:
        if topic == CLOCK:
            # File order is essential: two different Gazebo processes may both publish t=20.
            if current is None or (when < 5. and last_clock > when + 5.):
                if current is not None:
                    yield current
                current = dict(camera=[], terminal=[], diagnostic=[], start_clock=when)
            current['last_clock'] = when
            last_clock = when
        elif current is not None:
            if topic == CAMERA:
                current['camera'].append((when, payload))
            elif topic in (NAV, SPIN):
                current['terminal'].extend((topic, last_clock, *goal) for goal in payload)
            elif topic == STATUS:
                current['diagnostic'].append((last_clock, payload))
    if current is not None:
        yield current


def bag_messages(path):
    import rosbag2_py as bag
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    if not (path / 'metadata.yaml').is_file():
        raise ValueError('Use a finalized sensor recording: ' + str(path))
    reader = bag.SequentialReader()
    reader.open(bag.StorageOptions(uri=str(path), storage_id='mcap'), bag.ConverterOptions('', ''))
    try:
        if not reader.set_read_order(bag.ReadOrder(bag.ReadOrderSortBy.File, False)):
            raise ValueError('MCAP reader does not support file order')
        topics = {topic.name: get_message(topic.type) for topic in reader.get_all_topics_and_types()
                  if topic.name in (CLOCK, CAMERA, STATUS, NAV, SPIN)}
        if not {CLOCK, CAMERA} <= topics.keys():
            raise ValueError('No clock or processed camera topic in recording')
        reader.set_filter(bag.StorageFilter(topics=list(topics)))
        while reader.has_next():
            topic, raw, _ = reader.read_next()
            msg = deserialize_message(raw, topics[topic])
            if topic == CLOCK:
                yield topic, seconds(msg.clock), None
            elif topic == CAMERA:
                yield topic, seconds(msg.header.stamp), (msg.width, msg.height, tuple(msg.k))
            elif topic == STATUS:
                yield topic, 0., json.loads(msg.data).get('decoded', {})
            else:
                goals = [(bytes(g.goal_info.goal_id.uuid).hex(), seconds(g.goal_info.stamp), int(g.status))
                         for g in msg.status_list if g.status in (4, 5, 6)]
                yield topic, 0., goals
    finally:
        reader.close()


def audit_episode(folder, evidence, calibration):
    metrics = read(folder/'metrics.json')
    starts = [e['sim'] for e in (json.loads(line) for line in (folder/'events.jsonl').read_text().splitlines())
              if e['event'] == 'start_ack']
    if len(starts) != 1:
        raise ValueError('Cannot match start event: ' + folder.name)
    start = starts[0]
    end = start + min(600., metrics['elapsed_sim_s'])
    eligible = [(round(t-start, 8), k) for t, k in evidence['camera'] if start <= t <= end]
    stamps = np.unique([t for t, _ in eligible])
    expected = metrics['camera']['processed_frames']
    missing = []
    if not len(stamps) or abs(len(stamps)-expected) > max(3, .02 * expected):
        missing.append('processed_exposure_count_mismatch')
    if evidence['start_clock'] > start+.5 or evidence['last_clock'] < end-.5:
        missing.append('clock_window_incomplete')
    for width, height, k in {k for _, k in eligible}:
        if ((width, height) != (calibration['width'], calibration['height'])
                or not np.allclose(k, calibration['k'], rtol=0., atol=1.e-5)):
            missing.append('camera_calibration_mismatch')
    terminal = {NAV: {}, SPIN: {}}
    for topic, receipt, uid, accepted, status in evidence['terminal']:
        if start <= accepted <= end and start <= receipt <= end+.5:
            terminal[topic][uid] = status
    decoded = [counts for t, counts in evidence['diagnostic'] if start <= t <= end + .5]
    observed_ids = {i for counts in decoded for i in counts}
    logged_ids = set(metrics['last_detector_status'].get('decoded', {}))
    if logged_ids != observed_ids:
        missing.append('decoder_ids_at_epoch_boundary_disagree')
    def nav_counts(values):
        counts = Counter(values.values())
        return dict(succeeded=counts[4], canceled=counts[5], aborted=counts[6])
    result = dict(run=folder.name, valid=not missing, reasons=missing, processed_stamps=len(stamps),
                  observer_processed_frames=expected, navigate=nav_counts(terminal[NAV]),
                  spin=nav_counts(terminal[SPIN]))
    if not missing:
        track = np.loadtxt(folder/'headings.csv', skiprows=1, delimiter=',', ndmin=2)
        world = ET.parse(folder/'world/osp_studio.sdf').find('world')
        result['surface_coverage'] = evaluate(read(folder/'scene.json'), world, track, stamps, calibration)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('exam', type=Path)
    parser.add_argument('recordings', type=Path)
    parser.add_argument('--continuous-bag', type=Path)
    parser.add_argument('--calibration', type=Path, default=Path(__file__).resolve().parent.parent /
                        '2026-11-21_v151_track5_boxes5/camera_geometry.json')
    args = parser.parse_args()
    rows = read(args.exam/'episodes.json')
    calibration = read(args.calibration)
    if args.continuous_bag:
        segments = list(epochs(bag_messages(args.continuous_bag)))
    else:
        segments = []
        for row in rows:
            chunk = list(epochs(bag_messages(args.exam/'sensor-recordings'/row['name'])))
            if len(chunk) != 1:
                raise ValueError('A per-episode bag contains several clock epochs')
            segments.extend(chunk)
    if len(segments) != len(rows):
        raise ValueError(f'{len(segments)} clock epochs for {len(rows)} registered episodes')
    destination = args.exam/'sensor-audit'
    destination.mkdir(exist_ok=True)
    for row, segment in zip(rows, segments, strict=True):
        result = audit_episode(args.recordings/row['name'], segment, calibration)
        path = destination/(row['name']+'.json')
        temporary = path.with_suffix('.new')
        temporary.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
        temporary.replace(path)


if __name__ == '__main__':
    main()
