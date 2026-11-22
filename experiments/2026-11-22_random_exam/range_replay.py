"""Replay the native decoder and odometry rays on recorded exam camera frames."""
import argparse
import csv
import json
from collections import Counter, defaultdict
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from osp_mission.marker_pair import decode
from osp_mission.camera_multiview import CameraVisualRange
from balanced_print.distant_hint import DistantHint


def seconds(stamp):
    return stamp.sec + 1e-9 * stamp.nanosec


def transform(row):
    t = row.transform.translation
    q = row.transform.rotation
    mat = np.eye(4)
    mat[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    mat[:3, 3] = [t.x, t.y, t.z]
    return mat


def poses_at(samples, stamps):
    times = np.array(sorted(samples))
    if len(times) < 2 or stamps.min() < times[0] or stamps.max() > times[-1]:
        raise ValueError('The camera stamps are outside the recorded transform interval')
    poses = np.stack([samples[t] for t in times])
    out = np.tile(np.eye(4), (len(stamps), 1, 1))
    for dim in range(3):
        out[:, dim, 3] = np.interp(stamps, times, poses[:, dim, 3])
    out[:, :3, :3] = Slerp(times, Rotation.from_matrix(poses[:, :3, :3]))(stamps).as_matrix()
    return out


def recorded_transforms(bag, stamps):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CameraInfo
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/tf', '/tf_static', '/mission/processed_camera',
                          '/mission/marker_observations', '/mission/marker_candidates']))
    static, base, correction, cameras, accepted, candidates = {}, {}, {}, {}, {}, {}
    try:
        while reader.has_next():
            topic, raw, _ = reader.read_next()
            if topic.endswith('processed_camera'):
                msg = deserialize_message(raw, CameraInfo)
                cameras[round(seconds(msg.header.stamp), 6)] = (msg.header.frame_id, tuple(msg.k), tuple(msg.d))
            elif topic.endswith('marker_observations'):
                row = json.loads(deserialize_message(raw, String).data)
                accepted[(round(row['stamp'], 6), row['id'])] = row
            elif topic.endswith('marker_candidates'):
                row = json.loads(deserialize_message(raw, String).data)
                candidates.setdefault(str(row['id']), []).append(row['stamp'])
            else:
                for row in deserialize_message(raw, TFMessage).transforms:
                    parent, child = row.header.frame_id, row.child_frame_id
                    if topic == '/tf_static':
                        static[child] = (parent, transform(row))
                    elif (parent, child) == ('odom', 'base_link'):
                        base[seconds(row.header.stamp)] = transform(row)
                    elif (parent, child) == ('map', 'odom'):
                        correction[seconds(row.header.stamp)] = transform(row)
    finally:
        reader.close()
    camera_models = {cameras[round(t, 6)] for t in stamps if round(t, 6) in cameras}
    if len(camera_models) != 1:
        raise ValueError('Expected one calibrated camera model during the mission')
    frame, k, d = camera_models.pop()
    extrinsic = np.eye(4)
    while frame != 'base_link':
        frame, step = static[frame]
        extrinsic = step @ extrinsic
    first = max(min(base), min(correction))
    last = min(max(base), max(correction))
    mask = np.array([round(t, 6) in cameras and first <= t <= last for t in stamps])
    return (mask, np.reshape(k, (3, 3)), np.asarray(d), poses_at(base, stamps[mask]) @ extrinsic,
            poses_at(correction, stamps[mask]), accepted, candidates)


def extract(folder, bag):
    metrics = json.loads((folder / 'metrics.json').read_text())
    start, = [e['sim'] for e in map(json.loads, (folder/'events.jsonl').read_text().splitlines())
              if e['event'] == 'start_ack']
    end = min(600., metrics['elapsed_sim_s'])
    with (folder/'frames.csv').open() as csvfile:
        frames = [(int(row['frame']), float(row['elapsed_sim_s'])) for row in csv.DictReader(csvfile)
                  if 0 <= float(row['elapsed_sim_s']) <= end]
    stamps = np.array([start + t for _, t in frames])
    mask, k, d, pose, correction, live, candidates = recorded_transforms(bag, stamps)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    align = metrics['world_from_map_initial_alignment']
    gt = {str(x['id']): [x['x'], x['y']] for x in json.loads((folder/'scene.json').read_text())['targets']}
    rows, loc_errors = [], []
    pos = 0
    for (n, t), valid in zip(frames, mask):
        if not valid:
            continue
        stamp = round(start + t, 6)
        cam, fix = pose[pos], correction[pos]
        pos += 1
        image = cv2.imread(str(folder/'camera'/f'{n:05d}.jpg'), cv2.IMREAD_GRAYSCALE)
        if image is None or image.shape != (metrics['camera']['height'], metrics['camera']['width']):
            raise ValueError(f'Missing camera frame {n}')
        native = []
        corners, ids = decode(image, dictionary, None, native)
        if ids is None:
            continue
        for quad, ident in zip(corners, ids.ravel(), strict=True):
            ident = int(ident)
            q = quad[0]
            if np.any(q.min(axis=0) < 4) or np.any(q.max(axis=0) > np.array(image.shape[::-1]) - 5):
                continue
            pixels = float(np.linalg.norm(q - np.roll(q, 1, axis=0), axis=1).min())
            if pixels < 10.:
                continue
            xy = q.mean(axis=0)
            uv = cv2.undistortPoints(np.float64([[xy]]), k, d).ravel()
            direction = cam[:3,:3] @ np.r_[uv, 1.]
            direction /= np.linalg.norm(direction)
            matched = live.get((stamp, ident))
            if matched is not None and matched.get('odom_point') is not None:
                predicted = cam @ np.r_[matched['camera_point'], 1.]
                loc_errors.append(float(np.linalg.norm(predicted[:3]-matched['odom_point'])))
            rows.append(dict(id=ident, t=t, frame=n, pixels=pixels, uv=xy.tolist(),
                             native=any(i == ident and np.array_equal(nq[0], q) for i, nq, _ in native),
                             origin=cam[:3,3].tolist(), direction=direction.tolist(),
                             map_from_odom=fix.tolist()))
    accepted = defaultdict(list)
    for time, ident in live:
        accepted[str(ident)].append(time)
    def first_times(observations):
        return {ident: min(seq)-start for ident, values in observations.items()
                if (seq := [t for t in values if start <= t <= start+end])}
    return dict(run=folder.name, recorded_frames=len(frames), processed_with_tf=int(mask.sum()),
                registered_s=metrics['registered_s'], live_counts=metrics['last_detector_status']['by_id'],
                first_live_observation_s=first_times(accepted), first_live_candidate_s=first_times(candidates),
                world_from_map=align, target_xy=gt, rows=rows, matched_live_tf=len(loc_errors),
                max_reprojection_to_live_odom_m=max(loc_errors, default=None), cv2=cv2.__version__)


def evaluate(extracted, min_pixels, model_class=CameraVisualRange, native_only=False):
    model = model_class()
    ids = defaultdict(lambda: dict(frames=0, first_decoded_s=None, current_triplets=0, estimates=[]))
    align = extracted['world_from_map']
    for row in extracted['rows']:
        if row['pixels'] < min_pixels or (native_only and not row['native']):
            continue
        ident, stamp = str(row['id']), row['t']
        record = ids[ident]
        record['frames'] += 1
        if record['first_decoded_s'] is None:
            record['first_decoded_s'] = stamp
        origin, direction = np.array(row['origin']), np.array(row['direction'])
        point = model.observe(ident, stamp, origin, direction, row['pixels'])
        recent = model.history.get(ident, [])
        if recent and recent[-1].stamp == stamp and model.window(recent, stamp, origin):
            record['current_triplets'] += 1
        if point is not None:
            point_map = np.array(row['map_from_odom']) @ np.r_[point, 1.]
            pred = np.array(align['rotation']) @ point_map[:2] + align['translation']
            truth = extracted['target_xy'].get(ident)
            error = float(np.linalg.norm(pred - truth)) if truth is not None else None
            record['estimates'].append((stamp, error))
    result = {}
    for ident, record in ids.items():
        hits = record.pop('estimates')
        cutoff = extracted['registered_s'].get(ident)
        result[ident] = dict(record, ranges=len(hits), before_registration=sum(
            cutoff is None or t < cutoff for t, _ in hits), first_range_s=hits[0][0] if hits else None,
            third_range_s=hits[2][0] if len(hits)>=3 else None,
            first_xy_error_m=hits[0][1] if hits else None,
            largest_xy_error_m=max((e for _,e in hits if e is not None), default=None),
            wrong_over_025m=sum(e is None or e > .25 for _,e in hits))
    return result


def dump(path, obj):
    path.write_text(json.dumps(obj, indent=2, allow_nan=False)+'\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('bag', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--cache', type=Path)
    args = parser.parse_args()
    cv2.setNumThreads(1)
    if args.cache and args.cache.is_file():
        receipt = json.loads(args.cache.read_text())
        if receipt['run'] != args.run.name or any('native' not in row for row in receipt['rows']):
            raise ValueError('The exposure cache belongs to another recording')
    else:
        receipt = extract(args.run, args.bag)
        if args.cache:
            dump(args.cache, receipt)
    counts = Counter(str(row['id']) for row in receipt['rows'])
    result = {k:v for k,v in receipt.items() if k not in ('rows','world_from_map','target_xy')}
    result['all_border_valid_10px_decodes'] = dict(counts)
    result['same_18px_gate'] = evaluate(receipt, 18.)
    result['exploratory_10px_geometry_only'] = evaluate(receipt, 10.)
    result['distant_hint_native_only'] = evaluate(receipt, 10., DistantHint, native_only=True)
    result['method'] = ('Frozen JPEG path and processed-camera stamps. Recorded odom-camera TF rays; '
        'the deployed CameraVisualRange uses all eligible prior triplets and applies '
        'all native parallax, depth, residual and pixel-scale gates. The 10px replay is diagnostic only. '
        'Simulator truth measures XY error after estimation. No closed-loop approach or registration credit.')
    dump(args.output, result)
