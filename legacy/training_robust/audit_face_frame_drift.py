"""Audit stale face-sweep directions using recorded map/odom transforms only.

This is an offline direction comparison, not a closed-loop recognition result.
No scene file, marker position or evaluator ground truth enters the replay.
"""
import argparse
import csv
import hashlib
import importlib.util
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
from scipy.spatial.transform import Rotation


def matched_transforms(rows, maximum_skew=.04):
    fields = ('map_stamp', 'map_yaw', 'odom_stamp', 'odom_yaw')
    a = np.array([[float(r[k]) for k in fields] for r in rows])
    a = a[np.isfinite(a).all(axis=1) & (np.abs(a[:, 0]-a[:, 2]) <= maximum_skew)]
    if len(a) < 2:
        raise ValueError('Need at least two time-matched map/odom poses')
    a = a[np.argsort(a[:, 0], kind='stable')]
    a = a[np.r_[np.diff(a[:, 0]) > 0, True]]
    return np.column_stack((a[:, 0], np.unwrap(a[:, 1]-a[:, 3])))


def at_time(samples, stamp, maximum_gap=.35):
    i = int(np.searchsorted(samples[:, 0], stamp))
    if i < len(samples) and abs(samples[i, 0]-stamp) < 1e-9:
        return float(samples[i, 1])
    if i == 0 or i >= len(samples) or samples[i, 0]-samples[i-1, 0] > maximum_gap:
        raise ValueError('No bounded interpolation at event time')
    return float(np.interp(stamp, samples[:, 0], samples[:, 1]))


def replay_turn(cls, end_map_yaw, sweep, camera_map_yaw):
    # Real production method, a local optical-camera transform and local events.
    # No ROS node or broadcaster is constructed.
    a = camera_map_yaw
    q = Rotation.from_matrix([[math.sin(a), 0., math.cos(a)],
                              [-math.cos(a), 0., math.sin(a)], [0., -1., 0.]]).as_quat()
    tr = NS(transform=NS(rotation=NS(**dict(zip(('x', 'y', 'z', 'w'), q)))))
    requested = []
    def lookup(frame, camera, time):
        requested.append((frame, camera))
        return tr
    owner = NS(p=lambda key: 'map' if key == 'map_frame' else None,
               tf=NS(lookup_transform=lookup), inspection_camera_frame='rgb_optical',
               get_logger=lambda: NS(info=lambda *a: None))
    look = cls(owner)
    look.event = lambda *args, **kwargs: None
    look.select(dict(face_inspection=True,yaw=end_map_yaw-sweep,scan_angle=sweep), None)
    turn = look.consume()
    if requested != [('map', 'rgb_optical')]:
        raise AssertionError('Candidate did not use current map camera orientation')
    return turn


def audit(episodes, candidate):
    source = candidate/'src/osp_mission/osp_mission/face_directed_look.py'
    spec = importlib.util.spec_from_file_location('replayed_face_look', source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    rows, excluded, hashes = [], [], {str(source): hashlib.sha256(source.read_bytes()).hexdigest()}
    for episode in episodes:
        pose_path = episode/'pose_trace.csv'
        event_path = episode/'face_directed_looks.jsonl'
        samples = matched_transforms(list(csv.DictReader(pose_path.open())))
        for p in (pose_path, event_path):
            hashes[str(p)] = hashlib.sha256(p.read_bytes()).hexdigest()
        task = None
        for line in event_path.read_text().splitlines():
            event = json.loads(line)
            if event['event'] == 'selected':
                task = event
            elif event['event'] in ('cleared', 'unavailable'):
                task = None
            elif event['event'] == 'consumed' and task is not None:
                selection, task = task, None
                try:
                    end_odom = selection['end_odom_yaw']
                    if end_odom is None or not math.isclose(end_odom, event['end_odom_yaw'], abs_tol=1e-9):
                        raise ValueError('Selection and consumption do not share a valid endpoint')
                    # At a clamped command the camera heading is not recoverable
                    # from endpoint minus requested turn; reject that inference.
                    if abs(event['turn_rad']) >= 2*math.pi/3-1e-6:
                        raise ValueError('Original turn was clamped')
                    first = at_time(samples, selection['stamp'])
                    last = at_time(samples, event['stamp'])
                    end_map = end_odom+first
                    camera_map = end_odom-event['turn_rad']+last
                    proposed = replay_turn(module.FaceDirectedLook, end_map,
                                           selection['planned_sweep_rad'], camera_map)
                    rows.append(dict(episode=str(episode),selected_sim_s=selection['stamp'],
                        consumed_sim_s=event['stamp'],navigation_wait_s=event['stamp']-selection['stamp'],
                        map_from_odom_yaw_change_deg=math.degrees(last-first),
                        original_turn_deg=math.degrees(event['turn_rad']),
                        candidate_turn_deg=math.degrees(proposed),
                        candidate_current_map_camera_lookup=True))
                except (ValueError, KeyError, TypeError) as exc:
                    excluded.append(dict(episode=str(episode),event=event,reason=str(exc)))
    return dict(evidence_type='Offline replay of original face-sweep events; no new native trajectory',
        rows=rows,excluded=excluded,map_odom_maximum_timestamp_skew_s=.04,maximum_interpolation_gap_s=.35,
        source_sha256=hashes,ground_truth_used=False,ros_nodes_created=False,
        recognition_improvement_established=False,production_promoted=False,
        limits=['Map/odom transforms are reconstructed from sparse time-matched pose traces',
                'Current camera orientation is inferred from an unclamped logged original command',
                'Map-frame goal orientation matches Nav2 intent; physical face association after loop closure is unverified',
                'No alternate future camera frames, marker detections or completion outcome are predicted'])


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--episode', type=Path, action='append', required=True)
    p.add_argument('--candidate', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    result = audit([e.resolve() for e in a.episode], a.candidate.resolve())
    a.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(dict(replayed=len(result['rows']),excluded=len(result['excluded']),
        largest_frame_yaw_change_deg=max((abs(r['map_from_odom_yaw_change_deg']) for r in result['rows']),default=None))))
