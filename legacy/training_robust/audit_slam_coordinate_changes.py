"""Evaluator-only audit of native poses, finalized graphs and SLAM replays.

Truth never enters the mapper or landmark processing. Original alignment is
fixed; missing truth intervals are excluded rather than interpolated over.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import yaml

from audit_pose_trace import summarize


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_columns(path, columns):
    with path.open() as stream:
        return np.asarray([[float(row[k]) for k in columns] for row in csv.DictReader(stream)])


def align_errors(poses, truth, maximum_gap=.35):
    poses, truth = np.asarray(poses, dtype=float), np.asarray(truth, dtype=float)
    if (poses.ndim != 2 or truth.ndim != 2 or poses.shape[1] != 3 or truth.shape[1] != 3
            or len(truth) < 2 or not np.isfinite(truth).all()):
        raise ValueError('Require finite timestamped reference positions')
    if not np.all(np.diff(truth[:, 0]) > 0):
        raise ValueError('Require strictly increasing reference times')
    poses = poses[np.isfinite(poses).all(axis=1)]
    poses = poses[np.argsort(poses[:, 0], kind='stable')]
    if not len(poses):
        raise ValueError('No finite poses')
    poses = poses[np.r_[np.diff(poses[:, 0]) > 0, True]]
    indices = np.searchsorted(truth[:, 0], poses[:, 0], side='right')
    valid = (indices > 0) & (indices < len(truth))
    candidates = np.flatnonzero(valid)
    valid[candidates] &= truth[indices[candidates], 0]-truth[indices[candidates]-1, 0] <= maximum_gap
    matched = poses[valid]
    if not len(matched):
        raise ValueError('No poses with time-bounded independent truth')
    actual = np.column_stack([np.interp(matched[:, 0], truth[:, 0], truth[:, i]) for i in [1, 2]])
    errors = np.linalg.norm(matched[:, 1:]-actual, axis=1)
    return np.column_stack([matched[:, 0], errors]), dict(
        finite_unique_poses=len(poses), evaluated=len(matched), excluded=len(poses)-len(matched),
        maximum_truth_interpolation_gap_s=maximum_gap)


def trajectory_report(path, columns, truth):
    errors, accounting = align_errors(read_columns(path, columns), truth)
    first = np.flatnonzero(errors[:, 1] > .5)
    return dict(path=str(path), sha256=digest(path), accounting=accounting,
        position_error_m=summarize(errors[:, 1]),
        first_over_half_m=(dict(stamp_sim_s=float(errors[first[0], 0]),
                              error_m=float(errors[first[0], 1])) if len(first) else None),
        windows=[dict(start_sim_s=start, position_error_m=summarize(
            errors[(errors[:, 0] >= start) & (errors[:, 0] < start+50), 1]))
            for start in range(0, math.ceil(errors[-1, 0]/50)*50, 50)])


def audit(episode, scenario, component=None):
    evaluation_path = episode/'evaluation.json'
    evaluation = json.loads(evaluation_path.read_text())
    tx, ty, angle = evaluation['ground_truth_audit']['world_to_map']
    c, s = math.cos(angle), math.sin(angle)
    rotation = np.array([[c, -s], [s, c]])
    truth_path = episode/'ground_truth_trajectory.csv'
    truth = read_columns(truth_path, ['sim_seconds', 'world_x', 'world_y'])
    truth = truth[np.argsort(truth[:, 0], kind='stable')]
    truth = truth[np.r_[np.diff(truth[:, 0]) > 0, True]]
    truth[:, 1:] = truth[:, 1:] @ rotation.T + [tx, ty]
    scene = yaml.safe_load(scenario.read_text())
    expected = {r['id']: np.asarray([r['x'], r['y']]) @ rotation.T + [tx, ty] for r in scene['targets']}
    markers = []
    for row in evaluation.get('graph_landmark_update', {}).get('rows', []):
        if not row.get('applied') or row['id'] not in expected:
            continue
        before = float(np.linalg.norm(expected[row['id']]-row['original_point']))
        after = float(np.linalg.norm(expected[row['id']]-row['point']))
        # Cross-check our independent frame conversion against original rubric.
        recorded = evaluation['ground_truth_audit']['position_errors_m'][str(row['id'])]
        if abs(after-recorded) > 1e-8:
            raise ValueError('Final coordinate error does not reproduce original evaluator')
        markers.append(dict(id=row['id'], original_error_m=before, finalized_error_m=after,
                            shift_m=row['shift_m'], error_change_m=after-before))
    graph_files = list(episode.glob('*/graph_scan_poses.csv'))
    if len(graph_files) != 1:
        raise ValueError('Require exactly one original finalized graph')
    trajectories = dict(native_online=trajectory_report(episode/'pose_trace.csv',
        ['map_stamp', 'map_x', 'map_y'], truth),
        finalized_graph=trajectory_report(graph_files[0], ['stamp_sim_s', 'map_x', 'map_y'], truth))
    replay = {}
    sources = [evaluation_path, truth_path, scenario, graph_files[0], episode/'pose_trace.csv']
    if component is not None:
        config = json.loads((component/'configuration.json').read_text())
        for arm in config['variants']:
            result = component/(arm+'.result.json')
            if not result.exists() or json.loads(result.read_text()).get('status') != 'completed':
                raise ValueError('Do not report a partial component replay as final')
            events, params = component/(arm+'.events.log'), component/(arm+'.parameters.txt')
            report = trajectory_report(component/(arm+'.online.csv'), ['stamp_sim_s', 'map_x', 'map_y'], truth)
            lines = events.read_text().splitlines()
            report.update(accepted_loop_events=[line for line in lines if ' BEGIN ' in line],
                          events_sha256=digest(events), parameters_sha256=digest(params))
            replay[arm] = report
            sources += [result, events, params]
        sources += [component/'configuration.json',
                    Path(config['driver']) if config.get('driver') else component/'run.py']
    return dict(evidence_type='Independent evaluation of recorded native and retained-scan component coordinates',
        original_world_to_map=[tx, ty, angle], trajectories=trajectories, markers=markers, component_replays=replay,
        source_sha256={str(p): digest(p) for p in sources}, ground_truth_supplied_to_policy=False,
        original_results_modified=False, native_success_established=False, hardware_validated=False,
        limits=['Retained scans omit the original ROS transport, discarded scans and mapper scheduling.',
                'Component output is not a new navigation, recognition or completion result.',
                'Loop event timing plus a controlled no-loop comparison is evidence about this recording only.'])


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    for name in ['episode', 'scenario', 'output']:
        p.add_argument('--'+name, type=Path, required=True)
    p.add_argument('--component', type=Path)
    a = p.parse_args()
    if a.output.exists():
        raise FileExistsError('Preserve previous audit')
    report = audit(a.episode.resolve(), a.scenario.resolve(), a.component.resolve() if a.component else None)
    a.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(dict(markers=report['markers'],
        component={k: v['position_error_m'] for k, v in report['component_replays'].items()})))
