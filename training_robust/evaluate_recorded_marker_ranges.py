"""Independently evaluate sealed raw sensor range predictions against truth.

No estimator or ROS node is run here. Scene truth is opened only after the
sensor-only prediction report and calibration identity have been checked.
World errors use interpolated planar base truth and the verified fixed mount;
they do not qualify the SLAM map coordinates or closed-loop mission.
"""
import argparse
from collections import defaultdict
import csv
import hashlib
import io
import json
import math
from pathlib import Path

import numpy as np
import yaml

from audit_imu_shadow import ordered, interpolate
from audit_pose_trace import summarize


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def world_point(camera_point, base_from_camera, world_base_pose):
    base = (base_from_camera @ np.r_[camera_point, 1.])[:2]
    x, y, yaw = world_base_pose
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([x, y]) + np.array([[c, -s], [s, c]]) @ base


def evaluate(replay_path, job, calibration_path, output):
    report = json.loads(replay_path.read_text())
    if report.get('ground_truth_used') is not False or not report.get('rows'):
        raise ValueError('Require sealed sensor-only predictions')
    if not all(row.get('production_range_reproduced') is True for row in report['rows']):
        raise ValueError('Original range function must reproduce all predictions')
    inventory_path = job/'input_inventory.json'
    inventory = json.loads(inventory_path.read_text())
    calibration = json.loads(calibration_path.read_text())
    if (calibration.get('ground_truth_used') is not False or
            calibration['description_sha256'] != inventory['actual_description_sha256']):
        raise ValueError('Fixed sensor mount does not match this official robot')
    mount = np.array(calibration['base_from_camera'])
    if mount.shape != (4, 4) or not np.isfinite(mount).all():
        raise ValueError('Invalid verified fixed mount')
    raw_files = {Path(r['file']): r['sha256'] for r in report['rows']}
    for file, expected in raw_files.items():
        if digest(file) != expected:
            raise ValueError('Raw snapshot changed after sensor-only fitting')
    manifest_path = job/'manifest.json'
    manifest = json.loads(manifest_path.read_text())
    if len(manifest['scenarios']) != 1:
        raise ValueError('Require exactly one native scenario')
    scene = Path(manifest['scenarios'][0]['file'])
    if digest(scene) != manifest['scenarios'][0]['sha256']:
        raise ValueError('Evaluator scenario differs from frozen experiment')
    # Ground truth enters only this evaluator, after predictions are fixed.
    targets = {int(t['id']): np.array([t['x'], t['y']])
               for t in yaml.safe_load(scene.read_text())['targets']}
    episodes = list((job/'episodes').iterdir())
    if len(episodes) != 1:
        raise ValueError('Require one episode')
    original_truth = episodes[0]/'ground_truth_trajectory.csv'
    content = original_truth.read_bytes()
    last_complete = content.rfind(b'\n') + 1
    content = content[:last_complete]
    trajectory = ordered([[float(r[k]) for k in ['sim_seconds', 'world_x', 'world_y', 'world_yaw']]
        for r in csv.DictReader(io.StringIO(content.decode()))])
    output.mkdir(parents=True, exist_ok=False)
    truth_snapshot = output/'ground_truth_trajectory.csv'
    truth_snapshot.write_bytes(content)
    rows, excluded = [], []
    groups = defaultdict(list)
    for row in report['rows']:
        if not row['production_would_attempt_range']:
            excluded.append(dict(file=row['file'], id=row['id'], reason='below_10px_range_attempt_threshold'))
            continue
        if row['id'] not in targets:
            excluded.append(dict(file=row['file'], id=row['id'], reason='decoded_id_not_in_evaluator_truth'))
            continue
        try:
            pose = interpolate(trajectory, row['stamp'])
        except ValueError as exc:
            excluded.append(dict(file=row['file'], id=row['id'], reason=str(exc)))
            continue
        methods = [('original', row['original']), ('camera_origin_cone', row['camera_origin_cone']),
                   *[(str(v['half_angle_rad']), v['result']) for v in row['narrow_cones']]]
        if 'candidate' in row:
            methods.append(('candidate', row['candidate']))
        for method, result in methods:
            if result['reason'] != 'returned_range':
                continue
            estimate = world_point(result['camera_point'], mount, pose)
            error = float(np.linalg.norm(estimate-targets[row['id']]))
            rows.append(dict(file=row['file'], id=row['id'], image_stamp=row['stamp'],
                scan_age_s=row['scan_age_s'], min_edge_pixels=row['min_edge_pixels'],
                method=method, sensor_geometry_xy_error_m=error))
            groups[(row['id'], method)].append(error)
    sources = [replay_path, calibration_path, inventory_path, manifest_path, scene, truth_snapshot]
    result = dict(evidence_type='Independent sensor-geometry evaluation of recorded raw range predictions',
        rows=rows, excluded=excluded,
        groups=[dict(id=i, method=m, error_m=summarize(values)) for (i, m), values in sorted(groups.items())],
        truth_snapshot_source=str(original_truth), truth_snapshot_bytes=last_complete,
        base_from_camera=mount.tolist(), robot_description_sha256=inventory['actual_description_sha256'],
        original_range_checks=report['production_range_checks'],
        truth_supplied_to_fitting=False, truth_supplied_to_policy=False,
        map_coordinates_evaluated=False, native_success_established=False, production_promoted=False,
        source_sha256={str(p): digest(p) for p in sources},
        raw_snapshot_sha256={str(p): value for p, value in raw_files.items()},
        limits=['Planar base XY/yaw truth; unmeasured base roll/pitch and sensor motion are not reconstructed',
                'Independent base truth is interpolated only across gaps <=0.35 seconds',
                'Raw scan/image timestamps and CameraInfo are preserved, but passive scan pairing may differ from the detector',
                'Range geometry does not establish an accepted ID, accurate SLAM map pose, approach or faster mission',
                'This development recording is not a held-out generalization or hardware test'])
    (output/'evaluation.json').write_text(json.dumps(result, indent=2)+'\n')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ['replay', 'job', 'calibration', 'output']:
        parser.add_argument('--'+name, type=Path, required=True)
    args = parser.parse_args()
    result = evaluate(args.replay.resolve(), args.job.resolve(), args.calibration.resolve(), args.output.resolve())
    print(json.dumps(dict(evaluated_ranges=len(result['rows']), excluded=len(result['excluded']), groups=result['groups'])))
