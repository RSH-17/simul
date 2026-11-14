"""Read-only, time-aligned evaluator audit of completed native pose traces."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np


def summarize(values):
    return dict(samples=len(values), median=float(np.median(values)),
                p95=float(np.quantile(values, .95)), maximum=float(np.max(values))) if len(values) else dict(samples=0)


def audit(episode):
    result = json.loads((episode/'evaluation.json').read_text())
    alignment = result['ground_truth_audit']['world_to_map']
    rows = list(csv.DictReader((episode/'pose_trace.csv').open()))
    truth = np.array([[float(r[k]) for k in ('sim_seconds','world_x','world_y','world_yaw')]
                      for r in csv.DictReader((episode/'ground_truth_trajectory.csv').open())])
    truth = truth[np.argsort(truth[:, 0], kind='stable')]
    truth = truth[np.r_[np.diff(truth[:, 0]) > 0, True]]
    truth[:, 3] = np.unwrap(truth[:, 3])
    tx, ty, angle = alignment
    c, s = math.cos(angle), math.sin(angle)
    rotation = np.array([[c,-s],[s,c]])
    truth[:, 1:3] = truth[:, 1:3] @ rotation.T + [tx,ty]
    truth[:, 3] += angle
    # Initial map and odom transforms define a fixed coordinate conversion.
    # Never refit to the later trajectory, which would hide accumulated drift.
    first = rows[0]
    offset_angle = float(first['map_yaw']) - float(first['odom_yaw'])
    c, s = math.cos(offset_angle), math.sin(offset_angle)
    odom_rotation = np.array([[c,-s],[s,c]])
    offset = np.array([float(first['map_x']),float(first['map_y'])]) - np.array([
        float(first['odom_x']),float(first['odom_y'])]) @ odom_rotation.T
    output = {}
    for frame in ('map','odom'):
        poses = np.array([[float(r[f'{frame}_{k}']) for k in ('stamp','x','y','yaw')] for r in rows])
        poses = poses[np.isfinite(poses).all(axis=1)]
        poses = poses[np.argsort(poses[:, 0], kind='stable')]
        poses = poses[np.r_[np.diff(poses[:, 0]) > 0, True]]
        if frame == 'odom':
            poses[:, 1:3] = poses[:, 1:3] @ odom_rotation.T + offset
            poses[:, 3] += offset_angle
        indices = np.searchsorted(truth[:, 0], poses[:, 0], side='right')
        valid = (indices > 0) & (indices < len(truth))
        candidate = np.flatnonzero(valid)
        # No extrapolation and no interpolation over missing truth segments.
        valid[candidate] &= truth[indices[candidate],0] - truth[indices[candidate]-1,0] <= .35
        poses = poses[valid]
        actual = np.column_stack([np.interp(poses[:,0],truth[:,0],truth[:,i]) for i in (1,2,3)])
        error = np.linalg.norm(poses[:,1:3]-actual[:,:2],axis=1)
        angle_error = np.abs((poses[:,3]-actual[:,2]+math.pi) % (2*math.pi)-math.pi)*180/math.pi
        output[frame] = dict(position_error_m=summarize(error), heading_error_deg=summarize(angle_error))
    paths = [episode/n for n in ('evaluation.json','pose_trace.csv','ground_truth_trajectory.csv')]
    return dict(evidence_type='Finalized native evaluator audit; not policy input or a replayed success',
        episode=str(episode), world_to_map=alignment, fixed_initial_odom_to_map=[*offset.tolist(),offset_angle],
        pose_errors=output, full_task_success=result.get('success',False),
        truth_max_sample_gap_s=float(np.diff(truth[:,0]).max()),
        interpolation_limit_s=.35, ground_truth_supplied_to_policy=False,
        limitations=['Coarse independent truth is interpolated within 0.35 seconds; no high-rate motion precision claim.',
                     'One short run does not qualify long-run localization or physical hardware.'],
        source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths})


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('episode',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    args.output.write_text(json.dumps(audit(args.episode.resolve()),indent=2)+'\n')
