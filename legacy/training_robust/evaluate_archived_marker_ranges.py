"""Independent geometry evaluation of already-computed archival range results.

Truth is read only here, after range fitting, and never supplied to the fitting
routine, ROS, robot or policy. Sparse/video timestamps limit this evaluation.
"""
import argparse
from collections import defaultdict
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import yaml

from audit_imu_shadow import ordered, interpolate
from audit_pose_trace import summarize


def evaluate(report_path):
    report=json.loads(report_path.read_text())
    if report['ground_truth_used'] or report['policy_modified']:
        raise ValueError('Require an independent sensor-only archival calculation')
    job=Path(report['job']);episode=next((job/'episodes').iterdir())
    manifest=json.loads((job/'manifest.json').read_text());scene=Path(manifest['scenarios'][0]['file'])
    if hashlib.sha256(scene.read_bytes()).hexdigest()!=manifest['scenarios'][0]['sha256']:
        raise ValueError('Evaluator scenario no longer matches the original experiment')
    targets={int(t['id']):np.array([t['x'],t['y']]) for t in yaml.safe_load(scene.read_text())['targets']}
    truth_path=episode/'ground_truth_trajectory.csv'
    truth=ordered([[float(r[k]) for k in ['sim_seconds','world_x','world_y','world_yaw']]
                   for r in csv.DictReader(truth_path.open())])
    mount=np.array(report['base_from_camera']);rows=[];excluded=[];groups=defaultdict(list)
    for row in report['rows']:
        if row['id'] not in targets:
            excluded.append(dict(frame=row['frame'],id=row['id'],reason='decoded_id_not_in_evaluator_truth'))
            continue
        try:pose=interpolate(truth,row['video_sim_s'])
        except ValueError as exc:
            excluded.append(dict(frame=row['frame'],reason=str(exc)));continue
        c,s=math.cos(pose[2]),math.sin(pose[2]);rotation=np.array([[c,-s],[s,c]])
        results=[('original',row['original']),('camera_origin_cone',row['camera_origin_cone']),
                 *[(str(v['half_angle_rad']),v['result']) for v in row['narrow_cones']]]
        for method,result in results:
            if result['reason']!='returned_range':continue
            base=(mount@np.r_[result['camera_point'],1])[:2]
            error=float(np.linalg.norm(pose[:2]+rotation@base-targets[row['id']]))
            rows.append(dict(id=row['id'],video_sim_s=row['video_sim_s'],scan_sim_s=row['scan_sim_s'],
                source_min_edge_estimate=row['source_min_edge_estimate'],method=method,error_m=error))
            groups[(row['id'],method)].append(error)
    return dict(evidence_type='Approximate evaluator-only spatial check of archival range sensitivity',
        rows=rows,excluded=excluded,groups=[dict(id=i,method=m,error_m=summarize(v)) for (i,m),v in sorted(groups.items())],
        truth_supplied_to_range_fitting=False,truth_supplied_to_robot_or_policy=False,
        native_accuracy_qualified=False,production_promoted=False,
        source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest()
                       for p in [report_path,job/'manifest.json',scene,truth_path]},
        limits=['Recorder time substitutes for exact image exposure time',
                'Only base XY/yaw truth and ideal fixed camera geometry are used',
                'Video is lossy and scan/image skew can be up to 0.25 seconds',
                'A spatial discrepancy can be from the estimator or this reconstruction; neither is isolated here',
                'No acceptance, marker confirmation, approach or closed-loop success is inferred'])


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('report',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():raise FileExistsError('Preserve previous evaluation')
    result=evaluate(a.report.resolve());a.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(evaluated_ranges=len(result['rows']),excluded=len(result['excluded']),native_accuracy_qualified=False)))
