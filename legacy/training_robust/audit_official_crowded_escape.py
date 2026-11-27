"""Evaluator-only escape timing for rotated, generated official dock corridors."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import yaml


def trace_metrics(scene, rows):
    yaw=scene['official_start']['exit_heading_rad']
    sx,sy=scene['start'][:2]
    length=scene['crowded_start']['exit_length_m']
    # The whole circular body, plus the existing 1 cm margin, must clear the
    # far end of both rails. Rotated starts cannot use a hard-coded world X.
    threshold=length+.189+.01
    first=float(rows[0]['sim_seconds']) if rows else None
    projection=[(float(r['world_x'])-sx)*math.cos(yaw)+(float(r['world_y'])-sy)*math.sin(yaw) for r in rows]
    exit_seconds=next((float(r['sim_seconds'])-first for r,p in zip(rows,projection) if p>=threshold),None)
    distance=sum(math.hypot(float(a['world_x'])-float(b['world_x']),float(a['world_y'])-float(b['world_y']))
                 for a,b in zip(rows,rows[1:]))
    return dict(observed_exit_seconds=exit_seconds,actual_path_m=distance,
        maximum_exit_axis_progress_m=max(projection,default=None),required_progress_m=threshold,
        timing_origin='first independent ground-truth sample, including undock',
        criterion='body plus 1 cm fully beyond corridor rail end')


def audit(output):
    manifest=json.loads((output/'manifest.json').read_text())
    scenario=manifest['scenarios'][0]
    episode=output/'episodes'/str(scenario['seed'])
    scene_path=Path(scenario['file'])
    if hashlib.sha256(scene_path.read_bytes()).hexdigest()!=scenario['sha256']:
        raise ValueError('Scenario differs from the recorded evaluator manifest')
    scene=yaml.safe_load(scene_path.read_text())
    trace=episode/'ground_truth_trajectory.csv'
    rows=list(csv.DictReader(trace.open()))
    result_path=episode/'evaluation.json'
    result=json.loads(result_path.read_text()) if result_path.exists() else {}
    completed=result.get('reason') in ('timeout','all_targets_found')
    return dict(evidence_type='Independent native trajectory audit; no data supplied to policy',
        episode=str(episode),completed=completed,reason=result.get('reason'),
        contacts=result.get('collisions'),targets=result.get('targets'),
        full_task_success=result.get('success',False),**trace_metrics(scene,rows))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('output',type=Path)
    p.add_argument('--report',type=Path,required=True)
    a=p.parse_args()
    a.report.write_text(json.dumps(audit(a.output.resolve()),indent=2)+'\n')
