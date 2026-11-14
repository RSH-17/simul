"""Paired camera-view diagnostic; hand dispatch back to the preserved matrix."""
import argparse
import json
from pathlib import Path
import subprocess

import run_official_local_grid_screen as local
import run_official_local_grid_matrix as matrix

UNIT='osp-official-camera-view-screen-v1.service'
MATRIX='osp-official-local-grid-matrix-v1.service'


def prepare(output,workspace,baseline,campaign):
    local.prepare(output,workspace,baseline,campaign)
    config=json.loads((output/'configuration.json').read_text())
    config['only_configuration_change']='Camera-calibrated face viewpoints; odometry and clearance unchanged'
    config['resume_matrix_after_completion']=MATRIX
    for name in ['run_official_camera_view_screen.py','run_official_local_grid_matrix.py','audit_pose_trace.py']:
        p=local.ROOT/'training_robust'/name;config['driver_hashes'][str(p)]=local.screen.base.digest(p)
    local.screen.base.save(output/'configuration.json',config)
    state=json.loads((output/'state.json').read_text());state['coordinator_unit']=UNIT
    state['jobs'][0]['unit']='osp-official-camera-view-v1.service'
    state['scope']='Same-scene 90-second camera-view diagnostic; not full-task training'
    local.screen.base.save(output/'state.json',state)


def run(output):
    if local.screen.base.service_active(MATRIX):
        raise RuntimeError('Matrix dispatcher must be held; native runs remain independent')
    matrix.run(output)
    state=json.loads((output/'state.json').read_text())
    if state['status']=='completed':
        subprocess.run(['systemctl','--user','start',MATRIX],check=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    for name in ['workspace','baseline','campaign']:p.add_argument('--'+name,type=Path)
    p.add_argument('--run-prepared',action='store_true');a=p.parse_args()
    if a.run_prepared:run(a.output.resolve())
    else:prepare(a.output.resolve(),a.workspace.resolve(),a.baseline.resolve(),a.campaign.resolve())
