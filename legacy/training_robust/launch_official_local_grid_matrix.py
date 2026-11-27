"""Prepare a new constraint campaign only after the finer-grid screen finishes.

The original queued 6 cm runs remain unchanged. All 18 training conditions
are repeated on the new frozen runtime; reserved holdout scenes are not run.
"""
import argparse
import json
from pathlib import Path
import subprocess

from launch_official_tb4_curriculum_v2 import qualify
import run_official_constraint_screen as screen

UNIT='osp-official-local-grid-matrix-v1.service'


def prepare(output, matrix, previous, comparison):
    state=json.loads((comparison/'state.json').read_text())
    if state.get('status')!='completed' or len(state.get('jobs',[]))!=1:
        raise ValueError('Require a finalized native screen, not an interim escape')
    job=state['jobs'][0];workspace=Path(job['workspace'])
    if job.get('navigation_grid_gate') is not True or job.get('start_screen_passed') is not True:
        raise ValueError('Actual local grid and finalized physical escape must both pass')
    qualification=qualify(workspace,comparison)
    if screen.base.service_active(state['coordinator_unit']):
        raise ValueError('Previous screen coordinator is still active')
    config=json.loads((previous/'configuration.json').read_text())
    weights=json.loads((comparison/'policy.json').read_text())['parameters']
    if weights!=config['baseline_weights']:
        raise ValueError('Keep the same fixed weights throughout this diagnostic')
    config.update(workspace=str(workspace),baseline_workspace=str(workspace),
                  source_hashes={str(workspace):screen.base.source_hashes(workspace)})
    for name in ['launch_official_local_grid_matrix.py','run_official_local_grid_screen.py',
                 'run_official_local_grid_matrix.py','official_local_grid_probe.py','audit_pose_trace.py']:
        path=screen.ROOT/'training_robust'/name;config['driver_hashes'][str(path)]=screen.base.digest(path)
    inputs=output.with_name(output.name+'_inputs');inputs.mkdir(exist_ok=False)
    screen.base.save(inputs/'configuration.json',config)
    screen.prepare(output,matrix,inputs)
    prepared=json.loads((output/'configuration.json').read_text())
    prepared.update(resume_previous_after_completion=False,
        resume_condition='Further 600-second coefficient training requires a new campaign on this runtime',
        startup_qualification=qualification,previous_campaign=str(previous))
    prepared['prepared_hashes'][str(inputs/'configuration.json')]=screen.base.digest(inputs/'configuration.json')
    screen.base.save(output/'configuration.json',prepared)
    new=json.loads((output/'state.json').read_text());new['coordinator_unit']=UNIT
    new['scope']='18 native departure conditions on finer local grid; fixed weights; not full-task qualification'
    for j in new['jobs']:j['unit']=f'osp-official-local-matrix-v1-{j["number"]:02d}.service'
    screen.base.save(output/'state.json',new)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ['output','matrix','previous','comparison']:p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--start',action='store_true');a=p.parse_args()
    prepare(a.output.resolve(),a.matrix.resolve(),a.previous.resolve(),a.comparison.resolve())
    if a.start:
        subprocess.run(['systemd-run','--user','--unit='+UNIT,'--property=MemoryMax=768M',
            '--property=WorkingDirectory='+str(screen.ROOT),'--property=Restart=no',
            '--property=StandardOutput=append:'+str(a.output.resolve()/'coordinator.log'),
            '--property=StandardError=append:'+str(a.output.resolve()/'coordinator.log'),
            '/usr/bin/python3','-u',str(screen.ROOT/'training_robust/run_official_local_grid_matrix.py'),
            '--output',str(a.output.resolve())],check=True)
