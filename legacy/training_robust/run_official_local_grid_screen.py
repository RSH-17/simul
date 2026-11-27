"""Single unchanged-scene comparison with finer local navigation costmap."""
import argparse
import json
from pathlib import Path

import run_official_constraint_screen as screen
import official_local_grid_probe as local

ROOT=Path(__file__).resolve().parents[1]
UNIT='osp-official-local-grid-screen-v1.service'
NATIVE_UNIT='osp-official-local-grid-v1.service'


def prepare(output, workspace, baseline, campaign):
    if not (workspace/'install/setup.bash').exists():raise ValueError('Build the candidate first')
    previous=json.loads((campaign/'configuration.json').read_text())
    original=json.loads((baseline/'manifest.json').read_text())
    if original['duration']!=90. or len(original['scenarios'])!=1:raise ValueError('Require original 90s screen')
    final=json.loads((baseline/'report.json').read_text())
    if not final.get('complete'):raise ValueError('Preserve a finalized baseline before comparison')
    output.mkdir(parents=True,exist_ok=False)
    (output/'policy.json').write_bytes((baseline/'policy.json').read_bytes())
    screen.base.save(output/'manifest.json',original)
    sources=dict(previous['driver_hashes'])
    for name in ['run_official_local_grid_screen.py','official_local_grid_probe.py']:
        p=ROOT/'training_robust'/name;sources[str(p)]=screen.base.digest(p)
    scene=original['scenarios'][0]
    prepared=[output/'policy.json',output/'manifest.json',Path(scene['file'])]
    screen.base.save(output/'configuration.json',dict(source_hashes={str(workspace):screen.base.source_hashes(workspace)},
        driver_hashes=sources,prepared_hashes={str(p):screen.base.digest(p) for p in prepared},
        domains=[229],native_limit=2,duration=90.,resume_previous_after_completion=False,
        baseline=str(baseline),only_configuration_change='local costmap resolution 0.06 to 0.025 m',
        production_promoted=False,hardware_validated=False))
    job=dict(number=0,role='candidate',phase='diagnostic',generation=0,workspace=str(workspace),
        output=str(output),scenario=scene,status='prepared',stress_profile='official_nominal',unit=NATIVE_UNIT)
    screen.base.save(output/'state.json',dict(status='prepared',phase='diagnostic',generation=0,
        coordinator_unit=UNIT,jobs=[job],scope='Single paired 90-second navigation-grid diagnostic; not full-task training'))


def run(output):
    original_observe=screen.observe
    original_verify=screen.verify
    screen.probe=local.probe
    screen.configuration_gate=local.configuration_gate
    screen.GATES=(*screen.GATES,'navigation_grid_gate')
    def observe(job):
        original_observe(job)
        p=Path(job['output'])/'runtime_configuration.json'
        if p.exists():
            gate=local.navigation_grid_gate(json.loads(p.read_text()))
            job['navigation_grid_gate']=gate['passed']
            screen.base.save(Path(job['output'])/'navigation_grid_check.json',gate)
    def verify(config):
        original_verify(config)
        for unit in ['osp-official-constraint-screen-v1.service','osp-official-tb4-cem-v1.service']:
            if screen.base.service_active(unit):raise RuntimeError('Another dispatcher is active; preserve host limit')
    screen.observe=observe
    screen.verify=verify
    screen.run(output)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--workspace',type=Path);p.add_argument('--baseline',type=Path);p.add_argument('--campaign',type=Path)
    p.add_argument('--run-prepared',action='store_true');a=p.parse_args()
    if a.run_prepared:run(a.output.resolve())
    else:prepare(a.output.resolve(),a.workspace.resolve(),a.baseline.resolve(),a.campaign.resolve())
