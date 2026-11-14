"""Queue a fixed-weight, three-shape comparison after the frozen current campaign.

Reuses the established native launcher, resource limits, input gates and strict
independent outcome evaluator. The only runtime change is cumulative active
navigation time across same-goal resumes. This driver never stops older jobs.
"""
import argparse
import json
import math
from pathlib import Path
import time

import run_official_integrated_comparison as integrated

screen=integrated.screen
ROOT=Path(__file__).resolve().parents[1]
UNIT='osp-official-navigation-budget-comparison-v1.service'
EXPECTED_DELTA={'osp_mission/mission.py','osp_mission/navigation_budget.py','test/test_navigation_budget.py'}
ORIGINAL_OBSERVE=integrated.observe


def budget_gate(status,expected):
    observed=status.get('navigation_budget',{})
    if not expected:
        passed=not observed
    else:
        passed=(observed.get('limit_s')==65. and type(observed.get('assigned')) is bool
            and type(observed.get('active')) is bool and observed.get('clock_fault') is False
            and type(observed.get('resumes')) is int and observed['resumes']>=0
            and type(observed.get('active_s')) in [int,float]
            and math.isfinite(observed['active_s']) and observed['active_s']>=0.)
    return dict(passed=passed,expected_cumulative_budget=expected,observed=observed)


def prepare(output,baseline,candidate,original,policy):
    old,new=[screen.base.source_hashes(w) for w in [baseline,candidate]]
    if {k for k in old.keys()|new.keys() if old.get(k)!=new.get(k)}!=EXPECTED_DELTA:
        raise ValueError('Require isolated navigation budget changes')
    if not all((w/'install/setup.bash').exists() for w in [baseline,candidate]):
        raise ValueError('Both runtimes must already be built')
    prior=json.loads((original/'configuration.json').read_text());screen.verify(prior)
    state=json.loads((original/'state.json').read_text())
    output.mkdir(parents=True,exist_ok=False);jobs=[]
    for number in [0,2,4]:
        previous=next(j for j in state['jobs'] if j['number']==number)
        directory=Path(previous['output']);manifest=json.loads((directory/'manifest.json').read_text())
        if manifest['duration']!=600. or (directory/'policy.json').read_bytes()!=policy.read_bytes():
            raise ValueError('Preserve full duration and identical fixed policy weights/settings')
        for role,workspace in [('baseline',baseline),('candidate',candidate)]:
            index=len(jobs);destination=output/'jobs'/f'{index:02d}_{previous["scenario"]["arena"]["shape"]}_{role}'
            destination.mkdir(parents=True)
            (destination/'policy.json').write_bytes(policy.read_bytes())
            screen.base.save(destination/'manifest.json',dict(manifest,
                policy_sha256=screen.base.digest(destination/'policy.json'),
                purpose='Full 600-second paired navigation budget development comparison'))
            jobs.append(dict(number=index,role=role,phase='development',generation=0,
                workspace=str(workspace),output=str(destination),scenario=previous['scenario'],status='prepared',
                stress_profile='official_nominal',unit=f'osp-official-nav-budget-v1-{index:02d}.service',
                expected_loop=previous['expected_loop'],expected_navigation_budget=role=='candidate'))
    drivers=dict(prior['driver_hashes']);drivers[str(Path(__file__).resolve())]=screen.base.digest(Path(__file__))
    candidate_source=ROOT/'policy_candidates/2026-11-15_official_tb4_v20'
    for path in [candidate_source/'build.py',candidate_source/'candidate.json',*candidate_source.glob('overlay/**/*.py')]:
        drivers[str(path)]=screen.base.digest(path)
    prepared=[policy,*output.glob('jobs/*/policy.json'),*output.glob('jobs/*/manifest.json')]
    config=dict(prior,source_hashes={str(baseline):old,str(candidate):new},driver_hashes=drivers,
        prepared_hashes={**prior['prepared_hashes'],**{str(p):screen.base.digest(p) for p in prepared}},
        waits_for_campaign=str(original),waits_for_configuration_sha256=screen.base.digest(original/'configuration.json'),
        primary_comparison='Cumulative 65s active navigation budget; same v17 policy, scenes, odometry and SLAM',
        navigation_budget_seconds=65.,wait_limit_wall_s=43200.)
    screen.base.save(output/'configuration.json',config)
    screen.base.save(output/'state.json',dict(status='queued_after_original_campaign',phase='development',generation=0,
        coordinator_unit=UNIT,jobs=jobs,scope='Three shapes, full 600-second independent hidden-marker task'))


def original_finished(config):
    directory=Path(config['waits_for_campaign'])
    if screen.base.digest(directory/'configuration.json')!=config['waits_for_configuration_sha256']:
        raise ValueError('Original campaign configuration changed')
    state=json.loads((directory/'state.json').read_text())
    return (state['status']=='completed' and bool(state['jobs'])
        and all(j['status']=='done' for j in state['jobs'])
        and not screen.base.service_active(state['coordinator_unit'])
        and all(not screen.base.service_active(j['unit']) for j in state['jobs']))


def observe(job):
    ORIGINAL_OBSERVE(job)
    directory=Path(job['output'])
    live=directory/'episodes'/str(job['scenario']['seed'])/'live_status.json'
    if live.exists():
        gate=budget_gate(json.loads(live.read_text()).get('status',{}),job['expected_navigation_budget'])
        job['navigation_budget_gate']=gate['passed']
        screen.base.save(directory/'navigation_budget_gate.json',gate)


def run(output):
    config=json.loads((output/'configuration.json').read_text());started=time.monotonic()
    while True:
        screen.verify(config)
        if original_finished(config):break
        if time.monotonic()-started>config['wait_limit_wall_s']:
            raise TimeoutError('Original campaign has not finished; no new native job started')
        time.sleep(5.)
    # Changes are local to this coordinator process; frozen sources stay intact.
    previous=(integrated.observe,integrated.GATES,integrated.BLOCKERS)
    integrated.observe=observe
    integrated.GATES=(*integrated.GATES,'navigation_budget_gate')
    integrated.BLOCKERS=[*integrated.BLOCKERS,integrated.UNIT]
    try:integrated.run(output)
    finally:integrated.observe,integrated.GATES,integrated.BLOCKERS=previous


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    for name in ['baseline','candidate','original','policy']:parser.add_argument('--'+name,type=Path)
    parser.add_argument('--run-prepared',action='store_true');args=parser.parse_args()
    if args.run_prepared:run(args.output.resolve())
    else:prepare(*[getattr(args,k).resolve() for k in ['output','baseline','candidate','original','policy']])
