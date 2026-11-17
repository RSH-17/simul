"""Separate localization observation and guarded landmark finalization trials."""
import argparse
import json
import math
from pathlib import Path
import time

import run_official_integrated_comparison as integrated

screen=integrated.screen
ROOT=Path(__file__).resolve().parents[1]
UNIT='osp-official-localization-validation-v1.service'
EXPECTED_DELTA={'osp_mission/mission.py','setup.py','launch/robot.launch.py','osp_mission/validated_graph_landmarks.py','osp_mission/pose_gyro_stream.py','osp_mission/pose_increment_gyro.py','osp_mission/pose_gyro_shadow.py','test/test_localization_validation.py'}
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
                purpose='Full 600-second guarded coordinate development comparison with passive odometry observation'))
            jobs.append(dict(number=index,role=role,phase='development',generation=0,
                workspace=str(workspace),output=str(destination),scenario=previous['scenario'],status='prepared',
                stress_profile='official_nominal',unit=f'osp-official-localization-v1-{index:02d}.service',
                expected_loop=previous['expected_loop'],expected_navigation_budget=True,expected_hybrid=False,expected_localization_validation=role=='candidate'))
    drivers=dict(prior['driver_hashes']);drivers[str(Path(__file__).resolve())]=screen.base.digest(Path(__file__))
    candidate_source=ROOT/'policy_candidates/2026-11-17_localization_validation_v1'
    for path in [candidate_source/'build.py',candidate_source/'candidate.json',*candidate_source.glob('overlay/**/*.py')]:
        drivers[str(path)]=screen.base.digest(path)
    prepared=[policy,*output.glob('jobs/*/policy.json'),*output.glob('jobs/*/manifest.json')]
    config=dict(prior,source_hashes={str(baseline):old,str(candidate):new},driver_hashes=drivers,
        prepared_hashes={**prior['prepared_hashes'],**{str(p):screen.base.digest(p) for p in prepared}},
        waits_for_campaign=str(original),waits_for_configuration_sha256=screen.base.digest(original/'configuration.json'),
        primary_comparison='v20 versus guarded final reprojection plus file-only gyro odometry observation; live odom and TF unchanged',
        navigation_budget_seconds=65.,wait_limit_wall_s=43200.)
    screen.base.save(output/'configuration.json',config)
    screen.base.save(output/'state.json',dict(status='prepared',phase='development',generation=0,
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
        status=json.loads(live.read_text()).get('status',{})
        job['hybrid_algorithm_gate']=(status.get('search_algorithm')=='frontier_coverage_v1') if job['expected_hybrid'] else 'search_algorithm' not in status
        screen.base.save(directory/'navigation_budget_gate.json',gate)


def service_active(unit):
    result=integrated.subprocess.run(['systemctl','--user','show',unit,'-p','LoadState','-p','ActiveState'],capture_output=True,text=True)
    props=dict(line.split('=',1) for line in result.stdout.splitlines() if '=' in line)
    if props.get('LoadState')=='not-found':return False
    if result.returncode:raise RuntimeError('Cannot inspect service '+unit+': '+result.stderr)
    return props.get('ActiveState') in ['active','activating','deactivating']


def output_profiles(directory):
    path=Path(directory)/'report.json'
    if not path.exists():return False
    rows=json.loads(path.read_text()).get('rows',[])
    if len(rows)!=1:return False
    result=rows[0].get('result',{});face=result.get('face_directed_look',{})
    return (result.get('marker_directed_look',{}).get('version')==1
        and face.get('version')==2 and face.get('frame')=='map' and face.get('frame_parameter')=='map_frame'
        and math.isclose(face.get('maximum_turn_rad',0.),2*math.pi/3,abs_tol=1e-9)
        and face.get('creates_coverage_or_recognition_credit') is False
        and result.get('marker_focus_profile',{}).get('desired_edge_pixels')==36.
        and result.get('near_pass_inspection',{}).get('maximum_looks')==8
        and result.get('near_pass_inspection',{}).get('maximum_sim_s')==35.
        and result.get('graph_landmark_update',{}).get('ground_truth_inputs') is False
        and result.get('platform_contract',{}).get('passed') is True)


ORIGINAL_COLLECT=integrated.collect

def collect(job):
    ORIGINAL_COLLECT(job)
    episode=Path(job['output'])/'episodes'/str(job['scenario']['seed'])
    if not job['expected_localization_validation']:return
    evidence=json.loads((episode/'evaluation.json').read_text())
    report=evidence.get('graph_landmark_update',{})
    job['coordinate_validation']=dict(version=report.get('validation_version'),
        coordinates_verified=report.get('coordinates_verified'),requires_reobservation=report.get('requires_reobservation'))
    if report.get('coordinates_verified') is not True and job['outcome'].get('success'):
        job['raw_outcome_before_coordinate_validation']=dict(job['outcome'])
        job['outcome'].update(success=False,reason='unverified_final_coordinates')
    shadow=episode/'pose_gyro_shadow.json'
    if shadow.exists():job['shadow_odometry_observation']=json.loads(shadow.read_text())


def run(output):
    # A missing transient historical service is terminal, not a process failure.
    # Keep this compatibility fix local; queued original drivers remain frozen.
    previous=(integrated.observe,integrated.GATES,integrated.BLOCKERS,screen.base.service_active,screen.base.candidate_profiles,integrated.collect)
    integrated.observe=observe
    screen.base.candidate_profiles=output_profiles
    integrated.collect=collect
    integrated.GATES=(*integrated.GATES,'navigation_budget_gate','hybrid_algorithm_gate')
    integrated.BLOCKERS=[*integrated.BLOCKERS,integrated.UNIT,'osp-official-navigation-budget-comparison-v1.service','osp-official-frontier-coverage-v1.service']
    screen.base.service_active=service_active
    try:integrated.run(output)
    finally:integrated.observe,integrated.GATES,integrated.BLOCKERS,screen.base.service_active,screen.base.candidate_profiles,integrated.collect=previous


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    for name in ['baseline','candidate','original','policy']:parser.add_argument('--'+name,type=Path)
    parser.add_argument('--run-prepared',action='store_true');args=parser.parse_args()
    if args.run_prepared:run(args.output.resolve())
    else:prepare(*[getattr(args,k).resolve() for k in ['output','baseline','candidate','original','policy']])
