"""Prepare official TB4 CEM and new reserved worlds without touching older campaigns."""
import argparse
import json
from pathlib import Path

import numpy as np
import yaml

import official_tb4_curriculum_v2 as runner
from official_crowded_scenarios import make_official_scene
from launch_spatial_cem_v6 import start_unit
from audit_official_crowded_escape import audit as audit_escape

ROOT = Path(__file__).resolve().parents[1]
UNIT = 'osp-official-tb4-cem-v2.service'


def qualify(workspace, comparison):
    """One completed start screen permits further experiments, never promotion."""
    state = json.loads((comparison/'state.json').read_text())
    jobs = state.get('jobs',[])
    report = json.loads((comparison/'report.json').read_text())
    gates = ('runtime_configuration_gate','velocity_profile_gate','scan_adapter_gate',
             'floor_sensor_gate','drive_path_gate','official_description_gate','slam_transport_gate')
    if (len(jobs)!=1 or Path(jobs[0]['workspace'])!=workspace or
            not all(jobs[0].get(k) is True for k in gates) or not report.get('complete')):
        raise ValueError('Require completed native screen with all observed input gates')
    hashes = json.loads((comparison/'source_hashes.json').read_text())
    if runner.source_hashes(workspace)!=hashes:
        raise ValueError('Runtime differs from the actual diagnostic source snapshot')
    result = audit_escape(comparison)
    if not (result['completed'] and result['contacts']==0 and
            result['observed_exit_seconds'] is not None and 0<=result['observed_exit_seconds']<=90.):
        raise ValueError('Require finalized collision-free physical corridor escape')
    return dict(screen=str(comparison), result=result, hardware_validated=False,
                scope='Single-world start qualification only; subsequent full-task training remains experimental')


def prepare(workspace, destination, comparison):
    qualification = qualify(workspace, comparison)
    if destination.exists():
        raise FileExistsError('Existing campaigns must be preserved')
    destination.mkdir()
    (destination/'scenarios').mkdir()
    (destination/'reserved').mkdir()
    weights = json.loads((ROOT/'policy_releases/2026-11-13_face_intent_v2/policy.json').read_text())['parameters']
    reserved = []
    for i in range(40):
        cfg = make_official_scene(1204000000+i*101, (9,12,16,20)[i%4], shape=('c_shape','u_shape','zigzag')[i%3])
        file = destination/'reserved'/f'{cfg["seed"]}.yaml'
        file.write_text(yaml.safe_dump(cfg, sort_keys=False))
        reserved.append(dict(file=str(file), seed=cfg['seed'], sha256=runner.digest(file),
            requested_obstacles=len(cfg['obstacles']), actual_obstacles=len(cfg['obstacles']),
            mount_profile='hidden_obstacles', arena={k:cfg['arena'][k] for k in ('shape','width_m','height_m','area_m2')}))
    reserved_manifest = destination/'reserved_manifest.json'
    runner.save(reserved_manifest, dict(episodes=reserved, used_for_training=False))
    # Reuse unchanged evaluator dependencies, then freeze all new driver sources.
    previous = json.loads((ROOT/'training_robust/native90/official_tb4_cem_v1/configuration.json').read_text())
    files = set(previous['driver_hashes']) | {str(ROOT/'training_robust'/n) for n in (
        'official_tb4_curriculum_v2.py','launch_official_tb4_curriculum_v2.py','official_crowded_scenarios.py',
        'official_training_input_probe.py','official_fine_no_return_probe.py','official_no_return_probe.py',
        'official_rotation_probe.py','audit_official_crowded_escape.py',
        'crowded_challenge_scenarios.py','official_tb4_probe.py','probe_official_tb4.py',
        'run_crowded_start_regression_v4.py','probe_sim_drive_path.py')}
    files |= {str(p) for p in (ROOT/'src/osp_mission/osp_mission').glob('*.py')}
    cfg = dict(run_id='nov15-official2', seed=1203000000, workspace=str(workspace), baseline_workspace=str(workspace),
        baseline_weights=weights, map_resolution=.025, domains=[230,231],
        source_hashes={str(workspace):runner.source_hashes(workspace)}, driver_hashes={p:runner.digest(Path(p)) for p in files},
        reserved_manifest=str(reserved_manifest), reserved_manifest_sha256=runner.digest(reserved_manifest),
        native_limit=2, production_promoted=False, hardware_validated=False, official_robot_files_modified=False,
        primary_baseline='Frozen initial weights on exactly the same official robot and runtime',
        synthetic_profiles=[], real_robot_calibration=False, baseline_action_acknowledgement_timeout_ms=1000,
        baseline_live_local_costmap=True, startup_qualification=qualification,
        purpose='Native coefficient training with crowded starts and strict full 600-second hidden-marker task')
    runner.save(destination/'configuration.json',cfg)
    state = dict(status='running', generation=0, phase='train', planned=False, jobs=[], champion=weights,
        champion_workspace=str(workspace), mean=np.log(weights).tolist(), std=[0.,.3,.3,.3,.3],
        reserved_holdout_consumed=False, spatial_holdout_consumed=False, coordinator_unit=UNIT,
        source_fixed_during_campaign=True)
    runner.plan(destination,cfg,state)
    runner.save(destination/'state.json',state)
    return dict(prepared_jobs=len(state['jobs']), reserved_holdout_scenes=len(reserved),
                runtime=str(workspace), native_started=False)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('workspace','destination','comparison'):
        parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--start',action='store_true')
    args = parser.parse_args()
    print(json.dumps(prepare(args.workspace.resolve(),args.destination.resolve(),args.comparison.resolve())),flush=True)
    if args.start:
        start_unit(UNIT,['/usr/bin/python3','-u',str(ROOT/'training_robust/official_tb4_curriculum_v2.py'),
                        str(args.destination.resolve())],args.destination.resolve()/'coordinator.log')
