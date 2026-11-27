import json
from pathlib import Path

import numpy as np
import yaml

import official_tb4_curriculum_v2 as runner


def test_official_training_inputs_and_paired_controls(tmp_path):
    (tmp_path/'scenarios').mkdir()
    config = dict(seed=1200200000,workspace='/official',baseline_workspace='/official',run_id='test',
                  baseline_weights=[1.,2.,.5,.8,.6])
    state = dict(phase='train',generation=0,jobs=[],champion=config['baseline_weights'],
                 mean=np.log(config['baseline_weights']).tolist(),std=[0.,.3,.3,.3,.3])
    runner.plan(tmp_path,config,state)
    assert len(state['jobs']) == 12
    assert len({j['scenario']['physical_sha256'] for j in state['jobs']}) == 12
    for job in state['jobs']:
        manifest = json.loads((Path(job['output'])/'manifest.json').read_text())
        assert manifest['duration'] == 600 and manifest['camera_width'] == 320 and manifest['start_docked']
        assert manifest['sensor_stress_injected'] is False
        policy = json.loads((Path(job['output'])/'policy.json').read_text())
        assert not {'targets','start','scenario','stress_seed'} & policy.keys()
        scene = yaml.safe_load(Path(job['scenario']['file']).read_text())
        assert all(c['post_undock_marker_approaches_reachable'] for c in scene['official_start']['checks'])
    state['phase'] = 'pilot'
    runner.plan(tmp_path,config,state)
    for a,b in zip(state['jobs'][12::2],state['jobs'][13::2]):
        assert a['scenario'] == b['scenario']
        assert a['workspace'] == b['workspace'] == '/official'
        assert a['stress_profile'] == b['stress_profile'] == 'official_nominal'


def test_full_rubric_and_contact_penalty_are_retained():
    failures = [dict(success=False,targets=3,contacts=0,seconds=300.)]
    assert runner.quality(failures)['success_rate'] == 0
    assert runner.quality(failures)['par2_seconds'] == 1200
    baseline = dict(episodes=12,success_rate=1.,contacts=0,par2_seconds=500.)
    assert not runner.materially_better(dict(baseline,contacts=1,par2_seconds=300.),baseline,12)
    assert not runner.strict_credit_gate(dict(all_three_physical_credits_verified=True,rows=[]))
