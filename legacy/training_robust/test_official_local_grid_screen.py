import json

import pytest

import run_official_local_grid_screen as candidate


def test_same_physical_scene_weights_and_frozen_candidate_are_required(tmp_path):
    baseline=tmp_path/'baseline';baseline.mkdir()
    scene=tmp_path/'scene.yaml';scene.write_text('seed: 10\n')
    policy=baseline/'policy.json';policy.write_text('{"planner":"edge","parameters":[1,2,3,4,5]}')
    manifest=dict(duration=90.,policy_sha256=candidate.screen.base.digest(policy),
        scenarios=[dict(seed=10,file=str(scene),sha256=candidate.screen.base.digest(scene))])
    (baseline/'manifest.json').write_text(json.dumps(manifest))
    (baseline/'report.json').write_text('{"complete":true}')
    campaign=tmp_path/'campaign';campaign.mkdir();(campaign/'configuration.json').write_text('{"driver_hashes":{}}')
    workspace=tmp_path/'workspace';(workspace/'install').mkdir(parents=True)
    (workspace/'install/setup.bash').write_text('')
    package=workspace/'src/osp_mission';package.mkdir(parents=True)
    runtime=package/'setting.yaml';runtime.write_text('resolution: 0.025\n')
    output=tmp_path/'candidate';candidate.prepare(output,workspace,baseline,campaign)
    assert (output/'policy.json').read_bytes()==policy.read_bytes()
    assert json.loads((output/'manifest.json').read_text())==manifest
    config=json.loads((output/'configuration.json').read_text())
    assert config['resume_previous_after_completion'] is False
    candidate.screen.verify(config)
    runtime.write_text('resolution: 0.01\n')
    with pytest.raises(RuntimeError,match='Frozen runtime'):
        candidate.screen.verify(config)
