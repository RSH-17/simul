import json
import math
from pathlib import Path
import pytest
import run_localization_validation as runner


def profile():
    return dict(marker_directed_look=dict(version=1),face_directed_look=dict(version=2,frame='map',frame_parameter='map_frame',
        maximum_turn_rad=2*math.pi/3,creates_coverage_or_recognition_credit=False),
        marker_focus_profile=dict(desired_edge_pixels=36.),near_pass_inspection=dict(maximum_looks=8,maximum_sim_s=35.),
        graph_landmark_update=dict(ground_truth_inputs=False),platform_contract=dict(passed=True))


@pytest.mark.parametrize('bad',['none','version','frame','false_coverage','platform','truth'])
def test_output_contract_accepts_exact_v2_and_preserves_safety_checks(tmp_path,bad):
    result=profile()
    if bad=='version':result['face_directed_look']['version']=1
    elif bad=='frame':result['face_directed_look']['frame']='odom'
    elif bad=='false_coverage':result['face_directed_look']['creates_coverage_or_recognition_credit']=True
    elif bad=='platform':result['platform_contract']['passed']=False
    elif bad=='truth':result['graph_landmark_update']['ground_truth_inputs']=True
    (tmp_path/'report.json').write_text(json.dumps(dict(rows=[dict(result=result)])))
    assert runner.output_profiles(tmp_path)==(bad=='none')


def test_unverified_retained_coordinate_cannot_promote_full_task_success(tmp_path,monkeypatch):
    e=tmp_path/'episodes/1';e.mkdir(parents=True)
    (e/'evaluation.json').write_text(json.dumps(dict(graph_landmark_update=dict(validation_version=1,coordinates_verified=False,requires_reobservation=[7]))))
    job=dict(output=str(tmp_path),scenario=dict(seed=1),expected_localization_validation=True,outcome=dict(success=True,infrastructure_failure=False))
    monkeypatch.setattr(runner,'ORIGINAL_COLLECT',lambda _:None)
    runner.collect(job)
    assert not job['outcome']['success'] and job['outcome']['reason']=='unverified_final_coordinates'
    assert not job['outcome']['infrastructure_failure']


def test_new_candidate_keeps_same_host_limit_dispatcher(tmp_path,monkeypatch):
    p=tmp_path/'prior';p.mkdir();(p/'configuration.json').write_text('{}')
    runner.screen.base.save(tmp_path/'configuration.json',dict(prerequisite=str(p),
        prerequisite_source_sha256=runner.screen.base.digest(p/'configuration.json'),domains=[230,231]))
    runner.screen.base.save(tmp_path/'state.json',dict(jobs=[dict(status='prepared',output=str(tmp_path/'new'))]))
    monkeypatch.setattr(runner.integrated,'prerequisite_report',lambda _:dict(passed=True))
    monkeypatch.setattr(runner.screen,'verify',lambda _:None)
    monkeypatch.setattr(runner,'service_active',lambda _:False)
    monkeypatch.setattr(runner.screen.base,'occupied_slots',lambda _:2)
    monkeypatch.setattr(runner.integrated.subprocess,'run',lambda *a,**kw:pytest.fail('Third native launched'))
    def stop(_):raise InterruptedError()
    monkeypatch.setattr(runner.integrated.time,'sleep',stop)
    with pytest.raises(InterruptedError):runner.run(tmp_path)
