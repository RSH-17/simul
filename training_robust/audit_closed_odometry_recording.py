"""Quarantined long sensor replay after a known recorder shutdown exception.

Never changes the original complete=false report. Sensor bytes are accepted
only after their frozen hash, schema, ordering, gaps, passivity and terminal
episode checks pass. Ground truth is loaded only after both estimators exist.
"""
import argparse
import csv
import hashlib
import io
import json
from pathlib import Path
import sys

import numpy as np

from audit_official_odometry_inputs import parse_inputs,validate_axes,DESCRIPTION
from audit_imu_shadow import ordered,pose,compare
from pose_increment_gyro import integrate


def digest(data):return hashlib.sha256(data).hexdigest()


def audit(recording,workspace,output):
    if output.exists():raise FileExistsError('Preserve previous audit')
    evidence=json.loads((recording/'report.json').read_text())
    if not (evidence.get('stop_reason')=='episode_completed' and evidence.get('recorder_returncode')==1
            and evidence.get('complete') is False and evidence.get('platform_checks',0)>0):
        raise ValueError('Require the explicitly quarantined completed-episode recording')
    log=(recording/'recorder.log').read_text()
    if 'rcl_shutdown already called on the given context' not in log or 'KeyboardInterrupt' not in log:
        raise ValueError('Unrecognized recorder failure')
    if any(evidence.get(k) is not False for k in ['ground_truth_input','controls_robot','tf_published','benchmark_launched_or_restarted']):
        raise ValueError('Not a passive recording')
    locks=json.loads((recording/'source_sha256.json').read_text())
    if not locks:raise ValueError('Missing source locks')
    for name,expected in locks.items():
        if digest(Path(name).read_bytes())!=expected:raise ValueError('Recorder source changed')
    passive=json.loads((recording/'passivity_check.json').read_text())
    if passive.get('passed') is not True or not set(passive['observed_publishers'])<={'/rosout','/parameter_events'}:
        raise ValueError('Recorder graph endpoints are not passive')
    data=(recording/'inputs.csv').read_bytes()
    if digest(data)!=evidence['input_sha256'] or not data.endswith(b'\n'):raise ValueError('Input bytes changed or CSV incomplete')
    wheel,gyro,stats=parse_inputs(data)
    description=(DESCRIPTION/'create3.urdf.xacro').read_bytes();imu=(DESCRIPTION/'sensors/imu.urdf.xacro').read_bytes()
    validate_axes(description,imu)
    batch=integrate(wheel,gyro)
    sys.path.insert(0,str(workspace/'src/osp_mission'))
    from osp_mission.pose_gyro_stream import PoseGyroStream
    stream=PoseGyroStream();online=[]
    for row in csv.DictReader(io.StringIO(data.decode())):
        online+=(stream.imu(float(row['stamp']),float(row['angular_z'])) if row['sensor']=='imu' else stream.wheel(pose(row)))
    online=np.asarray(online)
    if online.shape!=batch.shape or not np.allclose(online,batch,atol=1e-8,rtol=0.):
        raise ValueError('Streaming arrival-order estimator differs from causal batch replay')
    # Evaluation inputs are accessed only after sensor-only estimates exist.
    episode=Path(evidence['episode']);truth_path=episode/'ground_truth_trajectory.csv'
    evaluation=json.loads((episode/'evaluation.json').read_text())
    if evaluation.get('platform_contract',{}).get('passed') is not True:raise ValueError('Final official platform gate failed')
    truth=ordered([[float(r[k]) for k in ['sim_seconds','world_x','world_y','world_yaw']]
        for r in csv.DictReader(truth_path.open())])
    result=compare(truth,dict(wheel=ordered(wheel),pose_increment_gyro=batch))
    paths=[Path(__file__),recording/'report.json',recording/'source_sha256.json',recording/'passivity_check.json',
        recording/'recorder.log',recording/'inputs.csv',truth_path,episode/'evaluation.json',
        workspace/'src/osp_mission/osp_mission/pose_gyro_stream.py',workspace/'src/osp_mission/osp_mission/pose_increment_gyro.py']
    result.update(recording=str(recording),evidence_type='Quarantined completed-episode sensor replay; original capture is not cleanly completed',
        original_recorder_complete=False,original_shutdown_error_preserved=True,input_validation=stats,
        streaming_batch_maximum_absolute_difference=float(np.abs(online-batch).max()),
        source_sha256={str(p.resolve()):digest(p.read_bytes()) for p in paths},
        imu_description_sha256=dict(create3=digest(description),imu=digest(imu)),
        ground_truth_supplied_to_estimator=False,scale_fitted=False,configuration_deployed=False,hardware_validated=False,
        limitations=['One U-shaped development scene, two trajectories; ideal simulated gyro, not hardware calibration.',
            'A known shutdown exception left complete=false; this audit validates sensor bytes without reclassifying the recorder.',
            'Fixed common initial alignment excludes drift before recording. Truth interpolation is bounded at 0.35s.',
            'Offline and file-only streaming agreement does not establish fused-TF navigation performance.'])
    output.write_text(json.dumps(result,indent=2)+'\n');return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ['recording','workspace','output']:p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args();r=audit(a.recording.resolve(),a.workspace.resolve(),a.output.resolve())
    print(json.dumps({k:r[k] for k in ['duration_s','actual_path_m','streaming_batch_maximum_absolute_difference','estimators']},indent=2))
