"""Evaluate a sensor-only odometry experiment after constructing its estimate.

Supports a completed, guarded official shadow recording. Truth is consumed only
by this evaluator, never by the integration module or a live robot process.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np

import pose_increment_gyro
from audit_imu_shadow import ordered,pose,compare


def audit(recording,truth_path):
    evidence=json.loads((recording/'recording.json').read_text())
    if not all(evidence.get(k) for k in ['complete','tf_endpoints_isolated','platform_checks_passed']):
        raise ValueError('Require completed guarded, isolated sensor recording')
    if any(evidence.get(k) for k in ['tf_published','commands_published','ground_truth_input']):
        raise ValueError('Require sensor-only recording')
    rows=list(csv.DictReader((recording/'inputs.csv').open()))
    wheel_rows=[r for r in rows if r['sensor']=='wheel']
    wheel=np.asarray([pose(r) for r in wheel_rows])
    gyro=np.asarray([[float(r['stamp']),float(r['wz'])] for r in rows if r['sensor']=='imu'])
    # No fitted parameter or evaluator data is available at this point.
    estimated=pose_increment_gyro.integrate(wheel,gyro)
    wheel=ordered(wheel)
    delta=np.diff(wheel[:,1:3],axis=0);turn=np.diff(wheel[:,3]);mid=(wheel[:-1,3]+wheel[1:,3])/2
    increments=np.sum(delta*np.column_stack([np.cos(mid),np.sin(mid)]),axis=1)/np.sinc(turn/(2*np.pi))
    dt=np.diff(wheel[:,0]);velocity=np.asarray([[float(r['vx']),float(r['wz'])] for r in wheel_rows])
    consistency=dict(scope='Recorded wheel pose versus its reported twist over the full wheel interval; no truth',
        pose_forward_distance_m=float(increments.sum()),reported_twist_forward_integral_m=float(np.sum(velocity[:-1,0]*dt)),
        pose_signed_turn_rad=float(turn.sum()),reported_twist_signed_turn_integral_rad=float(np.sum(velocity[:-1,1]*dt)),
        twist_integration_method='Past-sample hold at acquisition timestamps; no fitted scale or offset')
    estimates=dict(wheel=wheel,ekf=ordered([pose(r) for r in rows if r['sensor']=='ekf']),
                   pose_increment_gyro=estimated)
    truth=ordered([[float(r[k]) for k in ['sim_seconds','world_x','world_y','world_yaw']]
                   for r in csv.DictReader(truth_path.open())])
    result=compare(truth,estimates)
    sources=[Path(__file__),Path(pose_increment_gyro.__file__),recording/'recording.json',
             recording/'inputs.csv',truth_path]
    result.update(evidence_type='Offline full-rate recorded odometry/gyro integration; not native navigation',
        wheel_twist_pose_consistency=consistency,
        source_sha256={str(p.resolve()):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
        ground_truth_supplied_to_estimator=False,scale_fitted=False,absolute_imu_orientation_used=False,
        production_promoted=False,native_validated=False,hardware_validated=False,
        limitations=['One 17.6-second development segment, not a long-run or unseen-scenario qualification.',
            'Only common initial alignment is used; error accumulated before this interval is excluded.',
            'Requires planar motion and gyro expressed about the base vertical axis.',
            'No gyro bias, covariance, relocalization or missing-stream runtime adapter has been validated.'])
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ['recording','truth','output']:p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args()
    if a.output.exists():raise FileExistsError('Preserve previous audit')
    a.output.write_text(json.dumps(audit(a.recording.resolve(),a.truth.resolve()),indent=2)+'\n')
