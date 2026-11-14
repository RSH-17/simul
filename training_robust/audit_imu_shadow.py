"""Evaluate a finished shadow recording against independent Gazebo truth.

Each estimate is aligned only at the common start, then left unchanged. This
measures subsequent drift, not accumulated error before the observer attached.
No evaluation output is sent to the estimator or the navigation policy.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np

from audit_pose_trace import summarize


def ordered(rows):
    a=np.asarray(rows,dtype=float)
    if len(a)<2 or not np.isfinite(a).all():raise ValueError('Need finite timestamped samples')
    a=a[np.argsort(a[:,0],kind='stable')]
    a=a[np.r_[np.diff(a[:,0])>0,True]]
    if len(a)<2:raise ValueError('Need at least two distinct timestamps')
    a[:,3]=np.unwrap(a[:,3])
    return a


def pose(row):
    x,y,z,w=[float(row[k]) for k in ('qx','qy','qz','qw')]
    return [float(row['stamp']),float(row['x']),float(row['y']),
            np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z))]


def interpolate(a,t):
    idx=np.searchsorted(a[:,0],t,side='right')
    if idx==0 or idx>=len(a) or a[idx,0]-a[idx-1,0]>.35:
        raise ValueError('Start needs bounded interpolation without extrapolation')
    return np.array([np.interp(t,a[:,0],a[:,i]) for i in (1,2,3)])


def compare(truth, estimates):
    start=max(a[0,0] for a in [truth,*estimates.values()])
    end=min(a[-1,0] for a in [truth,*estimates.values()])
    if end-start<1.:raise ValueError('Less than one second of common data')
    initial_truth=interpolate(truth,start)
    results={}
    for name,a in estimates.items():
        first=interpolate(a,start);angle=initial_truth[2]-first[2]
        c,s=np.cos(angle),np.sin(angle);rotation=np.array([[c,-s],[s,c]])
        offset=initial_truth[:2]-rotation@first[:2]
        rows=a[(a[:,0]>=start)&(a[:,0]<end)].copy()
        idx=np.searchsorted(truth[:,0],rows[:,0],side='right')
        good=(idx>0)&(idx<len(truth))
        valid=np.flatnonzero(good)
        good[valid]&=truth[idx[valid],0]-truth[idx[valid]-1,0]<=.35
        rows=rows[good]
        if len(rows)<2:raise ValueError('Insufficient time-aligned independent truth')
        actual=np.column_stack([np.interp(rows[:,0],truth[:,0],truth[:,i]) for i in (1,2,3)])
        rows[:,1:3]=rows[:,1:3]@rotation.T+offset;rows[:,3]+=angle
        position=np.linalg.norm(rows[:,1:3]-actual[:,:2],axis=1)
        heading=np.abs((rows[:,3]-actual[:,2]+np.pi)%(2*np.pi)-np.pi)*180/np.pi
        results[name]=dict(position_error_m=summarize(position),heading_error_deg=summarize(heading),
            initial_alignment=[*offset.tolist(),float(angle)],
            final_position_error_m=float(position[-1]),final_heading_error_deg=float(heading[-1]))
    selected=truth[(truth[:,0]>=start)&(truth[:,0]<=end)]
    return dict(common_start_s=float(start),common_end_s=float(end),duration_s=float(end-start),
        actual_path_m=float(np.linalg.norm(np.diff(selected[:,1:3],axis=0),axis=1).sum()),
        actual_absolute_turn_rad=float(np.abs(np.diff(selected[:,3])).sum()),estimators=results)


def audit(recording,truth_path):
    evidence=json.loads((recording/'recording.json').read_text())
    if not all(evidence.get(k) for k in ['complete','tf_endpoints_isolated','platform_checks_passed']):
        raise ValueError('Require completed, isolated observation with the navigation guard continuously passing')
    if evidence['tf_published'] or evidence['commands_published'] or evidence['ground_truth_input']:
        raise ValueError('This audit requires a non-controlling, sensor-only shadow run')
    if evidence['counts'].get('imu',0)<2:raise ValueError('No IMU measurements')
    rows=list(csv.DictReader((recording/'inputs.csv').open()))
    estimates={k:ordered([pose(r) for r in rows if r['sensor']==k]) for k in ('wheel','ekf')}
    truth=ordered([[float(r[k]) for k in ('sim_seconds','world_x','world_y','world_yaw')]
        for r in csv.DictReader(truth_path.open())])
    result=compare(truth,estimates)
    result.update(evidence_type='Offline evaluation of live shadow estimator; not a new navigation success',
        recording=str(recording),truth_file=str(truth_path),ground_truth_supplied_to_estimator=False,
        limitations=['Initial fixed alignment removes error accumulated before recording.',
            'Short live interval does not qualify localization, marker accuracy or hardware.',
            'Truth is interpolated only over gaps up to 0.35 seconds.'],
        source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest()
            for p in [recording/'recording.json',recording/'inputs.csv',recording/'ekf.yaml',truth_path]})
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('recording',type=Path)
    p.add_argument('truth',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    a.output.write_text(json.dumps(audit(a.recording.resolve(),a.truth.resolve()),indent=2)+'\n')
