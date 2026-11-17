"""Evaluator-only separation of landmark observations and localization error.

Replace the observed robot pose with independent truth only in this offline
diagnostic. No corrected coordinates feed the detector, SLAM, policy or robot.
Truth/pose interpolation is bounded to 0.35 seconds; the residual is not a
calibration accuracy claim or a counterfactual navigation result.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml

from audit_pose_trace import audit as audit_pose, summarize
from audit_slam_coordinate_changes import audit as audit_graph


def rotation(angle):
    c,s=np.cos(angle),np.sin(angle)
    return np.array([[c,-s],[s,c]])


def bounded_pose(trace,stamp):
    index=np.searchsorted(trace[:,0],stamp,side='right')
    if not 0<index<len(trace) or trace[index,0]-trace[index-1,0]>.35:
        return None
    return np.array([np.interp(stamp,trace[:,0],trace[:,i]) for i in [1,2,3]])


def substitute_robot_pose(point,observed,actual):
    return actual[:2]+rotation(actual[2])@rotation(-observed[2])@(point-observed[:2])


def read_trace(path,fields):
    values=np.array([[float(r[k]) for k in fields] for r in csv.DictReader(path.open())])
    values=values[np.isfinite(values).all(axis=1)]
    values=values[np.argsort(values[:,0],kind='stable')]
    values=values[np.r_[np.diff(values[:,0])>0,True]]
    values[:,3]=np.unwrap(values[:,3])
    return values


def audit(episode):
    evaluation=episode/'evaluation.json';result=json.loads(evaluation.read_text())
    scenario=Path(result['scenario']);scene=yaml.safe_load(scenario.read_text())
    tx,ty,angle=result['ground_truth_audit']['world_to_map']
    expected={t['id']:rotation(angle)@np.array([t['x'],t['y']])+[tx,ty] for t in scene['targets']}
    truth_path=episode/'ground_truth_trajectory.csv';pose_path=episode/'pose_trace.csv'
    truth=read_trace(truth_path,['sim_seconds','world_x','world_y','world_yaw'])
    truth[:,1:3]=truth[:,1:3]@rotation(angle).T+[tx,ty];truth[:,3]+=angle
    poses=read_trace(pose_path,['map_stamp','map_x','map_y','map_yaw'])
    observations_path=episode/'marker_observations.json';rows={};excluded=0
    for observation in json.loads(observations_path.read_text()):
        ident=observation['id']
        if ident not in expected or observation.get('map_point') is None:continue
        stamp=observation['stamp'];actual=bounded_pose(truth,stamp);observed=bounded_pose(poses,stamp)
        if actual is None or observed is None:excluded+=1;continue
        point=np.array(observation['map_point'][:2]);substituted=substitute_robot_pose(point,observed,actual)
        rows.setdefault(ident,[]).append([np.linalg.norm(point-expected[ident]),
            np.linalg.norm(substituted-expected[ident]),np.linalg.norm(observed[:2]-actual[:2]),
            abs((observed[2]-actual[2]+np.pi)%(2*np.pi)-np.pi)*180/np.pi])
    metrics={str(ident):{name:summarize(np.array(values)[:,i]) for i,name in enumerate([
        'observed_marker_error_m','truth_pose_substituted_residual_m','robot_position_error_m','robot_heading_error_deg'])}
        for ident,values in rows.items()}
    graph=audit_graph(episode,scenario);pose=audit_pose(episode)
    sources=[Path(__file__),evaluation,scenario,truth_path,pose_path,observations_path,
        Path(__file__).with_name('audit_pose_trace.py'),Path(__file__).with_name('audit_slam_coordinate_changes.py')]
    return dict(episode=str(episode),evidence_type='Offline independent coordinate diagnosis of an original completed native run',
        observation_metrics=metrics,excluded_observations=excluded,marker_finalization=graph['markers'],
        native_pose_errors=pose['pose_errors'],graph_audit=graph,
        source_sha256={str(p.resolve()):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
        truth_supplied_to_policy=False,original_results_changed=False,hardware_calibration_claim=False,
        limitations=['Truth substitution is retrospective diagnosis, not deployable localization or navigation success.',
            'Robot poses are interpolated across at most 0.35s gaps; residuals include timing and interpolation uncertainty.',
            'This audit identifies localization/final-graph contributions, not the underlying cause of every SLAM update.'])


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--episode',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():raise FileExistsError('Preserve earlier audit')
    a.output.write_text(json.dumps(audit(a.episode.resolve()),indent=2)+'\n')
