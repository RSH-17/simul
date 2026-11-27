"""Development only: two archived balanced runs and the first R2 regression.

R3's src/osp_mission must be on PYTHONPATH. Ground truth is used after goal
generation for attribution, never to select or filter observation goals.
"""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import numpy as np

from osp_mission._r32_navigation_core import Grid
from osp_mission.adaptive_faces import clearance_grid
from osp_mission.box_views import local_boxes
from osp_mission.center_wall import CenterWall
from osp_mission.followthrough import face_match

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from near_miss import audit

NAMES=('v151-boxes5-track-m112601-trial1','v151-survey-candidate-track-m112601-1122',
       'v151-boxes5-track-m112605-trial5')


def read_lines(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line]


def rounded(values):
    return [round(float(x),3) for x in values]


def analyze(run,camera):
    calib=json.loads(camera.read_text())
    metrics=json.loads((run/'metrics.json').read_text())
    actions=read_lines(run/'native/center_wall.jsonl')
    dispatched=read_lines(run/'native/exploration_memory.jsonl')
    stamps=np.genfromtxt(run/'headings.csv',delimiter=',',names=True)
    heading=np.unwrap(stamps['yaw_rad'])
    scene=json.loads((run/'scene.json').read_text())
    target=next(t for t in scene['targets'] if t['id']==32)
    box=next(b for b in scene['obstacles'] if b['name']==target['attached_to'])
    projection=audit(run,run/'frames.csv',camera,run/'world/osp_studio.sdf',32)
    front=projection['front_local_x_sign']*np.array([math.cos(target['yaw']),math.sin(target['yaw'])])
    adaptive=[r for r in actions if 'survey_kind' in r]
    counts=dict(selections=len(actions),kinds=Counter(r['kind'] for r in actions),
                adaptive_kinds=Counter(r['survey_kind'] for r in adaptive),
                adaptive_reasons=Counter(r['reason'] for r in adaptive),
                overlap_other_kinds_with_nonzero_interior=Counter(r['kind'] for r in actions
                    if r['available']['interior'] and r['kind'] in ('wall_surface','interior_frontier')),
                locally_batched_interiors=sum(bool(r.get('local_batch')) for r in actions if r['kind']=='interior_surface'),
                fitted_interiors=sum(r.get('fitted_face') is not None for r in actions if r['kind']=='interior_surface'),
                owned_interiors=sum(r.get('batch_box') is not None for r in actions if r['kind']=='interior_surface'))
    visits=[]
    arrivals=[]
    wait_clock={}
    previous=None
    previous_dispatch=None
    for f in sorted((run/'native/policy_decisions').glob('*.npz')):
        with np.load(f,allow_pickle=False) as saved:
            elapsed=float(saved['elapsed']);pose=(*saved['robot'],float(saved['heading']))
            grid=Grid(saved['data'],float(saved['resolution']),(*saved['origin'],0.))
            viewed=saved['viewed']
            choices=[dict(x=float(x),y=float(y),yaw=float(angle),scan_angle=float(turn),features=np.array(feat),
                          distance=float(feat[2])) for x,y,angle,turn,*feat in saved['candidates']]
        live=min(actions,key=lambda row:abs(row['elapsed']-elapsed))
        logged=min(dispatched,key=lambda row:abs(row['elapsed_seconds']-elapsed))
        if abs(live['elapsed']-elapsed)>1 or abs(logged['elapsed_seconds']-elapsed)>1:
            raise ValueError('snapshot and goal log mismatch')
        wx,wy,wangle=[np.interp(elapsed,stamps['elapsed_sim_s'],v) for v in (stamps['x'],stamps['y'],heading)]
        angle=wangle-pose[2]
        rot=np.array([[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]])
        def as_world(point):return rot@(np.asarray(point)-pose[:2])+[wx,wy]
        box_map=rot.T@(np.array([box['x'],box['y']])-[wx,wy])+pose[:2]
        near=math.dist(pose[:2],box_map)
        selected_surface=live.get('surface')
        host_offset=math.dist(selected_surface,box_map) if selected_surface is not None else None
        if near<=3.5 or (host_offset is not None and host_offset<=1.):
            fits=local_boxes(grid,pose)
            fit_error=min((math.dist(c,box_map) for c,_ in fits),default=None)
            visits.append(dict(elapsed=round(elapsed,2),robot_to_host=round(near,3),
                selection_kind=live['kind'],surface_to_host=None if host_offset is None else round(host_offset,3),
                fitted_face=live.get('fitted_face'),logged_box_owner=live.get('batch_box') is not None,
                local_batch=live.get('local_batch'),near_fit_center_error=None if fit_error is None else round(fit_error,3)))
        if adaptive and previous and previous['kind']=='interior_surface' and previous.get('survey_kind')=='interior':
            # One-step experiment. Arrival and association are based only on the
            # previously dispatched SLAM goal and contemporaneous local occupancy.
            prev_pose=previous_dispatch['selected']
            if math.dist(pose[:2],prev_pose[:2])<=.65:
                selector=CenterWall()
                selector.survey.camera=(calib['k'][0],2*math.atan(calib['width']/(2*calib['k'][0])))
                selector.survey.waiting=dict(wait_clock)
                turn=math.radians(24 if previous['fitted_face'] else 36)
                seed=dict(r32_kind='interior_surface',survey_nav_odom=prev_pose[:2],
                          surface_odom=previous['surface'],normal_odom=prev_pose[2]+turn/2+math.pi,
                          survey_expected_s=previous['survey_expected_s'])
                selector.survey.follow.dispatched(seed,previous['elapsed'])
                clearance=clearance_grid(grid)
                cost=Grid(np.where(clearance>=.32,0,100).astype(np.int8),grid.resolution,grid.origin)
                fits=local_boxes(grid,pose)
                owners=[(c,fs) for c,fs in fits if math.dist(c,seed['surface_odom'])<.68]
                detail=dict(elapsed=round(elapsed,2),prior_elapsed=previous['elapsed'],prior_fitted=previous.get('fitted_face'),
                            current_live_kind=live['kind'],reason=live['reason'],local_fits=len(fits),
                            plausible_local_owners=len(owners),local_options=[])
                # The probe records options before the R3 rule. Projected GT
                # values below are written for evaluation only, never returned.
                original=selector.survey.follow.choose
                def probe(grid,pose,transform,eligible,waiting,period,elapsed,remaining):
                    if len(owners)==1:
                        c,fs=owners[0]
                        detail['local_center_map']=rounded(c)
                        detail['prior_anchor_to_fit']=round(math.dist(c,seed['surface_odom']),3)
                        detail['overlay_center_to_host_gt']=round(math.dist(c,box_map),3)
                        for opt in eligible.get('interior',[]):
                            g=opt.goal
                            if math.dist(c,g['surface_map'])>.65:continue
                            index=face_match(fs,np.asarray(g['surface_map']),g['normal_odom'],grid.resolution)
                            camera_xy=as_world((g['x'],g['y']))
                            delta=camera_xy-np.array([target['x'],target['y']])
                            length=np.linalg.norm(delta)
                            detail['local_options'].append(dict(face_index=index,walk_m=round(opt.walk,3),
                                estimated_s=round(opt.seconds,3),view_map=rounded((g['x'],g['y'])),
                                clearance_m=round(g.get('clearance_m',0),3),target32_dist=round(float(length),3),
                                target32_printed_side_cos=round(float(delta@front/max(length,1.e-6)),3)))
                    return original(grid,pose,transform,eligible,waiting,period,elapsed,remaining)
                selector.survey.follow.choose=probe
                selected=selector.decide(grid,cost,pose,(0.,0.,0.),viewed,elapsed,600.,lambda _:False,choices=choices)
                if selected:
                    detail.update(one_step_kind=selected.get('survey_kind'),one_step_reason=selector.reason,
                                  one_step_goal=rounded((selected['x'],selected['y'])))
                    if 'survey_follow_step' in selected:
                        detail.update(step=selected['survey_follow_step'], budget=round(selected['survey_follow_budget_s'],3))
                arrivals.append(detail)
        if 'survey_kind' in live:
            wait_clock[live['survey_kind']]=elapsed
        previous,previous_dispatch=live,logged
    return dict(run=run.name,final_ids=metrics['final_ids'],counts=counts,
                box32_exposure_projection=projection,host_snapshots=visits,one_step_replays=arrivals)


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('recordings',type=Path)
    p.add_argument('camera',type=Path)
    p.add_argument('output',type=Path)
    args=p.parse_args()
    rows=[analyze(args.recordings/n,args.camera) for n in NAMES]
    out=dict(scope='Three development recordings only. GT attribution aligns each archived SLAM robot pose to '
             'time-interpolated ground truth. This is not a stable absolute map registration. Live logs contain '
             'selected records/candidate counts, not per-option scores, all box tracks, rejected normals or Nav2 '
             'costmaps. R3 one-step replay uses actual SLAM occupancy+directional RGB bits+pose+Flow candidates, '
             '0.32m proxy costmap, identity map-to-odom, prior dispatched map anchor and most recent class-service '
             'times, fresh selector history, no live veto, tracks or ID interrupts. It never advances the robot, '
             'certifies Nav2 paths, or measures detection performance. Previous face normal is reconstructed from '
             'the recorded goal yaw + half of its corresponding scan arc + pi.',runs=rows)
    args.output.write_text(json.dumps(out,indent=2)+'\n')
