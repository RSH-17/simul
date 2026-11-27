"""Frozen past trajectories. Counterfactual goals on saved maps cannot prove navigation."""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from scipy import ndimage
from osp_mission.center_wall import CenterWall
from osp_mission._r32_navigation_core import Grid
from route_repeat import counts,resample

def analyze(folder):
    read=lambda p:json.loads(p.read_text())
    metrics=read(folder/'metrics.json')
    heads=np.loadtxt(folder/'headings.csv',delimiter=',',skiprows=1,ndmin=2)
    seq=[(x,np.zeros(len(x),dtype=bool)) for x in resample(heads)]
    geo=counts(seq,85.,[])
    actual=dict(name=folder.name,recorded_variant=metrics.get('variant'),elapsed_s=metrics['elapsed_sim_s'],
        registered=metrics.get('registered_s'),actual_geometric_repeat_m=geo['repeated_m'],
        actual_geometric_longest_m=geo['longest_continuous_repeat_m'])
    aln=read(folder/'audit.json').get('world_from_map_initial_alignment')
    if aln is None:return dict(actual,replay_unavailable='No measured world/map alignment')
    rot=np.asarray(aln['rotation']);shift=np.asarray(aln['translation'])
    oldfile=folder/'native/center_wall.jsonl'
    previous=[json.loads(l) for l in oldfile.read_text().splitlines() if l] if oldfile.exists() else []
    goals=folder/'native/hybrid_goals.jsonl'
    dispatch=[json.loads(l) for l in goals.read_text().splitlines() if l] if goals.is_file() else []
    begins=[json.loads(l) for l in (folder/'events.jsonl').read_text().splitlines() if l]
    start=next(e['sim'] for e in begins if e['event']=='start_ack')
    srcs=sorted((folder/'native/policy_decisions').glob('*.npz'))
    s=CenterWall()
    cam=metrics.get('camera',{}).get('camera_info') or {}
    fx=(cam.get('k') or [None])[0];width=cam.get('width',0)
    if not fx or not width:return dict(actual,replay_unavailable='No measured live camera calibration')
    s.survey.camera=fx,2*math.atan(width/(2*fx))
    high=-.01
    chosen=[]
    for source in srcs:
        with np.load(source) as d:
            if 'viewed' not in d:continue
            t=float(d['elapsed']);scale=float(d['resolution']);origin=(*d['origin'][:2],0.)
            pose=(*d['robot'],float(d['heading']));grid=Grid(d['data'].copy(),scale,origin)
            cells=grid.cells
            cost=Grid(np.where(cells<0,-1,np.where(ndimage.distance_transform_edt(cells<65)*scale<.25,99,0)),scale,origin)
            viewed=d['viewed'].copy()
            opts=[dict(x=a[0],y=a[1],yaw=a[2],scan_angle=a[3],distance=a[6],features=a[4:].copy()) for a in d['candidates']]
        for stamp,x,y,_ in heads[(heads[:,0]>high)&(heads[:,0]<=t)]:
            position=(np.array([x,y])-shift)@rot
            s.survey.position(*position,stamp)
        s.batch.update([],[],pose[:2])
        candidate=s.decide(grid,cost,pose,(0.,0.,0.),viewed,t,600.,lambda goal:False,opts)
        orig=min(previous,key=lambda r:abs(r['elapsed']-t)) if previous else {}
        if orig and abs(orig['elapsed']-t)>4.:orig={}
        chosen_before=min(dispatch,key=lambda r:abs(r['t']-(start+t))) if dispatch else {}
        if chosen_before and abs(chosen_before['t']-(start+t))>4.:chosen_before={}
        before=[chosen_before['x'],chosen_before['y']] if chosen_before else None
        after=[candidate['x'],candidate['y']] if candidate else None
        if candidate is not None:s.dispatched(candidate,t)
        chosen.append(dict(t=t,snapshot=source.name,sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
            requested=candidate.get('r32_kind') if candidate else None,reason=s.reason,
            escape=bool(candidate and candidate.get('survey_escape')),last_camera_gain_s=(
            s.survey.last_progress if np.isfinite(s.survey.last_progress) else None),old=orig.get('reason'),
            actual_goal_xy=before,replay_goal_xy=after,goal_separation_m=math.dist(before,after) if before and after else None))
        high=t
    return dict(actual,scored_snapshots=len(chosen),
        selected_wall=sum(r['requested']=='wall_surface' for r in chosen),selected_escape=sum(r['escape'] for r in chosen),
        actual_previous_reasons=dict(Counter(e.get('reason') for e in previous)),replay=chosen)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('output',type=Path);p.add_argument('folders',type=Path,nargs='+')
    args=p.parse_args();result=[]
    for f in args.folders:
        try:r=analyze(f)
        except (ValueError,KeyError,OSError,TypeError) as exc:r=dict(name=f.name,unavailable=repr(exc))
        result.append(r)
        print({k:v for k,v in r.items() if k not in ('replay','actual_previous_reasons')},flush=True)
    args.output.write_text(json.dumps(dict(scope='archived geometry; actual route; counterfactual choices with estimated costmap and no frozen object tracks',runs=result),allow_nan=False,indent=2)+'\n')
