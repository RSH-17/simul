"""Post-run evaluator. No scene truth, trajectory or metric is imported by the policy."""
import argparse
from collections import Counter
import csv
import json
import math
from pathlib import Path

import numpy as np


def load(path):
    return json.loads(path.read_text())


def revisit(run, obstacles):
    """Time inside 2 m after leaving that box by >=3 m. Count overlapping boxes once."""
    points=np.asarray([(b['x'],b['y']) for b in obstacles])
    with (run/'headings.csv').open() as f:
        rows=np.array([[float(r[k]) for k in ('elapsed_sim_s','x','y')] for r in csv.DictReader(f)])
    times,xy=rows[:,0],rows[:,1:]
    distances=np.linalg.norm(xy[:,None,:]-points[None,:,:],axis=2)
    inside=np.zeros(len(points),bool)
    visits=np.zeros(len(points),int)
    repeated=np.zeros(len(times),bool)
    for i,ds in enumerate(distances):
        visits+= ~inside & (ds<=2.)
        inside=(inside | (ds<=2.)) & (ds<3.)
        repeated[i]=np.any((ds<=2.) & (visits>=2))
    delta=np.maximum(0,np.diff(times))
    return float(delta[repeated[:-1]].sum()), times, distances.min(axis=1)


def low_gain_time(run, times, nearest_box):
    # Shared retrospective metric in SLAM map frame. Count only intervals with
    # stable matched occupied cells, sufficient local witnesses and RGB coverage.
    previous=None
    low=eligible=0.
    for path in sorted((run/'native/policy_decisions').glob('*.npz')):
        with np.load(path) as z:
            current=(float(z['elapsed']),z['origin'],float(z['resolution']),z['data'],z['viewed'],z['robot'])
        if previous is not None:
            t,origin,res,grid,seen,robot=previous
            t1,o1,r1,g1,v1,_=current
            if res==r1 and seen.ndim==v1.ndim==3 and t1>t:
                cr=np.ceil(2.5/res)
                x,y=np.floor((robot-origin)/res).astype(int)
                lo_r,lo_c=max(0,int(y-cr)),max(0,int(x-cr))
                hi_r,hi_c=min(grid.shape[0],int(y+cr+1)),min(grid.shape[1],int(x+cr+1))
                if hi_r>lo_r and hi_c>lo_c:
                    rr,cc=np.where(grid[lo_r:hi_r,lo_c:hi_c]>=65)
                    rr+=lo_r; cc+=lo_c
                    px,py=(cc+.5)*res+origin[0],(rr+.5)*res+origin[1]
                    rr1=np.floor((py-o1[1])/res).astype(int); cc1=np.floor((px-o1[0])/res).astype(int)
                    valid=(rr1>=0)&(cc1>=0)&(rr1<g1.shape[0])&(cc1<g1.shape[1]) & (np.hypot(px-robot[0],py-robot[1])<=2.5)
                    ref=int(valid.sum())
                    valid[valid]&=g1[rr1[valid],cc1[valid]]>=65
                    if ref>=8 and valid.sum()>=.65*ref:
                        gain=(v1[:,rr1[valid],cc1[valid]] & ~seen[:,rr[valid],cc[valid]]).sum()*res
                        # Count only the observed span of the simulation trajectory.
                        left=np.maximum(times[:-1],t)
                        right=np.minimum(times[1:],t1)
                        seconds=float(np.maximum(0,right-left)[nearest_box[:-1]<=2.].sum())
                        eligible+=seconds
                        if gain<.25:
                            low+=seconds
        previous=current
    return dict(eligible_near_box_s=eligible, low_new_local_surface_s=low)


def summarize(run,row):
    scene=load(run/'scene.json'); metrics=load(run/'metrics.json')
    wall=next(t['id'] for t in scene['targets'] if 'wall' in t['attached_to'])
    boxes={t['id'] for t in scene['targets']}-{wall}
    registered=set(row['registered_ids']); proof=set(row['sensor_verified_ids'])
    independent=set(map(int,metrics['registered_and_within_s']))
    approached=registered & proof & independent
    events=[json.loads(l) for l in (run/'native/center_wall.jsonl').read_text().splitlines()]
    revisit_s,times,nearest=revisit(run,scene['obstacles'])
    return dict(seed=row['seed'], run=run.name, initial_world_xy=row['start_world_pose'],
        world_sha256=row['world_sha256'],wall_id=wall, box_ids=sorted(boxes),
        registered=sorted(registered),approached=sorted(approached),wall_approached=wall in approached,
        box_approached=len(boxes & approached),strict=row['strict_saved_map_success'],
        wall_min_m=metrics['min_ground_truth_m'].get(str(wall)),
        first_wall_goal_s=next((e['elapsed'] for e in events if e['kind']=='wall_surface'),None),
        first_wall_strip_s=next((e['elapsed'] for e in events if e.get('reason') in
                          ('region_low_yield','wall_strip_due','continue_wall_strip') and e['kind']=='wall_surface'),None),
        region_low_yield_switches=sum(e['reason']=='region_low_yield' and e['kind']=='wall_surface' for e in events),
        selected_reasons=dict(Counter(e['reason'] for e in events)),
        transit_deadline_cancels=row['cancellations'].get('wall_region_due',0),
        elapsed_sim_s=row['elapsed_sim_s'],path_m=row['path_length_m'],bumps=row['bumper_events'],
        revisited_box_neighborhood_s=revisit_s,**low_gain_time(run,times,nearest))


def main(root,base,output):
    rows=load(root/'report.json')['runs']
    new_paths=list(map(Path,load(root/'runs.json')))
    old_paths=list(map(Path,load(base/'runs.json')))
    old_rows=load(base/'report.json')['runs']
    pairs=[]
    for path,row in zip(new_paths,rows,strict=True):
        old=next(r for r in old_rows if r['seed']==row['seed'])
        other=next(p for p in old_paths if p.name==old['run'])
        assert row['run']==path.name and row['world_sha256']==old['world_sha256']
        assert math.dist(row['start_world_pose'][:2],old['start_world_pose'][:2])<.05
        pairs.append(dict(baseline=summarize(other,old),candidate=summarize(path,row)))
    result=dict(complete=(root/'complete.json').exists(),planned=5,pairs=pairs)
    output.write_text(json.dumps(result,indent=2)+'\n')
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser()
    for name in ('root','base','output'):
        p.add_argument(name,type=Path)
    args=p.parse_args()
    main(args.root,args.base,args.output)
