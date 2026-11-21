"""Independent post-run camera/geometry overlap, stable metric for both policies."""
from collections import Counter
import csv
import json
import math
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def surfaces(scene):
    pts=[];normal=[];names=[];weights=[]
    def face(name,center,n,width):
        count=math.ceil(width/.1)
        tangent=np.array([-n[1],n[0]])
        for i in range(count):
            pts.append(np.asarray(center)+tangent*width*((i+.5)/count-.5))
            names.append(name);normal.append(n);weights.append(width/count)
    w,h=np.asarray(scene['room_size'])-.12
    for name,p,n,length in [('south',(0,-h/2),(0,1),w),('north',(0,h/2),(0,-1),w),
                            ('west',(-w/2,0),(1,0),h),('east',(w/2,0),(-1,0),h)]:
        face('wall/'+name,p,n,length)
    boxes=[]
    for i,b in enumerate(scene['obstacles']):
        center=np.array([b['x'],b['y']]);half=np.array([b['sx'],b['sy']])/2
        boxes.append((center-half,center+half))
        for name,n in [('east',(1,0)),('north',(0,1)),('west',(-1,0)),('south',(0,-1))]:
            face(f'box{i}/{name}',center+half*n,n,2*(half[1] if n[0] else half[0]))
    return np.array(pts),np.array(normal),np.array(names),np.array(weights),boxes


def visible_by_box(camera,points,boxes):
    d=points-camera
    blocked=np.zeros(len(d),bool)
    for lo,hi in boxes:
        # Segment against the box interior; touching just the endpoint is allowed.
        parallel=np.abs(d)<1e-10
        safe=np.where(parallel,1.,d)
        a=(lo-camera)/safe;b=(hi-camera)/safe
        entry=np.where(parallel,-np.inf,np.minimum(a,b)).max(axis=1)
        leave=np.where(parallel,np.inf,np.maximum(a,b)).min(axis=1)
        outside=np.any(parallel&((camera<lo)|(camera>hi)),axis=1)
        blocked|=(np.maximum(entry,0)<np.minimum(leave,1.-1e-5)) & ~outside
    return ~blocked


def exposure_coverage(scene,track,exposures):
    points,normals,names,weight,boxes=surfaces(scene)
    geometry=exposures['camera'];mount=exposures['base_from_camera']
    if geometry is None or mount is None or not exposures['elapsed_s'] or not len(track):
        return dict(valid=False,reason='camera_calibration_or_pose_missing')
    stamp=np.unique(exposures['elapsed_s'])
    stamp=stamp[(stamp>=track[0,0])&(stamp<=track[-1,0])]
    k=np.array(geometry['k']).reshape(3,3)
    optic=Rotation.from_quat(mount['xyzw']).as_matrix();trans=np.array(mount['translation'])
    world_yaw=np.unwrap(track[:,3]);hits=np.zeros(len(points),int);last=np.full(len(points),-np.inf)
    eligible=np.zeros(len(points),int);unobstructed=np.zeros(len(points),int)
    for t in stamp:
        i=np.searchsorted(track[:,0],t)
        if i>0 and i<len(track) and track[i,0]-track[i-1,0]>.5:continue
        x,y=[np.interp(t,track[:,0],track[:,j]) for j in (1,2)]
        yaw=np.interp(t,track[:,0],world_yaw);c,s=math.cos(yaw),math.sin(yaw)
        rot=np.array([[c,-s,0],[s,c,0],[0,0,1]])
        camera=np.array([x,y,0.])+rot@trans
        delta=points-camera[:2]
        rays=np.column_stack((delta,np.full(len(delta),.45-camera[2])))@rot@optic
        z=np.maximum(rays[:,2],1e-9);u=k[0,0]*rays[:,0]/z+k[0,2];v=k[1,1]*rays[:,1]/z+k[1,2]
        dist=np.linalg.norm(delta,axis=1)
        inc=-np.sum(normals*delta,axis=1)/np.maximum(dist,1e-9)
        good=((rays[:,2]>0)&(u>=0)&(u<geometry['width'])&(v>=0)&(v<geometry['height'])
              &(dist>.25)&(inc>.6)&(dist<.85*k[0,0]*.08*inc/18.))
        eligible+=good
        ids=np.flatnonzero(good)
        if len(ids):
            visible=ids[visible_by_box(camera[:2],points[ids],boxes)]
            unobstructed[visible]+=1
            credited=visible[t-last[visible]>=.15-1e-9];hits[credited]+=1;last[credited]=t
    observed=hits>=3
    state={}
    def fraction(mask):return float(np.sum(weight[mask]*observed[mask])/weight[mask].sum())
    for name in sorted(set(names)):
        mask=names==name;under=mask&~observed
        counts=Counter('fov_distance_or_incidence' if eligible[j]==0 else 'occluded' if unobstructed[j]==0
                       else 'insufficient_processed_exposures' for j in np.flatnonzero(under))
        state[name]=dict(fraction=fraction(mask),missing_samples=dict(counts))
    wall=np.char.startswith(names,'wall/')
    objects={}
    for i in range(len(boxes)):
        vals={k.split('/')[1]:v['fraction'] for k,v in state.items() if k.startswith(f'box{i}/')}
        objects[str(i)]=dict(faces=vals,complete_faces=sum(f>=.70 for f in vals.values()))
    return dict(valid=True,matched_exposures=len(stamp),wall_fraction=fraction(wall),
                object_surface_fraction=fraction(~wall),fully_observed_boxes=sum(o['complete_faces']==4 for o in objects.values()),
                total_boxes=len(boxes),objects=objects,surfaces=state)


def records(path):
    if not path.exists():return []
    return [json.loads(row) for row in path.read_text().splitlines() if row.strip()]


def revisit(path,track,alignment,elapsed):
    if not alignment:return None
    goals=records(path/'native/exploration_memory.jsonl')
    reached=[]
    for i,goal in enumerate(goals):
        a=goal['elapsed_seconds'];b=goals[i+1]['elapsed_seconds'] if i+1<len(goals) else elapsed
        target=np.array(alignment['rotation'])@goal['selected'][:2]+alignment['translation']
        span=track[(track[:,0]>=a)&(track[:,0]<=b),1:3]
        if len(span) and np.min(np.linalg.norm(span-target,axis=1))<=.30:reached.append(target)
    return sum(any(np.linalg.norm(p-q)<=.35 for q in reached[:j]) for j,p in enumerate(reached))


def evaluate(path):
    path=Path(path)
    read=lambda name:json.loads((path/name).read_text())
    scene,m,e=read('scene.json'),read('metrics.json'),read('processed_exposures.json')
    with (path/'headings.csv').open() as f:
        track=np.array([[float(row[k]) for k in ('elapsed_sim_s','x','y','yaw_rad')] for row in csv.DictReader(f)]).reshape(-1,4)
    if len(track):track=track[np.unique(track[:,0],return_index=True)[1]]
    audit=records(path/'native/global_coverage.jsonl')
    expected={t['id'] for t in scene['targets']};detected=set(m['final_ids'])
    coverage=exposure_coverage(scene,track,e)
    action=Counter(e['navigate_terminal'].values())
    internal=audit[-1] if audit else None
    return dict(case=scene['case'],policy=m['variant'],run=path.name,ids=sorted(detected),
                detected_fraction=len(expected&detected)/len(expected),unexpected_ids=sorted(detected-expected),
                witnesses_within_1_5m=m['registered_and_within_s'],strict=m['strict_success'],
                camera_coverage=coverage,elapsed_s=m['elapsed_sim_s'],distance_m=m['path_length_m'],
                bump_events=m['bumper_events'],nav_goals_aborted=action[6],nav_goals_canceled=action[5],
                nav_goals_succeeded=action[4],duplicate_reached_goals=revisit(path,track,m['world_from_map_initial_alignment'],m['elapsed_sim_s']),
                policy_snapshot=internal,stop_reason=m['reason'])
