"""Register a separate, paired stress test at two partially occluding movable obstacles."""
import argparse
import hashlib
import math
from pathlib import Path
import shutil
from tempfile import TemporaryDirectory

import numpy as np
from scipy import ndimage

from fixture import FILES, digest, read, save
from geometry import RESOLUTION, raster, table_parts
from route_repeat import boundary_distance, nearby_pairs


def ray_range(xy, directions, body):
    c,s=math.cos(body['yaw']),math.sin(body['yaw'])
    axes=np.array([[c,-s],[s,c]])
    origin=(xy-np.array([body['x'],body['y']]))@axes
    rays=directions@axes
    half=np.array([body['sx'],body['sy']])/2
    ray=np.where(abs(rays)>1.e-10,rays,1.e-10)
    first=(-half-origin)/ray
    last=(half-origin)/ray
    enter=np.minimum(first,last).max(axis=1)
    exit_=np.maximum(first,last).min(axis=1)
    return np.where(exit_>=np.maximum(enter,0.),np.maximum(0.,enter),np.inf)


def select(scene,tables,nonce):
    parts=table_parts(tables,scene['furniture'])
    spawn,origin=raster(scene,parts,for_spawn=True)
    wheel,_=raster(scene,parts)
    distance=lambda grid:ndimage.distance_transform_edt(np.pad(~grid,1))[1:-1,1:-1]*RESOLUTION
    clearance=distance(spawn)
    regions,_=ndimage.label(distance(wheel)>=.40)
    old=np.array([scene['robot_spawn'][k] for k in ('x','y')])
    col,row=np.floor((old-origin)/RESOLUTION).astype(int)
    if regions[row,col]==0:
        raise ValueError('Original start not in driving region')
    cells=np.argwhere((regions==regions[row,col])&(clearance>=.50))
    xy=origin+(cells[:,::-1]+.5)*RESOLUTION
    pairs=sorted(nearby_pairs(scene),key=lambda p:(p[2],p[0],p[1]))
    if not pairs:
        raise ValueError('No adjacent independent objects in this fixture')
    a,b,gap,A,B=pairs[0]
    center=(A.mean(axis=0)+B.mean(axis=0))*.5
    candidates=np.flatnonzero((boundary_distance(xy,A)<=2.)&(boundary_distance(xy,B)<=2.))
    bodies=[*scene['obstacles'],*scene['partitions'],*[p for p in parts if p['z']-p['height']/2<=.32]]
    ia=next(i for i,body in enumerate(bodies) if body.get('name')==a)
    ib=next(i for i,body in enumerate(bodies) if body.get('name')==b)
    rng=np.random.default_rng(int(nonce[:16],16))
    for k in rng.permutation(candidates):
        yaw=math.atan2(*(center-xy[k])[::-1])
        angles=np.linspace(yaw-.60,yaw+.60,121)
        directions=np.stack((np.cos(angles),np.sin(angles)),axis=1)
        d=np.stack([ray_range(xy[k],directions,o) for o in bodies])
        hit=np.isfinite(d)
        nearest=np.where(hit.any(axis=0),d.argmin(axis=0),-1)
        overlap=hit[ia]&hit[ib]&np.isin(nearest,[ia,ib])
        visible=(int((nearest==ia).sum()),int((nearest==ib).sum()))
        if overlap.sum()<8 or min(visible)<4:
            continue
        r,c=cells[k]
        return dict(x=float(xy[k,0]),y=float(xy[k,1]),yaw=float(yaw)),dict(
            pair=[a,b],grid_clearance_m=float(clearance[r,c]),gap_m=gap,
            same_direction_rays=int(overlap.sum()),rays_total=121,unique_first_hits=visible,
            ray_height_m=.32,ray_spacing_rad=.01,
            distance_to_bodies_m=[float(boundary_distance(xy[k:k+1],v)[0]) for v in (A,B)])
    raise ValueError('No start with separate and overlapping projections for this pair')


def prepare(official, destination, candidate_identity):
    official=official.resolve()
    if destination.exists():
        raise FileExistsError(destination)
    p=read(official/'protocol.json')
    r=read(official/'registration.json')
    if digest(official/'registration.json')!=p['registration_sha256'] or r['furniture']!='fixed':
        raise ValueError('Different exam room or unregistered parent')
    # Select for physical proximity only. Do not select a map by policy score.
    options=[]
    for world in p['worlds']:
        source=official/'fixtures'/f"map-{world['seed']}"
        pairs=nearby_pairs(read(source/'scene.json'))
        if pairs:options.append((min(z[2] for z in pairs),world,source))
    _,world,source=min(options,key=lambda entry:entry[0])
    nonce=hashlib.sha256((digest(official/'protocol.json')+'/overlapping-movable-v1').encode()).hexdigest()
    scene=read(source/'scene.json')
    for name,value in world['files'].items():
        if digest(source/name)!=value:raise ValueError('Source fixture changed: '+name)
    pose,visibility=select(scene,read(source/FILES[3])['tables'],nonce)
    start=[pose[k] for k in ('x','y','yaw')]
    metadata=dict(seed=world['seed'],original_start=world['start'],start=start,
        original_protocol_sha256=digest(official/'protocol.json'),generator_sha256=digest(Path(__file__)),
        fixture_role='diagnostic: closest physically separate obstacles; both partly visible at spawn',
        hypothesis='less repeated travel without a new readable surface near the pair, without sacrificing verified marks',
        far_hints=False,**visibility)
    with TemporaryDirectory(dir=destination.parent) as tmp:
        root=Path(tmp)/'stage'
        fixture=root/'fixtures'/source.name
        shutil.copytree(source,fixture)
        scene.update(robot_spawn=pose,robot_start_expected=pose)
        save(fixture/'scene.json',scene)
        shutil.copyfile(fixture/'scene.json',fixture/'run/simulation/scene.json')
        launch=read(fixture/'scene.yaml'); launch['start']=start
        save(fixture/'scene.yaml',launch)
        files={name:digest(fixture/name) for name in FILES}
        for name in ('world/osp_studio.sdf','run/simulation/demo_layout.json','run/simulation/demo_floor.png'):
            if files[name]!=world['files'][name]:raise ValueError('Changed original world or printed panels')
        save(root/'registration.json',dict(r,nonce=nonce,draws=[world['seed']],repeated_index=None,
            candidate=candidate_identity,candidate_label='Visual Survey · 겹친 장애물 진단',cluster_start=metadata))
        save(root/'protocol.json',dict(registration_sha256=digest(root/'registration.json'),
            worlds=[dict(world,start=start,files=files)],plan=[dict(pair=1,world=0,repeat=False,order=['baseline','candidate'])]))
        save(root/'start-comparison.json',metadata)
        root.rename(destination)
    return metadata


if __name__=='__main__':
    cli=argparse.ArgumentParser(description=__doc__)
    for name in ('official','separate_panel','visual_survey_enrollment'):
        cli.add_argument(name,type=Path)
    args=cli.parse_args()
    print(prepare(args.official,args.separate_panel,read(args.visual_survey_enrollment)['source']))
