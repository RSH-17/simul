"""Evaluator-only randomized worlds. Only geometry QA may reject a draw."""
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import cv2
import numpy as np
from scipy.ndimage import distance_transform_edt, label
import yaml

from osp_mission.space_geometry import make_arena, rasterize_space, wall
from osp_mission.world import generate


FAMILIES = ('wall_l_room', 'boxes_trapezoid', 'mixed_occluded_u', 'narrow_door',
            'same_box_two_prints', 'curved_perimeter')
SHAPES = ('l_shape', 'trapezoid', 'u_shape', 'rectangle', 'rectangle', 'circle')


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def random_geometry(rng, family, seed):
    base = make_arena(rng, SHAPES[FAMILIES.index(family)])
    w, h = float(rng.uniform(8.8, 11.3)), float(rng.uniform(6., 8.))
    polygon = [[w*x/base['width_m'], h*y/base['height_m']] for x, y in base['polygon']]
    walls = [wall(a,b,boundary=True) for a,b in zip(polygon,polygon[1:]+polygon[:1])]
    boundary = len(walls)
    doors = []
    if family == 'narrow_door':
        x, y = w*float(rng.uniform(.44,.58)), h*float(rng.uniform(.32,.68))
        gap = float(rng.uniform(1.12,1.40))
        for low,high in ((0.,y-gap/2-.02),(y+gap/2+.02,h)):
            walls.append(wall((x,low),(x,high),boundary=False))
        doors.append({'x':x,'y':y,'opening_m':gap})
    arena = dict(width_m=w,height_m=h,polygon=polygon,walls=walls,doorways=doors)
    cfg = dict(seed=seed, randomize=False, arena=arena, obstacles=[], targets=[])
    resolution=.06
    # Only place solids where their entire circumscribed disk fits. No placement
    # may close the intended doorway, including its approach on either side.
    for i in range(5 if family in ('boxes_trapezoid','mixed_occluded_u') else 3):
        free=distance_transform_edt(~rasterize_space(cfg,resolution))*resolution
        sx,sy=map(float,rng.uniform(.65,1.15,2))
        rows,cols=np.where(free > .5*math.hypot(sx,sy)+.62)
        if doors:
            ok=(abs((cols+.5)*resolution-doors[0]['x'])>1.3)|(abs((rows+.5)*resolution-doors[0]['y'])>1.1)
            rows,cols=rows[ok],cols[ok]
        if not len(rows):return None
        j=rng.integers(len(rows)); angle=float(rng.uniform(-math.pi,math.pi))
        cfg['obstacles'].append(dict(type='box',x=float((cols[j]+.5)*resolution),
                 y=float((rows[j]+.5)*resolution),sx=sx,sy=sy,height=.85,yaw=angle))
    free=distance_transform_edt(~rasterize_space(cfg,resolution))*resolution
    rows,cols=np.where(free > .75)
    if not len(rows):return None
    j=rng.integers(len(rows))
    cfg['start']=[float((cols[j]+.5)*resolution),float((rows[j]+.5)*resolution),float(rng.uniform(-math.pi,math.pi))]
    ids=list(map(int,rng.permutation([7,32,103])))
    def add(x,y,yaw,host):
        cfg['targets'].append(dict(id=ids.pop(),size=float(rng.uniform(.14,.18)),
            x=float(x),y=float(y),yaw=float(yaw),attached_to=host))
    def on_box(index, angle, shift):
        host=cfg['obstacles'][index]; a=host['yaw']+angle
        stand=(host['sx'] if abs(math.cos(angle))>.5 else host['sy'])/2+.014
        add(host['x']+stand*math.cos(a)-shift*math.sin(a),
            host['y']+stand*math.sin(a)+shift*math.cos(a),a,'box_'+str(index))
    nw={'wall_l_room':3,'boxes_trapezoid':0,'curved_perimeter':2}.get(family,1)
    for i in rng.choice(boundary,nw,replace=False):
        host=walls[int(i)]
        yaw=host['yaw']+math.pi/2
        shift=float(rng.uniform(-.3,.3))*max(0.,host['length']-.25)
        add(host['x']+math.cos(yaw)*.084+math.cos(host['yaw'])*shift,
            host['y']+math.sin(yaw)*.084+math.sin(host['yaw'])*shift,yaw,'wall_'+str(i))
    indices=list(map(int,rng.permutation(len(cfg['obstacles']))))
    if family=='same_box_two_prints':
        a=int(rng.integers(4))*math.pi/2
        for shift in (-.145,.145):on_box(indices[0],a,shift)
    else:
        for i in indices[:3-nw]:on_box(i,int(rng.integers(4))*math.pi/2,0.)
    return cfg


def qa(cfg, family):
    res=.05
    occupied=rasterize_space(cfg,res)
    safe=distance_transform_edt(~occupied)*res >= .42
    labels,_=label(safe)
    sx,sy=cfg['start'][:2]
    region=labels[int(sy/res),int(sx/res)]
    if not region:return None
    ys,xs=np.where(labels==region)
    x=(xs+.5)*res;y=(ys+.5)*res
    if len(xs)<.96*safe.sum():return None
    views={};blocked_by_other=0
    for t in cfg['targets']:
        tx,ty=t['x'],t['y']
        vector=np.array([math.cos(t['yaw']),math.sin(t['yaw'])])
        if not 0<=tx<cfg['arena']['width_m'] or not 0<=ty<cfg['arena']['height_m']:return None
        host_kind,host_index=t['attached_to'].split('_'); host_index=int(host_index)
        shapes=cfg['arena']['walls'] if host_kind=='wall' else cfg['obstacles']
        obj=shapes[host_index]
        # Check location against the actual host plane and paper footprint.
        c,s=math.cos(obj['yaw']),math.sin(obj['yaw']);dx,dy=tx-obj['x'],ty-obj['y']
        u,v=c*dx+s*dy,-s*dx+c*dy
        nu,nv=c*vector[0]+s*vector[1],-s*vector[0]+c*vector[1]
        if abs(nu)>.99:plane=nu*u-obj['sx']/2; tangent=abs(v);width=obj['sy']
        elif abs(nv)>.99:plane=nv*v-obj['sy']/2;tangent=abs(u);width=obj['sx']
        else:return None
        if not (.008<plane<.025 and tangent+t['size']*.625<width/2):return None
        dx=x-tx;dy=y-ty;distance=np.hypot(dx,dy)
        ok=(distance>=.75)&(distance<=1.3)&(dx*vector[0]+dy*vector[1]>=distance*math.cos(math.radians(30)))
        # Exclude only the final 12 cm around the printed wall itself, to
        # accommodate the graphics support and conservative raster discretization.
        valid=0
        for i in np.flatnonzero(ok):
            f=np.linspace(.12/distance[i],1.,40)
            if not occupied[((ty+f*dy[i])/res).astype(int),((tx+f*dx[i])/res).astype(int)].any():valid+=1
        if not valid:return None
        views[t['id']]=valid
        if family=='mixed_occluded_u':
            other=dict(cfg,targets=[],arena=dict(cfg['arena']),obstacles=list(cfg['obstacles']))
            if host_kind=='wall':other['arena']['walls']=[w for j,w in enumerate(cfg['arena']['walls']) if j!=host_index]
            else:other['obstacles'].pop(host_index)
            occ=rasterize_space(other,res);f=np.linspace(.04,.98,180)
            blocked_by_other+=int(occ[((ty+(sy-ty)*f)/res).astype(int),((tx+(sx-tx)*f)/res).astype(int)].any())
    if family=='mixed_occluded_u' and not blocked_by_other:return None
    if family=='narrow_door':
        door=cfg['arena']['doorways'][0]
        if len({t['x']<door['x'] for t in cfg['targets']})<2:return None
    return dict(visible_connected_camera_cells=views,center_clearance_m=.42,grid_m=res,
                approach_range_m=[.75,1.3],max_incidence_deg=30,
                initial_rays_blocked_by_other_geometry=blocked_by_other if family=='mixed_occluded_u' else None)


def build(parent, seed, family):
    rng=np.random.default_rng(seed)
    for attempt in range(100):
        cfg=random_geometry(rng,family,seed)
        if cfg:
            result=qa(cfg,family)
            if result:break
    else:raise ValueError(f'No valid geometry for seed {seed}')
    dst=parent/f'map-{seed}';dst.mkdir(exist_ok=True)
    source=dst/'world_source.yaml';source.write_text(yaml.safe_dump(cfg))
    generate(source,dst/'world')
    sdf=dst/'world/osp_studio.sdf';tree=ET.parse(sdf);world=tree.find('world')
    world.find('physics/max_step_size').text='.004'
    w,h=cfg['arena']['width_m'],cfg['arena']['height_m']
    for model in world.findall('model'):
        pose=model.find('pose')
        if pose is None:pose=ET.SubElement(model,'pose');pose.text='0 0 0 0 0 0'
        values=list(map(float,pose.text.split()))
        values[0]-=w/2;values[1]-=h/2
        pose.text=' '.join(map(str,values))
    # Verify rendered, front-facing bit cells, including a white quiet margin.
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    for index,target in enumerate(cfg['targets']):
        model=world.find(f"model[@name='target{index}']")
        bits=np.full((9,9),255,np.uint8)
        actual=list(map(float,model.findtext('pose').split()))
        if not np.allclose([actual[0],actual[1],actual[5]],[target['x']-w/2,target['y']-h/2,target['yaw']]):
            raise ValueError('SDF marker pose mismatch')
        for v in model.findall('link/visual'):
            name=v.get('name')
            if name=='white':continue
            row,col=map(int,name.removeprefix('cell').split('_'))
            depth,y,z=map(float,v.findtext('pose').split()[:3])
            if depth<=0 or abs((col-3)*target['size']/7-y)>1e-6 or abs((3-row)*target['size']/7-z)>1e-6:
                raise ValueError('Invalid printed cell orientation')
            bits[1+row,1+col]=0
        _,ids,_=cv2.aruco.detectMarkers(np.kron(bits,np.ones((12,12),np.uint8)),dictionary)
        if ids is None or ids.ravel().tolist()!=[target['id']]:raise ValueError('Unreadable SDF marker')
    tree.write(sdf,encoding='utf-8',xml_declaration=True)
    def centered(item):return dict(item,x=item['x']-w/2,y=item['y']-h/2)
    targets=list(map(centered,cfg['targets']))
    start=[cfg['start'][0]-w/2,cfg['start'][1]-h/2,cfg['start'][2]]
    scene=dict(seed=seed,topology=family,room_size=[w,h],targets=targets,
               obstacles=list(map(centered,cfg['obstacles'])),partitions=list(map(centered,cfg['arena']['walls'])),
               robot_spawn=dict(zip(('x','y','yaw'),start)))
    (dst/'scene.json').write_text(json.dumps(scene,indent=2)+'\n')
    (dst/'scene.yaml').write_text(yaml.safe_dump(dict(start=start,duration=600.,targets=targets)))
    # Physics/render inputs are pinned; the SDF contains an absolute floor mesh URI.
    files={f:digest(dst/f) for f in ('world/osp_studio.sdf','world/arena_floor.obj','world_source.yaml','scene.yaml','scene.json')}
    return dict(seed=seed,topology=family,start=start,files=files,placement_draw=attempt+1,qa=result)
