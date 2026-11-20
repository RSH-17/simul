"""Freeze two development fixtures and three distinct unseen marker-face layouts."""
import argparse
import json
import math
from pathlib import Path
import random
import shutil
import sys
from xml.etree import ElementTree as ET
import yaml
from osp_mission.world import generate


def prepare(root,development=(112021,112022),held_out=(112101,112102,112103),distractor=False):
    root.mkdir(parents=True,exist_ok=False)
    for seed in development:
        source=Path.home()/f'camera-v12-expanded-1120/map-{seed}'
        if not source.exists():source=Path.home()/f'camera-aware-policy-1121/map-{seed}'
        if not source.exists():source=Path.home()/f'camera-refine-policy-1121/map-{seed}'
        shutil.copytree(source,root/f'map-{seed}')
    for index,seed in enumerate(held_out):
        rng=random.Random(seed);folder=root/f'map-{seed}';folder.mkdir()
        spread=.5 if distractor else .3
        boxes=[dict(type='box',x=3.4+rng.uniform(-spread,spread),y=1.7+rng.uniform(-spread,spread),sx=.7,sy=.7,height=.9),
               dict(type='box',x=5.3+rng.uniform(-spread,spread),y=4.1+rng.uniform(-spread,spread),sx=.8,sy=.7,height=.9)]
        faces=[('east','south'),('west','east'),('north','south')][index]
        targets=[]
        for ident,box,face,size in zip((103,32),boxes,faces,(.14,.18)):
            dx,dy,angle={'east':(box['sx']/2+.01,0.,0.),'west':(-box['sx']/2-.01,0.,math.pi),
                'north':(0.,box['sy']/2+.01,math.pi/2),'south':(0.,-box['sy']/2-.01,-math.pi/2)}[face]
            targets.append(dict(id=ident,x=box['x']+dx,y=box['y']+dy,yaw=angle,size=size,attached_to=f'obstacle{len(targets)}'))
        wall=[(6.4,5.90,-math.pi/2,'north'),(6.1,.10,math.pi/2,'south'),(.10,4.5,0.,'west')][index]
        if distractor:
            jitter=rng.uniform(-.8,.8)
            wall=(wall[0]+jitter,wall[1],wall[2],wall[3]) if index<2 else (wall[0],wall[1]+jitter,wall[2],wall[3])
        targets.append(dict(id=7,x=wall[0],y=wall[1],yaw=wall[2],size=.16,attached_to='wall_'+wall[3]))
        obstacles=boxes+([dict(type='cylinder',x=6.6,y=2.6,radius=.2,height=.7)] if distractor else [])
        cfg=dict(seed=seed,randomize=False,start=[1.4,3.,0.],targets=targets,obstacles=obstacles,
            arena=dict(width_m=8.,height_m=6.,polygon=[[0,0],[8,0],[8,6],[0,6]],walls=[
                dict(x=4,y=0,sx=8.04,sy=.12,height=1.4,yaw=0),dict(x=4,y=6,sx=8.04,sy=.12,height=1.4,yaw=0),
                dict(x=0,y=3,sx=.12,sy=6.04,height=1.4,yaw=0),dict(x=8,y=3,sx=.12,sy=6.04,height=1.4,yaw=0)]))
        path=folder/'world_source.yaml';path.write_text(yaml.safe_dump(cfg));generate(path,folder/'world')
        path=folder/'world/osp_studio.sdf';tree=ET.parse(path);world=tree.find('world')
        world.find('physics/max_step_size').text='.004'
        for model in world.findall('model'):
            pose=model.find('pose')
            if pose is None:pose=ET.SubElement(model,'pose');pose.text='0 0 0 0 0 0'
            values=list(map(float,pose.text.split()));values[0]-=4;values[1]-=3;pose.text=' '.join(map(str,values))
        tree.write(path,encoding='utf-8',xml_declaration=True)
        translated=[dict(t,x=t['x']-4,y=t['y']-3) for t in targets]
        (folder/'scene.yaml').write_text(yaml.safe_dump(dict(start=[-2.6,0.,0.],duration=300.,targets=translated)))
        (folder/'scene.json').write_text(json.dumps(dict(seed=seed,room_size=[8.,6.],partitions=[],
            obstacles=[dict(b,x=b['x']-4,y=b['y']-3) for b in obstacles],targets=translated,
            robot_spawn=dict(x=-2.6,y=0.,yaw=0.)),indent=2)+'\n')
    (root/'split.json').write_text(json.dumps(dict(development=list(development),held_out=list(held_out),
        untagged_distractor=distractor,
        held_out_used_for_selection=False,criteria='strict sensor proof + independent distance + map accuracy + zero bumps'),indent=2)+'\n')


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('root',type=Path)
    parser.add_argument('--refinement',action='store_true');args=parser.parse_args()
    if args.refinement:
        prepare(args.root,development=(112021,112101),held_out=(112201,112202,112203),distractor=True)
    else:prepare(args.root)
