"""Run unchanged v1.5.1 with five additional, untagged random obstacles."""
import fcntl
import json
import math
import os
from pathlib import Path
import random
import sys
import time
from xml.etree import ElementTree as ET
import yaml
from osp_mission.world import generate

BASE=Path.home()/'camera-aware-tools/experiments/2026-11-21_camera_aware_policy'
sys.path.insert(0,str(BASE))
import campaign


def extents(b):
    return (b['sx']/2,b['sy']/2) if b['type']=='box' else (b['radius'],b['radius'])


def main(seed=112401):
    locks=[open(Path.home()/'track-recordings'/name,'a') for name in ('.domain.lock','.second-domain.lock')]
    for lock in locks:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    stamp=str(time.time_ns());root=Path.home()/('camera-clutter-'+stamp);root.mkdir()
    fixture=root/f'map-{seed}';fixture.mkdir()
    cfg=yaml.safe_load((Path.home()/'camera-final-policy-1121/map-112301/world_source.yaml').read_text())
    rng=random.Random(seed);original=len(cfg['obstacles']);protected=[cfg['start'][:2]]
    protected += [(t['x']+1.1*math.cos(t['yaw']),t['y']+1.1*math.sin(t['yaw'])) for t in cfg['targets']]
    for attempt in range(10000):
        if len(cfg['obstacles'])==original+5:break
        index=len(cfg['obstacles'])-original
        b=dict(type='box',sx=rng.uniform(.38,.58),sy=rng.uniform(.38,.58),height=.75) if index%2==0 else dict(type='cylinder',radius=rng.uniform(.18,.24),height=.8)
        b.update(x=rng.uniform(.75,7.25),y=rng.uniform(.75,5.25))
        ex,ey=extents(b)
        if any(abs(b['x']-x)<ex+.8 and abs(b['y']-y)<ey+.8 for x,y in protected):continue
        if any(abs(b['x']-old['x'])<ex+extents(old)[0]+.65 and abs(b['y']-old['y'])<ey+extents(old)[1]+.65 for old in cfg['obstacles']):continue
        cfg['obstacles'].append(b)
    assert len(cfg['obstacles'])==original+5,'Could not place five separated obstacles'
    cfg['seed']=seed;source=fixture/'world_source.yaml';source.write_text(yaml.safe_dump(cfg))
    generate(source,fixture/'world')
    sdf=fixture/'world/osp_studio.sdf';tree=ET.parse(sdf);world=tree.find('world');world.find('physics/max_step_size').text='.004'
    for model in world.findall('model'):
        pose=model.find('pose')
        if pose is None:pose=ET.SubElement(model,'pose');pose.text='0 0 0 0 0 0'
        values=list(map(float,pose.text.split()));values[0]-=4;values[1]-=3;pose.text=' '.join(map(str,values))
    tree.write(sdf,encoding='utf-8',xml_declaration=True)
    targets=[dict(t,x=t['x']-4,y=t['y']-3) for t in cfg['targets']]
    (fixture/'scene.yaml').write_text(yaml.safe_dump(dict(start=[-2.6,0.,0.],duration=600.,targets=targets)))
    (fixture/'scene.json').write_text(json.dumps(dict(seed=seed,room_size=[8,6],partitions=[],
        obstacles=[dict(b,x=b['x']-4,y=b['y']-3) for b in cfg['obstacles']],targets=targets,robot_spawn=dict(x=-2.6,y=0,yaw=0)),indent=2))
    plan=dict(version='v1.5.1-balanced',seed=seed,extra_obstacles=5,total_obstacles=len(cfg['obstacles']),
        source_map=112301,camera='480x480@5',domain=174,physical_drive=False,root=str(root),
        note='Geometry spacing is not a Nav2 reachability proof; policy and collision margins unchanged')
    (root/'scenario.json').write_text(json.dumps(plan,indent=2)+'\n');print(json.dumps(plan),flush=True)
    campaign.WORKSPACES['sync']=Path.home()/'flow_v151_exposure_sync_r2_ws'
    os.environ.pop('CAMERA_BENCH_TRIAL_SCRIPT',None)
    run=campaign.execute(root,'sync','balanced',seed,'clutter-extra5-'+stamp,600)
    (root/'runs.json').write_text(json.dumps([str(run)])+'\n')
    (root/'complete.json').write_text(json.dumps(dict(completed=True,run=str(run),physical_drive=False))+'\n')


if __name__=='__main__':main(int(sys.argv[1]) if len(sys.argv)>1 else 112401)
