"""Generate two matched random compact scenes; truth is evaluator-only."""
import argparse,json,math,random
from pathlib import Path
from xml.etree import ElementTree as ET
import yaml
from osp_mission.world import generate


def prepare(root,seed):
    root.mkdir(parents=True,exist_ok=False)
    rng=random.Random(seed)
    boxes=[dict(type='box',x=3.7+rng.uniform(-.2,.2),y=1.8+rng.uniform(-.2,.2),sx=.65,sy=.65,height=.9),
           dict(type='box',x=5.1+rng.uniform(-.2,.2),y=4.1+rng.uniform(-.2,.2),sx=.75,sy=.7,height=.9)]
    targets=[dict(id=7,x=7.90,y=3.3+rng.uniform(-.25,.25),yaw=math.pi,size=.16,attached_to='wall_east'),
             dict(id=103,x=boxes[0]['x']-.335,y=boxes[0]['y'],yaw=math.pi,size=.14,attached_to='obstacle0'),
             dict(id=32,x=boxes[1]['x']-.385,y=boxes[1]['y'],yaw=math.pi,size=.18,attached_to='obstacle1')]
    cfg=dict(seed=seed,randomize=False,start=[1.4,3.,0.],arena=dict(width_m=8.,height_m=6.,
        polygon=[[0,0],[8,0],[8,6],[0,6]],walls=[
            dict(x=4,y=0,sx=8.04,sy=.12,height=1.4,yaw=0),dict(x=4,y=6,sx=8.04,sy=.12,height=1.4,yaw=0),
            dict(x=0,y=3,sx=.12,sy=6.04,height=1.4,yaw=0),dict(x=8,y=3,sx=.12,sy=6.04,height=1.4,yaw=0)]),
        obstacles=boxes,targets=targets)
    source=root/'world_source.yaml';source.write_text(yaml.safe_dump(cfg))
    generate(source,root/'world')
    path=root/'world/osp_studio.sdf';tree=ET.parse(path);world=tree.find('world')
    world.find('physics/max_step_size').text='.004'
    for model in world.findall('model'):
        pose=model.find('pose')
        if pose is None:pose=ET.SubElement(model,'pose');pose.text='0 0 0 0 0 0'
        values=list(map(float,pose.text.split()));values[0]-=4.;values[1]-=3.;pose.text=' '.join(map(str,values))
    tree.write(path,encoding='utf-8',xml_declaration=True)
    targets=[dict(t,x=t['x']-4,y=t['y']-3) for t in targets]
    scene=dict(start=[-2.6,0.,0.],duration=90.,targets=targets)
    (root/'scene.yaml').write_text(yaml.safe_dump(scene))
    (root/'scene.json').write_text(json.dumps(dict(seed=seed,room_size=[8.,6.],partitions=[],
        obstacles=[dict(b,x=b['x']-4,y=b['y']-3) for b in boxes],targets=targets,
        robot_spawn=dict(x=-2.6,y=0.,yaw=0.)),indent=2)+'\n')
    return root


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('root',type=Path);args=parser.parse_args()
    for seed in (112021,112022):prepare(args.root/f'map-{seed}',seed)
