"""Six deterministic matched Gazebo scenes. Geometry is visible only to the judge."""
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
from osp_mission.world import generate

# Map coordinates in metres, before centering. D has a far face behind the
# first box and a free path around it; actual SLAM discovery time is measured.
CASES = {
    'A': ([(3.4,1.7),(5.0,4.4)], ['south','east','north']),
    'B': ([(3.4,1.7),(5.0,4.4),(6.3,2.7)], [(0,'west'),(1,'north'),(2,'south')]),
    'C': ([(3.4,1.7),(5.0,4.4),(6.3,2.7)], ['south',(1,'east'),(0,'west')]),
    'D': ([(3.9,3.),(5.6,4.),(5.5,2.1)], ['north',(0,'east'),(2,'south')]),
    'E': ([(3.25,3.),(4.85,3.),(6.05,3.1)], ['east',(2,'north'),(1,'east')]),
    'F': ([(4.,.9),(5.4,4.5),(6.8,3.)], ['south',(0,'south'),(1,'west')]),
}
NORMAL={'east':(1,0),'north':(0,1),'west':(-1,0),'south':(0,-1)}
WALL={'east':(7.90,3.7,'west'),'north':(5.5,5.90,'south'),'south':(2.5,.10,'north')}
def prepare(root,case,seed):
    centers,placements=CASES[case]
    boxes=[dict(type='box',x=x,y=y,sx=.72,sy=.72,height=.9) for x,y in centers]
    targets=[]
    for ident,where in zip((7,32,103),placements):
        if isinstance(where,str):
            x,y,normal=WALL[where];owner='wall_'+where
        else:
            index,normal=where;b=boxes[index];dx,dy=NORMAL[normal]
            x,y=b['x']+dx*(b['sx']/2+.01),b['y']+dy*(b['sy']/2+.01)
            owner='obstacle'+str(index)
        nx,ny=NORMAL[normal]
        targets.append(dict(id=ident,x=x,y=y,yaw=math.atan2(ny,nx),size=.16,attached_to=owner))
    arena=dict(width_m=8.,height_m=6.,polygon=[[0,0],[8,0],[8,6],[0,6]],
               walls=[dict(x=x,y=y,sx=w,sy=h,height=1.4,yaw=0)
               for x,y,w,h in [(4,0,8.04,.12),(4,6,8.04,.12),(0,3,.12,6.04),(8,3,.12,6.04)]])
    out=root/f'map-{seed}';out.mkdir()
    source=out/'world_source.yaml'
    source.write_text(yaml.safe_dump(dict(seed=seed,randomize=False,start=[1.4,3.,0.],arena=arena,
                                           targets=targets,obstacles=boxes)))
    generate(source,out/'world')
    sdf=out/'world/osp_studio.sdf';tree=ET.parse(sdf);world=tree.find('world')
    world.find('physics/max_step_size').text='.004'
    for model in world.findall('model'):
        pose=model.find('pose')
        if pose is None:pose=ET.SubElement(model,'pose');pose.text='0 0 0 0 0 0'
        vals=list(map(float,pose.text.split()));vals[0]-=4;vals[1]-=3
        pose.text=' '.join(map(str,vals))
    tree.write(sdf,encoding='utf-8',xml_declaration=True)
    def translate(o):return dict(o,x=o['x']-4,y=o['y']-3)
    stage=dict(start=[-2.6,0.,0.],duration=180.,targets=list(map(translate,targets)))
    (out/'scene.yaml').write_text(yaml.safe_dump(stage))
    scene=dict(seed=seed,case=case,room_size=[8.,6.],partitions=[],obstacles=list(map(translate,boxes)),
               targets=stage['targets'],robot_spawn=dict(x=-2.6,y=0.,yaw=0.))
    (out/'scene.json').write_text(json.dumps(scene,indent=2)+'\n')
    return hashlib.sha256(sdf.read_bytes()).hexdigest()
