"""Build a new layout in the actual Gazebo exam room using the captured models."""
import argparse
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import cv2
import numpy as np

from geometry import EXAM_BOX_COUNT, sample
from surface_audit import aim_at, blockers, readable_indices


ORIGINAL_SDF = '589d152dcfef420f6fda37fccdc30c3d73e9f90ebbdab8359d04bf94a5538b1a'
PORTABLE_SDF = '564be4776da7c708f0b16453c84bafcb7b71d77807bdfa89ec0f49f37ed585f6'
FIXED_TABLE_ROOM = 'beff18e9879eaa96cc036e4967575987282c1d951f6d9488b145e4d59f080c6f'
FILES = ('world/osp_studio.sdf','scene.json','scene.yaml','run/simulation/demo_layout.json','run/simulation/demo_floor.png')
HERE = Path(__file__).resolve().parent
CALIBRATION = HERE.parent/'2026-11-21_v151_track5_boxes5/camera_geometry.json'
VIEW_SOURCES = (HERE/'surface_audit.py', HERE.parent/'2026-11-22_v151_adaptive_survey/development/exposure_opportunity.py', CALIBRATION)
INK_X = -.004


def read(path):
    return json.loads(path.read_text())


def save(path, value):
    temp=path.with_name(path.name+'.new')
    temp.write_text(json.dumps(value,indent=2,allow_nan=False,ensure_ascii=False)+'\n')
    temp.replace(path)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def verify_boxes(scene, world, template):
    bodies=scene['obstacles']
    dimensions={(b['sx'],b['sy'],b['height']) for b in template['obstacles']}
    if (scene.get('randomization',{}).get('obstacles')!='exam_boxes' or len(bodies)!=EXAM_BOX_COUNT or
            len({b['object_id'] for b in bodies})!=EXAM_BOX_COUNT):
        raise ValueError('Expected five independent exam boxes')
    models={m.get('name'):m for m in world.findall('world/model') if m.get('name','').startswith('movable_')}
    if set(models)!={b['name'] for b in bodies}:
        raise ValueError('Missing or additional movable Gazebo models')
    for b in bodies:
        expected=(b['sx'],b['sy'],b['height'])
        if b['shape']!='box' or b['name']!=b['object_id'] or expected not in dimensions:
            raise ValueError('The movable obstacle differs from the original exam boxes')
        model=models[b['name']]
        pose=[float(x) for x in model.findtext('pose').split()]
        if not np.allclose(pose,[b['x'],b['y'],b['height']/2,0,0,b['yaw']],rtol=0,atol=1e-8):
            raise ValueError('Gazebo box pose differs from the recorded scene')
        for tag in ('collision','visual'):
            pieces=model.findall('.//'+tag)
            size=pieces[0].findtext('geometry/box/size','') if len(pieces)==1 else ''
            if not size or tuple(map(float,size.split()))!=expected:
                raise ValueError('Gazebo box '+tag+' geometry differs from the original dimensions')


def architecture(world, scene, include_tables):
    tree=deepcopy(world)
    w=tree.find('world')
    names={b['name'] for b in scene['obstacles']} | {f"target_{t['id']}" for t in scene['targets']}
    if not include_tables:names.add('demo_furniture')
    for m in list(w.findall('model')):
        if m.get('name') in names:
            names.remove(m.get('name'));w.remove(m)
    if names:raise ValueError(f'Absent Gazebo models {names}')
    textures=list(tree.iter('albedo_map'))
    if len(textures)!=1:raise ValueError('Unexpected painted floor')
    textures[0].text='demo_floor.png'
    data=ET.canonicalize(ET.tostring(tree,encoding='unicode'),strip_text=True)
    return hashlib.sha256(data.encode()).hexdigest()


def value(x):
    return ' '.join(format(float(v),'.15g') for v in x)


def original_dictionary():
    return cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)


def symbol(dictionary, ident):
    size=dictionary.markerSize+2
    draw=getattr(cv2.aruco,'generateImageMarker',None) or cv2.aruco.drawMarker
    return draw(dictionary,int(ident),size)==0


def marker_model(source, target, dictionary):
    marker=deepcopy(source)
    marker.set('name',f"target_{target['id']}")
    marker.find('pose').text=value([target['x'],target['y'],0,0,0,target['yaw']])
    paper=marker.find("model[@name='paper']")
    cells=[m for m in marker.findall('model') if m is not paper]
    if not cells:raise ValueError('Missing physical black cells')
    template=cells[0]
    for m in cells:marker.remove(m)
    size,z=target['size'],target['z']
    paper.find('pose').text=value([-.002,0,z,0,0,0])
    paper.find('.//box/size').text=value([.002,1.3*size,1.3*size])
    bits=symbol(dictionary,target['id']);pitch=size/len(bits)
    for row,col in zip(*np.nonzero(bits)):
        model=deepcopy(template);model.set('name',f'cell_{row}_{col}')
        model.find('pose').text=value([INK_X,size/2-(col+.5)*pitch,z+size/2-(row+.5)*pitch,0,0,0])
        model.find('.//box/size').text=value([.002,pitch,pitch])
        marker.append(model)
    return marker


def update_tables(furniture, before, after):
    # Table instances in the original SDF share one link. Transform all four
    # legs and the top in the collision and visual records together.
    for old,new in zip(before,after):
        name=old['name']
        if new['name']!=name:raise ValueError('Changed table identity')
        angle=new['yaw']-old['yaw'];c,s=math.cos(angle),math.sin(angle)
        pieces=[e for e in furniture.find('link') if e.get('name','').startswith(name+'_')]
        if len(pieces)!=10:raise ValueError('Missing physical table geometry: '+name)
        for piece in pieces:
            p=piece.find('pose');x,y,z,r,pitch,yaw=map(float,p.text.split());dx=x-old['x'];dy=y-old['y']
            p.text=value([new['x']+c*dx-s*dy,new['y']+s*dx+c*dy,z,r,pitch,yaw+angle])


def verify_prints(scene, world, dictionary):
    w=world.find('world')
    for target in scene['targets']:
        model=w.find(f"model[@name='target_{target['id']}']")
        bits=symbol(dictionary,target['id']); rows=len(bits)
        rendered=np.zeros_like(bits)
        if model is None:raise ValueError('Missing printed marker')
        pose=list(map(float,model.findtext('pose').split()))
        if math.dist(pose[:2],(target['x'],target['y']))>1e-7 or abs(pose[-1]-target['yaw'])>1e-7:
            raise ValueError('Printed marker pose differs from evaluation')
        for part in model.findall('model'):
            if part.get('name')=='paper':continue
            x,y,z,*_=map(float,part.findtext('pose').split())
            if x>=-.002:raise ValueError('Black ink is behind the paper')
            col=round((.5-y/target['size'])*rows-.5)
            row=round((.5-(z-target['z'])/target['size'])*rows-.5)
            if not (0<=col<rows and 0<=row<rows) or rendered[row,col]:raise ValueError('Misplaced print cell')
            rendered[row,col]=True
        if not np.array_equal(bits,rendered):raise ValueError('Physical pattern differs from dictionary ID')


def rendered_view_check(world):
    calibration=read(CALIBRATION)
    solids=blockers(world,-math.inf,math.inf)

    def visible(view,point,inward,size,height):
        direction=np.array([*inward,0.])
        center=np.array([*point,height])+INK_X*direction
        normal=-direction
        tangent=np.array([-normal[1],normal[0],0.])
        origins,axes=aim_at(view,center,calibration)
        indices,_=readable_indices(center,normal,tangent,size,18.,origins,axes,calibration,solids)
        return bool(len(indices))

    return visible


def build(original, out, seed, marker_count=3, furniture='fixed', host_stress=None, obstacle_set='boxes'):
    original=original.resolve();out=out.resolve()
    source_sdf=original/FILES[0]
    source_digest=digest(source_sdf)
    if source_digest not in (ORIGINAL_SDF,PORTABLE_SDF):raise ValueError('Expected the photographed Gazebo exam room')
    out.mkdir(parents=True,exist_ok=False)
    raw=read(original/'scene.json');layout=read(original/FILES[3])
    if source_digest==PORTABLE_SDF:
        # The verified team archive has the same shell and furniture. Match the
        # source room's draw order so a shared seed produces identical prints.
        by_size={target['size']:target for target in raw['targets']}
        raw['targets']=[by_size[size] for size in (.30,.20,.25)]
    dictionary=original_dictionary()
    root=ET.parse(source_sdf).getroot();w=root.find('world')
    shell=architecture(root,raw,False);fixed=architecture(root,raw,True)
    if fixed!=FIXED_TABLE_ROOM:raise ValueError('The original exam architecture or furniture changed')
    by_name={m.get('name'):m for m in w.findall('model')}
    body_template=by_name[raw['obstacles'][0]['name']]
    marker_template=by_name[f"target_{raw['targets'][0]['id']}"]
    for b in raw['obstacles']:w.remove(by_name[b['name']])
    for t in raw['targets']:w.remove(by_name[f"target_{t['id']}"])
    def prepare_room(scene,tables):
        if furniture=='shuffle':update_tables(by_name['demo_furniture'],layout['tables'],tables)
        for b in scene['obstacles']:
            m=deepcopy(body_template);m.set('name',b['name'])
            m.find('pose').text=value([b['x'],b['y'],b['height']/2,0,0,b['yaw']])
            for dim in m.findall('.//box/size'):dim.text=value([b['sx'],b['sy'],b['height']])
            w.append(m)
        return rendered_view_check(w)

    scene,tables,witnesses=sample(raw,layout['tables'],seed,len(dictionary.bytesList),marker_count,
                                  furniture,host_stress,prepare_room,obstacle_set)
    for t in scene['targets']:w.append(marker_model(marker_template,t,dictionary))
    sim=out/'run/simulation';sim.mkdir(parents=True)
    floor=sim/'demo_floor.png'
    shutil.copyfile(original/FILES[-1],floor)
    if (original/'run/simulation/board-floor.jpg').exists():
        shutil.copyfile(original/'run/simulation/board-floor.jpg',sim/'board-floor.jpg')
    if furniture=='fixed':shutil.copyfile(original/FILES[3],sim/'demo_layout.json')
    else:save(sim/'demo_layout.json',dict(layout,tables=tables))
    root.find('.//albedo_map').text=('@TEAM_MAP_ROOT@/run/simulation/demo_floor.png'
                                   if source_digest==PORTABLE_SDF else floor.as_uri())
    (out/'world').mkdir();ET.ElementTree(root).write(out/FILES[0],encoding='utf-8',xml_declaration=True)
    save(out/'scene.json',scene);shutil.copyfile(out/'scene.json',sim/'scene.json')
    pose=scene['robot_spawn']
    # JSON is a YAML subset, including for the existing Jazzy launch and recorder.
    save(out/'scene.yaml',dict(start=[pose[k] for k in ('x','y','yaw')],duration=600,targets=scene['targets']))
    actual=ET.parse(out/FILES[0]).getroot()
    verify_prints(scene,actual,dictionary)
    if obstacle_set=='boxes':verify_boxes(scene,actual,raw)
    if shell!=architecture(actual,scene,False) or (furniture=='fixed' and fixed!=architecture(actual,scene,True)):
        raise ValueError('Architectural shell, original camera-room appearance or physics changed')
    files={n:digest(out/n) for n in FILES}
    objects={b['object_id']:b['shape'] for b in scene['obstacles']}
    return dict(seed=int(seed),movable_objects=len(objects),object_shapes=objects,tables=furniture,
                marker_count=marker_count,host_stress=host_stress,start=[pose[k] for k in ('x','y','yaw')],
                files=files,fixed_shell_sha256=shell,
                hosts=[t['attached_to'] for t in scene['targets']],ids=[t['id'] for t in scene['targets']],
                geometric_witnesses_m=witnesses)


if __name__=='__main__':
    cli=argparse.ArgumentParser(description=__doc__)
    cli.add_argument('original',type=Path);cli.add_argument('output',type=Path);cli.add_argument('seed',type=int)
    cli.add_argument('--markers',type=int,default=3);cli.add_argument('--tables',choices=('fixed','shuffle'),default='fixed')
    args=cli.parse_args()
    print(json.dumps(build(args.original,args.output,args.seed,args.markers,args.tables),indent=2))
