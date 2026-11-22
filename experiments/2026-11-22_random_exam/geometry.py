"""Sample only simulator fixtures. Nothing in this module is imported by a policy."""
from copy import deepcopy
import math

import numpy as np
from scipy import ndimage


RESOLUTION = .05
EXAM_BOX_COUNT = 5


def polygon(body, margin=0.):
    # A margin expands both half-extents, in metres.
    a, b = body['sx'] / 2 + margin, body['sy'] / 2 + margin
    c, s = math.cos(body['yaw']), math.sin(body['yaw'])
    return np.array([[-a,-b],[a,-b],[a,b],[-a,b]]) @ [[c,s],[-s,c]] + [body['x'],body['y']]


def intersects(first, second):
    """Separating axes of the two physical oriented rectangles."""
    for corners in (first, second):
        for axis in (corners[1]-corners[0], corners[3]-corners[0]):
            a,b = first @ axis, second @ axis
            if a.max() < b.min() or b.max() < a.min():
                return False
    return True


def wall_segments(scene):
    half_x,half_y=np.asarray(scene['room_size'])/2.
    recess=scene['south_recess']; left,right=recess['x_min'],recess['x_max']
    back=-half_y-recess['depth']
    pts=[(-half_x,-half_y),(left,-half_y),(left,back),(right,back),(right,-half_y),
         (half_x,-half_y),(half_x,half_y),(-half_x,half_y),(-half_x,-half_y)]
    names=['south_west','south_recess_west','south_recess','south_recess_east',
           'south_east','east','north','west']
    segments=[(n,np.array(a),np.array(b)) for n,a,b in zip(names,pts,pts[1:])]
    # The step along the east wall and protruding columns have real camera-
    # facing surfaces. The clockwise winding puts their solid on the right.
    for body in scene['partitions']:
        corners=polygon(body)[::-1]
        for i in range(len(corners)):
            segments.append((f"structure_{body['name']}_{i}",corners[i],corners[(i+1)%len(corners)]))
    return segments


def clear_of_shell(points, scene):
    half_x,half_y=np.asarray(scene['room_size'])/2.
    recess=scene['south_recess']
    x,y=points.T
    bottom=np.where((x>recess['x_min']+.07)&(x<recess['x_max']-.07),-half_y-recess['depth'],-half_y)
    return bool(np.all((abs(x)<half_x-.07)&(y<half_y-.07)&(y>bottom+.07)))


def draw_pose(body, rng, scene, footprints):
    half_x,half_y=np.asarray(scene['room_size'])/2.
    for _ in range(600):
        candidate=dict(body,x=float(rng.uniform(-half_x,half_x)),y=float(rng.uniform(-half_y,half_y)),
                       yaw=float(rng.uniform(-math.pi,math.pi)))
        poly=polygon(candidate,.06)
        if clear_of_shell(poly,scene) and all(not intersects(poly,f) for f in footprints):
            return candidate
    raise ValueError('No nonintersecting pose for a movable body')


def spread_box(template, rng, scene, footprints, sector):
    previous = [(b['x'],b['y']) for b in scene['obstacles']]
    minimum = .48*min(scene['room_size'])
    for _ in range(600):
        box = draw_pose(template,rng,scene,footprints)
        here = (box['x'],box['y'])
        quadrant = int(here[0]>=0)+2*int(here[1]>=0)
        if quadrant==sector and all(math.dist(here,xy)>=minimum for xy in previous):
            return box
    raise ValueError('No separated exam box in the assigned sector')


def draw_obstacle(index, rng, scene, footprints):
    host=f'movable_{index+1}'
    kind=str(rng.choice(('box','box','box','screen','elbow')))
    if kind=='box':
        sx,sy=rng.uniform(.46,1.25),rng.uniform(.46,1.1)
    elif kind=='screen':
        sx,sy=rng.uniform(.85,2.1),rng.uniform(.16,.32)
    else:
        sx,sy=rng.uniform(.95,1.75),rng.uniform(.95,1.55)
    frame=draw_pose(dict(name=host,object_id=host,shape=kind,sx=float(sx),sy=float(sy),
                         height=float(rng.uniform(.55,.95))),rng,scene,footprints)
    if kind!='elbow':
        return [frame]
    # Two physical prisms make the L. The two arms share a single object ID.
    thickness=float(rng.uniform(.22,.36))
    c,s=math.cos(frame['yaw']),math.sin(frame['yaw'])
    pieces=[]
    for suffix,lx,ly,width,depth in (('a',0.,(thickness-sy)/2,sx,thickness),
                                     ('b',(thickness-sx)/2,thickness/2,thickness,sy-thickness)):
        pieces.append(dict(frame,name=host+'_'+suffix,x=frame['x']+c*lx-s*ly,
                           y=frame['y']+s*lx+c*ly,sx=float(width),sy=float(depth)))
    return pieces


def raster(scene, parts, for_spawn=False):
    # At wheel and camera height the original tables have four thin legs. Do
    # not fill a whole tabletop for visibility at marker height.
    hx,hy=np.asarray(scene['room_size'])/2.; recess=scene['south_recess']
    xmin,ymin=-hx,-hy-recess['depth']
    xs=np.arange(xmin+RESOLUTION/2,hx,RESOLUTION); ys=np.arange(ymin+RESOLUTION/2,hy,RESOLUTION)
    x,y=np.meshgrid(xs,ys)
    bottom=np.where((x>recess['x_min']+.05)&(x<recess['x_max']-.05),ymin,-hy)
    occ=(np.abs(x)>=hx-.05)|(y>=hy-.05)|(y<=bottom+.05)
    for body in [*scene['partitions'],*scene['obstacles'],*parts]:
        if not for_spawn and body.get('z',0)-body.get('height',0)/2 > .40:
            continue
        c,s=math.cos(body['yaw']),math.sin(body['yaw'])
        dx,dy=x-body['x'],y-body['y']
        # Include half a cell for thin table legs so an unlucky grid alignment
        # never makes them disappear.
        occ |= ((abs(c*dx+s*dy)<=body['sx']/2+.5*RESOLUTION)&
                (abs(-s*dx+c*dy)<=body['sy']/2+.5*RESOLUTION))
    return occ,np.array([xmin,ymin])


def table_parts(tables, config):
    parts=[]
    for t in tables:
        z=config['tabletop_height']; angle=t['yaw']; c,s=math.cos(angle),math.sin(angle)
        parts.append(dict(t,z=z-.025,height=.05))
        for lx in (-t['sx']/2+.09,t['sx']/2-.09):
            for ly in (-t['sy']/2+.08,t['sy']/2-.08):
                parts.append(dict(x=t['x']+c*lx-s*ly,y=t['y']+s*lx+c*ly,
                                  sx=.055,sy=.055,yaw=angle,z=.35,height=.70))
    return parts


class Access:
    def __init__(self, scene, tables, rng):
        parts=table_parts(tables,scene['furniture'])
        self.occ,self.origin=raster(scene,parts)
        spawn_occ,_=raster(scene,parts,for_spawn=True)
        clearance=ndimage.distance_transform_edt(np.pad(~self.occ,1))[1:-1,1:-1]*RESOLUTION
        spawn_clear=ndimage.distance_transform_edt(np.pad(~spawn_occ,1))[1:-1,1:-1]*RESOLUTION
        spawn=np.argwhere(spawn_clear>=.80)
        if not len(spawn):raise ValueError('No legal start in the exam room')
        row,col=spawn[rng.integers(len(spawn))]
        labels,_=ndimage.label(clearance>=.40)
        self.connected=labels==labels[row,col]
        xy=self.origin+(np.array([col,row])+.5)*RESOLUTION
        self.start=dict(x=float(xy[0]),y=float(xy[1]),yaw=float(rng.uniform(-math.pi,math.pi)))

    def cells(self, points):
        return np.floor((np.asarray(points)-self.origin)/RESOLUTION).astype(int)

    def clear(self, points, grid):
        cells=self.cells(points); x,y=cells.T
        inside=(x>=0)&(x<grid.shape[1])&(y>=0)&(y<grid.shape[0])
        return bool(inside.all() and grid[y,x].all())

    def witness(self, point, inward, size, z=None, visible=None):
        # Connected 2D geometric examples, never Nav2 paths or policy inputs.
        tangent=np.array([-inward[1],inward[0]])
        for distance in (.95,1.25,.75):
            for lateral in (0.,-.32,.32):
                view=point-distance*inward+lateral*tangent
                if not self.clear([view],self.connected):continue
                ends=[point-.10*inward+offset*tangent for offset in (-size*.65,0,size*.65)]
                rays=np.concatenate([np.linspace(view,end,70) for end in ends])
                if self.clear(rays,~self.occ) and (visible is None or visible(view,point,inward,size,z)):
                    return [float(v) for v in view]
        return None


def mount(rng, scene, size, kind):
    if kind=='wall':
        segments=wall_segments(scene)
        lengths=np.array([np.linalg.norm(b-a) for _,a,b in segments])
        name,a,b=segments[rng.choice(len(segments),p=lengths/lengths.sum())]
        span=np.linalg.norm(b-a); margin=size*.65+.08
        if span<=2*margin:return None
        tangent=(b-a)/span
        inward=np.array([tangent[1],-tangent[0]])  # points into the structure, away from the print
        stand_off=.002 if name.startswith('structure_') else .075
        point=a+tangent*rng.uniform(margin,span-margin)-stand_off*inward
        return point,inward,'wall_'+name,None
    body=scene['obstacles'][rng.integers(len(scene['obstacles']))]
    side=int(rng.integers(4)); yaw=body['yaw']+side*math.pi/2
    inward=np.array([math.cos(yaw),math.sin(yaw)]);tangent=np.array([-inward[1],inward[0]])
    depth,width=(body['sx'],body['sy']) if side%2==0 else (body['sy'],body['sx'])
    allowance=width/2-size*.65-.02
    if allowance<0:return None
    point=np.array([body['x'],body['y']])-inward*depth/2+tangent*rng.uniform(-allowance,allowance)
    return point,inward,body['name'],body['height']


def sample(template, tables, seed, dictionary_size, marker_count=3, furniture='fixed', host_stress=None,
           prepare_room=None, obstacle_set='boxes'):
    if (not 1<=marker_count<=dictionary_size or furniture not in ('fixed','shuffle') or
            host_stress not in (None,'wall','object') or obstacle_set not in ('boxes','legacy_mixed','spread_three')):
        raise ValueError('Invalid exam sampling parameters')
    spread=obstacle_set=='spread_three'
    if spread and (furniture!='fixed' or marker_count!=3 or host_stress is not None):
        raise ValueError('The spread exam requires fixed desks, two box prints and one outer-wall print')
    rng=np.random.default_rng(seed)
    scene=deepcopy(template)
    scene['seed']=int(seed); scene['targets']=[];scene['obstacles']=[]
    static=[polygon(b) for b in scene['partitions']]
    tables=deepcopy(tables)
    if furniture=='shuffle':
        for i,old in enumerate(tables):
            tables[i]=draw_pose(old,rng,scene,static)
            static.append(polygon(tables[i]))
    else:static.extend(polygon(t) for t in tables)
    count=3 if spread else EXAM_BOX_COUNT if obstacle_set=='boxes' else int(rng.integers(1 if host_stress=='object' else 0,10))
    sectors=rng.choice(4,3,replace=False) if spread else ()
    box_dimensions=sorted({(b['sx'],b['sy'],b['height']) for b in template['obstacles']})
    for i in range(count):
        if obstacle_set!='legacy_mixed':
            sx,sy,height=box_dimensions[rng.integers(len(box_dimensions))]
            name=f'movable_{i+1}'
            box=dict(name=name,object_id=name,shape='box',sx=sx,sy=sy,height=height)
            bodies=[spread_box(box,rng,scene,static,sectors[i]) if spread else draw_pose(box,rng,scene,static)]
        else:
            bodies=draw_obstacle(i,rng,scene,static)
        scene['obstacles'].extend(bodies)
        static.extend(polygon(b) for b in bodies)
    access=Access(scene,tables,rng); scene['robot_spawn']=access.start;scene['robot_start_expected']=access.start
    visible=prepare_room(scene,tables) if prepare_room is not None else None
    sizes=[t['size'] for t in template['targets']]
    ids=rng.choice(dictionary_size,size=marker_count,replace=False)
    witnesses={}
    hosts=rng.permutation(['wall','object','object']) if spread else ()
    for index,ident in enumerate(ids):
        size=float(rng.choice(sizes))
        kind=hosts[index] if spread else host_stress or ('wall' if count==0 or rng.integers(2)==0 else 'object')
        for _ in range(800):
            placement=mount(rng,scene,size,kind)
            if placement is None:continue
            point,inward,host,height=placement
            if spread and (host.startswith('wall_structure_') or
                    any(t['attached_to']==host or math.dist(point,(t['x'],t['y']))<.38*min(scene['room_size'])
                        for t in scene['targets'])):
                continue
            low=max(.25,size*.65+.02);high=min(.40,height-size*.65-.02) if height else .40
            if high<low:continue
            z=float(rng.uniform(low,high));yaw=math.atan2(inward[1],inward[0])
            # Keep complete printed sheets separate on the same plane. An object
            # may carry several IDs, including on adjacent or opposite faces.
            object_id=next((b['object_id'] for b in scene['obstacles'] if b['name']==host),host)
            if any(t['mount_object']==object_id and math.cos(yaw-t['yaw'])>.95 and
                   abs(z-t['z'])<.65*(size+t['size']) and
                   math.dist(point,(t['x'],t['y']))<.65*(size+t['size'])+.015 for t in scene['targets']):
                continue
            view=access.witness(point,inward,size,z,visible)
            if view is not None:break
        else:raise ValueError('No geometrically observable print on the sampled host class')
        scene['targets'].append(dict(id=int(ident),size=size,z=z,x=float(point[0]),y=float(point[1]),
                                     yaw=yaw,attached_to=host,mount_object=object_id))
        witnesses[str(ident)]=view
    scene['randomization']=dict(enabled=False,seed=int(seed),generator='exam-random-layout',furniture=furniture)
    if obstacle_set=='boxes':
        scene['randomization']['obstacles']='exam_boxes'
    elif spread:
        scene['randomization'].update(obstacles='spread_exam_boxes',box_count=3,
                                      box_marker_count=2,outer_wall_marker_count=1)
    scene['description']='Original Gazebo exam room with independently sampled movable obstacles, spawn and printed markers.'
    return scene,tables,witnesses
