"""Evaluator-only saved-image ray and conservative map reachability audit. No Nav2 call."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import cv2
import numpy as np
from scipy import ndimage

from exposure_opportunity import blockers, occluded, pose


def rotation(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]])


def ray(origin, yaw, limit, grid):
    points = np.arange(0, limit + .001, grid['resolution'] / 5)
    xy = origin + points[:, None] * np.array([math.cos(yaw), math.sin(yaw)])
    ij = np.floor((xy-grid['origin'])/grid['resolution']).astype(int)
    a = grid['data']
    inside = (ij[:, 0] >= 0) & (ij[:, 0] < a.shape[1]) & (ij[:, 1] >= 0) & (ij[:, 1] < a.shape[0])
    values = np.full(len(points), -1, dtype=int)
    values[inside] = a[ij[inside, 1], ij[inside, 0]]
    def first(mask):
        return round(float(points[np.argmax(mask)]), 3) if mask.any() else None
    return {'first_occupied_m':first(values >= 65), 'first_unknown_m':first(values < 0),
            'first_ambiguous_m':first((values > 25) & (values < 65))}


def connected_views(grid, camera, target, normal, clearance_m):
    a, origin, res = grid['data'], grid['origin'], grid['resolution']
    free = (a >= 0) & (a <= 25)
    clearance = ndimage.distance_transform_edt(np.pad(free, 1))[1:-1,1:-1]*res
    reach, _ = ndimage.label(clearance >= clearance_m)
    sx, sy = np.floor((camera-origin)/res).astype(int)
    region = reach[sy,sx]
    if not region:
        return {'start_free':False,'cells':0}
    iy, ix = np.where(reach == region)
    xy = origin+(np.stack((ix,iy), axis=1)+.5)*res
    delta = xy-target
    distance = np.linalg.norm(delta,axis=1)
    valid = (distance >= .75) & (distance <= 1.3) & (delta@normal >= distance*math.cos(math.radians(45)))
    count = 0
    first = None
    for point, dist in zip(xy[valid], distance[valid]):
        yaw = np.arctan2(*(target-point)[::-1])
        line = ray(point, yaw, dist-.12, grid)
        if all(v is None for v in line.values()):
            count += 1
            if first is None:
                first = point.round(3).tolist()
    return {'start_free':True,'cells':count,'example_map_xy':first}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('run',type=Path);p.add_argument('calibration',type=Path)
    p.add_argument('before',type=Path);p.add_argument('after',type=Path);p.add_argument('--output',type=Path,required=True)
    args = p.parse_args(); run=args.run
    gj=lambda p:json.loads(p.read_text())
    calibration=gj(args.calibration)
    world=ET.parse(run/'world/osp_studio.sdf').find('world')
    tag=next(t for t in gj(run/'scene.json')['targets'] if t['id']==32)
    marker=world.find("model[@name='target_32']")
    cell=pose(next(m for m in marker.findall('model') if m.get('name','').startswith('cell_')))
    paper=pose(marker.find("model[@name='paper']"))
    normal=np.sign(cell[0]-paper[0])*rotation(tag['yaw'])[:,0]
    target=np.r_[np.array([tag['x'],tag['y']])+cell[0]*rotation(tag['yaw'])[:,0], tag['z']]
    xycam=np.array(calibration['translation'][:2]); camz=calibration['translation'][2]
    qx,qy,qz,qw=calibration['quaternion']
    optical_yaw=math.atan2(2*(qy*qz-qx*qw),2*(qx*qz+qy*qw))
    head=np.genfromtxt(run/'headings.csv',names=True,delimiter=',')
    stamps=np.genfromtxt(run/'frames.csv',names=True,delimiter=',')
    def gt(t):
        return np.array([np.interp(t,head['elapsed_sim_s'],f) for f in
                        (head['x'],head['y'],np.unwrap(head['yaw_rad']))])
    align=gj(run/'audit.json')['world_from_map_initial_alignment']
    matrix=np.array(align['rotation']); origin=np.array(align['translation'])
    snapshots={}
    for path in (args.before,args.after):
        g=np.load(path)
        snapshots[str(round(float(g['elapsed']),3))]={k:g[k] for k in ('origin','resolution','data','robot','heading','candidates')}
    static=blockers(world,min(camz,tag['z']-.15),max(camz,tag['z']+.15))
    records=[]
    for frame in (972,975):
        t=float(stamps['elapsed_sim_s'][stamps['frame']==frame][0])
        base=gt(t)
        cyaw=base[2]+optical_yaw; camera=base[:2]+rotation(base[2])@xycam
        delta=target[:2]-camera;dist=np.linalg.norm(delta)
        angle=float(np.arctan2(delta[1],delta[0]))
        im=cv2.imread(str(run/'camera'/f'{frame:05d}.jpg'),0)
        dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        corners,ids,_=cv2.aruco.detectMarkers(im,dictionary)
        quad=corners[list(ids.ravel()).index(32)].reshape(4,2)
        u,v=quad.mean(0)
        imageangle=cyaw+math.atan((calibration['k'][2]-u)/calibration['k'][0])
        wrap=lambda a:math.atan2(math.sin(a),math.cos(a))
        cast=np.r_[camera,camz][None,:]
        center_clear=not bool(occluded(cast,target,static)[0])
        row={'t':round(t,3),'frame':frame,'measured_quad_px':quad.tolist(),'jpeg_center_uv':[float(u),float(v)],
             'camera_ground_range_m':round(dist,4),'camera_range_3d_m':round(float(np.linalg.norm(target-cast[0])),4),
             'printed_front_incidence_deg':round(math.degrees(math.acos(np.clip((-delta)@normal/dist,-1.,1.))),3),
             'image_world_bearing_deg':round(math.degrees(wrap(imageangle)),3),
             'true_world_bearing_deg':round(math.degrees(wrap(angle)),3),
             'image_bearing_error_deg':round(math.degrees(wrap(imageangle-angle)),3),
             'center_clear_static_gazebo_visuals':center_clear,'map_snapshots':[]}
        for ts,g in snapshots.items():
            body_snap=gt(float(ts))
            snapped_rot=rotation(body_snap[2]-g['heading'])
            snapped_offset=body_snap[:2]-snapped_rot@g['robot']
            results={}
            for kind,R,T in [('fixed_initial',matrix,origin),('snapshot_robot_anchor',snapped_rot,snapped_offset)]:
                mc=R.T@(camera-T); mt=R.T@(target[:2]-T);mn=R.T@normal
                myaw=imageangle-math.atan2(R[1,0],R[0,0])
                r=ray(mc,myaw,dist+1.,g)
                if r['first_occupied_m'] is not None:
                    r['hit_minus_true_m']=round(r['first_occupied_m']-dist,3)
                c=g['candidates'][:,:2]-mt
                cd=np.linalg.norm(c,axis=1)
                front=(cd >= .75)&(cd <= 1.3)&(c@mn >= cd*math.cos(math.radians(45)))
                results[kind]={'image_ray':r,'connected_front_cells':{
                    str(clear):connected_views(g, R.T@(base[:2]-T), mt,mn,clear) for clear in (.32,.42)},
                    'archived_flow_candidates_in_front_envelope':int(front.sum())}
            row['map_snapshots'].append({'t':float(ts),'delta_from_exposure_s':round(float(ts)-t,3),
                                          'by_alignment':results})
        records.append(row)
    out={'run':run.name,'target':32,'opencv':cv2.__version__,'frames':records,
         'input_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in
                         (args.before,args.after,args.calibration,run/'frames.csv',run/'headings.csv',run/'audit.json')},
         'limitations':'SDF only for evaluation; pixel bearing uses decoded JPEG. Occupancy >=65, known-free <=25. Snapshots bracket actual exposure; static initial transform and independent local body alignment bound drift. Occupancy first hit is not target identity. Connected 4-neighbor free cells after .32/.42m obstacle+unknown clearance with .75–1.3m standoff, <=45deg incidence and clear ray (last .12m untested) is a map proxy, not actual Nav2, live costmap, scanner depth or pose-tracking guarantee. Stored Flow list excludes final adaptive candidates and facing/all costmap filters.'}
    args.output.write_text(json.dumps(out,indent=2)+'\n')


if __name__=='__main__':
    main()
