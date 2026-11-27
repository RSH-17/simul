"""Check projected black-square width and incidence on the last missed box using RGB stamps."""
import argparse
import csv
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np


def audit(recording, frames, camera, sdf, ident):
    target=next(t for t in json.loads((recording/'scene.json').read_text())['targets'] if t['id']==ident)
    calibration=json.loads(camera.read_text())
    poses=list(csv.DictReader((recording/'headings.csv').open()))
    stamps=list(csv.DictReader(frames.open()))
    t=np.array([float(f['elapsed_sim_s']) for f in stamps])
    pt=np.array([float(p['elapsed_sim_s']) for p in poses])
    yaw=np.interp(t,pt,np.unwrap([float(p['yaw_rad']) for p in poses]))
    x=np.interp(t,pt,[float(p['x']) for p in poses]); y=np.interp(t,pt,[float(p['y']) for p in poses])
    body_near=np.hypot(x-target['x'],y-target['y'])<=1.5
    tx,ty,_=calibration['translation']
    x=x+tx*np.cos(yaw)-ty*np.sin(yaw)
    y=y+tx*np.sin(yaw)+ty*np.cos(yaw)
    qx,qy,qz,qw=calibration['quaternion']
    yaw+=math.atan2(2*(qy*qz-qx*qw),2*(qx*qz+qy*qw))
    marker=ET.parse(sdf).find(f".//world/model[@name='target_{ident}']")
    paper=float(marker.findtext("model[@name='paper']/pose").split()[0])
    cell=float(next(m for m in marker.findall('model') if m.get('name','').startswith('cell_')).findtext('pose').split()[0])
    sign=math.copysign(1.,cell-paper)
    nx,ny=sign*math.cos(target['yaw']),sign*math.sin(target['yaw'])
    dx=x-target['x'];dy=y-target['y']
    bearing_cos=np.clip((dx*nx+dy*ny)/np.hypot(dx,dy),-1,1)
    incidence=np.degrees(np.arccos(bearing_cos))
    side=target['size']/2
    edges=[];facing=bearing_cos>0
    for s in (-1,1):
        vx=-dx-s*ny*side;vy=-dy+s*nx*side
        z=vx*np.cos(yaw)+vy*np.sin(yaw)
        facing &= z>.1
        edges.append(calibration['k'][2]+calibration['k'][0]*(vx*np.sin(yaw)-vy*np.cos(yaw))/np.maximum(z,.1))
    visible=facing & (np.minimum(*edges)>=4) & (np.maximum(*edges)<=calibration['width']-5)
    widths=np.abs(edges[0]-edges[1])
    best=np.argmax(np.where(visible,widths,-1))
    return dict(id=ident, front_local_x_sign=sign, camera_frames=len(t),body_near_frames=int(body_near.sum()),
                frustum_front_near_frames=int((visible & body_near).sum()),
                frustum_front_near_18px_frames=int((visible & body_near & (widths>=18)).sum()),
                near_best_incidence_deg=round(float(incidence[visible & body_near].min()),1) if (visible&body_near).any() else None,
                all_frustum_front_18px_frames=int((visible & (widths>=18)).sum()), best_frame=int(stamps[best]['frame']) if visible.any() else None,
                best_t=round(t[best],3) if visible.any() else None, best_width_px=round(widths[best],2) if visible.any() else None,
                limitations='planar horizontal pinhole projection; omits occlusion, vertical cropping, chassis pitch and roll')


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('folder',type=Path);parser.add_argument('frames',type=Path)
    parser.add_argument('camera',type=Path);parser.add_argument('sdf',type=Path);parser.add_argument('id',type=int)
    args=parser.parse_args()
    print(json.dumps(audit(args.folder,args.frames,args.camera,args.sdf,args.id),indent=2))
