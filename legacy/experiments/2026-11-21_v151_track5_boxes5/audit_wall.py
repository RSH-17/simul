"""Horizontal projection opportunities, not an occlusion/recognition oracle."""
import argparse
import csv
import json
import math
from pathlib import Path
import numpy as np

def audit(run,geometry):
    scene=json.loads((run/'scene.json').read_text());g=json.loads(geometry.read_text())
    with (run/'headings.csv').open() as stream:poses=list(csv.DictReader(stream))
    with (run/'frames.csv').open() as stream:frames=list(csv.DictReader(stream))
    pt=np.array([float(p['elapsed_sim_s']) for p in poses]);ft=np.array([float(p['elapsed_sim_s']) for p in frames])
    heading=np.interp(ft,pt,np.unwrap([float(p['yaw_rad']) for p in poses]))
    x=np.interp(ft,pt,[float(p['x']) for p in poses]);y=np.interp(ft,pt,[float(p['y']) for p in poses])
    qx,qy,qz,qw=g['quaternion'];front=np.array([2*(qx*qz+qy*qw),2*(qy*qz-qx*qw)])
    offset=math.atan2(front[1],front[0]);yaw=heading+offset
    bx,by,_=g['translation'];x=x+np.cos(heading)*bx-np.sin(heading)*by;y=y+np.sin(heading)*bx+np.cos(heading)*by
    fx,cx=g['k'][0],g['k'][2];results=[]
    metrics=json.loads((run/'metrics.json').read_text())
    for target in scene['targets']:
        if not target['attached_to'].startswith('wall_'):continue
        tangent=np.array([-math.sin(target['yaw']),math.cos(target['yaw'])])*target['size']/2
        edges=[];forward=[]
        for sign in (-1,1):
            dx=target['x']+sign*tangent[0]-x;dy=target['y']+sign*tangent[1]-y
            z=np.cos(yaw)*dx+np.sin(yaw)*dy;right=np.sin(yaw)*dx-np.cos(yaw)*dy
            edges.append(cx+fx*right/np.maximum(z,1e-9));forward.append(z)
        visible=(forward[0]>.1)&(forward[1]>.1)&(np.minimum(*edges)>=4)&(np.maximum(*edges)<=g['width']-5)
        pixels=np.abs(edges[1]-edges[0]);eligible=np.where(visible)[0]
        chosen=sorted(eligible,key=lambda i:pixels[i],reverse=True)
        best=[]
        for i in chosen:
            if all(abs(ft[i]-r['elapsed_s'])>=1 for r in best):
                best.append(dict(frame=int(frames[i]['frame']),elapsed_s=float(ft[i]),horizontal_pixels=float(pixels[i]),
                    center_u=float((edges[0][i]+edges[1][i])/2),camera_xy=[float(x[i]),float(y[i])],camera_yaw=float(yaw[i]),
                    image=str(run/'camera'/f"{int(frames[i]['frame']):05d}.jpg")))
            if len(best)==5:break
        detail=metrics['last_detector_status'].get('by_id',{}).get(str(target['id']),{})
        results.append(dict(id=target['id'],size_m=target['size'],closest_actual_m=metrics['min_ground_truth_m'].get(str(target['id'])),
            horizontal_fov_frames=int(visible.sum()),max_horizontal_pixels=float(pixels[visible].max()) if visible.any() else None,
            opportunity_frames={str(n):int((visible&(pixels>=n)).sum()) for n in (6,8,18)},best_frames=best,detector_id_diagnostics=detail))
    return dict(run=run.name,walls=results,limitations='Horizontal planar approximation only: ignores occlusion, vertical clipping, pitch and body roll. It does not prove a marker was visible or readable. Actual JPEG frames must be inspected.')

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('geometry',type=Path);p.add_argument('output',type=Path)
    a=p.parse_args();result=audit(a.run,a.geometry);a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
