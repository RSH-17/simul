"""Project the actual SDF solids through the measured OAK-D lens on its central image row."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent/'2026-11-22_v151_adaptive_survey/development'))
from exposure_opportunity import blockers


def visible_columns(world, pose, calibration, pair):
    base=Rotation.from_euler('z',pose[2])
    camera=base*Rotation.from_quat(calibration['quaternion'])
    origin=base.apply(calibration['translation'])+np.array([*pose[:2],0.])
    width=calibration['width'];k=calibration['k']
    rays=camera.apply(np.column_stack(((np.arange(width)-k[2])/k[0],np.zeros(width),np.ones(width))))
    objects=blockers(world,0.,3.)
    if len(pair)!=2 or len(set(pair))!=2:raise ValueError('Two different solids are required')
    distances=[]
    names=np.array([o.name.split('/')[0] for o in objects])
    for obj in objects:
        offset=(origin-obj.center)@obj.axes
        direction=rays@obj.axes
        parallel=abs(direction)<1e-12
        div=np.where(parallel,1.,direction)
        first=(-obj.half-offset)/div;last=(obj.half-offset)/div
        entry=np.where(parallel,-np.inf,np.minimum(first,last)).max(axis=1)
        exit_=np.where(parallel,np.inf,np.maximum(first,last)).min(axis=1)
        hit=(~(parallel&(abs(offset)>obj.half)).any(axis=1))&(exit_>=np.maximum(entry,0.))&(exit_>0.)
        distances.append(np.where(hit,np.maximum(0.,entry),np.inf))
    depth=np.asarray(distances)
    closest=np.where(np.isfinite(depth).any(axis=0),names[depth.argmin(axis=0)],'')
    projection=[np.isfinite(depth[names==obj]).any(axis=0) for obj in pair]
    shared=projection[0]&projection[1]&np.isin(closest,pair)
    def spans(mask):
        edges=np.flatnonzero(np.diff(np.r_[False,mask,False])).reshape(-1,2)
        return [[int(a),int(b-1)] for a,b in edges]
    return dict(camera_origin_world=origin.tolist(),center_row_px=k[5],width_px=width,objects=pair,
        projected_columns={o:spans(p) for o,p in zip(pair,projection)},
        first_visible_columns={o:spans(closest==o) for o in pair},overlapping_columns=spans(shared),
        overlapping_count=int(shared.sum()))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for k in ('world','setup','calibration','output'):parser.add_argument(k,type=Path)
    a=parser.parse_args();setup=json.loads(a.setup.read_text());cal=json.loads(a.calibration.read_text())
    r=visible_columns(ET.parse(a.world).find('world'),setup['start'],cal,setup['pair'])
    r.update(world_sha256=hashlib.sha256(a.world.read_bytes()).hexdigest(),calibration_sha256=hashlib.sha256(a.calibration.read_bytes()).hexdigest(),
             type='geometry at enrolled start, not a camera recording')
    a.output.write_text(json.dumps(r,indent=2)+'\n')
