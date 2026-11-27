"""Open-loop selection on archived SLAM and directional RGB-coverage snapshots."""
import argparse
import collections
import hashlib
import json
import math
from pathlib import Path
import sys
import time

import numpy as np
from scipy import ndimage


def replay(folder, calibration):
    from osp_mission._r32_navigation_core import Grid
    from osp_mission.center_wall import CenterWall
    selector=CenterWall()
    focal=calibration['k'][0]
    if hasattr(selector,'survey'):
        selector.survey.camera=(focal,2*math.atan(calibration['width']/(2*focal)))
    rows=[]
    for path in sorted((folder/'native/policy_decisions').glob('*.npz')):
        with np.load(path,allow_pickle=False) as snap:
            elapsed=float(snap['elapsed'])
            grid=Grid(snap['data'],float(snap['resolution']),(*snap['origin'],0.))
            free=(grid.cells>=0)&(grid.cells<=25)
            clearance=ndimage.distance_transform_edt(np.pad(free,1))[1:-1,1:-1]*grid.resolution
            cost=Grid(np.where(clearance>=.32,0,100).astype(np.int8),grid.resolution,grid.origin)
            pose=(*snap['robot'],float(snap['heading']))
            choices=[dict(x=x,y=y,yaw=a,scan_angle=turn,features=np.array(feat),distance=float(feat[2]))
                     for x,y,a,turn,*feat in snap['candidates']]
            start=time.monotonic()
            goal=selector.decide(grid,cost,pose,(0.,0.,0.),snap['viewed'],elapsed,600.,lambda _:False,choices=choices)
            wall_time=time.monotonic()-start
            row=dict(file=path.name,elapsed=elapsed,reason=selector.reason,wall_time_s=round(wall_time,4))
            if goal:
                selector.dispatched(goal,elapsed)
                row.update(kind=goal.get('survey_kind',goal.get('r32_kind','other')),
                           surface=goal.get('surface_map'),goal_xy=[float(goal['x']),float(goal['y'])],
                           gain_wall=goal.get('survey_wall_gain_m'),gain_interior=goal.get('survey_interior_gain_m'))
            rows.append(row)
    return dict(run=folder.name, selections=dict(collections.Counter(r.get('kind') for r in rows)), rows=rows)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('source',type=Path);p.add_argument('camera',type=Path)
    p.add_argument('output',type=Path);p.add_argument('runs',type=Path,nargs='+')
    args=p.parse_args()
    sys.path.insert(0,str(args.source.resolve()))
    calibration=json.loads(args.camera.read_text())
    data=dict(scope='Open loop: archived SLAM map, pose, Flow candidates and camera viewed flags. Costmap estimated by .32m clearance; identity map-to-odom, no live box fitting, veto, costmap, marker interrupts, new images or action outcomes. Replayed selector maintains its own dispatch history.',runs=[])
    files = args.source / 'osp_mission'
    data['source_sha256'] = {name:hashlib.sha256((files/name).read_bytes()).hexdigest()
        for name in ('center_wall.py','box_batch.py','survey.py','survey_distance.py','deferred_views.py')
        if (files/name).is_file()}
    for folder in args.runs:
        data['runs'].append(replay(folder,calibration))
        args.output.write_text(json.dumps(data,indent=2)+'\n')
        print(folder.name,data['runs'][-1]['selections'],flush=True)
