"""Replay recorded planning inputs; the costmap substitute is explicitly reported."""
import argparse
import importlib.util
import json
from pathlib import Path
import numpy as np
from scipy import ndimage
from osp_mission._r32_navigation_core import Grid
from osp_mission.center_wall import CenterWall

def compare(run,parent):
    spec=importlib.util.spec_from_file_location('parent_center_wall',parent)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    models=[module.CenterWall(),CenterWall()]
    alignment=json.loads((run/'metrics.json').read_text())['world_from_map_initial_alignment']
    scene=json.loads((run/'scene.json').read_text())
    east_step=next(o for o in scene['partitions'] if o['name']=='east_wall_step')
    east_straight=scene['room_size'][0]/2-.075
    rows=[]
    for p in sorted((run/'native/policy_decisions').glob('*.npz')):
        with np.load(p) as log:
            grid=Grid(log['data'],float(log['resolution']),(*log['origin'],0.))
            # Only SLAM and viewed inputs were retained in these snapshots.
            free=(grid.cells>=0)&(grid.cells<=25)
            room=ndimage.distance_transform_edt(np.pad(free,1))[1:-1,1:-1]*grid.resolution
            cost=Grid(np.where(room>=.32,0,100).astype(np.int8),grid.resolution,grid.origin)
            pose=(*log['robot'],float(log['heading']))
            choices=[dict(x=x,y=y,yaw=a,scan_angle=turn,features=np.array(feat),distance=float(feat[2]))
                     for x,y,a,turn,*feat in log['candidates']]
            elapsed=float(log['elapsed'])
            outputs=[]
            for model in models:
                goal=model.decide(grid,cost,pose,(0.,0.,0.),log['viewed'],elapsed,600.,lambda g:False,choices=choices)
                record=dict(kind=goal.get('r32_kind','other') if goal else None,reason=model.reason)
                if goal:
                    model.dispatched(goal,elapsed)
                    record.update(section=goal.get('wall_section'),wait_s=goal.get('wall_wait_s'))
                    if 'surface_map' in goal and goal['r32_kind']=='wall_surface':
                        target=np.array(alignment['rotation'])@goal['surface_map']+alignment['translation']
                        x,y=target
                        east=(east_step['x']-east_step['sx']/2 if abs(y-east_step['y'])<=east_step['sy']/2 else east_straight)
                        record.update(surface_world_m=target.tolist(),
                                      near_east_boundary=bool(abs(x-east)<.35 and abs(y)<scene['room_size'][1]/2))
                outputs.append(record)
            rows.append(dict(elapsed=elapsed,baseline=outputs[0],candidate=outputs[1]))
    return dict(run=run.name,decisions=len(rows),rows=rows)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('parent',type=Path);p.add_argument('output',type=Path);p.add_argument('runs',nargs='+',type=Path)
    a=p.parse_args()
    results=[compare(run,a.parent) for run in a.runs]
    a.output.write_text(json.dumps(dict(protocol='Open-loop replay of saved SLAM, viewpoint coverage and original trajectory; each selector keeps its own dispatch history. Static map-to-odom, SLAM-derived costmap proxy, no live Nav2, box fits, marker interrupts or new camera observations.',results=results),indent=2)+'\n')
    for r in results:
        print(r['run'],r['decisions'],[(k,sum(v[k]['kind']=='wall_surface' for v in r['rows'])) for k in ['baseline','candidate']])
