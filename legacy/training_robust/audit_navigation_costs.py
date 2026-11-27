"""Compare recorded costmaps using installed C++ Nav2 footprint checking.

Each received map is kept unchanged. This is a component diagnostic, not a
replayed controller trajectory, synchronized sensor comparison or native run.
"""
import argparse
import ast
import json
import math
from pathlib import Path
import subprocess

import numpy as np


def costs(binary, grid, resolution, origin, footprint, padding, poses):
    if abs(origin[2]) > 1e-8:
        raise ValueError('Nav2 Costmap2D requires an axis-aligned grid')
    lines=[f'{grid.shape[1]} {grid.shape[0]} {resolution} {origin[0]} {origin[1]}',
           ' '.join(str(int(v)) for v in grid.ravel()),f'{len(footprint)} {padding}',
           *[f'{x} {y}' for x,y in footprint],str(len(poses)),
           *[' '.join(str(float(v)) for v in pose) for pose in poses]]
    result=subprocess.run([str(binary)],input='\n'.join(lines)+'\n',capture_output=True,text=True,check=True)
    values=np.fromstring(result.stdout,sep='\n')
    if len(values)!=len(poses):raise ValueError('Incomplete component output')
    return values


def transform(tf):
    q=tf['rotation'];return [tf['translation']['x'],tf['translation']['y'],
        math.atan2(2*(q['w']*q['z']+q['x']*q['y']),1-2*(q['y']**2+q['z']**2))]


def analyze(capture, binary):
    report=json.loads((capture/'report.json').read_text())
    parameters=report['parameters']['local_costmap/local_costmap']
    footprint=ast.literal_eval(parameters['footprint']['string_value'])
    padding=parameters['footprint_padding']['double_value']
    robot=transform(report['transforms']['odom<-base_link']['transform'])
    map_from_odom=transform(report['transforms']['map<-odom']['transform'])
    angles=np.linspace(0.,2*math.pi,361)
    steps=np.linspace(0.,.6,61)
    rotation=np.column_stack([np.full(361,robot[0]),np.full(361,robot[1]),robot[2]+angles])
    forward=np.column_stack([robot[0]+steps*math.cos(robot[2]),robot[1]+steps*math.sin(robot[2]),np.full(61,robot[2])])
    rows={}
    for name,parent in [('local_costmap','odom'),('global_costmap','map')]:
        saved=np.load(capture/(name+'_costmap_raw.npz'))
        groups={}
        for kind,points in [('rotation',rotation),('forward',forward)]:
            poses=points.copy()
            if parent=='map':
                x,y,a=map_from_odom;c,s=math.cos(a),math.sin(a)
                poses[:,0]=x+c*points[:,0]-s*points[:,1]
                poses[:,1]=y+s*points[:,0]+c*points[:,1]
                poses[:,2]+=a
            values=costs(binary,saved['data'],float(saved['resolution']),saved['origin'],footprint,padding,poses)
            groups[kind]=dict(samples=len(values),lethal_or_unknown=int((values>=254).sum()),
                maximum=float(values.max()),initial=float(values[0]),values=values.tolist())
        rows[name]=dict(resolution_m=float(saved['resolution']),**groups)
    return dict(evidence_type='Offline installed Nav2 C++ footprint checker on unchanged received grids',
        grids=rows,physical_footprint=footprint,padding_m=padding,ground_truth_input=False,
        native_success=False,limits=['Map snapshots are asynchronous; not a synchronized sensor replay',
        'This does not rerun MPPI, costmap updates or model dynamics',
        'Differences may reflect grid resolution and/or layer contents; new native comparison is required'])


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--capture',type=Path,required=True)
    p.add_argument('--binary',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();r=analyze(a.capture.resolve(),a.binary.resolve());a.output.write_text(json.dumps(r,indent=2)+'\n')
    print(json.dumps({name:{k:v for k,v in row.items() if k=='resolution_m'}|{kind:{k:v for k,v in row[kind].items() if k!='values'} for kind in ['rotation','forward']} for name,row in r['grids'].items()},indent=2))
