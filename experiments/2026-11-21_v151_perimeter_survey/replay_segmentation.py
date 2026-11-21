"""Replay the same recorded SLAM/coverage snapshot through two wall segmenters."""
import importlib.util
import json
from pathlib import Path
import sys
import numpy as np
from osp_mission.center_wall import surface_cells
from osp_mission._r32_navigation_core import Grid
from osp_mission.perimeter_route import PerimeterRoute

old_file=Path.home()/'flow_v151_perimeter_survey_r2_ws/src/osp_mission/osp_mission/perimeter_route.py'
spec=importlib.util.spec_from_file_location('old_perimeter_route',old_file)
old=importlib.util.module_from_spec(spec);sys.modules[spec.name]=old;spec.loader.exec_module(old)
run=Path.home()/'corridor-board-73bbb05/recordings/v151-perimeter-track-m112601-1795267796273583198'
files=sorted((run/'native/policy_decisions').glob('*.npz'))
selected=[]
for desired in (40.,100.,140.):
    path=min(files,key=lambda p:abs(float(np.load(p)['elapsed'])-desired))
    if path not in selected:selected.append(path)
rows=[]
for path in selected:
    with np.load(path) as z:
        grid=Grid(z['data'],float(z['resolution']),(*z['origin'],0.))
        pose=(*z['robot'],float(z['heading']))
        samples=surface_cells(grid,pose)
        result={}
        for label,cls in [('r2_grid_axes',old.PerimeterRoute),('r3_measured_normals',PerimeterRoute)]:
            model=cls()
            model.observe(samples,grid,z['viewed'],(0,0,0),lambda *args:None,float(z['elapsed']))
            result[label]=dict(sections=len(model.sections),observed=sum(s.observed for s in model.sections.values()),
                              maximum_fraction=max((s.fraction for s in model.sections.values()),default=0))
        rows.append(dict(elapsed_s=float(z['elapsed']),snapshot=path.name,**result))
print(json.dumps(dict(protocol='Identical stored SLAM and RGB boundary flags. One-pass segmentation only; map-to-odom set to identity for both. Not new navigation or marker detection.',rows=rows),indent=2))
