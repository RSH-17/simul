"""Replay frozen observed maps; no hidden scene, future map or robot commands."""
import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys

import numpy as np
from scipy.ndimage import distance_transform_edt, label


def audit(inputs, baseline, candidate):
    source = baseline / 'src/osp_mission/osp_mission'
    target = candidate / 'src/osp_mission/osp_mission'
    digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    old = {p.name: digest(p) for p in source.glob('*.py')}
    new = {p.name: digest(p) for p in target.glob('*.py')}
    if {k for k in old.keys() | new.keys() if old.get(k) != new.get(k)} != {'start_waypoints.py'}:
        raise ValueError('Require an isolated start-waypoint source change')
    sys.path.insert(0, str(source.parent))
    from osp_mission import start_waypoints as original
    if Path(original.__file__).resolve() != (source / 'start_waypoints.py').resolve():
        raise ValueError('Original helper imported from another workspace')
    spec = importlib.util.spec_from_file_location('pocket_candidate', target / 'start_waypoints.py')
    revised = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(revised)
    rows = []
    for file in sorted(inputs.glob('*.npz')):
        with np.load(file, allow_pickle=False) as d:
            data, resolution, origin, robot = d['data'], float(d['resolution']), d['origin'], d['robot']
            heading = float(d['heading'])
            x, y = np.floor((robot-origin) / resolution).astype(int)
            if not (0 <= y < data.shape[0] and 0 <= x < data.shape[1]):
                raise ValueError('Invalid recorded robot position')
            normal = (data == 0) & (distance_transform_edt(data <= 50) * resolution >= .32)
            groups, _ = label(normal)
            region = groups == groups[y, x] if groups[y, x] else np.zeros_like(normal)
            clearance = distance_transform_edt(np.pad(data == 0, 1))[1:-1, 1:-1] * resolution
            minimum = .189 + resolution / math.sqrt(2) + .01
            old_choices, _ = original.options(data, resolution, origin, robot, heading)
            choices, previous = revised.options(data, resolution, origin, robot, heading, True)
            proposals = []
            for c in choices:
                cell = c['observed_frontier_cell']
                chain = set()
                path_clearance = []
                while True:
                    if cell in chain or data[cell] != 0 or clearance[cell] < minimum:
                        raise AssertionError('Invalid full-body observed-free path')
                    chain.add(cell)
                    path_clearance.append(clearance[cell])
                    if cell == (y, x):
                        break
                    index = previous[cell[0] * data.shape[1] + cell[1]]
                    if index < 0:
                        raise AssertionError('Path is disconnected from robot')
                    parent = divmod(int(index), data.shape[1])
                    if cell[0] != parent[0] and cell[1] != parent[1]:
                        if min(clearance[cell[0], parent[1]], clearance[parent[0], cell[1]]) < minimum:
                            raise AssertionError('Path cuts a blocked corner')
                    cell = parent
                if c['cell'] not in chain:
                    raise AssertionError('Waypoint is not on endpoint path')
                proposals.append(dict(xy=[c['x'], c['y']], distance_m=c['distance'],
                    pocket_continuation=c['pocket_continuation'],
                    endpoint_xy=[origin[0]+(c['observed_frontier_cell'][1]+.5)*resolution,
                                 origin[1]+(c['observed_frontier_cell'][0]+.5)*resolution],
                    full_path_minimum_clearance_m=float(min(path_clearance)), required_clearance_m=minimum))
            recorded_index = int(d['selected_index'])
            rows.append(dict(file=str(file),sha256=digest(file),elapsed_seconds=float(d['elapsed']),
                robot=robot.tolist(),normal_component_m2=float(region.sum()*resolution**2),
                original_transit_count=len(old_choices),candidate_transit_count=len(choices),
                recorded_selected_xy=d['candidates'][recorded_index,:2].tolist() if recorded_index >= 0 else None,
                proposals=proposals))
    return dict(evidence_type='Offline observed-map geometry replay, not a new trajectory',
        baseline=str(baseline),candidate=str(candidate),rows=rows,
        changed_source_sha256=dict(original=old['start_waypoints.py'],candidate=new['start_waypoints.py']),
        ground_truth_used=False,weights_retrained=False,native_success_established=False,
        limits=['No future maps or navigation outcomes predicted',
                'Candidate proposals before mission memory filtering and marker/approach overrides',
                'Safety here means recorded map body clearance; native contact testing remains required'])


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    for name in ['inputs', 'baseline', 'candidate', 'output']:
        p.add_argument('--'+name, type=Path, required=True)
    a = p.parse_args()
    report = audit(a.inputs.resolve(), a.baseline.resolve(), a.candidate.resolve())
    a.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(dict(decisions=len(report['rows']),new_transit_decisions=sum(
        bool(r['candidate_transit_count']) and not r['original_transit_count'] for r in report['rows']),
        all_proposal_paths_verified=True)))
