"""Replay saved policy observations; never read hidden scenario targets."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np


def replay(workspace, baseline):
    package=workspace/'src/osp_mission'
    sys.path.insert(0,str(package))
    from osp_mission.edge_policy import options, readable_distance
    inventory=baseline/'input_inventory.json'
    focal=json.loads(inventory.read_text())['streams']['info']['K'][0]
    files=sorted(baseline.glob('episodes/*/policy_decisions/*.npz'))
    if not files:raise ValueError('Need recorded policy observations')
    rows=[]
    for path in files:
        with np.load(path) as d:
            goals,_=options(d['data'],float(d['resolution']),tuple(d['origin']),tuple(d['robot']),
                float(d['heading']),(),d['viewed'],inspection_focal=focal,collect_faces=True)
            directed=[g for g in goals if g.get('face_inspection')]
            rows.append(dict(snapshot=path.name,elapsed=float(d['elapsed']),candidates=len(goals),
                directed_readable_candidates=sum(int(g['features'][0]>0) for g in directed),
                readable_candidates=sum(int(g['features'][0]>0) for g in goals),
                directed_face_length_m=max((float(g['features'][0]) for g in directed),default=0.),
                sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
    return dict(evidence_type='Offline candidate generation from recorded map/pose/coverage; not navigation success',
        workspace=str(workspace),baseline=str(baseline),focal_px=focal,readable_distance_m=readable_distance(focal),
        rows=rows,source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest()
            for p in [inventory,package/'osp_mission/edge_policy.py']},
        ground_truth_input=False,limitations=['Exploration-memory filtering and future sensor feedback are not replayed.',
            'More proposed views do not establish QR detection, safe arrival or full-task success.'])


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--workspace',type=Path,required=True)
    p.add_argument('--baseline',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();a.output.write_text(json.dumps(replay(a.workspace.resolve(),a.baseline.resolve()),indent=2)+'\n')
