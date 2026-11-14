"""Compare only face allocation on recorded policy inputs, never evaluator data.

The parent is reproduced by restoring its uniform-index sampler in the otherwise
identical candidate module. No future motion, memory update or detection is replayed.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time

import numpy as np


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def face_set(goals):
    return {tuple(int(x) for x in f) for g in goals for f in g.get('visible_faces',())}


def matrix(goals):
    return [[g['x'],g['y'],g['yaw'],g.get('scan_angle',0.),*g['features'].tolist()] for g in goals]


def audit(baseline,candidate,jobs,transit_episodes):
    old=baseline/'src/osp_mission/osp_mission';new=candidate/'src/osp_mission/osp_mission'
    files={str(p.relative_to(old)) for p in old.glob('*.py')}|{str(p.relative_to(new)) for p in new.glob('*.py')}
    changed={f for f in files if not (old/f).exists() or not (new/f).exists() or digest(old/f)!=digest(new/f)}
    if changed!={'edge_policy.py','face_proposals.py'}:raise ValueError('Require isolated proposal allocation change')
    restored=(new/'edge_policy.py').read_text().replace('from osp_mission.face_proposals import select_faces\n','')
    restored=restored.replace('select_faces(data,resolution,f,valid,distance,inspection_standoffs(inspection_focal))',
        "np.linspace(0,len(f['y'])-1,min(48,len(f['y'])),dtype=int)")
    if restored!=(old/'edge_policy.py').read_text():raise ValueError('Parent reproduction does not match source')
    sys.path.insert(0,str(new.parent))
    from osp_mission import edge_policy
    if Path(edge_policy.__file__).resolve()!=new/'edge_policy.py':raise ValueError('Unexpected planner import')
    select=edge_policy.select_faces
    paths=[]
    for job in jobs:
        saved=sorted(job.glob('episodes/*/policy_decisions/*.npz'))
        if not saved:raise ValueError('Missing recorded policy inputs')
        paths.extend([(saved[len(saved)//2],'midpoint'),(saved[-1],'last')])
    for episode in transit_episodes:
        for path in sorted((episode/'policy_decisions').glob('*.npz')):
            with np.load(path,allow_pickle=False) as d:
                if float(d['elapsed'])<30.:paths.append((path,'early_departure'))
    rows=[];sources={str(p):digest(p) for p in [Path(__file__),old/'edge_policy.py',new/'edge_policy.py',new/'face_proposals.py']}
    for path,kind in paths:
        memory_path=path.parent.parent/'exploration_memory.jsonl'
        memories=[json.loads(line) for line in memory_path.read_text().splitlines()]
        sources[str(memory_path)]=digest(memory_path)
        with np.load(path,allow_pickle=False) as d:
            elapsed=float(d['elapsed']);memory=next(r for r in memories if abs(r['elapsed_seconds']-elapsed)<1e-6)
            focal=memory['inspection_focal_px'];res=float(d['resolution'])
            row=dict(input=str(path),input_sha256=digest(path),elapsed_seconds=elapsed,selection=kind,arms={})
            goals_by_arm={}
            for arm in ['uniform','feasible_spread']:
                allocation={}
                def sampler(data,resolution,faces,valid,distance,standoffs):
                    eligible=select(data,resolution,faces,valid,distance,standoffs,limit=len(faces['y']))
                    chosen=(np.linspace(0,len(faces['y'])-1,min(48,len(faces['y'])),dtype=int)
                            if arm=='uniform' else select(data,resolution,faces,valid,distance,standoffs))
                    allocation.update(unseen_faces=len(faces['y']),feasible_faces=len(eligible),
                        selected_faces=len(chosen),selected_feasible_faces=int(np.isin(chosen,eligible).sum()))
                    return chosen
                edge_policy.select_faces=sampler
                started=time.perf_counter()
                try:
                    goals,_=edge_policy.options(d['data'],res,d['origin'],d['robot'],float(d['heading']),(),
                        d['viewed'],inspection_focal=focal,collect_faces=True)
                finally:
                    edge_policy.select_faces=select
                directed=[g for g in goals if g.get('face_inspection')]
                goals_by_arm[arm]=goals
                row['arms'][arm]=dict(allocation=allocation,raw_candidates=len(goals),
                    directed_candidates=len(directed),readable_boundary_length_m=len(face_set(goals))*res,
                    directed_boundary_length_m=len(face_set(directed))*res,
                    wall_seconds_including_audit=time.perf_counter()-started,
                    all_transit=bool(goals) and all(g.get('start_escape') for g in goals))
            before,after=goals_by_arm.values();a,b=face_set(before),face_set(after)
            row['new_readable_boundary_m']=len(b-a)*res;row['lost_readable_boundary_m']=len(a-b)*res
            row['raw_candidates_identical']=matrix(before)==matrix(after)
            row['raw_parent_candidates_preserved']=all(value in matrix(after) for value in matrix(before))
            rows.append(row)
            print(json.dumps(dict(input=str(path),arms=row['arms'],new_m=row['new_readable_boundary_m'],lost_m=row['lost_readable_boundary_m'])),flush=True)
    return dict(evidence_type='Offline proposal allocation on fixed observed policy maps; not a new trajectory',
        baseline=str(baseline),candidate=str(candidate),source_sha256=sources,rows=rows,
        parent_source_reproduction_checked=True,ground_truth_used=False,weights_retrained=False,
        native_validated=False,production_promoted=False,
        limitations=['Midpoint and last maps are selected diagnostics, not representative success statistics.',
            'No memory filtering, deadline ranking, safe arrival, camera feedback or later SLAM updates are rolled out.',
            'Readable boundary length is geometric proposal coverage, not correctly detected marker IDs.',
            'Timing includes additional audit work and host contention; it is not runtime performance.'])


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ['baseline','candidate','output']:p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--job',type=Path,action='append',default=[])
    p.add_argument('--transit-episode',type=Path,action='append',default=[])
    a=p.parse_args();report=audit(a.baseline.resolve(),a.candidate.resolve(),a.job,a.transit_episode)
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(report,indent=2)+'\n')
