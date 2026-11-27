"""Change only frontier reward in already recorded eligible candidate lists.

Preserve observed map, candidate poses, path costs, revisit penalties and
weights. Reproduce the old selection before comparing the new feature. This
does not simulate subsequent navigation or camera detections.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
from scipy.ndimage import distance_transform_edt,label


def replay(workspace,baseline):
    package=workspace/'src/osp_mission';sys.path.insert(0,str(package))
    from osp_mission.escape_connectivity import connect_start
    from osp_mission.frontier import reachable_gain
    from osp_mission.frontier_support import exploration_gain,supported_unknown
    from osp_mission.surface_endpoint_policy import choose
    rows=[]
    for path in sorted(baseline.glob('episodes/*/policy_decisions/*.npz')):
        with np.load(path) as d:
            grid=d['data'];resolution=float(d['resolution']);origin=d['origin'];robot=d['robot']
            candidates=d['candidates'];selected=int(d['selected_index']);heading=float(d['heading'])
            row=dict(snapshot=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),elapsed=float(d['elapsed']))
            if not len(candidates) or selected<0 or np.all(candidates[:,3]==0):
                row['skipped']='crowded-start or no selected ordinary goal';rows.append(row);continue
            xy=np.floor((robot-origin)/resolution).astype(int);ry,rx=int(xy[1]),int(xy[0])
            safe=(grid==0)&(distance_transform_edt(grid<=50)*resolution>=.32)
            safe=connect_start(grid,resolution,(ry,rx),safe);groups,_=label(safe)
            safe&=groups==groups[ry,rx]
            old=reachable_gain(grid,resolution,safe);new=exploration_gain(grid,resolution,safe)
            cells=np.floor((candidates[:,:2]-origin)/resolution).astype(int)
            mismatch=max(abs(float(c[5])-5*old[y,x]) for c,(x,y) in zip(candidates,cells))
            original=[dict(index=i,features=c[4:].copy()) for i,c in enumerate(candidates)]
            prior=choose(original,d['weights'])
            if mismatch>1e-5 or prior['index']!=selected:
                row.update(skipped='nonstandard features or overridden choice',maximum_old_feature_error=float(mismatch))
                rows.append(row);continue
            altered=[]
            for candidate,(x,y) in zip(original,cells):
                features=candidate['features'].copy();features[1]=5*new[y,x]
                if features[0]==0 and features[1]<.075:continue
                altered.append(dict(index=candidate['index'],features=features))
            after=choose(altered,d['weights'])
            def details(choice):
                if choice is None:return None
                i=choice['index'];pose=candidates[i,:3];projection=(pose[:2]-robot)@np.array([np.cos(heading),np.sin(heading)])
                return dict(index=i,pose=pose.tolist(),features=choice['features'].tolist(),
                            goal_projection_along_current_heading_m=float(projection))
            row.update(original_choice_reproduced=True,old=details(prior),new=details(after),
                old_goal_new_frontier_feature=float(5*new[cells[selected,1],cells[selected,0]]),
                original_candidates=len(original),retained_candidates=len(altered),
                supported_unknown_cells=int(supported_unknown(grid,resolution).sum()),
                raw_unknown_cells=int((grid<0).sum()))
            rows.append(row)
    return dict(evidence_type='Offline re-ranking of saved eligible candidates; not native success',
        rows=rows,ground_truth_input=False,weights_retrained=False,
        unchanged=['map','candidate positions','path distance','turn feature','revisit penalty','weights'],
        limitations=['Actual new candidate generation may also change sampling; this isolates the reward.',
            'Projection behind current heading is not by itself evidence of a repeated physical path.'],
        runtime_source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in
            [package/'osp_mission/frontier_support.py',package/'osp_mission/frontier.py',
             package/'osp_mission/surface_endpoint_policy.py',package/'osp_mission/escape_connectivity.py']})


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ['workspace','baseline','output']:p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args();a.output.write_text(json.dumps(replay(a.workspace.resolve(),a.baseline.resolve()),indent=2)+'\n')
