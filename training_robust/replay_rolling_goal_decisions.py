"""Diagnostic rolling-goal checks between stored decision snapshots.

These maps were saved at mission decisions, not at periodic mid-navigation
ticks. Comparing a preceding destination on the next map is a robustness
diagnostic only, never a replayed trajectory or an estimate of time saved.
No scenario or evaluator truth is opened.
"""
import argparse
import hashlib
import io
import json
from pathlib import Path
import sys

import numpy as np


def decode(row):
    return dict(x=float(row[0]),y=float(row[1]),yaw=float(row[2]),scan_angle=float(row[3]),
        distance=float(row[6]),features=np.array(row[4:9]),
        coverage_kind='surface' if row[4]>0 else 'frontier')


def audit(workspace,episodes):
    sys.path.insert(0,str(workspace/'src/osp_mission'))
    from osp_mission.rolling_goal import RollingGoal
    import osp_mission.rolling_goal as module
    expected=workspace/'src/osp_mission/osp_mission/rolling_goal.py'
    if Path(module.__file__).resolve()!=expected:raise ValueError('Unexpected imported runtime')
    rows=[];sources={str(Path(__file__).resolve()):hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        str(expected):hashlib.sha256(expected.read_bytes()).hexdigest()}
    for episode in episodes:
        previous=None
        for path in sorted((episode/'policy_decisions').glob('*.npz')):
            content=path.read_bytes();sources[str(path)]=hashlib.sha256(content).hexdigest()
            with np.load(io.BytesIO(content),allow_pickle=False) as d:
                values=d['candidates'];index=int(d['selected_index']);now=float(d['elapsed'])
                if previous is not None and now-previous[1]>=12. and len(values):
                    incumbent,started=previous;policy=RollingGoal();policy.selected(incumbent,started)
                    choices=[decode(r) for r in values if r[4]>0 or r[6]>.7]
                    old_features=incumbent['features'].copy()
                    choice=policy.consider(choices,(incumbent['x'],incumbent['y'],incumbent['yaw']),
                        d['robot'],d['weights'],now,600.-now)
                    assert np.array_equal(incumbent['features'],old_features)
                    rows.append(dict(input=str(path),elapsed_s=now,gap_s=now-started,
                        decision=choice,retained=choice is None,candidates=len(choices),
                        previous_destination=[incumbent['x'],incumbent['y'],incumbent['yaw']]))
                previous=(decode(values[index]),now) if 0<=index<len(values) and values[index,4]>0 else None
    return dict(evidence_type='Offline diagnostic across stored decision maps, not mid-navigation rollout',
        rows=rows,checks=len(rows),suggested_reselections=sum(not r['retained'] for r in rows),
        source_sha256=sources,ground_truth_input=False,native_improvement_proven=False,
        limitations=['Input lacks per-tick active-action state, cancellation handshake and exact candidate metadata.',
            'Current-goal features are matched on the subsequent map; no future trajectory is simulated.',
            'Exploration-memory filtering is already in the recorded candidates; no new memory rollout occurs.',
            'No hardware result, completion improvement or saved navigation time is established.'])


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--workspace',type=Path,required=True);p.add_argument('--episode',type=Path,action='append',required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():raise FileExistsError('Preserve earlier diagnostic')
    result=audit(a.workspace.resolve(),[e.resolve() for e in a.episode])
    a.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(checks=result['checks'],suggested_reselections=result['suggested_reselections'])))
