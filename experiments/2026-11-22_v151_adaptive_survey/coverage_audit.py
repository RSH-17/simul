"""Count live policy RGB-credited faces retained in archived decision snapshots."""
import argparse
import json
from pathlib import Path

import numpy as np


def summarize(run):
    rows=[]
    for path in sorted((run/'native/policy_decisions').glob('*.npz')):
        with np.load(path,allow_pickle=False) as z:
            rows.append(dict(elapsed_s=round(float(z['elapsed']),3),seen_face_cells=int(z['viewed'].sum()),
                             shape=list(z['data'].shape),origin_xy=z['origin'].tolist()))
    changes=[]
    for before,after in zip(rows,rows[1:]):
        loss=before['seen_face_cells']-after['seen_face_cells']
        if loss>0:
            changes.append(dict(from_s=before['elapsed_s'],to_s=after['elapsed_s'],net_loss_cells=loss,
                                geometry_changed=before['shape']!=after['shape'] or before['origin_xy']!=after['origin_xy']))
    return dict(run=run.name,snapshots=rows,net_decreases=changes,
                caveat='Occupancy evidence, camera credit, SLAM origin and geometry all evolve between snapshots. A net drop is observed; identity of an unchanged physical face is not independently known.')


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('runs',type=Path,nargs='+')
    args=parser.parse_args()
    print(json.dumps([summarize(run) for run in args.runs],indent=2))
