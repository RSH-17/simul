"""Wait for completed refinement, then evaluate frozen choice on three new maps."""
import fcntl
import importlib.util
import json
import os
from pathlib import Path
import sys
import time

import campaign
from prepare import prepare
HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('final_camera_summary',HERE/'summarize.py')
summary=importlib.util.module_from_spec(spec);spec.loader.exec_module(summary)


def main(root):
    previous=Path.home()/'camera-matrix-policy-1121'
    print('WAITING for completed refinement and untouched selection',flush=True)
    while not (previous/'complete.json').exists():time.sleep(2.)
    report=summary.analyze(previous);selection=json.loads((previous/'selection.json').read_text())
    selected=selection['selected']
    profile=selection['profile']
    held=[r for r in report['runs'] if r['variant']==selected and r['profile']==profile]
    if len(held)!=2 or not all(r['strict_saved_map_success'] for r in held):
        root.parent.joinpath(root.name+'.needs_review.json').write_text(json.dumps(dict(
            reason='Selected camera profile did not pass both controlled ratio maps',selected=selected))+'\n')
        return
    locks=[open(Path.home()/'track-recordings'/name,'a') for name in ('.domain.lock','.second-domain.lock')]
    for lock in locks:fcntl.flock(lock,fcntl.LOCK_EX)
    if not root.exists():prepare(root,development=(),held_out=(112301,112302,112303),distractor=True)
    campaign.WORKSPACES.clear()
    campaign.WORKSPACES.update(camera=Path.home()/'flow_v15_camera_aware_r2_ws',
        sync=Path.home()/'flow_v151_exposure_sync_r2_ws',committed=Path.home()/'flow_v16_local_commitment_r3_ws')
    os.environ['CAMERA_BENCH_TRIAL_SCRIPT']=str(HERE/'warm_trial.py')
    (root/'selection.json').write_text(json.dumps(dict(selected=selected,profile=profile,
        chosen_before_final_map_results=True,warmup_sim_s=12.,basis=str(previous/'selection.json')),indent=2)+'\n')
    rows=[]
    for index,seed in enumerate((112301,112302,112303)):
        for variant in (('camera',selected) if index%2==0 else (selected,'camera')):
            run=campaign.execute(root,variant,profile,seed,'heldout-final',300)
            rows.append(str(run));(root/'runs.json').write_text(json.dumps(rows,indent=2)+'\n')
    (root/'complete.json').write_text(json.dumps(dict(completed=True,runs=len(rows),selected=selected,
        profile=profile,warmup_sim_s=12.,physical_drive=False),indent=2)+'\n')


if __name__=='__main__':main(Path(sys.argv[1]).resolve())
