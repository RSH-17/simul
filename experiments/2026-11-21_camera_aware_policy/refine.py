"""Second round: isolate synchronization and bounded local-focus commitment."""
import fcntl
import importlib.util
import json
import os
from pathlib import Path
import statistics
import sys

import campaign
spec=importlib.util.spec_from_file_location('camera_aware_summary',Path(__file__).with_name('summarize.py'))
summary_module=importlib.util.module_from_spec(spec);spec.loader.exec_module(summary_module)
analyze=summary_module.analyze

STAGES=dict(camera=Path.home()/'flow_v15_camera_aware_r2_ws',
    sync=Path.home()/'flow_v151_exposure_sync_r2_ws',
    committed=Path.home()/'flow_v16_local_commitment_r3_ws')


def native(root,variant,seed,phase,duration):
    # Resume only finalized recordings. A legitimate policy failure is retained
    # as a result; retries are limited to excluded launch/recording failures.
    base=f'v15-{variant}-detail-m{seed}-{phase}'
    for attempt in range(1,4):
        label=phase if attempt==1 else phase+f'-retry{attempt}'
        name=base if attempt==1 else base+f'-retry{attempt}'
        run=Path.home()/'corridor-board-73bbb05/recordings'/name
        if run.exists():
            if (run/'metrics.json').is_file() and (run/'audit.json').is_file() and not (run/'trial_error.json').exists():
                return run
            continue
        try:return campaign.execute(root,variant,'detail',seed,label,duration)
        except Exception:
            if attempt==3:raise
    raise RuntimeError('Three incomplete/excluded infrastructure attempts: '+base)


def save(root,rows):
    temp=root/'runs.tmp';temp.write_text(json.dumps(rows,indent=2)+'\n');temp.replace(root/'runs.json')


def main(root):
    locks=[open(Path.home()/'track-recordings'/name,'a') for name in ('.domain.lock','.second-domain.lock')]
    for lock in locks:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    assert os.environ['ROS_DOMAIN_ID']=='174' and os.environ['ROS_LOCALHOST_ONLY']=='1'
    proof=json.loads((Path.home()/'camera-sync-delay250.json').read_text())
    assert proof['registered_ids']==[42] and proof['diagnostics']['errors']==0
    module=Path('src/osp_mission/osp_mission/detector.py')
    assert (STAGES['sync']/module).read_bytes()==(STAGES['committed']/module).read_bytes()
    campaign.WORKSPACES.clear();campaign.WORKSPACES.update(STAGES)
    rows=[]
    for index,seed in enumerate((112021,112101)):
        variants=tuple(STAGES) if index==0 else tuple(reversed(STAGES))
        for variant in variants:
            rows.append(str(native(root,variant,seed,'dev2',180)));save(root,rows)
    report=analyze(root);(root/'development.json').write_text(json.dumps(report,indent=2)+'\n')
    groups={g['variant']:g for g in report['groups']}
    sync,committed=groups['sync'],groups['committed']
    # Both changes share the same validated perception code. Prefer the simpler
    # sync-only policy unless commitment improves success or >=5% mean time.
    quality=lambda g:(-g['strict_saved_map_successes'],g['bumper_events'],g['sensor_interruptions'])
    choose_commit=(quality(committed)<quality(sync) or
        (quality(committed)==quality(sync) and
         committed['mean_sim_s']<=.95*sync['mean_sim_s']))
    selected='committed' if choose_commit else 'sync'
    selection=dict(selected=selected,profile='detail',selection_maps=[112021,112101],
        rule='Require synchronized perception; success, contacts and sensor interruptions first; prefer sync unless commitment improves quality or >=5% time',
        baseline_reference='camera',held_out_used=False,
        caution='112101 was a first-round holdout counterexample and is explicitly second-round tuning data')
    (root/'selection.json').write_text(json.dumps(selection,indent=2)+'\n')
    for index,seed in enumerate((112201,112202,112203)):
        for variant in (('camera',selected) if index%2==0 else (selected,'camera')):
            rows.append(str(native(root,variant,seed,'heldout2',300)));save(root,rows)
    (root/'complete.json').write_text(json.dumps(dict(completed=True,runs=len(rows),development_runs=6,
        held_out_runs=6,selected=selected,physical_drive=False),indent=2)+'\n')


if __name__=='__main__':main(Path(sys.argv[1]).resolve())
