"""Recheck all camera ratios on the frozen selected algorithm before final test."""
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
spec=importlib.util.spec_from_file_location('ratio_summary',HERE/'summarize.py')
summary=importlib.util.module_from_spec(spec);spec.loader.exec_module(summary)


def main(root):
    previous=Path.home()/'camera-refine-policy-1121'
    print('WAITING for refinement validation',flush=True)
    while not (previous/'complete.json').exists():time.sleep(2.)
    chosen=json.loads((previous/'selection.json').read_text())['selected']
    previous_report=summary.analyze(previous)
    held=[r for r in previous_report['runs'] if r['split']=='held_out' and r['variant']==chosen]
    if len(held)!=3 or not all(r['strict_saved_map_success'] for r in held):
        root.with_suffix('.needs_review.json').write_text(json.dumps(dict(reason='Refinement did not pass all three validation maps'))+'\n')
        return
    locks=[open(Path.home()/'track-recordings'/name,'a') for name in ('.domain.lock','.second-domain.lock')]
    for lock in locks:fcntl.flock(lock,fcntl.LOCK_EX)
    if not root.exists():prepare(root,development=(112201,112202),held_out=(),distractor=True)
    campaign.WORKSPACES[chosen]=Path.home()/('flow_v151_exposure_sync_r2_ws' if chosen=='sync' else 'flow_v16_local_commitment_r3_ws')
    os.environ['CAMERA_BENCH_TRIAL_SCRIPT']=str(HERE/'warm_trial.py')
    rows=[]
    for index,seed in enumerate((112201,112202)):
        profiles=tuple(campaign.PROFILES) if index==0 else tuple(reversed(campaign.PROFILES))
        for profile in profiles:
            run=campaign.execute(root,chosen,profile,seed,'matrix-dev',300)
            rows.append(str(run));(root/'runs.json').write_text(json.dumps(rows,indent=2)+'\n')
    report=summary.analyze(root);groups=report['groups']
    quality=lambda g:(-g['strict_saved_map_successes'],g['bumper_events'],g['sensor_interruptions'])
    best_quality=min(map(quality,groups));eligible=[g for g in groups if quality(g)==best_quality]
    fastest=min(g['mean_sim_s'] for g in eligible)
    close=[g for g in eligible if g['mean_sim_s']<=1.05*fastest]
    payload=lambda g:(campaign.PROFILES[g['profile']][0]**2*campaign.PROFILES[g['profile']][1],g['mean_sim_s'])
    selected=min(close,key=payload)
    selection=dict(selected=chosen,profile=selected['profile'],quality_maps=[112201,112202],warmup_sim_s=12.,
        rule='Success, contacts and sensor interruptions first; among profiles within 5% of fastest mean choose lower raw RGB payload',
        final_maps_seen=False)
    (root/'selection.json').write_text(json.dumps(selection,indent=2)+'\n')
    (root/'complete.json').write_text(json.dumps(dict(completed=True,runs=6,selection=selection,physical_drive=False),indent=2)+'\n')


if __name__=='__main__':main(Path(sys.argv[1]).resolve())
