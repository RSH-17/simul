"""Analyze strict navigation, camera load and stop/turn behavior from saved runs."""
import argparse
import collections
import csv
import json
import math
from pathlib import Path
import re
import statistics
import numpy as np


def motion(run):
    with (run/'headings.csv').open() as stream:
        rows=[row for row in csv.DictReader(stream) if float(row['elapsed_sim_s'])>=0]
    t=np.array([float(row['elapsed_sim_s']) for row in rows])
    xy=np.array([[float(row['x']),float(row['y'])] for row in rows])
    yaw=np.unwrap(np.array([float(row['yaw_rad']) for row in rows]))
    dt=np.diff(t);good=dt>0
    speed=np.linalg.norm(np.diff(xy,axis=0),axis=1)/np.maximum(dt,1e-9)
    rotation=np.abs(np.diff(yaw))/np.maximum(dt,1e-9)
    translating=good&(speed>=.02);turning=good&(speed<.02)&(rotation>=.05)
    stopped=good&(speed<.02)&(rotation<.05)
    return dict(translation_s=float(dt[translating].sum()),rotation_only_s=float(dt[turning].sum()),
        stopped_s=float(dt[stopped].sum()),integrated_abs_turn_rad=float(np.abs(np.diff(yaw))[good].sum()))


def analyze(root):
    rows=[]
    for name in json.loads((root/'runs.json').read_text()):
        run=Path(name);metric=json.loads((run/'metrics.json').read_text());audit=json.loads((run/'audit.json').read_text())
        resources=json.loads((run/'resource_samples.json').read_text());bins={}
        for sample in resources:
            key=(sample['t'],sample['kind']);old=bins.setdefault(key,[0.,0.])
            old[0]+=sample['cpu_percent_one_core'];old[1]+=sample['rss_mib']
        cpu={kind:dict(median_percent_one_core=float(np.median([value[0] for (_,k),value in bins.items() if k==kind])),
            p95_percent_one_core=float(np.percentile([value[0] for (_,k),value in bins.items() if k==kind],95)),
            max_sampled_rss_mib=max(value[1] for (_,k),value in bins.items() if k==kind))
            for kind in {sample['kind'] for sample in resources}}
        events=[json.loads(line) for line in (run/'events.jsonl').read_text().splitlines()]
        start=next(event for event in events if event['event']=='start_ack')
        text=(run/'launch.log').read_text(errors='replace')
        cancellations=collections.Counter(re.findall(r'Cancel (?:nav|spin): ([a-z_]+)',text))
        goals=[]
        if (run/'native/hybrid_goals.jsonl').exists():
            goals=[json.loads(line) for line in (run/'native/hybrid_goals.jsonl').read_text().splitlines()]
            goals=[goal for goal in goals if start['sim']<=goal['t']<=start['sim']+metric['elapsed_sim_s']+.1]
        duplicate_selected=sum(a.get('kind')==b.get('kind') and math.hypot(a['x']-b['x'],a['y']-b['y'])<.1 for a,b in zip(goals,goals[1:]))
        alignment=metric['world_from_map_initial_alignment'];errors={}
        if alignment:
            rotation=np.array(alignment['rotation']);offset=np.array(alignment['translation'])
            import yaml
            targets={t['id']:(t['x'],t['y']) for t in yaml.safe_load((run/'scene.yaml').read_text())['targets']}
            errors={str(i):float(np.linalg.norm(rotation@np.array(p)+offset-np.array(targets[int(i)])))
                for i,p in audit['saved_map_xy'].items() if int(i) in targets}
        saved_map_ok=set(map(int,errors))=={7,32,103} and max(errors.values(),default=math.inf)<=.25
        status=metric['last_policy_status']
        rows.append(dict(run=run.name,variant=metric['variant'],profile=metric['profile'],
            split='held_out' if '-heldout' in run.name else 'development',
            seed=int(re.search(r'-m(\d+)-',run.name)[1]),
            strict_live_success=metric['strict_success'],strict_saved_map_success=bool(metric['strict_success'] and saved_map_ok),
            independent_approach_complete=metric['independent_approach_complete'],
            policy_sensor_proof_complete=metric['policy_sensor_proof_complete'],
            registered_ids=metric['final_ids'],sensor_verified_ids=status.get('approached_ids',[]),
            elapsed_sim_s=metric['elapsed_sim_s'],elapsed_wall_s=metric['elapsed_wall_s'],
            path_length_m=metric['path_length_m'],motion=motion(run),
            bumper_events=metric['bumper_events'],minimum_valid_scan_range_m=metric['minimum_valid_scan_range_m'],
            camera=metric['camera'],cpu=cpu,
            sensor_interruptions=status.get('sensor_interruptions'),navigation_flow=status.get('navigation_flow'),
            map_errors_live_m=metric['marker_position_errors_m'],map_errors_saved_m=errors,
            initial_registered_ids=start['ids'],world_sha256=metric['world_sha256'],
            selected_goal_counts=dict(collections.Counter(goal['kind'] for goal in goals)),
            consecutive_same_selected_goal_count=duplicate_selected,cancellations=dict(cancellations),
            nav2_control_rate_warnings=sum('control loop' in line.lower() and 'missed' in line.lower() for line in text.splitlines()),
            detector=metric['last_detector_status'],video=audit['video']))
    groups=[]
    for split in ('development','held_out'):
        for variant in ('baseline','candidate'):
            for profile in ('balanced','glance','detail'):
                selected=[r for r in rows if (r['split'],r['variant'],r['profile'])==(split,variant,profile)]
                if not selected:continue
                groups.append(dict(split=split,variant=variant,profile=profile,n=len(selected),
                    strict_saved_map_successes=sum(r['strict_saved_map_success'] for r in selected),
                    strict_live_successes=sum(r['strict_live_success'] for r in selected),
                    mean_sim_s=statistics.mean(r['elapsed_sim_s'] for r in selected),
                    mean_wall_s=statistics.mean(r['elapsed_wall_s'] for r in selected),
                    mean_path_m=statistics.mean(r['path_length_m'] for r in selected),
                    mean_stopped_s=statistics.mean(r['motion']['stopped_s'] for r in selected),
                    bumper_events=sum(r['bumper_events'] for r in selected),
                    worst_saved_map_error_m=max((error for r in selected for error in r['map_errors_saved_m'].values()),default=None)))
    return dict(complete=(root/'complete.json').is_file(),runs=rows,groups=groups,
        selection=json.loads((root/'selection.json').read_text()) if (root/'selection.json').exists() else None,
        notes=['Native runs use identical world files and poses, not identical sensor/control trajectories.',
               'Initial prestart registrations are recorded and can differ; camera processing runs during startup.',
               'Strict live criterion and final saved-map audit are listed separately.',
               'Stopped time excludes in-place rotation using independent simulator pose and heading.',
               'Heartbeat publication moved to the end of processing in v1.5; raw heartbeat ages differ in semantics.',
               'Selected goals are not an exact action dispatch count or proof of unnecessary replanning.',
               'Minimum scan range is not footprint-adjusted clearance.',
               'VM software-rendered camera does not validate RPi/Create3 load or physical reliability.'])


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('root',type=Path);parser.add_argument('output',type=Path)
    args=parser.parse_args();result=analyze(args.root);args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(complete=result['complete'],n=len(result['runs']),groups=result['groups']),indent=2))
