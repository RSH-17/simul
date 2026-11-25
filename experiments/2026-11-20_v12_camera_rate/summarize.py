"""Summarize actual simulated capture/processing rates, resources and navigation."""
import hashlib,json,sys,math,csv
from pathlib import Path
import numpy as np
import yaml

root=Path(sys.argv[1]);rows=[]
for name in json.loads((root/'runs.json').read_text()):
    run=Path(name);metric=json.loads((run/'metrics.json').read_text());audit=json.loads((run/'audit.json').read_text())
    resource=json.loads((run/'resource_samples.json').read_text());camera=metric['camera']
    grouped={};memory={}
    for row in resource:
        key=(row['t'],row['kind']);grouped[key]=grouped.get(key,0.)+row['cpu_percent_one_core']
        memory[key]=memory.get(key,0.)+row['rss_mib']
    cpu={kind:dict(median_percent_one_core=float(np.median([value for (_,k),value in grouped.items() if k==kind])),
                   p95_percent_one_core=float(np.percentile([value for (_,k),value in grouped.items() if k==kind],95)),
                   max_sampled_rss_mib=max(value for (_,k),value in memory.items() if k==kind))
         for kind in sorted({r['kind'] for r in resource})}
    status=metric.get('last_policy_status',{})
    launch=(run/'launch.log').read_text(errors='replace').lower()
    events=[json.loads(line) for line in (run/'events.jsonl').read_text().splitlines()]
    start=next(event for event in events if event['event']=='start_ack')
    expected_start=yaml.safe_load((run/'scene.yaml').read_text())['start']
    with (run/'headings.csv').open() as stream:
        first_heading=next(row for row in csv.DictReader(stream) if float(row['elapsed_sim_s'])>=0.)
    pose_error=math.dist(start['pose'],expected_start[:2])
    yaw_error=abs(math.remainder(float(first_heading['yaw_rad'])-expected_start[2],2*math.pi))
    rows.append(dict(run=run.name,camera=camera,cpu=cpu,elapsed_sim_s=metric['elapsed_sim_s'],elapsed_wall_s=metric['elapsed_wall_s'],
        registered_ids=metric['final_ids'],registered_s=metric['registered_s'],approached_ids=sorted(map(int,metric['registered_and_within_s'])),
        policy_sensor_verified_ids=status.get('approached_ids',[]),
        nearest_ground_truth_m=metric['min_ground_truth_m'],bumper_events=metric['bumper_events'],
        navigation_flow=status.get('navigation_flow',{}),sensor_interruptions=status.get('sensor_interruptions'),
        nav2_control_rate_warnings=sum('control loop' in line and 'missed' in line for line in launch.splitlines()),
        readable_ids=status.get('visual_ids',[]),policy_registered_targets=status.get('targets'),
        ids_present_at_mission_start=start['ids'],start_world_pose_xy=start['pose'],expected_start_world_pose=expected_start,
        start_position_error_m=pose_error,start_yaw_error_rad=yaw_error,
        saved_map_xy=audit['saved_map_xy'],offline_rgb=audit['offline_rgb'],video=audit['video']))
result=dict(policy='Mixed v1.2, default forward limits 0.25/0.27 m/s',
    scope='Up to 90 seconds per matched-world validation, not a ten-minute completion or physical reliability test',
    sensor_change_only=True,sequential=True,domain=174,
    world_sha256={p.name:hashlib.sha256((p/'world/osp_studio.sdf').read_bytes()).hexdigest() for p in root.glob('map-*')},
    image_format='square RGB; same simulated field of view and optical transform',
    limitations=['No physical OAK-D exposure, USB transport or Create3 DDS load',
                 'Software rendering shares VM resources; camera render cost affects real-time factor',
                 'Unbalanced closed-loop repetitions: 480/5 and 480/10 repeated on map112021; trajectories may diverge after visual observations',
                 'Detector runs during startup: some IDs can be registered before mission time zero; initial IDs are recorded',
                 'Approached IDs use independent simulator distance <=1.5m after registration; policy sensor-witness completion is reported separately'],runs=rows)
(root/'comparison.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
