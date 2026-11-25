"""Common post-run metrics from already recorded, independently measured poses."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import shutil
import statistics
import tarfile


def load(p):return json.loads(p.read_text())

def common(ids,witness,errors,expected,bumps):
    ids=set(ids);expected=set(expected);near=ids & set(map(int,witness)) & expected
    ok=ids==expected and near==expected and set(map(int,errors))==expected
    return bool(ok and max(errors.values(),default=math.inf)<=.25 and bumps==0),sorted(near)

def record(run):
    m=load(run/'metrics.json');a=load(run/'audit.json');scene=load(run/'scene.json')
    expected={t['id'] for t in scene['targets']};wall=next(t['id'] for t in scene['targets'] if 'wall' in t['attached_to'])
    # Initial-time alignment remains fixed. Do not fit a transform to the final markers.
    alignment=m.get('world_from_map_initial_alignment')
    errors={}
    if alignment:
        rotation,translation=alignment['rotation'],alignment['translation']
        for t in scene['targets']:
            xy=a.get('saved_map_xy',{}).get(str(t['id']))
            if xy is not None:
                p=[sum(rotation[i][j]*xy[j] for j in range(2))+translation[i] for i in range(2)]
                errors[str(t['id'])]=math.dist(p,[t['x'],t['y']])
    ok,near=common(m['final_ids'],m['registered_and_within_s'],errors,expected,m['bumper_events'])
    with (run/'headings.csv').open() as f:trajectory=list(csv.DictReader(f))
    poses=[(float(p['elapsed_sim_s']),float(p['x']),float(p['y'])) for p in trajectory]
    distance=sum(math.dist(a[1:],b[1:]) for a,b in zip(poses,poses[1:]) if b[0]<=m['elapsed_sim_s']+.01)
    started=next(loadline for loadline in map(json.loads,(run/'events.jsonl').read_text().splitlines()) if loadline['event']=='start_ack')
    registered=set(m['final_ids']) & expected
    return dict(run=run.name,world_sha256=m['world_sha256'],initial_world_xy=started['pose'],
        registered=sorted(registered),unexpected_ids=sorted(set(m['final_ids'])-expected),approached=near,
        expected_ids=sorted(expected),wall_id=wall,wall_registered=wall in registered,wall_approached=wall in near,
        box_registered=len(registered-{wall}),box_approached=len(set(near)-{wall}),common_success=ok,
        saved_map_error_m=errors,elapsed_sim_s=m['elapsed_sim_s'],elapsed_wall_s=m['elapsed_wall_s'],
        all_approached_s=max(m['registered_and_within_s'].values()) if set(map(int,m['registered_and_within_s']))>=expected else None,
        path_m=distance,bumps=m['bumper_events'],reason=m['reason'],policy_specific_strict=m.get('strict_success'),
        policy_specific_approached=m['last_policy_status'].get('approached_ids'),video=a['video'])

def export(root,baseline,rescue,destination):
    assert (root/'complete.json').exists()
    destination.mkdir(exist_ok=False)
    roots=dict(balanced=baseline,rescue360=rescue,latest_r32=root)
    paths={k:list(map(Path,load(p/'runs.json'))) for k,p in roots.items()}
    assert all(len(v)==5 for v in paths.values())
    protocol=load(root/'protocol.json');shutil.copy2(root/'protocol.json',destination/'protocol.json')
    pairs=[]
    for seed in protocol['seeds']:
        pair=dict(seed=seed)
        for k,runs in paths.items():
            run=next(p for p in runs if f'm{seed}' in p.name)
            pair[k]=row=record(run)
            track=destination/'tracks'/run.name;track.mkdir(parents=True)
            shutil.copy2(run/'scene.json',track/'scene.json');shutil.copy2(run/'run/simulation/demo_layout.json',track/'layout.json')
            with (run/'headings.csv').open() as f,(track/'track.csv').open('w') as o:
                r=csv.DictReader(f);w=csv.DictWriter(o,fieldnames=r.fieldnames);w.writeheader();last=-1.
                for p in r:
                    t=float(p['elapsed_sim_s'])
                    if t>=0 and t-last>=.25:w.writerow(p);last=t
            if k=='latest_r32':
                episode=destination/'training_robust/native90/r32-latest-d8704f6-1122'/run.name
                video=episode/'video';video.mkdir(parents=True)
                shutil.copy2(run/'mission.mp4',video/'mission.mp4')
                assert hashlib.sha256((video/'mission.mp4').read_bytes()).hexdigest()==row['video']['sha256']
                (video/'recording.json').write_text(json.dumps(dict(frames=row['video']['frames'],duration_seconds=row['video']['duration_s'])))
                (episode/'evaluation.json').write_text(json.dumps(dict(success=row['common_success'],elapsed_seconds=row['elapsed_sim_s'],
                    targets=len(row['registered']),collisions=row['bumps'],reason=row['reason'],
                    ground_truth_audit={'closest_approach_m':load(run/'metrics.json')['min_ground_truth_m']})))
        assert len({pair[k]['world_sha256'] for k in roots})==1
        assert all(math.dist(pair[k]['initial_world_xy'],pair['balanced']['initial_world_xy'])<.05 for k in roots)
        pairs.append(pair)
    groups={}
    for k in roots:
        rows=[p[k] for p in pairs]
        groups[k]=dict(n=len(rows),successes=sum(r['common_success'] for r in rows),registered=sum(len(r['registered']) for r in rows),
            approached=sum(len(r['approached']) for r in rows),wall_registered=sum(r['wall_registered'] for r in rows),wall_approached=sum(r['wall_approached'] for r in rows),
            box_registered=sum(r['box_registered'] for r in rows),box_approached=sum(r['box_approached'] for r in rows),
            mean_sim_s=statistics.mean(r['elapsed_sim_s'] for r in rows),mean_path_m=statistics.mean(r['path_m'] for r in rows),bumps=sum(r['bumps'] for r in rows))
    result=dict(complete=True,common_protocol='3 exact IDs, independent post-registration approach <=1.5m, saved fixed-frame errors <=0.25m, zero bumps',
        groups=groups,pairs=pairs,held_out=False,upstream_commit=protocol['upstream_commit'],
        limitations=['End-to-end original Nav2 profiles differ; not a speed-normalized algorithm ablation.',
            'One run per policy per development map; stochastic sensor and control trajectories differ.',
            'Balanced/rescue recordings waited for their additional sensor witness; report all_approached_s separately.',
            'Raw video is the same source resolution and rate; real hardware not driven.'])
    (destination/'comparison.json').write_text(json.dumps(result,indent=2)+'\n')
    with tarfile.open(destination.with_suffix('.tgz'),'w:gz') as archive:archive.add(destination,arcname=destination.name)

if __name__=='__main__':
    p=argparse.ArgumentParser()
    for k in ('root','baseline','rescue','destination'):p.add_argument(k,type=Path)
    a=p.parse_args();export(a.root,a.baseline,a.rescue,a.destination)
