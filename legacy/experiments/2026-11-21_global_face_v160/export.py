"""Verify and archive paired evaluator outputs plus the synthetic recordings."""
import csv
import hashlib
import json
from pathlib import Path
import shutil
import tarfile
import argparse

def export(root,dest):
    read=lambda p:json.loads(p.read_text())
    assert read(root/'complete.json')==dict(completed=True,pairs=6,runs=12)
    rows=read(root/'results.json');protocol=read(root/'protocol.json')
    assert {(r['case'],r['policy']) for r in rows}=={(case,v) for case in 'ABCDEF' for v in ['mixed153','global160']}
    dest.mkdir(exist_ok=False)
    for name in ('results.json','observer_tests.txt','protocol.json','complete.json'):
        shutil.copy2(root/name,dest/name)
    known={Path(p).name:Path(p) for p in read(root/'runs.json')}
    for row in rows:
        run=known[row['run']];audit=read(run/'audit.json');m=read(run/'metrics.json')
        assert m['world_sha256']==protocol['sdf_sha256'][row['case']]
        track=dest/'paths'/run.name;track.mkdir(parents=True)
        shutil.copy2(run/'scene.json',track/'scene.json')
        with (run/'headings.csv').open() as f,(track/'track.csv').open('w') as w:
            rd=csv.DictReader(f);out=csv.DictWriter(w,fieldnames=rd.fieldnames);out.writeheader()
            last=-1
            for item in rd:
                t=float(item['elapsed_sim_s'])
                if t>=0 and t-last>=.25:
                    out.writerow(item);last=t
        section=dest/'training_robust/native90/global-face-six-1121'/run.name
        video=section/'video';video.mkdir(parents=True)
        shutil.copy2(run/'mission.mp4',video/'mission.mp4')
        v=audit['video']
        assert hashlib.sha256((video/'mission.mp4').read_bytes()).hexdigest()==v['sha256']
        (video/'recording.json').write_text(json.dumps(dict(frames=v['frames'],duration_seconds=v['duration_s'])))
        (section/'evaluation.json').write_text(json.dumps(dict(success=row['strict'],elapsed_seconds=row['elapsed_s'],
                targets=len(row['ids']),collisions=row['bump_events'],reason=row['stop_reason'],
                ground_truth_audit={'closest_approach_m':m['min_ground_truth_m']})))
    with tarfile.open(dest.with_suffix('.tgz'),'w:gz') as tar:tar.add(dest,arcname=dest.name)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);p.add_argument('destination',type=Path)
    a=p.parse_args();export(a.root,a.destination)
