"""Collect the five completed runs and their verified, synthetic-only recordings."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import shutil
import tarfile
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'2026-11-21_region_wall_v153'))
from compare import load, main as compare


def export(root, base, destination):
    destination.mkdir(exist_ok=False)
    pairs=compare(root,base,destination/'comparison.json')
    assert pairs['complete'] and len(pairs['pairs'])==5
    for name in ('protocol.json','report.json'):
        shutil.copy2(root/name,destination/name)
    manifest=load(root/'protocol.json')['source_manifest']
    source=Path.home()/'flow_v151_object_survey_r2_ws/src/osp_mission/osp_mission'
    assert all(hashlib.sha256((source/p).read_bytes()).hexdigest()==sha for p,sha in manifest['source_sha256'].items())
    for pair in pairs['pairs']:
        for variant,row in pair.items():
            run=Path.home()/'corridor-board-73bbb05/recordings'/row['run']
            folder=destination/'tracks'/run.name
            folder.mkdir(parents=True)
            shutil.copy2(run/'scene.json',folder)
            shutil.copy2(run/'run/simulation/demo_layout.json',folder/'layout.json')
            with (run/'headings.csv').open() as f, (folder/'track.csv').open('w') as out:
                reader=csv.DictReader(f)
                writer=csv.DictWriter(out,fieldnames=reader.fieldnames)
                writer.writeheader()
                last=-1
                for pose in reader:
                    t=float(pose['elapsed_sim_s'])
                    if t>=0 and t-last>=.25:
                        writer.writerow(pose)
                        last=t
            if variant!='candidate':
                continue
            episode=destination/'training_robust/native90/v151-object-survey-1121'/run.name
            video=episode/'video'
            video.mkdir(parents=True)
            info=load(run/'audit.json')['video']
            shutil.copy2(run/'mission.mp4',video/'mission.mp4')
            assert hashlib.sha256((video/'mission.mp4').read_bytes()).hexdigest()==info['sha256']
            (video/'recording.json').write_text(json.dumps(dict(frames=info['frames'], duration_seconds=info['duration_s'])))
            (episode/'evaluation.json').write_text(json.dumps(dict(success=row['strict'],elapsed_seconds=row['elapsed_sim_s'],
                targets=len(row['registered']),collisions=row['bumps'],reason=load(run/'metrics.json')['reason'],
                ground_truth_audit={'closest_approach_m':load(run/'metrics.json')['min_ground_truth_m']})))
    with tarfile.open(destination.with_suffix('.tgz'),'w:gz') as tar:
        tar.add(destination,arcname=destination.name)


if __name__=='__main__':
    p=argparse.ArgumentParser()
    for name in ('root','base','destination'):
        p.add_argument(name,type=Path)
    a=p.parse_args()
    export(a.root,a.base,a.destination)
