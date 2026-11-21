"""Export lightweight paired metrics + SHA-verified synthetic clips for this suite."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import shutil

from compare import report


def save_track(run,dst):
    host=dst/'tracks'/run.name
    host.mkdir(parents=True)
    for filename in ('scene.json','policy_revision.txt','policy_manifest.json'):
        if (run/filename).is_file():
            shutil.copy2(run/filename,host/filename)
    if (run/'run/simulation/demo_layout.json').is_file():
        shutil.copy2(run/'run/simulation/demo_layout.json',host/'layout.json')
    with (run/'headings.csv').open() as source, (host/'track.csv').open('w') as output:
        reader=csv.DictReader(source)
        writer=csv.DictWriter(output,fieldnames=reader.fieldnames)
        writer.writeheader()
        last=-1.
        for pose in reader:
            time=float(pose['elapsed_sim_s'])
            if time >= 0 and time-last >= .25:
                writer.writerow(pose)
                last=time


def export(root,dst,partial=False):
    result=report(root)
    if not result['complete'] and not partial:
        raise RuntimeError('Native paired comparison is incomplete')
    dst.mkdir(exist_ok=False)
    (dst/'comparison.json').write_text(json.dumps(result,indent=2)+'\n')
    shutil.copy2(root/'protocol.json',dst/'protocol.json')
    for name in ('superseded_preflight.json','scorecard_status.json'):
        if (root/name).exists():
            shutil.copy2(root/name,dst/name)
    archive=json.loads((root/'protocol.json').read_text())['archived_runs']
    archived_names={Path(row).name for row in archive}
    for path in archive:
        save_track(Path(path),dst)
    collected=[]
    for role in ('baseline','candidate'):
        runs_file=root/role/'runs.json'
        if not runs_file.is_file():
            continue
        for folder in json.loads(runs_file.read_text()):
            run=Path(folder)
            if run.name in archived_names:
                continue
            metrics=json.loads((run/'metrics.json').read_text())
            info=json.loads((run/'audit.json').read_text())['video']
            summary=json.loads((root/role/'report.json').read_text())
            row=next(item for item in summary['runs'] if item['run']==run.name)
            save_track(run,dst)
            episode=dst/'training_robust/native90/v151-adaptive-survey-1122'/run.name
            media=episode/'video'
            media.mkdir(parents=True)
            shutil.copy2(run/'mission.mp4',media/'mission.mp4')
            checksum=hashlib.sha256((media/'mission.mp4').read_bytes()).hexdigest()
            if checksum!=info['sha256']:
                raise ValueError(f'Video hash mismatch in {run}')
            stats=dict(frames=info['frames'],duration_seconds=info['duration_s'])
            (media/'recording.json').write_text(json.dumps(stats,indent=2)+'\n')
            evaluation=dict(success=row['strict_saved_map_success'],reason=row['reason'],
                            elapsed_seconds=row['elapsed_sim_s'],targets=row['registered_ids'],
                            collisions=row['bumper_events'],ground_truth_audit=dict(
                                closest_approach_m=metrics['min_ground_truth_m'],
                                registered_and_within_s=metrics['registered_and_within_s']))
            (episode/'evaluation.json').write_text(json.dumps(evaluation,indent=2)+'\n')
            collected.append(dict(run=run.name,seed=row['seed'],role=role,sha256=checksum,**stats))
    (dst/'media.json').write_text(json.dumps(collected,indent=2)+'\n')


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('root',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--partial',action='store_true')
    a=p.parse_args();export(a.root.resolve(),a.output.resolve(),a.partial)
