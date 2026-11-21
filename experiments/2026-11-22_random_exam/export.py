"""Export only finalized synthetic episodes, frozen scores, and video checksums."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import tempfile


def read(path):
    return json.loads(path.read_text())


def write(path,value):
    path.write_text(json.dumps(value,indent=2,ensure_ascii=False,allow_nan=False)+'\n')


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def export(root,recordings,destination):
    import cv2
    registration=root/'registration.json';protocol=root/'protocol.json'
    result=read(root/'comparison.json');plan=read(protocol);state=read(root/'status.json')
    if state['state'] not in ('complete','invalid_pair') or sha(registration)!=plan['registration_sha256']:
        raise ValueError('A terminal paired campaign with matching registration is required')
    rows=read(root/'episodes.json')
    if len(rows)!=2*len(plan['plan']) or len({r['name'] for r in rows})!=len(rows):
        raise ValueError('Incomplete or duplicate episodes')
    destination.parent.mkdir(parents=True,exist_ok=True)
    if destination.exists():raise FileExistsError(destination)
    with tempfile.TemporaryDirectory(dir=destination.parent) as work:
        stage=Path(work)/'bundle';stage.mkdir()
        for name in ('comparison.json','protocol.json','episodes.json','status.json','registration.json'):
            shutil.copy2(root/name,stage/name)
        if (root/'routes.png').is_file():shutil.copy2(root/'routes.png',stage/'routes.png')
        for folder in ('camera-opportunities', 'sensor-audit'):
            if (root/folder).is_dir():
                shutil.copytree(root/folder,stage/folder)
        media=[]
        for row in rows:
            name=row['name'];run=recordings/name
            scene=read(run/'scene.json');audit=read(run/'audit.json');metrics=read(run/'metrics.json')
            score=row['score'];clip=run/'mission.mp4'
            if (scene['seed']!=row['seed'] or audit['run']!=name or
                sha(clip)!=row['evidence']['mission.mp4'] or sha(run/'metrics.json')!=row['evidence']['metrics.json']):
                raise ValueError('Changed recording '+name)
            cap=cv2.VideoCapture(str(clip))
            try:
                frames=int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
                if frames!=audit['video']['frames'] or not cap.read()[0]:raise ValueError('Video start '+name)
                cap.set(cv2.CAP_PROP_POS_FRAMES,frames-1)
                if not cap.read()[0]:raise ValueError('Video end '+name)
            finally:cap.release()
            relative=Path('training_robust/native90')/root.name/name/'video/mission.mp4'
            dest=stage/relative;dest.parent.mkdir(parents=True)
            try:os.link(clip,dest)
            except OSError:shutil.copy2(clip,dest)
            write(dest.parent/'recording.json',dict(frames=frames,duration_seconds=audit['video']['duration_s']))
            expected=sorted(int(i) for i in score['records'])
            verified=sorted(int(i) for i,r in score['records'].items() if r['verified_s'] is not None)
            write(dest.parents[1]/'evaluation.json',dict(success=score['strict'],elapsed_seconds=score['elapsed_s'],
                  targets=score['registered'],collisions=score['bumps'],reason=score['stop_reason'],
                  policy_training_sample_valid=score['valid'],
                  ground_truth_audit=dict(closest_approach_m=metrics['min_ground_truth_m'],
                                          expected_ids=expected,verified_after_registration_ids=verified)))
            media.append(dict(name=name,seed=row['seed'],role=row['role'],asset=root.name+'__'+name+'.mp4',
                              relative=str(relative),sha256=row['evidence']['mission.mp4'],bytes=dest.stat().st_size))
        write(stage/'media.json',media)
        stage.rename(destination)
    return dict(episodes=len(rows),valid_pairs=result['valid_pairs'],bundle=str(destination))


if __name__=='__main__':
    cli=argparse.ArgumentParser(description=__doc__)
    for name in ('exam','recordings','destination'):cli.add_argument(name,type=Path)
    args=cli.parse_args();print(json.dumps(export(args.exam,args.recordings,args.destination)))
