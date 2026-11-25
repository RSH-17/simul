"""Run exactly three independently drawn three-box exams with two box prints and one outer-wall print."""
import argparse
from contextlib import ExitStack
import fcntl
import hashlib
from itertools import combinations
import math
import os
from pathlib import Path
import secrets
import shutil

from fixture import build, digest, read, save


def spacing(scene):
    boxes=scene['obstacles']
    marks=scene['targets']
    hosts=[t['attached_to'] for t in marks]
    outer=[h for h in hosts if h.startswith('wall_') and not h.startswith('wall_structure_')]
    box_ids={b['name'] for b in boxes}
    sectors={(b['x']>=0,b['y']>=0) for b in boxes}
    def shortest(items):
        return min(math.hypot(a['x']-b['x'],a['y']-b['y']) for a,b in combinations(items,2))
    if (len(boxes)!=3 or len(sectors)!=3 or len(marks)!=3 or len(set(hosts)&box_ids)!=2 or
            len(outer)!=1 or len({m['id'] for m in marks})!=3 or
            shortest(boxes)<.48*min(scene['room_size']) or shortest(marks)<.38*min(scene['room_size'])):
        raise ValueError('Incorrect spacing or marker mounts')
    return dict(min_box_centers_m=shortest(boxes),min_marker_centers_m=shortest(marks),outer_wall=outer[0])


def prepare(root,workspace,engine):
    root.mkdir(exist_ok=False)
    fixtures=root/'fixtures'
    fixtures.mkdir()
    reg=dict(nonce=secrets.token_hex(32),source=engine.identity(workspace),known_count=3,camera=[480,480,5],
             duration_s=600,study_files=engine.file_identity(),driver_sha256=digest(Path(__file__)),physical_drive=False)
    enrolled=read(Path(__file__).with_name('focus-enrollment.json'))
    if reg['source']['source_manifest_sha256']!=enrolled['source_manifest_sha256']:
        raise ValueError('Expected registered Focus Continuity')
    reg['seeds']=[int(hashlib.sha256(f"{reg['nonce']}/{i}".encode()).hexdigest()[:8],16) for i in range(3)]
    if len(set(reg['seeds']))!=3:
        raise ValueError('Nonunique seeds')
    save(root/'registration.json',reg)
    anchor=engine.runner.HOME/'track-recordings/.layouts/uFjtqfyJ'
    worlds=[]
    for seed in reg['seeds']:
        directory=fixtures/f'map-{seed}'
        world=build(anchor,directory,seed,obstacle_set='spread_three')
        world['spacing']=spacing(read(directory/'scene.json'))
        worlds.append(world)
    save(root/'protocol.json',dict(registration_sha256=digest(root/'registration.json'),worlds=worlds))
    save(root/'status.json',dict(state='enrolled',episodes=0,limit=3))


def run(root,workspace,engine):
    protocol=read(root/'protocol.json')
    reg=read(root/'registration.json')
    if len(protocol['worlds'])!=3 or [w['seed'] for w in protocol['worlds']]!=reg['seeds']:
        raise ValueError('Exactly three registered layouts required')
    ledger=root/'episodes.json'
    rows=read(ledger) if ledger.exists() else []
    os.environ.update(OSP_PUBLIC_MARKER_COUNT='3',OSP_SURVEY_DISTANT_HINTS='false')
    engine.runner.HELDOUT=()
    with ExitStack() as stack:
        for filename in ('.domain.lock','.second-domain.lock'):
            lock=stack.enter_context((engine.runner.HOME/'track-recordings'/filename).open('a'))
            save(root/'status.json',dict(state='waiting_for_native_simulator',episodes=len(rows),limit=3))
            fcntl.flock(lock,fcntl.LOCK_EX)
        for i,world in enumerate(protocol['worlds'],1):
            seed=world['seed']
            if (reg['source']!=engine.identity(workspace) or reg['study_files']!=engine.file_identity() or
                    digest(root/'registration.json')!=protocol['registration_sha256'] or digest(Path(__file__))!=reg['driver_sha256']):
                raise ValueError('Study inputs changed')
            engine.paired.checked_files(root/'fixtures'/f'map-{seed}',world)
            if i<=len(rows):
                row=rows[i-1]
                folder=engine.runner.HOME/'corridor-board-73bbb05/recordings'/row['name']
                if row['seed']!=seed or any(digest(folder/k)!=v for k,v in row['evidence'].items()):
                    raise ValueError('Changed result')
                continue
            suffix=int(hashlib.sha256((reg['nonce']+str(i)).encode()).hexdigest()[:15],16)
            stage=root/f'trial-{suffix}'
            stage.mkdir()
            (stage/'fixtures').symlink_to(root/'fixtures',target_is_directory=True)
            save(stage/'protocol.json',dict(worlds={str(seed):world['files']['world/osp_studio.sdf']}))
            engine.runner.guard()
            if shutil.disk_usage(root).free<3*1024**3:
                raise RuntimeError('Insufficient space for a full camera drive')
            name=f'v151-survey-focus-track-m{seed}-{suffix}'
            save(root/'status.json',dict(state='native_episode',pair=i,role='focus',seed=seed,name=name,episodes=len(rows),limit=3))
            original=engine.runner.HERE
            try:
                engine.runner.HERE=engine.HERE
                folder=engine.runner.trial(stage,'focus',seed,workspace)
            finally:
                engine.runner.HERE=original
            if folder.name!=name:
                raise ValueError('Unexpected native recording')
            result=engine.paired.adjudicate(folder,world,reg['source'],engine.measure)
            score=engine.score(result,read(folder/'metrics.json'),read(folder/'scene.json'))
            row=dict(pair=i,role='focus',name=name,seed=seed,score=score,start=result['start_world_xyyaw'],
                     world_sha256=result['world_sha256'],source_sha256=result['source_manifest_sha256'],
                     evidence=result['evidence_sha256'])
            rows.append(row)
            save(ledger,rows)
            save(root/'comparison.json',engine.totals(rows))
            from board_results import apply
            try:
                cards=Path(os.environ['CAMERA_BENCH_CARDS'])
                engine.reconcile(folder,cards,engine.paired.BOARD)
                apply(cards/name,row,'Mixed v1.5.2.1 · 분산 상자 3개')
            except (OSError,ValueError,KeyError,RuntimeError) as error:
                print('Display:',error,flush=True)
    save(root/'status.json',dict(state='complete',episodes=len(rows),limit=3,
                                valid=sum(r['score']['valid'] for r in rows)))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=('prepare','run'))
    parser.add_argument('root',type=Path)
    parser.add_argument('workspace',type=Path)
    args=parser.parse_args()
    from visual_survey_trial import native
    engine=native(Path(__file__).resolve().parent)
    if args.action=='prepare':
        prepare(args.root,args.workspace,engine)
    else:
        with (args.root/'owner.lock').open('a') as owner:
            fcntl.flock(owner,fcntl.LOCK_EX|fcntl.LOCK_NB)
            try:
                run(args.root,args.workspace,engine)
            except Exception as exc:
                save(args.root/'status.json',dict(state='infrastructure_hold',error=repr(exc)))
                raise
