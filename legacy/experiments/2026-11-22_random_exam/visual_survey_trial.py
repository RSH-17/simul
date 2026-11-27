"""Evaluate Visual Survey on the existing fixed-desk worlds and photographed starting poses."""
import argparse
from contextlib import ExitStack
import fcntl
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import sys


def read(p):
    return json.loads(p.read_text())


def save(path, obj):
    tmp=path.with_suffix('.next')
    tmp.write_text(json.dumps(obj,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
    tmp.replace(path)


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def native(exam):
    sys.path.insert(0,str(exam))
    spec=importlib.util.spec_from_file_location('frozen_native_exam',exam/'campaign.py')
    engine=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(engine)
    return engine


def enroll(root, official, alternate, ws, m):
    if root.exists():
        raise FileExistsError(root)
    original=read(official/'protocol.json')
    alt=read(alternate/'protocol.json')
    if len(original['worlds'])!=5 or len(alt['worlds'])!=1:
        raise ValueError('Expected five fixed layouts and one prespecified alternative starting pose')
    fixtures=[(official,i) for i in (1,2,3,4,0)] + [(alternate,0)]
    cases=[]
    for i,(parent,index) in enumerate(fixtures):
        world=read(parent/'protocol.json')['worlds'][index]
        m.paired.checked_files(parent/'fixtures'/f"map-{world['seed']}",world)
        cases.append(dict(number=i+1,panel=str(parent),world=world,source_index=index,
                          purpose='development' if parent==official and index<2 else
                                  'untuned' if parent==official else 'alternate_pose'))
    seed=cases[0]['world']['seed']
    if (seed!=1817236217 or cases[-1]['world']['files']['world/osp_studio.sdf']!=cases[1]['world']['files']['world/osp_studio.sdf']
            or cases[-1]['world']['start']==cases[1]['world']['start']):
        raise ValueError('Unexpected diagnostic or alternate-pose case')
    root.mkdir(parents=True)
    reg=dict(source=m.identity(ws),original_protocol=sha(official/'protocol.json'),
             alternate_protocol=sha(alternate/'protocol.json'),cases=cases,runner_sha256=sha(Path(__file__)),
             frozen_native=m.file_identity(),known_count=3,duration_s=600,pixels=[480,480],fps=5,
             parents=['original Balanced','Balanced Print from the same world and verified initial pose'],
             gate='Enable distant hints for the scored Visual Survey panel only if the preregistered three-layout exposure screen passes. Otherwise run that feature separately on development world 2.',
             candidate_failures_retried=False)
    save(root/'registration.json',reg)
    save(root/'status.json',dict(state='enrolled',new_runs_completed=0))
    return reg


def evaluate(root, official, alternate, ws, m):
    reg=read(root/'registration.json')
    def verify():
        if (reg['source']!=m.identity(ws) or reg['runner_sha256']!=sha(Path(__file__))
                or reg['original_protocol']!=sha(official/'protocol.json')
                or reg['alternate_protocol']!=sha(alternate/'protocol.json') or reg['frozen_native']!=m.file_identity()):
            raise ValueError('Source, controls or original enrollment changed')
    verify()
    board=Path.home()/'corridor-board-73bbb05/recordings'
    states=(official/'status.json',alternate/'status.json')
    screen=official/'distant-screen/holdout.json'
    if (not all(f.is_file() and read(f)['state']=='complete' for f in states)
            or not screen.is_file() or not read(screen)['complete']):
        raise ValueError('Reference drives and the independent distant-hint screen must be finalized')
    gate=bool(read(screen)['advance'])
    cases=[dict(c,far=gate) for c in reg['cases']]
    if not gate:
        cases.insert(0, dict(reg['cases'][0],number=0,far=True,purpose='distant_development_only'))
    ledger=root/'episodes.json'
    rows=read(ledger) if ledger.is_file() else []
    m.runner.HELDOUT=()
    os.environ['OSP_PUBLIC_MARKER_COUNT']='3'
    with ExitStack() as stack:
        for filename in ('.domain.lock','.second-domain.lock'):
            slot=stack.enter_context((Path.home()/'track-recordings'/filename).open('a'))
            fcntl.flock(slot,fcntl.LOCK_EX)
        for case in cases:
            parent=Path(case['panel'])
            world=case['world']
            seed=world['seed']
            cache=parent/'fixtures'/f'map-{seed}'
            m.paired.checked_files(cache,world)
            references=[r for r in read(parent/'episodes.json') if r['seed']==seed and
                        r['pair']==(case['source_index']+1)]
            if {r['role'] for r in references}!={'baseline','candidate'}:
                raise ValueError('Missing original control pair')
            key=hashlib.sha256((sha(root/'registration.json')+f"/{case['number']}/{case['far']}").encode()).hexdigest()[:15]
            suffix=int(key,16)
            stage=root/f"stage-{suffix}"
            stage.mkdir(exist_ok=True)
            link=stage/'fixtures'
            if not link.exists():link.symlink_to(cache.parent,target_is_directory=True)
            if link.resolve()!=cache.parent.resolve():raise ValueError('Unexpected fixture link')
            save(stage/'protocol.json',dict(worlds={str(seed):world['files']['world/osp_studio.sdf']}))
            name=f'v151-survey-visualsurvey-track-m{seed}-{suffix}'
            run=board/name
            old=next((r for r in rows if r['name']==name),None)
            if old:
                if any(sha(run/k)!=v for k,v in old['evidence'].items()):raise ValueError('Changed scored evidence')
                continue
            claim=stage/'attempt.json'
            if run.exists() or claim.exists():raise FileExistsError('Unscored attempt preserved: '+name)
            if shutil.disk_usage(root).free < 3*1024**3:raise RuntimeError('Insufficient disk for a full camera episode')
            m.runner.guard()
            verify()
            with claim.open('x') as f:json.dump(dict(name=name,seed=seed,far=case['far']),f)
            save(root/'status.json',dict(state='native_episode',name=name,seed=seed,number=case['number'],
                                          purpose=case['purpose'],far=case['far'],episodes=len(rows)))
            os.environ['OSP_SURVEY_DISTANT_HINTS']='true' if case['far'] else 'false'
            original_here=m.runner.HERE
            try:
                m.runner.HERE=m.HERE
                got=m.runner.trial(stage,'visualsurvey',seed,ws)
            finally:
                m.runner.HERE=original_here
            if got!=run:raise ValueError('Unexpected recording path')
            verify()
            record=m.paired.adjudicate(run,world,reg['source'],m.measure)
            result=m.score(record,read(run/'metrics.json'),read(run/'scene.json'))
            for ref in references:
                xy,yaw=m.paired.errors(record['start_world_xyyaw'],ref['start'])
                if (ref['world_sha256']!=record['world_sha256'] or xy>m.paired.XY_TOL or yaw>m.paired.YAW_TOL
                        or not ref['score']['valid']):
                    raise ValueError('Control world, measured starting pose or prior score differs')
            if not result['valid']:raise ValueError('Unusable measurement preserved: '+name)
            rows.append(dict(name=name,role='visualsurvey',seed=seed,pair=case['number'],purpose=case['purpose'],
                far_hints=case['far'],start=record['start_world_xyyaw'],world_sha256=record['world_sha256'],
                source_sha256=record['source_manifest_sha256'],score=result,evidence=record['evidence_sha256'],
                controls={r['role']:r['name'] for r in references}))
            save(ledger,rows)
            try:m.reconcile(run,Path(os.environ['CAMERA_BENCH_CARDS']),m.paired.BOARD)
            except (KeyError,OSError,RuntimeError,ValueError) as exc:print('Display refresh:',exc,flush=True)
            print('Scored',case['purpose'],seed,result['verified'],result['strict'],flush=True)
    save(root/'status.json',dict(state='complete',episodes=len(rows),screen_passed=gate))
    return rows


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=['enroll','run'])
    for name in ('output','official','alternate','workspace','native_exam'):
        parser.add_argument(name,type=Path)
    args=parser.parse_args()
    m=native(args.native_exam.resolve())
    if args.action=='enroll':
        enroll(args.output,args.official,args.alternate,args.workspace,m)
    else:
        try:evaluate(args.output,args.official,args.alternate,args.workspace,m)
        except BaseException as exc:
            save(args.output/'status.json',dict(state='infrastructure_hold',reason=repr(exc)))
            raise
