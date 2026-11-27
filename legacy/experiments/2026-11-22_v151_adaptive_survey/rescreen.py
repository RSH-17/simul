"""Conditional, new-revision two-case futility screen; never starts blind trials."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import shutil
import sys

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE/'validation'))
from decisions import screen


def read(path):
    return json.loads(path.read_text())



def early_screen(rows):
    required={112601,112605}
    if len(rows)!=2 or {r['seed'] for r in rows}!=required:
        raise ValueError('Two prespecified completed diagnostic pairs are required')
    reasons=[]
    if any(row['candidate']['bumps'] or row['candidate']['unexpected_ids'] or
           row['candidate'].get('unexpected_ids_ever') for row in rows):
        reasons.append('bumper contact or false registered ID')
    if not any(row['candidate']['strict'] for row in rows):
        reasons.append('neither original strict failure was solved')
    return dict(eligible_to_continue=not reasons,reasons=reasons)


def frozen_world_and_pose(root,protocol):
    from run import digest
    results={}
    for seed,sha in protocol['worlds'].items():
        fixture=root/'fixtures'/f'map-{seed}'
        if sha!=digest(fixture/'world/osp_studio.sdf'):
            raise ValueError('Parent world changed on '+str(seed))
        results[seed]={name:digest(fixture/name) for name in ('world/osp_studio.sdf','scene.yaml','scene.json')}
    return results


def parent_gate(parent):
    from compare import report
    if not (parent/'candidate/complete.json').exists():
        raise ValueError('Prior seven native cases are unfinished')
    d=report(parent)
    if not d['complete']:
        raise ValueError('Prior comparison incomplete')
    g=screen(d)
    if g['eligible']:
        raise ValueError('Prior revision passed its screen; conditional next revision is ineligible')
    manifests=[read(Path(name)/'policy_manifest.json') for name in read(parent/'candidate/runs.json')]
    if len(manifests)!=7 or any(m!=manifests[0] for m in manifests):
        raise ValueError('Prior native run fingerprints differ')
    return g,manifests[0]


def main(args):
    import run as runner
    from compare import report
    from validate import identity
    from board_audit import reconcile
    root,parent,ws=args.root.resolve(),args.parent.resolve(),args.workspace.resolve()
    token=root.name.rsplit('-',1)[-1]
    if token==parent.name.rsplit('-',1)[-1] or not token.isdecimal():
        raise ValueError('Supply a fresh numeric card suffix distinct from the previous experiment')
    locks=[(Path.home()/'track-recordings'/n).open('a') for n in ('.domain.lock','.second-domain.lock')]
    for lock in locks:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    runner.guard()
    gate,prior_source=parent_gate(parent)
    old=read(parent/'protocol.json')
    files=frozen_world_and_pose(parent,old)
    child=read(ws/'followthrough_manifest.json')
    if child.get('version')!='1.5.1-object-followthrough':
        raise ValueError('Expected separately staged object-followthrough revision')
    source=identity(ws)
    for filename,sha in child['child_modules'].items():
        if runner.digest(ws/'src/osp_mission/osp_mission'/filename)!=sha:
            raise ValueError('Child policy source changed: '+filename)
    if runner.digest(ws/'adaptive_survey_manifest.json')!=child['parent_manifest_sha256']:
        raise ValueError('Wrong parent stage for new revision')
    original_parent=read(ws/'adaptive_survey_manifest.json')
    if any(prior_source.get('osp_mission/'+name)!=sha for name,sha in original_parent['changed_source_sha256'].items()):
        raise ValueError('Previous screen used a different parent policy')
    old_base_files=[]
    for name in old['archived_runs']+read(parent/'baseline/runs.json'):
        run=Path(name)
        old_base_files.append({key:runner.digest(run/key) for key in ('scene.yaml','metrics.json','audit.json')})
    registration=dict(prior_screen=str(parent),prior_gate=gate,source=source,fixture_files=files,
       baseline_evaluations_sha256=old_base_files,child_manifest=child,
       order=[112601,112605,112801,112802,112602,112603,112604],
       classification='All seven are engineering screen only. No blind outcome obtained or consumed.',
       early_stop='After both first cases, require at least one strict success and zero bumps and unexpected IDs in both.')
    if root.exists():
        if read(root/'r3_registration.json')!=registration:
            raise ValueError('Resumption requires identical registration, policies and baseline measurements')
    else:
        # Enroll only before any new outcome exists and refuse any previous dashboard ID.
        for seed in registration['order']:
            split='heldout' if seed in runner.HELDOUT else 'track'
            name=f'v151-survey-candidate-{split}-m{seed}-{token}'
            if (Path.home()/'corridor-board-73bbb05/recordings'/name).exists() or (Path.home()/'track-recordings'/name).exists():
                raise FileExistsError(name)
        root.mkdir()
        (root/'fixtures').symlink_to(parent/'fixtures',target_is_directory=True)
        (root/'baseline').symlink_to(parent/'baseline',target_is_directory=True)
        runner.write(root/'protocol.json',{**old,'split_note':registration['classification'],'revision':child['version']})
        runner.write(root/'r3_registration.json',registration)
        (root/'candidate').mkdir()
    if args.action=='prepare':
        print('READY',root,'no native runs started',flush=True)
        return
    phase=root/'candidate'
    paths=read(phase/'runs.json') if (phase/'runs.json').exists() else []
    if (root/'futility_stop.json').exists():
        print('STOPPED',read(root/'futility_stop.json'),flush=True)
        return
    def adjudicate():
        data=report(root)
        runner.write(root/'paired_comparison.json',data)
        first=[pair for pair in data['regression']['pairs'] if pair['seed'] in (112601,112605)]
        if len(first)!=2:
            return None
        for pair in first:
            trial=Path(next(name for name in paths if Path(name).name==pair['candidate']['run']))
            ids=set()
            for line in (trial/'events.jsonl').read_text().splitlines():
                event=json.loads(line)
                if event['event'] in ('ids','start_ack'):
                    ids.update(event.get('ids',[]))
            pair['candidate']['unexpected_ids_ever']=sorted(ids-set(pair['candidate']['expected_ids']))
        criterion=early_screen(first)
        runner.write(root/'first_two_gate.json',criterion)
        if not criterion['eligible_to_continue']:
            runner.write(root/'futility_stop.json',dict(complete_native_screen=False,
                finished_cases=len(paths),blind_eligible=False,**criterion))
        return criterion
    for index,seed in enumerate(registration['order']):
        if index < len(paths):
            if f'-m{seed}-' not in Path(paths[index]).name:
                raise ValueError('Saved order differs')
            continue
        if index>=2 and not adjudicate()['eligible_to_continue']:
            return
        if identity(ws)!=source or frozen_world_and_pose(parent,old)!=files:
            raise ValueError('Frozen sources or pairing changed')
        if shutil.disk_usage(root).free<3*1024**3:
            raise RuntimeError('Insufficient storage for next recording')
        runner.guard()
        completed=runner.trial(root,'candidate',seed,ws)
        paths.append(str(completed))
        runner.write(phase/'runs.json',paths)
        measured=runner.analyze(phase)
        runner.write(phase/'report.json',measured)
        try:
            reconcile(completed,Path(os.environ['CAMERA_BENCH_CARDS']),'http://192.168.252.2:8780')
        except (OSError,RuntimeError,ValueError) as exc:
            print('BOARD',str(exc),flush=True)
        row=measured['runs'][-1]
        print('DONE',seed,row['strict_saved_map_success'],row['registered_ids'],flush=True)
        criterion=adjudicate()
        if criterion is not None and not criterion['eligible_to_continue']:
            print('FUTILITY',criterion,flush=True)
            return
    runner.write(phase/'complete.json',dict(complete=True,count=len(paths)))
    runner.write(phase/'report.json',runner.analyze(phase))
    data=report(root)
    runner.write(root/'paired_comparison.json',data)
    runner.write(root/'seven_case_gate.json',screen(data))
    print('COMPLETE',root,screen(data),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('action',choices=('prepare','run'))
    p.add_argument('parent',type=Path);p.add_argument('root',type=Path);p.add_argument('workspace',type=Path)
    main(p.parse_args())
