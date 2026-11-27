"""Publish experiment summaries/videos only, in an isolated checkout.

This does not edit policy code, review code, promote a policy or control a robot.
The local timer runs only while this host's user services are available.
"""
import argparse
import collections
import datetime
import fcntl
import json
from pathlib import Path
import subprocess
import sys

START='<!-- ROBOT_TRANSFER_PROGRESS:START -->'
END='<!-- ROBOT_TRANSFER_PROGRESS:END -->'
FILES=['README.md','repository_snapshot/status.json','repository_snapshot/robot_transfer_progress.json',
       'repository_snapshot/videos.json','repository_snapshot/videos.html']
COMMIT_MESSAGE='Update native robot-transfer progress and finalized video index'


def run(args, cwd=None, timeout=120):
    return subprocess.run(args,cwd=cwd,text=True,capture_output=True,check=True,timeout=timeout).stdout


def sync_generated_checkout(checkout):
    """Recover our rejected generated commits after a concurrent main push.

    This operates only on the isolated publication checkout. Preserve a local
    recovery ref before regenerating; never discard unrelated commits or force
    push. A failed push must not permanently strand the recurring publisher.
    """
    run(['git','fetch','origin','main'],checkout)
    outgoing=run(['git','rev-list','origin/main..HEAD'],checkout).splitlines()
    for commit in outgoing:
        subject=run(['git','show','-s','--format=%s',commit],checkout).strip()
        parents=run(['git','show','-s','--format=%P',commit],checkout).split()
        changed=run(['git','diff-tree','--no-commit-id','--name-only','-r',commit],checkout).splitlines()
        if subject!=COMMIT_MESSAGE or len(parents)!=1 or not changed or set(changed)-set(FILES):
            raise RuntimeError('Unpublished non-generated commit in publication checkout; preserved')
    if outgoing:
        head=run(['git','rev-parse','HEAD'],checkout).strip()
        run(['git','update-ref','refs/osp-progress/rejected/'+head,head],checkout)
        run(['git','reset','--hard','origin/main'],checkout)
    else:
        run(['git','merge','--ff-only','origin/main'],checkout)


def service_status(unit):
    """Observe a live systemd process; a stale campaign state is insufficient."""
    try:
        result=subprocess.run(['systemctl','--user','show',unit,
            '--property=ActiveState','--property=MainPID'],capture_output=True,text=True,timeout=5)
        fields=dict(line.split('=',1) for line in result.stdout.splitlines() if '=' in line)
        pid=int(fields.get('MainPID','0'))
        active=result.returncode==0 and fields.get('ActiveState')=='active' and pid>0
        active=active and Path('/proc',str(pid)).exists()
        return dict(service_active=bool(active),main_pid=pid if active else 0)
    except (OSError,ValueError,subprocess.TimeoutExpired):
        return dict(service_active=False,main_pid=0,observation_failed=True)


def summarize(campaign):
    state=json.loads((campaign/'state.json').read_text())
    done=[j for j in state['jobs'] if j['status']=='done']
    valid=[j for j in done if not j['outcome']['infrastructure_failure']]
    by_group={}
    for job in valid:
        key='/'.join([job['phase'],job['role'],job.get('stress_profile','historical'),job['scenario']['arena']['shape']])
        row=by_group.setdefault(key,dict(completed=0,successes=0,contacts=0))
        row['completed']+=1;row['successes']+=int(job['outcome']['success']);row['contacts']+=job['outcome']['contacts']
    active=[]
    for j in state['jobs']:
        if j['status']!='running':continue
        observed=service_status(j['unit'])
        row=dict(number=j['number'],unit=j['unit'],**observed,
                 shape=j['scenario']['arena']['shape'],profile=j.get('stress_profile'),
                 gates={k:v for k,v in j.items() if k.endswith('_gate')})
        p=Path(j['output'])/'episodes'/str(j['scenario']['seed'])/'live_status.json'
        if p.exists():
            live=json.loads(p.read_text()).get('status',{})
            row['live']={k:live.get(k) for k in ('state','targets','collisions','elapsed_seconds','readiness_reason')}
        active.append(row)
    coordinator=service_status(state['coordinator_unit'])
    return dict(campaign=campaign.name,status=state['status'],coordinator_active=coordinator['service_active'],
        scope=state.get('scope'),
        generation=state['generation'],phase=state['phase'],recorded_completed=len(done),
        valid_completed=len(valid),successes=sum(j['outcome']['success'] for j in valid),
        infrastructure_failures=len(done)-len(valid),by_phase_role_profile_shape=by_group,active=active,
        evidence_type='native_gazebo_training_and_paired_development_not_hardware',
        production_promoted=False,hardware_validated=False)


def followed_summary(campaign, follow_campaigns=()):
    """Keep separate results for explicitly allowed campaigns, prioritizing live jobs.

    Never scan arbitrary directories or infer success from an active coordinator.
    Prepared trials and short departure screens do not become full-task passes.
    """
    paths=list(dict.fromkeys([*(Path(p).resolve() for p in follow_campaigns),campaign.resolve()]))
    summaries=[];unavailable=[]
    for path in paths:
        try:
            summaries.append(summarize(path))
        except (OSError,ValueError,KeyError,TypeError) as error:
            unavailable.append(dict(campaign=path.name,error=type(error).__name__))
    if not summaries:raise RuntimeError('No readable campaign states; preserving published report')
    live=[s for s in summaries if any(j['service_active'] for j in s['active'])]
    coordinators=[s for s in summaries if s['coordinator_active']]
    fallback=next((s for s in summaries if s['campaign']==campaign.name),summaries[0])
    selected=(live or coordinators or [fallback])[0]
    result=dict(selected)
    if follow_campaigns:
        result['campaign_selection']=dict(
            reason='live_native_process' if live else 'live_coordinator' if coordinators else 'fallback',
            live_native_campaigns=[s['campaign'] for s in live],unavailable=unavailable,
            policy='Explicit allowlist; live native MainPID first; keep campaign results separate')
        result['followed_campaigns']=summaries
    return result


def snapshot(source, checkout, campaign, follow_campaigns=()):
    summary=followed_summary(campaign,follow_campaigns)
    summary['captured_utc']=datetime.datetime.now(datetime.timezone.utc).isoformat()
    summary['synthetic_stress_is_real_robot_calibration']=False
    summary['automation_scope']='local systemd timer: summary and finished-video publication only'
    path=checkout/'repository_snapshot/robot_transfer_progress.json'
    path.write_text(json.dumps(summary,indent=2)+'\n')
    status_path=checkout/'repository_snapshot/status.json'
    status=json.loads(status_path.read_text())
    status['robot_transfer_progress']='repository_snapshot/robot_transfer_progress.json'
    status['robot_transfer_training_observation']=summary
    official=[s for s in summary.get('followed_campaigns',[summary]) if s['campaign'].startswith(('official_','frontier_coverage_'))]
    if official:
        live=[s['campaign'] for s in official if any(j['service_active'] for j in s['active'])]
        status['active_tb4_campaigns']=live
        status['active_tb4_campaign']=live[0] if live else None
    status_path.write_text(json.dumps(status,indent=2)+'\n')
    readme=checkout/'README.md';text=readme.read_text()
    block='\n'.join([START,'',
        f"실물 대응 후보 `{summary['campaign']}`: 상태 `{summary['status']}`, 세대 {summary['generation']}, 단계 `{summary['phase']}`.",
        f"독립 입력 검사를 통과한 native 개발 회차 {summary['valid_completed']}개 중 엄격한 완주 {summary['successes']}개; 인프라 실패 {summary['infrastructure_failures']}개 별도.",
        '형태·잡음·후보/기준별 결과는 [진행 기록](repository_snapshot/robot_transfer_progress.json)에 분리합니다. 출발 진단·전체 과제·계수 학습은 각각의 scope/phase를 확인해야 하며 실물 검증·production 승격 결과가 아닙니다.',
        f"관측 UTC: {summary['captured_utc']}. 로컬 게시 작업은 요약·완성 영상만 갱신하며 자동 코드 수정/리뷰를 수행하지 않습니다.",'',END])
    if START in text and END in text:
        a=text.index(START);b=text.index(END,a)+len(END);text=text[:a]+block+text[b:]
    else:text+='\n'+block+'\n'
    readme.write_text(text)
    return summary


def publication_remote(source):
    """Push to the user's own repository (remote `mine`); see AGENTS.md."""
    for name in ('mine','origin'):
        try:return run(['git','remote','get-url',name],source).strip()
        except subprocess.CalledProcessError:continue
    raise RuntimeError('No publication remote in the source checkout')


def publish(source, checkout, campaign, videos, follow_campaigns=()):
    checkout.parent.mkdir(parents=True,exist_ok=True)
    remote=publication_remote(source)
    if not (checkout/'.git').exists():
        run(['git','clone','--quiet','--branch','main','--single-branch',remote,str(checkout)],timeout=180)
    # An older publication checkout may still point at the upstream repository.
    run(['git','remote','set-url','origin',remote],checkout)
    # Only our fixed generated-file list may be dirty from an interrupted run.
    changed=run(['git','diff','--name-only','HEAD'],checkout).splitlines()
    if set(changed)-set(FILES):raise RuntimeError('Unexpected edits in publication checkout')
    if changed:run(['git','restore','--source=HEAD','--staged','--worktree','--',*changed],checkout)
    sync_generated_checkout(checkout)
    if videos:
        run([sys.executable,str(checkout/'scripts/publish_simulation_videos.py'),
             '--data-root',str(source),'--upload'],cwd=checkout,timeout=1800)
    snapshot(source,checkout,campaign,follow_campaigns)
    run(['git','add','--',*FILES],checkout)
    if not run(['git','diff','--cached','--name-only'],checkout).strip():return
    for field in ('user.name','user.email'):
        run(['git','config',field,run(['git','config',field],source).strip()],checkout)
    run(['git','commit','-m',COMMIT_MESSAGE],checkout)
    # Normal push rejects concurrent main changes. Reconcile on the next tick;
    # never force-push or alter the user's working checkout.
    run(['git','push','origin','HEAD:main'],checkout,timeout=180)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',type=Path,default=Path(__file__).resolve().parents[1])
    p.add_argument('--campaign',type=Path,required=True)
    p.add_argument('--follow-campaign',type=Path,action='append',default=[],
        help='Allowed successor campaign, repeatable; live native processes take priority')
    p.add_argument('--checkout',type=Path)
    p.add_argument('--publish',action='store_true');p.add_argument('--videos',action='store_true')
    a=p.parse_args();source=a.source.resolve();campaign=a.campaign.resolve()
    lock_path=source/'deployment/training_progress.lock';lock_path.parent.mkdir(exist_ok=True)
    with lock_path.open('w') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        if a.publish:
            if a.checkout is None:p.error('--publish requires an isolated --checkout')
            if a.checkout.resolve()==source:p.error('Publication checkout must be isolated')
            publish(source,a.checkout.resolve(),campaign,a.videos,a.follow_campaign)
        else:print(json.dumps(snapshot(source,source,campaign,a.follow_campaign)))
