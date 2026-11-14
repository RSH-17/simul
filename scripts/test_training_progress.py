import json
from pathlib import Path
from types import SimpleNamespace
import pytest
import publish_training_progress as publisher


def test_snapshot_separates_infrastructure_and_preserves_unrelated_readme(tmp_path,monkeypatch):
    campaign=tmp_path/'campaign';campaign.mkdir();(tmp_path/'repository_snapshot').mkdir()
    (tmp_path/'README.md').write_text('Keep this original text.\n')
    (tmp_path/'repository_snapshot/status.json').write_text('{"original_result":42}')
    jobs=[]
    for number,(success,infra) in enumerate([(True,False),(False,False),(False,True)]):
        jobs.append(dict(number=number,status='done',phase='train',role='0',stress_profile='mild',
            scenario=dict(arena=dict(shape='u_shape')),outcome=dict(success=success,infrastructure_failure=infra,contacts=0)))
    (campaign/'state.json').write_text(json.dumps(dict(status='running',generation=0,phase='train',jobs=jobs,coordinator_unit='osp-test.service')))
    monkeypatch.setattr(publisher,'service_status',lambda *a:dict(service_active=True,main_pid=1))
    a=publisher.snapshot(tmp_path,tmp_path,campaign)
    assert a['recorded_completed']==3 and a['valid_completed']==2 and a['successes']==1 and a['infrastructure_failures']==1
    assert a['hardware_validated'] is False and a['production_promoted'] is False
    assert json.loads((tmp_path/'repository_snapshot/status.json').read_text())['original_result']==42
    publisher.snapshot(tmp_path,tmp_path,campaign)
    readme=(tmp_path/'README.md').read_text()
    assert readme.startswith('Keep this original text.') and readme.count(publisher.START)==1


def diverged_checkouts(tmp_path, generated=True):
    run=publisher.run
    remote=tmp_path/'remote.git';author=tmp_path/'author';checkout=tmp_path/'publication'
    run(['git','init','--bare',str(remote)])
    run(['git','clone',str(remote),str(author)])
    run(['git','checkout','-b','main'],author)
    for field,value in [('user.name','Test'),('user.email','test@example.invalid')]:
        run(['git','config',field,value],author)
    (author/'README.md').write_text('Initial\n')
    run(['git','add','.'],author);run(['git','commit','-m','Initial'],author)
    run(['git','push','origin','main'],author)
    run(['git','clone','--branch','main',str(remote),str(checkout)])
    for field,value in [('user.name','Test'),('user.email','test@example.invalid')]:
        run(['git','config',field,value],checkout)
    (checkout/('README.md' if generated else 'policy.py')).write_text('Unpublished change\n')
    run(['git','add','.'],checkout)
    run(['git','commit','-m',publisher.COMMIT_MESSAGE],checkout)
    previous=run(['git','rev-parse','HEAD'],checkout).strip()
    (author/'README.md').write_text('New project milestone\n')
    run(['git','add','.'],author);run(['git','commit','-m','Project milestone'],author)
    run(['git','push','origin','main'],author)
    return checkout,previous


def test_rejected_generated_commit_is_preserved_and_rebased_by_regeneration(tmp_path):
    checkout,previous=diverged_checkouts(tmp_path)
    publisher.sync_generated_checkout(checkout)
    assert (checkout/'README.md').read_text()=='New project milestone\n'
    assert publisher.run(['git','show',f'refs/osp-progress/rejected/{previous}:README.md'],checkout)=='Unpublished change\n'
    assert publisher.run(['git','rev-list','origin/main..HEAD'],checkout).strip()==''


def test_unrelated_unpublished_commit_is_never_reset(tmp_path):
    checkout,previous=diverged_checkouts(tmp_path,generated=False)
    with pytest.raises(RuntimeError,match='non-generated'):
        publisher.sync_generated_checkout(checkout)
    assert publisher.run(['git','rev-parse','HEAD'],checkout).strip()==previous
    assert (checkout/'policy.py').read_text()=='Unpublished change\n'


def make_campaign(root, name, jobs=(), phase='development'):
    path=root/name;path.mkdir()
    (path/'state.json').write_text(json.dumps(dict(status='running',generation=0,
        phase=phase,jobs=list(jobs),coordinator_unit=name+'.service',scope=name)))
    return path


def running_job(root, unit):
    return dict(number=0,status='running',unit=unit,output=str(root/'empty'),
        scenario=dict(arena=dict(shape='u_shape'),seed=1))


def test_live_native_beats_stale_job_and_waiting_coordinator(tmp_path,monkeypatch):
    old=make_campaign(tmp_path,'old',[running_job(tmp_path,'old-native.service')])
    waiting=make_campaign(tmp_path,'waiting')
    new=make_campaign(tmp_path,'new',[running_job(tmp_path,'new-native.service')])
    monkeypatch.setattr(publisher,'service_status',lambda unit:dict(
        service_active=unit in {'waiting.service','new-native.service'},main_pid=42))
    report=publisher.followed_summary(old,[waiting,new])
    assert report['campaign']=='new'
    assert report['campaign_selection']['reason']=='live_native_process'
    assert report['campaign_selection']['live_native_campaigns']==['new']
    assert not report['followed_campaigns'][2]['active'][0]['service_active']


def test_parallel_campaigns_stay_separate_and_screen_is_not_full_success(tmp_path,monkeypatch):
    done=dict(status='done',phase='diagnostic',role='candidate',
        scenario=dict(arena=dict(shape='c_shape')),
        outcome=dict(success=False,start_screen_passed=True,infrastructure_failure=False,contacts=0))
    old=make_campaign(tmp_path,'old',[running_job(tmp_path,'old-native.service')])
    screen=make_campaign(tmp_path,'screen',[done,running_job(tmp_path,'screen-native.service')],phase='diagnostic')
    monkeypatch.setattr(publisher,'service_status',lambda unit:dict(service_active=True,main_pid=42))
    report=publisher.followed_summary(old,[screen])
    assert report['campaign_selection']['live_native_campaigns']==['screen','old']
    assert report['valid_completed']==1 and report['successes']==0
    assert report['followed_campaigns'][1]['valid_completed']==0
    assert report['phase']=='diagnostic' and report['production_promoted'] is False


def test_no_process_uses_fallback_and_reports_unavailable_followup(tmp_path,monkeypatch):
    old=make_campaign(tmp_path,'old');waiting=make_campaign(tmp_path,'waiting')
    monkeypatch.setattr(publisher,'service_status',lambda unit:dict(service_active=False,main_pid=0))
    report=publisher.followed_summary(old,[waiting,tmp_path/'missing'])
    assert report['campaign']=='old' and report['campaign_selection']['reason']=='fallback'
    assert report['campaign_selection']['unavailable']==[dict(campaign='missing',error='FileNotFoundError')]


@pytest.mark.parametrize('fields,expected',[
    ('ActiveState=active\nMainPID=0\n',False),
    ('ActiveState=inactive\nMainPID={pid}\n',False),
    ('ActiveState=active\nMainPID={pid}\n',True),
])
def test_service_status_requires_running_process(monkeypatch,fields,expected):
    import os
    monkeypatch.setattr(publisher.subprocess,'run',lambda *a,**k:SimpleNamespace(
        returncode=0,stdout=fields.format(pid=os.getpid())))
    assert publisher.service_status('test.service')['service_active'] is expected
