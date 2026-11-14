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
    monkeypatch.setattr(publisher.subprocess,'run',lambda *a,**k:SimpleNamespace(returncode=0))
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
