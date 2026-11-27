"""Temporary-file/fake-native tests. No ROS, network, Gazebo or robot commands."""
import json
import math
import os
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import paired as p


def write(f,data):
    f.parent.mkdir(parents=True,exist_ok=True)
    f.write_text(json.dumps(data))


class TestExamPairs(unittest.TestCase):
    def setUp(self):
        temp=TemporaryDirectory();self.addCleanup(temp.cleanup)
        self.root=Path(temp.name)/'exam';self.home=Path(temp.name)/'home'
        self.cards=self.home/'board';self.cards.mkdir(parents=True)
        (self.home/'track-recordings').mkdir()
        self.records=self.home/'corridor-board-73bbb05/recordings'
        self.records.mkdir(parents=True)
        self.manifest={n:{'workspace':'/fake/'+n,'source_manifest_sha256':p.fingerprint({'native':n})}
                       for n in ('baseline','R3','R4')}
        worlds=[]
        for seed in (p.ANCHOR,20):
            path=self.root/'fixtures'/f'map-{seed}'
            start=[1.,2.,math.pi-.001]
            for rel in p.REQUIRED:write(path/rel,{})
            write(path/'scene.yaml',{'start':start})
            write(path/'scene.json',{'targets':[{'id':7}]})
            worlds.append(dict(seed=seed,start=start,boxes=4,files={k:p.sha(path/k) for k in p.REQUIRED}))
        self.protocol=dict(domain=174,physical_drive=False,duration_s=600,camera=[480,480,5],
                           anchor_seed=p.ANCHOR,nonce='0123456789abcdef',policies=self.manifest,
                           worlds=worlds,plan=[
                               dict(pair=1,seed=p.ANCHOR,repeat=0,order=['baseline','candidate']),
                               dict(pair=2,seed=20,repeat=0,order=['candidate','baseline']),
                               dict(pair=3,seed=p.ANCHOR,repeat=1,order=['candidate','baseline'])])
        self.protocol['execution']={f.name:p.sha(f) for f in p.execution_files()}
        write(self.root/'enrollment.json',{k:self.protocol[k] for k in ('execution','policies','nonce')})
        write(self.root/'protocol.json',self.protocol)
        self.calls=[]
        self.runner=SimpleNamespace(HOME=self.home,HELDOUT=(1,2),guard=Mock(),trial=self.fixture_trial)
        self.env=patch.dict(os.environ,CAMERA_BENCH_CARDS=str(self.cards))
        self.env.start();self.addCleanup(self.env.stop)
        self.native=patch.object(p,'native_tools',return_value=(
            self.runner,lambda ws:self.manifest[ws.name],self.measure,lambda *args:'current'))
        self.native.start();self.addCleanup(self.native.stop)
        self.free=patch.object(p.shutil,'disk_usage',return_value=SimpleNamespace(free=8*1024**3))
        self.free.start();self.addCleanup(self.free.stop)

    def fixture_trial(self,stage,role,seed,ws):
        self.calls.append((role,seed,ws.name))
        run=self.records/f'v151-survey-{role}-track-m{seed}-{stage.name.split("-")[-1]}'
        run.mkdir()
        for rel in p.REQUIRED:
            path=run/rel;path.parent.mkdir(parents=True,exist_ok=True)
            path.symlink_to(stage/'fixtures'/f'map-{seed}'/rel)
        write(run/'policy_manifest.json',{'native':ws.name})
        (run/'headings.csv').write_text('elapsed_sim_s,x,y,yaw_rad\n-.02,0,0,0\n.01,1.,2.,-3.141\n')
        (run/'trajectory.csv').write_text('elapsed_sim_s,registered_ids\n-2,50\n5,7\n601,800\n')
        (run/'events.jsonl').write_text('{"event":"ids","sim":4,"ids":[33]}\n{"event":"start_ack","sim":5,"ids":[]}\n')
        write(run/'metrics.json',dict(elapsed_wall_s=3,elapsed_sim_s=4,final_ids=[7]))
        (run/'mission.mp4').write_bytes(b'test double for audited video')
        write(run/'audit.json',{'video':{'sha256':p.sha(run/'mission.mp4')}})
        return run

    @staticmethod
    def measure(r,w):
        return dict(valid=True,strict=True,elapsed_s=4.,world_sha256=w['files']['world/osp_studio.sdf'],
                    after_registration_near_s={7:2.},unexpected_ids=[],bumps=0,map_errors_m={7:.1})

    def test_anchor_reuse_never_counts_repeats_as_independent_or_restarts(self):
        a=p.native_run(self.root,'R3',True)
        self.assertEqual((a['valid_pairs'],a['distinct_completed_layouts'],a['conclusion']),(2,1,'undetermined'))
        self.assertEqual(self.calls,[('baseline',p.ANCHOR,'baseline'),('candidate',p.ANCHOR,'R3'),
                                     ('candidate',p.ANCHOR,'R3'),('baseline',p.ANCHOR,'baseline')])
        p.native_run(self.root,'R3',True);b=p.native_run(self.root,'R3')
        self.assertEqual(len(self.calls),6)
        self.assertTrue(b['complete_valid']);self.assertEqual(b['distinct_completed_layouts'],2)
        self.assertEqual(self.runner.HELDOUT,(1,2))
        p.native_run(self.root,'R4',True)
        self.assertEqual(len(self.calls),10)
        self.assertNotEqual(p.tag(self.protocol,'R3',1),p.tag(self.protocol,'R4',1))

    def test_during_failure_records_once_and_will_not_restart(self):
        self.runner.trial=Mock(side_effect=RuntimeError('observer stopped'))
        with self.assertRaisesRegex(RuntimeError,'observer stopped'):p.native_run(self.root,'R3',True)
        with self.assertRaisesRegex(RuntimeError,'Incomplete attempt'):p.native_run(self.root,'R3',True)
        self.assertEqual(self.runner.trial.call_count,1)
        row=p.read(self.root/'R3/episodes.json')[0]
        self.assertEqual(row['state'],'incomplete')
        self.assertEqual(p.read(self.root/'R3/result.json')['conclusion'],'undetermined')

    def test_abandoned_claim_blocks_launch_and_evidence_tampering_blocks_reuse(self):
        session=self.root/'R3';session.mkdir();p.ledger(session,self.protocol,'R3')
        stage=session/f'pair-{p.tag(self.protocol,"R3",1)}';stage.mkdir()
        claim=stage/'baseline.attempt.json';claim.write_text('{}')
        with self.assertRaisesRegex(FileExistsError,'preserved'):p.native_run(self.root,'R3',True)
        self.assertEqual(self.calls,[])
        p.native_run(self.root,'R4',True)
        old=p.read(self.root/'R4/episodes.json')[0]
        (Path(old['run'])/'metrics.json').write_text('{}')
        with self.assertRaisesRegex(ValueError,'evidence changed'):p.native_run(self.root,'R4',True)
        self.assertEqual(len(self.calls),4)

    def test_changed_enrollment_changed_fixture_and_physical_settings_fail_before_dispatch(self):
        p.native_run(self.root,'R4',True)
        altered={**self.protocol,'nonce':'abcd0000'}
        write(self.root/'protocol.json',altered)
        with self.assertRaisesRegex(ValueError,'Cannot reuse'):p.native_run(self.root,'R4',True)
        write(self.root/'protocol.json',self.protocol)
        (self.root/'fixtures'/f'map-{p.ANCHOR}'/'world/osp_studio.sdf').write_bytes(b'drift')
        with self.assertRaisesRegex(ValueError,'fixture'):p.native_run(self.root,'R4',True)
        for change in ({'physical_drive':True},{'domain':4},{'camera':[720,720,5]}):
            with self.assertRaises(ValueError):p.plan({**self.protocol,**change},'R3')
        self.assertEqual(len(self.calls),4)

    def test_first_pose_direction_and_transient_after_start_false_id(self):
        p.native_run(self.root,'R3',True)
        rows=p.read(self.root/'R3/episodes.json');old=rows[0];run=Path(old['run'])
        self.assertEqual(old['false_registered_ids_after_start'],[])
        self.assertLess(old['start_yaw_error_rad'],.01)
        with (run/'events.jsonl').open('a') as f:f.write('{"event":"ids","sim":20,"ids":[7,1023]}\n')
        v=p.adjudicate(run,self.protocol['worlds'][0],self.manifest['baseline'],self.measure)
        self.assertTrue(v['valid']);self.assertFalse(v['strict'])
        self.assertEqual(v['false_registered_ids_after_start'],[1023])
        (run/'headings.csv').write_text('elapsed_sim_s,x,y,yaw_rad\n0.01,1,2,0\n')
        v=p.adjudicate(run,self.protocol['worlds'][0],self.manifest['baseline'],self.measure)
        self.assertFalse(v['valid']);self.assertFalse(v['strict'])
        rows[1]['start_world_xyyaw']=[1,2,0.]
        result=p.summarize(self.protocol,'R3',rows)
        self.assertFalse(result['pairs'][0]['valid'])
        self.assertEqual(result['conclusion'],'undetermined')


if __name__=='__main__':unittest.main()
