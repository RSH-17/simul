import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import select_finalist as s
spec=importlib.util.spec_from_file_location('exam_dispatch',s.HERE/'queue.py')
queue=importlib.util.module_from_spec(spec)
spec.loader.exec_module(queue)


def leg(seed,strict=False,candidate=False):
    ids=[7,32,103] if strict else [7,32]
    return dict(run=f'{"candidate" if candidate else "base"}-{seed}',source='src',valid=True,world_sha256=str(seed),
         configured_start=[0.,0.,.3],settled_xy=[.01,0.],registered_ids=list(ids),near_ids=list(ids),
         raw_near_time_s={str(i):t for t,i in enumerate(ids,1)},strict=strict,original_strict=strict,raw_strict=strict,
         wall_ids=[7],box_ids=[32,103],wall_reg=1,box_reg=len(ids)-1,wall_near=1,box_near=len(ids)-1,
         elapsed_s=90.,unexpected_ids=[],bumps=0,run_source_sha256='src' if candidate else None)


def result(rev,win=True,early=False):
    seeds=s.ORDER[:2] if early else s.ORDER
    rows=[dict(seed=z,baseline=leg(z),candidate=leg(z,win and i==0,True)) for i,z in enumerate(seeds)]
    return s.adjudicate(rev,rows,{z:str(z) for z in seeds},'src',early)


class Selection(unittest.TestCase):
    def test_both_decline_always_four_anchor_episodes_without_adoption(self):
        decision=s.choose(result('R3',False,True),result('R4',False))
        self.assertEqual((decision['revision'],decision['mode'],decision['required_pairs'],decision['development_qualified']),
                         ('R4','anchor_only',2,False))

    def test_one_qualifies_without_auxiliary_vote(self):
        d=s.choose(result('R3'),result('R4',False,True))
        self.assertEqual((d['mode'],d['revision'],d['required_pairs']),('full','R3',9))

    def test_lexicographic_and_tie(self):
        a,b=result('R3'),result('R4')
        self.assertEqual(s.choose(a,b)['revision'],'R3')
        for key,value in [('strict',2),('verified_near',15),('capped_mean_s',1.)]:
            better=copy.deepcopy(b);better[key]=value
            self.assertEqual(s.choose(a,better)['revision'],'R4')

    def test_source_world_raw_and_membership_checked(self):
        r=result('R4')['pairs']
        worlds={z:str(z) for z in s.ORDER}
        for key,v in [('world_sha256','bad'),('run_source_sha256','bad'),('valid',False),
                      ('raw_near_time_s',{}),('wall_near',0),('configured_start',[0,0,2.])]:
            rows=copy.deepcopy(r);rows[0]['candidate'][key]=v
            with self.assertRaises(ValueError):
                s.adjudicate('R4',rows,worlds,'src',False)
        rows=copy.deepcopy(r);rows[-1]['seed']=112802
        with self.assertRaises(ValueError):
            s.adjudicate('R4',rows,worlds,'src',False)

    def test_reference_runs_must_be_identical(self):
        a,b=result('R3'),result('R4')
        b['pairs'][0]['baseline']['settled_xy']=[.02,.01]
        with self.assertRaisesRegex(ValueError,'Different baseline'):
            s.choose(a,b)

    def test_prior_valid_futility_disagrees_with_current_passing_measurement(self):
        with self.assertRaises(ValueError):
            result('R3',True,True)


def write(file,data):
    file.write_text(json.dumps(data))


class Dispatch(unittest.TestCase):
    def simulate(self,root,decision,invalid=False):
        p={'selection_files':{f.name:queue.digest(f) for f in (s.HERE/'select_finalist.py',s.HERE/'queue.py',s.R4_TOOLS/'track_screen.py')},
           'policies':{k:{'source_manifest_sha256':k} for k in ('R3','R4','baseline')}}
        write(root/'protocol.json',p)
        decision=copy.deepcopy(decision)
        decision['protocol_sha256']=queue.digest(root/'protocol.json')
        launched=[]
        def native(cmd,**ignored):
            launched.append(cmd)
            r=decision['revision'];n=decision['required_pairs'];folder=root/r;folder.mkdir()
            write(folder/'enrollment.json',dict(protocol_sha256=s.stable(p),revision=r,baseline_source='baseline',candidate_source=r))
            write(folder/'result.json',dict(revision=r,recorded_episodes=2*n,completed_pairs=n,valid_pairs=n-int(invalid),
                 complete_valid=n==9 and not invalid,valid_anchor_repetitions=2))
            return subprocess.CompletedProcess(cmd,0)
        with patch.object(queue,'select',return_value=decision),patch.object(queue,'locks_free',return_value=True), \
             patch.object(queue.subprocess,'run',side_effect=native):
            first=queue.run_queue(root,1,1);second=queue.run_queue(root,1,1)
        self.assertEqual(first,second)
        self.assertEqual(len(launched),1)
        return first,launched[0]

    def test_exact_anchor_dispatch_and_idempotent(self):
        with tempfile.TemporaryDirectory() as temp:
            d,cmd=self.simulate(Path(temp),s.choose(result('R3',False,True),result('R4',False)))
            self.assertEqual(d['state'],'completed_anchor_only')
            self.assertIn('--anchor-only',cmd)

    def test_incomplete_or_invalid_is_not_policy_loss(self):
        with tempfile.TemporaryDirectory() as temp:
            d,cmd=self.simulate(Path(temp),s.choose(result('R3'),result('R4')),True)
            self.assertEqual(d['state'],'infrastructure_hold')
            self.assertNotIn('--anchor-only',cmd)

    def test_missing_evidence_expiration_and_pin_failure_hold_without_launch(self):
        for with_protocol in (False,True):
            with tempfile.TemporaryDirectory() as temp:
                root=Path(temp)
                if with_protocol:write(root/'protocol.json',{'selection_files':{}})
                job=root/'official-queue';job.mkdir()
                write(job/'start.json',{'deadline_unix':0})
                with patch.object(queue.subprocess,'run',side_effect=AssertionError):
                    self.assertEqual(queue.run_queue(root,1,1)['state'],'infrastructure_hold')

    def test_standard_library_queue_still_loads(self):
        subprocess.run([sys.executable,'-B','-c','import queue; q=queue.Queue();q.put(3);assert q.get()==3'],
                       cwd=s.HERE,check=True)


if __name__=='__main__':
    unittest.main()
