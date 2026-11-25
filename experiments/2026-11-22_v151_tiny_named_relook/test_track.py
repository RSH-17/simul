import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import r4_track as r
from track_screen import ORDER, accepted_glances, common_maps, judge, predecessor


def leg(valid=True,strict=False):
    return dict(valid=valid,strict=strict,world_sha256='sha',configured_start=[0,0,.1],settled_xy=[0,0],
                wall_ids=[7],box_ids=[32,103],wall_reg=1,box_reg=1,wall_near=1,box_near=1,bumps=0,
                unexpected_ids=[],elapsed_s=60,near_ids=[7,32])


def pairs():
    return [dict(seed=s,baseline=leg(),candidate=leg(strict=i==0)) for i,s in enumerate(ORDER)]


class TrackQA(unittest.TestCase):
    def test_two_cases_require_a_real_recovery_and_no_bump_or_false_id(self):
        rows=pairs()[:2]
        self.assertEqual(judge(rows,False)['state'],'passed')
        for key,val in [('bumps',1),('unexpected_ids',[77]),('strict',False)]:
            rows=pairs()[:2];rows[0]['candidate'][key]=val
            self.assertEqual(judge(rows,False)['state'],'declined')
        rows=pairs()[:2];rows[0]['baseline']['strict']=True
        self.assertEqual(judge(rows,False)['state'],'declined')

    def test_compact_maps_inadmissible_and_no_per_type_regression(self):
        rows=pairs()
        self.assertEqual(judge(rows,True)['state'],'passed')
        rows[-1]['seed']=112802
        self.assertEqual(judge(rows,True)['state'],'hold')
        rows=pairs();rows[1]['candidate']['wall_near']=0
        self.assertEqual(judge(rows,True)['state'],'declined')

    def test_different_world_or_start_or_invalid_is_infrastructure(self):
        for key,value in [('valid',False),('configured_start',[0,0,1]),('settled_xy',[1,0]),('world_sha256','different')]:
            rows=pairs();rows[0]['candidate'][key]=value
            self.assertEqual(judge(rows,True)['state'],'hold')

    def test_operator_decides_after_engineering_queue_finishes(self):
        self.assertEqual(predecessor('skipped_r2_eligible',True,False,False),'ready')
        self.assertEqual(predecessor('complete',True,False,True),'ready')
        self.assertEqual(predecessor('futility_stop',True,True,False),'ready')
        self.assertEqual(predecessor('needs_investigation',True,True,True),'infrastructure_hold')
        self.assertNotEqual(predecessor('running_r3',True,False,False),'ready')

    def test_two_locks_and_no_duplicate_attempt(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp)
            with r.native_locks(root),self.assertRaises(BlockingIOError),r.native_locks(root):
                self.fail('duplicate native owner')
            with r.native_locks(root):
                r.claim(root,112605)
            with self.assertRaises(FileExistsError):
                r.claim(root,112605)

    def test_common_map_telemetry_and_false_association(self):
        result=accepted_glances(['Accepted subthreshold print 103 glance -6.0 deg',
                'Accepted tiny edge glance 6.0 deg','Rejected subthreshold print 7 glance -6.0 deg'])
        self.assertEqual(result,{'accepted_named':[{'id':103,'turn_deg':-6.0}],'accepted_anonymous_count':1})
        rows=common_maps({112601:leg(strict=True),112801:leg()},{112601:leg(),112605:leg(),112801:leg()})
        self.assertEqual(len(rows),1)
        self.assertEqual(rows[0]['capped_time_difference_s'],540.)

    def test_only_reviewed_artifacts_are_admitted(self):
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'approval.json'
            sample=dict(workspace='/stage',jazzy={'path':'j'},replay={'path':'r'},jazzy_passed=False,replay_passed=False)
            path.write_text(json.dumps(sample))
            with patch.object(r,'receipt',return_value=dict(sample)):
                with self.assertRaisesRegex(ValueError,'Missing reviewed'):
                    r.check_receipt(path,r.digest(path))
                with self.assertRaisesRegex(ValueError,'SHA differs'):
                    r.check_receipt(path,'changed')


class NativeDriverWithoutMotion(unittest.TestCase):
    def exercise(self, plan, insufficient=False, interrupt=False):
        with tempfile.TemporaryDirectory() as tmp:
            home=Path(tmp); archive=home/'archive'; archive.mkdir(); runs=home/'native'; runs.mkdir()
            (home/'track-recordings').mkdir(); root=home/'job-112204'
            evidence=archive/'archived'; evidence.mkdir()
            for filename in ('metrics.json','audit.json','events.jsonl','trajectory.csv'):
                (evidence/filename).write_text('diagnostic')
            calls=[]; stage=home/'stage'
            def native(folder,role,seed,ws):
                calls.append(seed)
                if interrupt:
                    raise RuntimeError('native recording failed')
                output=runs/f'v151-survey-candidate-track-m{seed}-112204';output.mkdir();return output
            def finished(folder,approval,hashes):
                processed=[p for p in plan if p['seed'] in calls]
                return dict(pairs=processed,first_two=judge(processed[:2],False) if len(processed)>=2 else None,
                            five_track=judge(processed,True) if len(processed)==5 else None)
            from contextlib import ExitStack
            with ExitStack() as stack:
                for name,value in [('HOME',home),('ARCHIVE',archive),('ROOT',root),('R2',home/'r2'),('RECORDINGS',runs)]:
                    stack.enter_context(patch.object(r,name,value))
                for name,value in [('fixture_hashes',{str(s):{'world/osp_studio.sdf':'sha'} for s in ORDER}),
                        ('archived_paths',{s:(evidence,{}) for s in ORDER}),('predecessor_state','ready'),('same_identity',True)]:
                    stack.enter_context(patch.object(r,name,return_value=value))
                stack.enter_context(patch.object(r,'report',side_effect=finished))
                stack.enter_context(patch.dict(sys.modules,{'run':SimpleNamespace(guard=Mock(),trial=native,analyze=lambda p:{'complete':False}),
                    'supervise':SimpleNamespace(disk_budget=lambda n:10**30 if insufficient else 0),
                    'board_audit':SimpleNamespace(reconcile=lambda *a:'current')}))
                stack.enter_context(patch.dict(r.os.environ,{'CAMERA_BENCH_CARDS':str(home)}))
                args=({'workspace':str(stage)},'sha')
                if interrupt:
                    with self.assertRaisesRegex(RuntimeError,'recording failed'):
                        r.launch(*args)
                    result=r.launch(*args)
                else:
                    result=r.launch(*args)
                    self.assertEqual(result,r.launch(*args))
                return result,calls

    def test_native_futility_stops_at_two_and_second_invocation_is_inert(self):
        plan=pairs();plan[0]['candidate']['strict']=False
        result,calls=self.exercise(plan)
        self.assertEqual((result['state'],calls),('screen_declined',list(ORDER[:2])))

    def test_native_five_and_never_schedules_a_sixth(self):
        result,calls=self.exercise(pairs())
        self.assertEqual((result['state'],calls),('five_track_screen_passed',list(ORDER)))

    def test_under_space_does_not_start_or_claim(self):
        result,calls=self.exercise(pairs(),insufficient=True)
        self.assertEqual((result['state'],calls),('storage_hold',[]))

    def test_infrastructure_error_is_never_retried(self):
        result,calls=self.exercise(pairs(),interrupt=True)
        self.assertEqual((result['state'],calls),('infrastructure_hold',[112601]))


if __name__=='__main__':
    unittest.main()
