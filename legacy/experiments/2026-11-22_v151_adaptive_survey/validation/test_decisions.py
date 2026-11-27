import unittest
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
from decisions import pairs_report, screen


def comparison():
    result={'complete':True}
    for key,seeds in [('regression',range(112601,112606)),('heldout',(112801,112802))]:
        pairs=[]
        for seed in seeds:
            fields=dict(strict=True,wall_near=1,wall_reg=1,box_near=2,box_reg=2,bumps=0,unexpected_ids=[],
                        expected_ids=[7,32,103],sha256='world',configured_start=[0,0,0])
            pairs.append(dict(seed=seed,baseline=dict(fields),candidate=dict(fields),settled_start_distance_m=0.))
        result[key]=dict(pairs=pairs)
    return result


class Gate(unittest.TestCase):
    def test_equal_is_not_an_improvement(self):
        self.assertFalse(screen(comparison())['eligible'])

    def test_each_split_and_safety_have_individual_vetoes(self):
        r=comparison()
        r['regression']['pairs'][0]['baseline']['strict']=False
        self.assertTrue(screen(r)['eligible'])
        for key,value in [('box_near',1),('wall_reg',0),('bumps',1),('unexpected_ids',[22])]:
            fresh=comparison()
            fresh['regression']['pairs'][0]['baseline']['strict']=False
            fresh['heldout']['pairs'][1]['candidate'][key]=value
            self.assertFalse(screen(fresh)['eligible'])

    def test_missing_pairs_or_world_mismatch(self):
        r=comparison();r['heldout']['pairs'][0]['baseline']['strict']=False
        r['heldout']['pairs'][0]['candidate']['sha256']='other'
        self.assertFalse(screen(r)['eligible'])
        r['regression']['pairs'].pop()
        self.assertFalse(screen(r)['eligible'])

    def new_trial(self,i,role,success,valid=True):
        return dict(pair=i,role=role,strict=success,valid=valid,world_sha256='paired',settled_xy=[0,0],
                    elapsed_s=100 if success else 550)

    def suite(self,baseline,candidate):
        plan=[];rows=[]
        for i in range(9):
            plan.append(dict(pair=i,seed=i if i<6 else i-5,topology=str(i%6),repeat=int(i>=6),order=[]))
            rows.extend([self.new_trial(i,'baseline',baseline),self.new_trial(i,'candidate',candidate)])
        return plan,rows

    def test_perfect_tie_qualifies_task_but_no_statistical_winner(self):
        plan,rows=self.suite(True,True);r=pairs_report(plan,rows)
        self.assertTrue(r['complete_valid'])
        self.assertEqual(r['task_qualification'],'met_on_all_tested_runs')
        self.assertEqual(r['primary_paired_exact']['interpretation'],'undetermined')
        self.assertEqual(r['primary_paired_exact']['both_success'],6)
        self.assertAlmostEqual(r['idealized_zero_failure_95_upper_bound'],.39303776899708276)

    def test_unique_layouts_are_primary_not_repeats(self):
        plan,rows=self.suite(False,True);r=pairs_report(plan,rows)
        self.assertEqual(r['primary_paired_exact']['candidate_only'],6)
        self.assertEqual(r['primary_paired_exact']['two_sided_p'],.03125)
        self.assertEqual(r['first_run_mean_capped_difference_s'],-500.)
        self.assertEqual(r['first_run_joint_success_time_differences_s'],[])

    def test_repeat_failure_disqualifies_coverage(self):
        p,r=self.suite(False,True);r[-1]['strict']=False
        report=pairs_report(p,r)
        self.assertEqual(report['task_qualification'],'not_met')
        self.assertEqual(report['primary_paired_exact']['candidate_only'],6)

    def test_invalid_or_interrupted_are_not_counted_as_policy_losses(self):
        p,r=self.suite(True,True);r[0]['valid']=False
        report=pairs_report(p,r[:-1])
        self.assertEqual(report['task_qualification'],'undetermined')
        self.assertFalse(report['complete_valid'])
        self.assertEqual(report['primary_paired_exact']['distinct_geometries'],5)
        self.assertEqual(report['primary_paired_exact']['baseline_only'],0)


if __name__=='__main__':
    unittest.main()
