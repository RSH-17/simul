import os
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
import xml.etree.ElementTree as ET

import numpy as np

from fixture import architecture,build, original_dictionary, read, verify_prints
from geometry import sample
from scoring import score


class RandomExamTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.anchor=Path(os.environ.get('EXAM_SOURCE',str(Path.home()/'track-recordings/.layouts/uFjtqfyJ')))
        if not (cls.anchor/'world/osp_studio.sdf').is_file():
            raise unittest.SkipTest('The native exam archive is required for geometry tests')
        cls.original=read(cls.anchor/'scene.json')
        cls.tables=read(cls.anchor/'run/simulation/demo_layout.json')['tables']
        cls.dictionary=original_dictionary()

    def draw(self,seed,markers=3,furniture='fixed',stress=None):
        return sample(self.original,self.tables,seed,len(self.dictionary.bytesList),markers,furniture,stress)

    def test_independent_ids_counts_host_mix_and_same_host(self):
        counts=set();mix=set();shared=False;ids=set();yaws=set();shapes=set()
        for seed in range(34,66):
            scene,tables,w=self.draw(seed)
            self.assertEqual(tables,self.tables)
            ts=scene['targets']
            self.assertEqual(len(ts),len(set(t['id'] for t in ts)))
            self.assertEqual(len(w),len(ts))
            n=len({b['object_id'] for b in scene['obstacles']})
            self.assertLessEqual(n,9);counts.add(n)
            shapes.update(b['shape'] for b in scene['obstacles'])
            mix.add(sum(t['attached_to'].startswith('wall_') for t in ts))
            ids.update(t['id'] for t in ts);shared|=len(set(t['attached_to'] for t in ts))<len(ts)
            yaws.update(round(b['yaw'],2) for b in scene['obstacles'])
        self.assertGreater(len(counts),5);self.assertEqual(mix,{0,1,2,3})
        self.assertTrue(shared);self.assertGreater(len(ids),75);self.assertGreater(len(yaws),50)
        self.assertEqual(shapes,{'box','screen','elbow'})

    def test_repeatable_and_variable_roster(self):
        for n in (1,3,5):
            a=self.draw(1904,n);self.assertEqual(a,self.draw(1904,n))
            self.assertEqual(len(a[0]['targets']),n)
        for host in ('wall','object'):
            a=self.draw(18,3,stress=host)
            self.assertTrue(all(t['attached_to'].startswith('wall_')==(host=='wall') for t in a[0]['targets']))

    def test_sdf_uses_real_room_and_actual_black_cells(self):
        with TemporaryDirectory() as d:
            for i,mode in enumerate(('fixed','shuffle')):
                dest=Path(d)/mode
                meta=build(self.anchor,dest,89145+i,3,mode)
                actual=ET.parse(dest/'world/osp_studio.sdf').getroot()
                scene=read(dest/'scene.json');base=ET.parse(self.anchor/'world/osp_studio.sdf').getroot()
                self.assertEqual(meta['fixed_shell_sha256'],architecture(base,self.original,False))
                self.assertEqual(architecture(actual,scene,False),architecture(base,self.original,False))
                self.assertEqual(architecture(actual,scene,True)==architecture(base,self.original,True),mode=='fixed')
                base_fac=base.find("world/model[@name='demo_furniture']/link")
                actual_fac=actual.find("world/model[@name='demo_furniture']/link")
                self.assertEqual(len(base_fac.findall('collision')),len(actual_fac.findall('collision')))
                for collision in actual_fac.findall('collision'):
                    visual=actual_fac.find(f"visual[@name='{collision.get('name')}_visual']")
                    self.assertEqual(collision.findtext('pose'),visual.findtext('pose'))
                marker=actual.find(f"world/model[@name='target_{scene['targets'][0]['id']}']")
                marker.remove(next(m for m in marker.findall('model') if m.get('name').startswith('cell_')))
                with self.assertRaisesRegex(ValueError,'pattern'):verify_prints(scene,actual,self.dictionary)


class ScoreTests(unittest.TestCase):
    def case(self):
        ids=[0,702,1023,49]; registered={i:float(i%3+8) for i in ids}
        scene=dict(targets=[dict(id=i,attached_to='wall_east' if i%2 else 'movable_1') for i in ids])
        row=dict(valid=True,after_registration_near_s={i:40+i%30 for i in ids},map_errors_m={i:.12 for i in ids},
                 false_registered_ids_after_start=[],unexpected_ids=[],distance_m=89.,bumps=0)
        metrics=dict(registered_s=registered,policy_sensor_proof_complete=True,sensor_verified_at_s={i:100. for i in ids},
                     elapsed_sim_s=600.004,final_ids=ids,judge_stopped_policy_on_truth=False,reason='common_time_limit')
        return row,metrics,scene

    def test_independent_proof_before_budget_and_slow_terminal_clock(self):
        row,m,s=self.case()
        result=score(row,m,s)
        self.assertTrue(result['strict']);self.assertEqual(result['goal_complete_by_s'],100.);self.assertEqual(result['expected'],4)

    def test_late_or_missing_false_ids_and_ground_truth_stop(self):
        row,m,s=self.case()
        m['sensor_verified_at_s'][0]=601.
        self.assertFalse(score(row,m,s)['strict']);self.assertEqual(score(row,m,s)['verified'],3)
        row,m,s=self.case();row['false_registered_ids_after_start']=[113]
        self.assertFalse(score(row,m,s)['strict'])
        row,m,s=self.case();row['after_registration_near_s'][0]=1.
        self.assertIsNone(score(row,m,s)['records'][0]['verified_s'])
        m['judge_stopped_policy_on_truth']=True
        with self.assertRaisesRegex(ValueError,'passive'):score(row,m,s)


if __name__=='__main__':unittest.main()
