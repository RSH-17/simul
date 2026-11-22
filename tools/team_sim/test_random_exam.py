import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from board import detail
from random_exam import fixture_for, selection
from run import survey_hint_setting


class RandomTeamExamTest(unittest.TestCase):
    def test_visual_survey_records_default_and_explicit_distant_cue_options(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            self.assertIsNone(survey_hint_setting(root,{}))
            (root/'visual_survey_manifest.json').write_text('{}')
            reg=Path(__file__).resolve().parents[2]/'experiments/2026-11-22_random_exam/visual_survey/enrollment.json'
            expected=str(json.loads(reg.read_text())['board_distant_hints']).lower()
            self.assertEqual(survey_hint_setting(root,{}),expected)
            self.assertEqual(survey_hint_setting(root,{'OSP_SURVEY_DISTANT_HINTS':'True'}),'true')
            with self.assertRaises(ValueError):survey_hint_setting(root,{'OSP_SURVEY_DISTANT_HINTS':'maybe'})
    def test_legacy_seed_fresh_draw_and_public_roster(self):
        legacy=range(112601,112606)
        self.assertEqual(selection(112601,False,None,legacy),(112601,False,3))
        self.assertEqual(selection(112601,True,None,legacy),(112601,True,3))
        self.assertEqual(selection(0,False,'three',legacy),(0,True,3))
        self.assertEqual(selection(112601,False,'unknown',legacy),(112601,True,0))
        with patch('random_exam.secrets.randbits',return_value=112601):
            self.assertEqual(selection(None,False,None,legacy),(112601,True,3))
        with self.assertRaises(ValueError):
            selection(2**32,False,None,legacy)

    @unittest.skipUnless(os.environ.get('TEAM_EXAM_ASSETS'),'requires verified team fixture package')
    def test_replay_unknown_count_and_cache_integrity(self):
        assets=Path(os.environ['TEAM_EXAM_ASSETS'])
        with tempfile.TemporaryDirectory() as tmp:
            cache=Path(tmp)
            a=fixture_for(cache,assets,2,0)
            manifest=json.loads((a/'fixture.json').read_text())
            self.assertEqual((manifest['marker_count'],manifest['tables']),(5,'fixed'))
            self.assertIn('@TEAM_MAP_ROOT@',(a/'world/osp_studio.sdf').read_text())
            self.assertEqual(fixture_for(cache,assets,2,0),a)
            b=fixture_for(cache,assets,2,3)
            self.assertEqual(json.loads((b/'fixture.json').read_text())['marker_count'],3)
            self.assertNotEqual(a,b)
            tables='run/simulation/demo_layout.json'
            self.assertEqual((b/tables).read_bytes(),(assets/'maps/map-112601'/tables).read_bytes())
            shuffled=fixture_for(cache,assets,2,3,'shuffle')
            self.assertNotEqual((b/tables).read_bytes(),(shuffled/tables).read_bytes())
            self.assertNotEqual(b,shuffled)
            (a/'scene.json').write_text('{}')
            with self.assertRaises(ValueError):
                fixture_for(cache,assets,2,0)

    def test_dashboard_separates_distance_from_sensor_proof(self):
        with tempfile.TemporaryDirectory() as tmp:
            base=Path(tmp)
            (base/'protocol.json').write_text(json.dumps({'fixture_mode':'randomized_exam'}))
            (base/'metrics.json').write_text(json.dumps({'registered_and_within_s':{'91':.2,'2':5.}}))
            self.assertEqual(detail(base)['approached'],[])
            self.assertFalse(detail(base)['finished'])
            (base/'exam-score.json').write_text(json.dumps({'strict':False,'records':{
                '91':{'verified_s':None},'2':{'verified_s':6.}}}))
            self.assertEqual(detail(base)['approached'],[2])
            self.assertTrue(detail(base)['finished'])


if __name__=='__main__':
    unittest.main()
