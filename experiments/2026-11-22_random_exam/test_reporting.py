import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from board_results import apply
from report import report


class ReportingTests(unittest.TestCase):
    def test_completed_card_keeps_zero_and_distinguishes_a_pass_by(self):
        with TemporaryDirectory() as temp:
            card=Path(temp)
            (card/'simulation_evaluation.json').write_text('{"stamp": 3}')
            latest=card/'mission_results/flow/latest.json';latest.parent.mkdir(parents=True)
            latest.write_text('{"snapshot":"x","status":{"targets":[{"id":0},{"id":542}]}}')
            row=dict(score=dict(valid=True,strict=False,registered=2,expected=3,
                                records={'0':{'verified_s':15},'542':{'verified_s':None}}))
            self.assertTrue(apply(card,row));self.assertTrue(apply(card,row))
            evaluation=json.loads((card/'simulation_evaluation.json').read_text())
            self.assertIn('접근 1/3',evaluation['evaluation']);self.assertEqual(evaluation['stamp'],3)
            self.assertEqual(json.loads(latest.read_text())['status']['targets'],
                             [{'id':0,'approached':True},{'id':542,'approached':False}])

    def test_no_pair_has_no_published_score(self):
        with TemporaryDirectory() as temp:
            root=Path(temp)
            (root/'protocol.json').write_text('{"worlds":[]}')
            (root/'comparison.json').write_text(json.dumps(dict(planned_pairs=6,paired=0,valid_pairs=0,pairs=[],
                  first_runs={'baseline':{'runs':0},'candidate':{'runs':0}})))
            output=report(root)
            self.assertIn('0쌍의 양쪽 주행',output)
            self.assertNotIn('| R5 |',output)
