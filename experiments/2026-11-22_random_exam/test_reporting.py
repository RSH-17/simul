import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from board_results import apply
from report import layout_results, missed_markers, report, supplemental


class ReportingTests(unittest.TestCase):
    def test_layout_and_camera_failure_counts_only_use_matched_independent_worlds(self):
        with TemporaryDirectory() as temp:
            root = Path(temp)
            audit = root/'camera-opportunities'
            audit.mkdir()
            score = dict(strict=False, wall_near=0, wall=1, obstacle=2, obstacle_near=1,
                         records={'0': {}, '34': {}, '981': {}})
            pair = dict(valid=True, repeat=False, seed=73, baseline=dict(name='b', score=score),
                        candidate=dict(name='c', score=score))
            pairs = [pair, dict(pair, repeat=True), dict(pair, valid=False)]
            layout = '\n'.join(layout_results(pairs, {73: {'tables': 'shuffle'}}))
            self.assertIn('책상까지 무작위 | R5 | 0/1 | 0/1 | 1/2', layout)
            self.assertNotIn('원래 책상 배치', layout)
            for name in ('b', 'c'):
                audit_file = audit/(name+'.json')
                if name == 'c':
                    self.assertEqual(missed_markers(root, pairs), [])
                audit_file.write_text(json.dumps(dict(run=name, targets={
                    '0': {'stage': 'decoded_without_registration'}, '34': {'stage': 'verified'},
                    '981': {'stage': 'registered_without_both_near_witnesses'}})))
            stages = '\n'.join(missed_markers(root, pairs))
            self.assertIn('주행 중 번호 판독 후 미등록 | 1 | 1', stages)
            self.assertIn('1개 독립 배치', stages)

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

    def test_surface_comparison_uses_only_paired_nonrepeat_valid_recordings(self):
        with TemporaryDirectory() as temp:
            root=Path(temp)
            folder=root/'sensor-audit';folder.mkdir()
            pair=dict(valid=True,repeat=False,baseline={'name':'b'},candidate={'name':'c'})
            info=dict(valid=True,navigate=dict(aborted=1,canceled=2),surface_coverage=dict(
                wall=dict(mountable_center_span_m=8,observed_span_m=2),
                obstacle=dict(mountable_center_span_m=0,observed_span_m=0),boxes=0,boxes_all_four=0))
            (folder/'b.json').write_text(json.dumps(info))
            self.assertEqual(supplemental(root,[pair]),[])
            (folder/'c.json').write_text(json.dumps(info))
            text='\n'.join(supplemental(root,[pair,dict(pair,repeat=True),dict(pair,valid=False)]))
            self.assertIn('1쌍',text)
            self.assertIn('25.0% | 해당 없음 | 0/0 | 1 | 2',text)
