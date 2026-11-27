import hashlib
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest

from focus_box_results import scores
from focus_box_trial import controls


class FocusBoxTrialTests(unittest.TestCase):
    def test_reference_uses_each_matched_world_once_and_checks_both_recordings(self):
        with TemporaryDirectory() as tmp:
            home = Path(tmp)
            panel, reference = home/'panel', home/'parent'
            panel.mkdir()
            reference.mkdir()
            def save(path, data):
                path.write_text(json.dumps(data))
            save(panel/'protocol.json', {'worlds':[{'seed':17,'files':{'world/osp_studio.sdf':'world'}}]})
            save(reference/'protocol.json', {'plan':[{'pair':1},{'pair':2}]})
            save(panel/'registration.json', {'reference_protocol_sha256':hashlib.sha256(
                (reference/'protocol.json').read_bytes()).hexdigest()})
            pair = dict(pair=1,seed=17,valid=True,repeat=False)
            paths = {}
            for role in ('baseline','candidate'):
                f = home/'corridor-board-73bbb05/recordings'/role/'metrics.json'
                f.parent.mkdir(parents=True)
                f.write_text('sealed')
                paths[role] = f
                pair[role] = dict(role=role,name=role,world_sha256='world',score={'valid':True},
                                  evidence={'metrics.json':hashlib.sha256(f.read_bytes()).hexdigest()})
            save(reference/'comparison.json', {'valid_pairs':2,'pairs':[pair,dict(pair,pair=2,repeat=True)]})
            engine = SimpleNamespace(runner=SimpleNamespace(HOME=home))
            self.assertEqual(controls(panel,reference,engine),[pair['baseline']])
            paths['candidate'].write_text('changed')
            with self.assertRaisesRegex(ValueError,'evidence changed'):
                controls(panel,reference,engine)

    def test_held_out_counts_registration_and_approach_separately(self):
        row = dict(registered=3,expected=3,near=2,wall=1,wall_near=0,objects=2,
                   object_near=2,strict=False,bump=0,capped_s=600.,travel_m=32.,false_ids=[])
        result = scores([{'focus':row}, {'focus':dict(row, near=3,wall_near=1,strict=True,capped_s=300.)}],
                        ['focus'])['focus']
        self.assertEqual((result['runs'],result['registered'],result['near'],result['strict']),(2,6,5,1))
        self.assertEqual(result['mean_capped_s'],450.)
