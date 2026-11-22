from contextlib import ExitStack
import json
import os
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import Mock, patch

from board_controls import MODES, OPTIONS, frontend
from board_run import MODES as RUNTIME_MODES, finish_card, prepare, reserve
from fixture import FILES


class SharedRandomBoardTests(unittest.TestCase):
    def test_async_card_is_scored_only_after_its_final_trace_exists(self):
        viewer=Mock()
        viewer.reconcile.side_effect=['no finalized board metadata','no final trace','current']
        scores=Mock()
        scores.apply.return_value=True
        with patch.dict('sys.modules',board_audit=viewer,board_results=scores), patch('board_run.time.sleep'):
            finish_card(Path('/records/run-a'),Path('/cards'),'http://example.test',{'valid':True},'Balanced')
        self.assertEqual(viewer.reconcile.call_count,3)
        scores.apply.assert_called_once_with(Path('/cards/run-a'),{'score':{'valid':True}},'Balanced')

    def test_incomplete_card_reports_the_saved_scored_recording(self):
        viewer=Mock()
        viewer.reconcile.return_value='no final trace'
        scores=Mock()
        with patch.dict('sys.modules',board_audit=viewer,board_results=scores):
            with self.assertRaisesRegex(RuntimeError,'/records/run-b'):
                finish_card(Path('/records/run-b'),Path('/cards'),'http://example.test',{},'Balanced',timeout=0)
        scores.apply.assert_not_called()

    def test_an_active_simulation_prevents_a_second_launch(self):
        with TemporaryDirectory() as tmp:
            track=Path(tmp)
            with ExitStack() as first:
                reserve(first,track)
                with ExitStack() as second, self.assertRaises(BlockingIOError):reserve(second,track)
            with ExitStack() as after:reserve(after,track)

    def test_new_menu_has_both_tested_policies_and_keeps_previous_choices(self):
        text='트랙 정책 <select><option value="pair">Pair</option></select></label> <button type="submit">새 랜덤 트랙 주행</button>'
        result=frontend(text)
        self.assertEqual(frontend(result),result)
        self.assertLess(result.index('exam-balanced'),result.index('value="pair"'))
        self.assertEqual(set(MODES),set(RUNTIME_MODES))
        for opt,_ in OPTIONS:self.assertEqual(result.count('value="'+opt+'"'),1)

    @unittest.skipUnless(os.environ.get('EXAM_SOURCE'),'requires the actual exam room')
    def test_existing_random_run_replays_the_same_files_and_public_count(self):
        with TemporaryDirectory() as tmp:
            base=Path(tmp);track=base/'track';(track/'.layouts').mkdir(parents=True)
            (track/'.layouts/uFjtqfyJ').symlink_to(Path(os.environ['EXAM_SOURCE']).resolve(),target_is_directory=True)
            first=base/'first';first.mkdir()
            a=prepare(first,track,2,0,'')
            self.assertEqual(a['marker_count'],5)
            ref=track/'test-track-prior';ref.symlink_to(first/'fixtures/map-2',target_is_directory=True)
            second=base/'second';second.mkdir()
            b=prepare(second,track,None,0,str(ref))
            self.assertEqual(b['files'],a['files'])
            self.assertEqual(set(b['files']),set(FILES))
            third=base/'third';third.mkdir()
            with self.assertRaisesRegex(ValueError,'undisclosed-count'):prepare(third,track,None,3,str(ref))

    def test_existing_older_record_cannot_be_called_a_new_random_exam(self):
        with TemporaryDirectory() as tmp:
            root=Path(tmp);ref=root/'test-track-baseline';ref.mkdir()
            (ref/'scene.json').write_text(json.dumps({'seed':1,'targets':[]}))
            stage=root/'stage';stage.mkdir()
            with self.assertRaisesRegex(ValueError,'randomized exam'):prepare(stage,root,None,3,str(ref))
