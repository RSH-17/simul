import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import supervise as s


class Supervision(unittest.TestCase):
    def test_branch_selection_and_terminal_futility(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);q=root/'queue.json';r3=root/'r3';r3.mkdir()
            with patch.object(s,'QUEUE',q),patch.object(s,'R3',r3):
                self.assertEqual(s.observe()[0],'waiting_for_engineering_screen')
                q.write_text(json.dumps({'state':'skipped_r2_eligible','r2_gate':{'eligible':True}}))
                self.assertEqual(s.observe(),('ready','R2'))
                q.write_text(json.dumps({'state':'running_r3','r2_gate':{'eligible':False}}))
                self.assertEqual(s.observe()[0],'waiting_for_r3_screen')
                (r3/'futility_stop.json').write_text('{"reason":"screen"}')
                self.assertEqual(s.observe()[0],'finished_r3_futility')
                (r3/'futility_stop.json').unlink();(r3/'candidate').mkdir()
                (r3/'candidate/complete.json').write_text('{}')
                (r3/'seven_case_gate.json').write_text('{"eligible":false}')
                self.assertEqual(s.observe()[0],'finished_both_screens_declined')
                (r3/'seven_case_gate.json').write_text('{"eligible":true}')
                self.assertEqual(s.observe(),('ready','R3'))

    def test_persisted_terminal_job_never_dispatches_again(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);(root/'finished.json').write_text('{"state":"done"}')
            with patch.object(s,'JOB',root),patch.object(s,'command',side_effect=AssertionError):
                s.monitor()

    def test_mixed_available_and_pending_pairs_report_separately(self):
        from summarize_blind import summarize
        a=dict(expected_wall=1,expected_box=2,strict=True,wall_registered=1,wall_near=1,box_registered=2,
               box_near=2,unexpected_ids=[],bumps=0,elapsed_s=12.)
        b=dict(a,strict=False,box_registered=1,box_near=1,elapsed_s=22.)
        p=dict(topology='case',valid=True,baseline=a,candidate=b,capped_candidate_minus_baseline_s=588.,
               jointly_successful_time_delta_s=None)
        q=dict(p,valid=False)
        row=summarize({'pairs':[p,q]})['case']
        self.assertEqual(row['invalid_pairs'],1)
        self.assertEqual(row['roles']['candidate']['mean_capped_s'],600.)


if __name__=='__main__':unittest.main()
