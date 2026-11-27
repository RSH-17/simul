"""The judge's detection log and the board's reading of it (no ROS needed)."""
import json
import sys
import tempfile
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from judge.detections import DetectionLog  # noqa: E402
import board  # noqa: E402


class DetectionLogTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.path = Path(self.tmp.name)/'detections.jsonl'
        self.log = DetectionLog(self.path)

    def tearDown(self):
        self.log.close()
        self.tmp.cleanup()

    def test_times_are_since_start_and_outside_readings_are_dropped(self):
        start = 10.
        self.assertIsNone(self.log.record('accepted', dict(id=7, stamp=5.), None, False))  # before start
        self.log.record('candidate', dict(id=7, stamp=40., min_edge_pixels=8.04, map_point=[1., 2., .3]), start, False)
        self.log.record('accepted', dict(id=7, stamp=45.5), start, False)
        self.log.record('accepted', dict(id=7, stamp=50.), start, False)
        self.assertIsNone(self.log.record('accepted', dict(id=9, stamp=60.), start, True))  # after close
        self.log.record('accepted', dict(id='bad', stamp=60.), start, False)
        rows = [json.loads(line) for line in self.path.read_text().splitlines()]
        self.assertEqual([r['t'] for r in rows], [30., 35.5, 40.])
        self.assertEqual(rows[0], dict(t=30., kind='candidate', id=7, min_edge_px=8., map_point=[1., 2.]))
        report = self.log.report(registered=set(), expected={7: (0, 0), 32: (1, 1)})
        self.assertEqual(report['first_detected_s'], {'7': 30.})
        self.assertEqual(report['first_detected_by_kind_s'], {'candidate': {'7': 30.}, 'accepted': {'7': 35.5}})
        self.assertEqual(report['detection_counts']['accepted'], {'7': 2})
        self.assertEqual(report['detected_not_registered'], [7])
        self.assertEqual(report['detection_messages_rejected'], 1)

    def test_registered_and_unexpected_ids(self):
        self.log.record('accepted', dict(id=7, stamp=1.), 0., False)
        self.log.record('accepted', dict(id=99, stamp=2.), 0., False)
        report = self.log.report(registered={7}, expected={7: (0, 0)})
        self.assertEqual(report['detected_not_registered'], [99])
        self.assertEqual(report['detected_unexpected_ids'], [99])

    def test_no_readings_writes_no_file(self):
        self.log.close()
        self.assertFalse(self.path.exists())
        self.assertEqual(self.log.report(set(), {})['first_detected_s'], {})


class BoardTest(unittest.TestCase):
    def test_judge_log_preferred_and_policy_logs_as_fallback(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = Path(tmp)
            (run/'native').mkdir()
            (run/'native/marker_followups.jsonl').write_text(json.dumps(dict(id=7, stamp=99.))+'\n')
            self.assertEqual(board.detections(run), ({'7': 99.}, 'policy'))
            (run/'detections.jsonl').write_text(
                json.dumps(dict(t=12.34, kind='candidate', id=7))+'\n'
                + json.dumps(dict(t=20., kind='accepted', id=32))+'\n'
                + '{"t": 21, "kind"')  # a line still being written
            self.assertEqual(board.detections(run), ({'7': 12.3, '32': 20.}, 'judge'))


if __name__ == '__main__':
    unittest.main()
