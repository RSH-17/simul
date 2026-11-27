"""The board's provisional live verdict (no ROS needed)."""
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import board  # noqa: E402


class LiveVerdictTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.folder = Path(self.tmp.name)
        # Robot starts at (1, 2) facing +y, so map (x, y) -> world (1 - y, 2 + x).
        (self.folder/'headings.csv').write_text(f'elapsed_sim_s,x,y,yaw_rad\n0.0,1,2,{math.pi/2}\n')
        self.scene = dict(targets=[dict(id=7, x=3., y=2.), dict(id=32, x=-5., y=-5.)])

    def tearDown(self):
        self.tmp.cleanup()

    def rows(self, *items):
        return [dict(elapsed_sim_s=str(t), world_x=str(x), world_y=str(y), registered_ids=ids, bumper='False')
                for t, x, y, ids in items]

    def test_approach_counts_only_after_registration(self):
        trace = self.rows((1, 2.5, 2, ''), (2, 2.4, 2, '7'), (3, 0, 0, '7 99'))
        (self.folder/'detections.jsonl').write_text(
            json.dumps(dict(kind='accepted', id=7, map_point=[0., -1.9])) + '\n{"half')
        live = board.live_verdict(self.folder, self.scene, trace)
        seven = live['targets']['7']
        self.assertEqual((seven['registered_s'], seven['near_s'], seven['closest_m']), (2., 2., .5))
        self.assertAlmostEqual(seven['map_error_m'], .1, places=3)  # world (2.9, 2.0)
        self.assertIsNone(live['targets']['32']['near_s'])
        self.assertEqual(live['wrong_ids'], [99])
        self.assertFalse(live['conditions_met'])

    def test_bumps_come_from_judge_events(self):
        (self.folder/'events.jsonl').write_text('{"event": "start_ack"}\n{"event": "bumper"}\n')
        self.assertEqual(board.live_verdict(self.folder, self.scene, [])['bumps'], 1)


if __name__ == '__main__':
    unittest.main()
