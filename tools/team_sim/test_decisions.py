"""The board's decision timeline (no ROS needed)."""
import json
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import board  # noqa: E402

COLUMNS = ['x', 'y', 'yaw', 'scan_angle', 'surface_gain', 'frontier_gain', 'distance', 'turn', 'revisit']


class DecisionTimelineTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.folder = Path(self.tmp.name)
        (self.folder/'native/policy_decisions').mkdir(parents=True)
        (self.folder/'headings.csv').write_text('elapsed_sim_s,x,y,yaw_rad\n0.0,10,0,0\n')

    def tearDown(self):
        self.tmp.cleanup()

    def save(self, index, elapsed, rows, selected):
        np.savez_compressed(self.folder/f'native/policy_decisions/{index:04d}.npz', elapsed=elapsed, planner='edge',
                            robot=np.array([0., 0.]), weights=np.array([1., 2., 1., .3, 1.]),
                            candidate_columns=COLUMNS, candidates=np.array(rows, dtype=float),
                            selected_index=selected)

    def test_layers_reasons_and_world_points(self):
        good = [1, 1, 0, 0, 0, 5, 1, 0, 0]
        poor = [2, 0, 0, 0, 0, 1, 1, 0, 0]
        rule = [3, 3, 0, 0, 0, 0, 0, 0, 0]
        self.save(0, 2.9, [good, poor], 0)
        self.save(1, 12.4, [good, poor, rule], 2)
        self.save(2, 20.0, [good, poor], 1)
        (self.folder/'native/center_wall.jsonl').write_text('\n'.join(json.dumps(r) for r in [
            dict(elapsed=2.9, kind='flow_or_marker', reason='explore_open_frontier'),
            dict(elapsed=12.4, kind='interior_surface', reason='unseen_reachable_object'),
            dict(elapsed=20.0, kind='flow_or_marker', reason='verified_marker_approach', marker_id=7)]))
        rows = board.decisions(self.folder)
        self.assertEqual([r['layer'] for r in rows], ['점수식', '상위 규칙', '마커 접근·경로 필터'])
        self.assertEqual(rows[1]['reason_text'], '아직 안 본 닿을 수 있는 물체')
        self.assertEqual(rows[1]['selected']['world'], [13., 3.])  # start pose (10, 0)
        self.assertEqual(rows[1]['top'][0]['index'], 0)
        self.assertAlmostEqual(rows[1]['top'][0]['score'], 10/(1/.26+5), places=3)
        self.assertEqual(rows[2]['marker_id'], 7)

    def test_missing_logs_give_an_empty_timeline(self):
        self.assertEqual(board.decisions(self.folder/'nowhere'), [])


if __name__ == '__main__':
    unittest.main()
