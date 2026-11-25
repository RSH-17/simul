"""Campaign summaries must keep failures, split wall/box markers and refuse mixed conditions."""
import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import summarize  # noqa: E402

TARGETS = [dict(id=7, attached_to='wall_south'), dict(id=32, attached_to='aisle_box_w'),
           dict(id=103, attached_to='aisle_box_c')]


def make_run(root, name, *, strict, final_ids, within, render='system'):
    run = root/name
    run.mkdir()
    (run/'scene.json').write_text(json.dumps(dict(targets=TARGETS)))
    (run/'protocol.json').write_text(json.dumps(dict(policy_source_sha256={'a.py': 'x'}, render_backend=render,
                                                     camera_noise=dict(sigma=0, blur=0, drop=0), duration_sim_s=600.)))
    (run/'metrics.json').write_text(json.dumps(dict(strict_success=strict, final_ids=final_ids,
                                                    registered_and_within_s=within, elapsed_sim_s=600.,
                                                    path_length_m=50., bumper_events=0, reason='x')))
    return str(run)


class SummaryTest(unittest.TestCase):
    def test_counts_failures_and_splits_marker_kinds(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            ok = make_run(root, 'ok', strict=True, final_ids=[7, 32, 103], within={'7': 300., '32': 100., '103': 200.})
            miss = make_run(root, 'miss', strict=False, final_ids=[32, 103], within={'32': 50.})
            attempts = [dict(mode='legacy', seed=1, run=ok, completed=True),
                        dict(mode='legacy', seed=1, run=miss, completed=True),
                        dict(mode='legacy', seed=1, run=None, completed=False, error_tail=['crash'])]
            rows = [summarize.attempt_row(a) for a in attempts]
            g = summarize.summarize(rows)['development_maps']
            self.assertEqual(g['trials'], 3)
            self.assertEqual(g['strict'][:2], (1, 3))
            # The attempt without a run folder has an unknown layout and is reported as invalid.
            self.assertEqual((g['wall_registered'], g['wall_approached'], g['wall_markers']), (1, 1, 2))
            self.assertEqual((g['box_registered'], g['box_approached'], g['box_markers']), (4, 3, 4))
            self.assertEqual(g['mean_finish_s_of_successes'], 300.)
            self.assertEqual(g['invalid'], 1)
            low, high = g['strict'][2]
            self.assertLess(low, 1/3)
            self.assertGreater(high, 1/3)

    def test_refuses_mixed_render_backends(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            a = make_run(root, 'a', strict=True, final_ids=[7, 32, 103], within={'7': 1., '32': 1., '103': 1.})
            b = make_run(root, 'b', strict=True, final_ids=[7, 32, 103], within={'7': 1., '32': 1., '103': 1.},
                         render='software')
            manifest = root/'m.json'
            manifest.write_text(json.dumps(dict(attempts=[dict(mode='legacy', seed=1, run=a, completed=True),
                                                          dict(mode='legacy', seed=1, run=b, completed=True)])))
            sys.argv = ['summarize', str(manifest)]
            with self.assertRaises(SystemExit):
                summarize.main()


if __name__ == '__main__':
    unittest.main()
