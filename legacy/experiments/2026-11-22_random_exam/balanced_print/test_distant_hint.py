import unittest

import numpy as np

from distant_hint import DistantHint
from screen_results import score_run, summarize


class DistantPrintTest(unittest.TestCase):
    def view(self, memory, ident, start, xy, target):
        pos = np.array([*xy, .24])
        direction = np.asarray(target)-pos
        distance = np.linalg.norm(direction)
        direction /= distance
        return [memory.observe(ident, start + dt, pos, direction, 85./distance)
                for dt in (0., .2, .4, .6, .8, 1.)]

    def test_two_positions_and_three_consistent_ranges_required(self):
        target = [5., 0., .31]
        memory = DistantHint()
        self.assertTrue(all(r is None for r in self.view(memory, 12, 2., (0.,-2.), target)))
        results = self.view(memory, 12, 10., (0.,0.), target)
        self.assertTrue(all(r is None for r in results[:4]))
        np.testing.assert_allclose(results[-1], target, atol=1.e-8)

    def test_separate_ids_collinear_translations_and_out_of_range(self):
        memory = DistantHint()
        target = [5., 0., .24]
        self.view(memory, 12, 1., (0.,0.), target)
        self.assertTrue(all(r is None for r in self.view(memory, 13, 8., (0.,-2.), target)))
        self.assertTrue(all(r is None for r in self.view(memory, 12, 12., (-1.,0.), target)))
        memory = DistantHint()
        target = [9., 0., .24]
        self.view(memory, 12, 1., (0.,-4.), target)
        self.assertTrue(all(r is None for r in self.view(memory, 12, 12., (0.,0.), target)))

    def test_wrong_location_and_prior_live_cue_get_no_early_credit(self):
        raw = dict(run='sample',recorded_frames=35,processed_with_tf=34,
                   registered_s={'1':55.,'2':None}, first_live_candidate_s={'1':10.},
                   first_live_observation_s={},
                   distant_hint_native_only={'1':dict(first_range_s=12.,first_xy_error_m=.02),
                                             '2':dict(first_range_s=22.,first_xy_error_m=.44)})
        results = score_run(raw)['markers']
        self.assertFalse(any(r['novel_early'] for r in results))
        self.assertEqual([r['id'] for r in results if r['wrong_first']], [2])
        partial = summarize([dict(name='x',seed=1,role='candidate')], {'x':raw}, {'untuned_layouts':[1,2,3]})
        self.assertFalse(partial['complete'])
        self.assertFalse(partial['advance'])
