import unittest

import numpy as np

from route_repeat import counts, first_complete, motion_repeats, nearby_pairs, resample


def drive(xs,ys):
    return np.column_stack((np.arange(len(xs))*.2,xs,ys))


class RouteTests(unittest.TestCase):
    def test_a_single_pass_and_single_crossing_are_not_recurring_loops(self):
        x=np.arange(76)*.1
        one,=resample(drive(x,np.zeros_like(x)))
        self.assertFalse(motion_repeats(one).any())
        points=[[0.,0.],[5.,0.],[5.,4.],[2.5,4.],[2.5,-2.]]
        line=np.vstack([np.linspace(a,b,80,endpoint=False) for a,b in zip(points,points[1:])])
        crossing,=resample(drive(*line.T))
        self.assertFalse(motion_repeats(crossing).any())

    def test_reverse_travel_is_measured_and_newly_seen_faces_have_credit(self):
        x=np.r_[np.arange(51)*.1,np.arange(49,-1,-1)*.1]
        p,=resample(drive(x,x*0.))
        tags=motion_repeats(p,85.)
        self.assertLess(tags.min(),0)
        self.assertEqual(tags.max(),0)
        area=np.ones(len(p),dtype=bool)
        d=counts([(p,area)],85.,[])
        fresh=counts([(p,area)],85.,[14.])
        self.assertGreater(d['cluster_repeat_no_new_view_m'],0.)
        self.assertEqual(fresh['cluster_repeat_no_new_view_m'],0.)
        self.assertEqual(d['repeated_m'],fresh['repeated_m'])

    def test_discontinuity_splits_the_track(self):
        x=np.r_[np.arange(30)*.1,5.+np.arange(30)*.1]
        self.assertEqual(len(list(resample(drive(x,x*0.)))),2)

    def test_separate_solids_only_and_exact_footprint_gap(self):
        a=dict(name='A1',object_id='A',x=0.,y=0.,sx=1.,sy=1.,yaw=0.)
        b=dict(a,name='A2',x=.2)
        c=dict(a,name='B',object_id='B',x=1.34)
        pairs=nearby_pairs({'obstacles':[a,b,c]})
        self.assertEqual([(p[0],p[1]) for p in pairs],[('A1','B'),('A2','B')])
        self.assertAlmostEqual(pairs[0][2],.34)

    def test_first_three_witnesses_must_share_a_window(self):
        self.assertIsNone(first_complete(np.array([.1, 2., 4.])))
        self.assertIsNone(first_complete(np.array([.1, .14, .149])))
        self.assertEqual(first_complete(np.array([.1, 2., 2.2, 2.4, 5.])),2.4)
