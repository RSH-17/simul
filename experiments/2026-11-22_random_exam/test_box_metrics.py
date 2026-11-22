import unittest

from box_metrics import paired_camera_summary


class BoxMetricsTests(unittest.TestCase):
    def test_only_pairs_with_evidence_on_both_sides_enter_the_same_denominator(self):
        def obs(frac,travel):
            return dict(sensor_valid=frac is not None,boxes=5,boxes_four=2,wall_fraction=frac,object_fraction=frac,
                        repeat={'recent':dict(repeated_m=travel,repeated_no_new_view_m=travel,
                            cluster_visit_m=travel,cluster_repeat_no_new_view_m=travel)} if travel is not None else None)
        pairs=[dict(baseline=obs(.5,2),candidate=obs(.75,1)),dict(baseline=obs(None,None),candidate=obs(1.,9)),
               dict(baseline=obs(.3,8),candidate=obs(.45,None))]
        result=paired_camera_summary(pairs)
        self.assertEqual((result['matched_camera_pairs'],result['matched_route_pairs']),(2,1))
        self.assertEqual(result['baseline']['wall_fraction'],dict(pairs=2,mean=.4))
        self.assertEqual(result['candidate']['boxes'],10)
        self.assertEqual(result['candidate']['repeated_m']['total'],1)
        self.assertIsNone(paired_camera_summary([])['baseline']['repeated_m']['total'])
