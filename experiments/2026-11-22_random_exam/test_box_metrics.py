import unittest

from box_metrics import outer_walls, paired_camera_summary
from geometry import wall_segments


class BoxMetricsTests(unittest.TestCase):
    def test_south_includes_its_recess_and_columns_do_not_double_count_wall_length(self):
        import math
        scene=dict(room_size=[10.,8.],south_recess=dict(x_min=-1.,x_max=1.,depth=.8),
                   partitions=[dict(name='column',x=2.,y=3.8,sx=.3,sy=.3,yaw=0.)])
        faces={}
        for name,a,b in wall_segments(scene):
            n=math.ceil(max(0.,math.dist(a,b)-.26)/.4)
            faces[name]=dict(sampled_centers=n,counts=dict(observed=n if name.startswith('south_') else 0))
        data=outer_walls(scene,dict(groups={'wall':{'faces':faces}},parameters=dict(print_m=.2,sample_interval_max_m=.4)))
        self.assertEqual(set(data),{'west','east','north','south'})
        self.assertEqual(data['south']['observed_m'],data['south']['mountable_m'])
        self.assertAlmostEqual(data['north']['mountable_m'],9.74)
        self.assertAlmostEqual(data['north']['observed_m'],0.)

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
