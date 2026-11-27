import copy
import unittest

from visual_survey_results import aggregate, matched


def sample():
    return dict(registered=2,near=1,expected=3,wall=2,wall_near=1,objects=1,object_near=0,
                strict=False,bump=0,capped_s=600.,travel_m=12.,false_ids=[],world_sha256='same',
                seed=9,markers={'103':{},'7':{},'32':{}},start=[1.,2.,0.],wall_fraction=.5,
                object_fraction=None,repeat=None)


class ComparisonTests(unittest.TestCase):
    def test_same_seed_but_different_world_or_launch_direction_is_rejected(self):
        reference=sample()
        for field,other in [('world_sha256','different'),('start',[1.,2.,.20]),('start',[1.,2.2,0.])]:
            change=copy.deepcopy(reference);change[field]=other
            with self.assertRaises(ValueError):matched(reference,change)
        matched(reference,dict(reference,start=[1.01,2.,.01]))

    def test_missing_surface_or_path_measurement_is_never_scored_as_zero(self):
        row=sample()
        result=aggregate([dict(balanced=row,print=row,survey=row)])['survey']
        self.assertEqual(result['near'],1)
        self.assertEqual(result['object_fraction'],dict(mean=None,n=0))
        self.assertEqual(result['repeated_no_new_view_m'],dict(mean=None,n=0))
        self.assertEqual(result['wall_fraction'],dict(mean=.5,n=1))
