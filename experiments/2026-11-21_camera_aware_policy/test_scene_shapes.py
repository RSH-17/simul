import unittest
from camera_scene_shapes import obstacle_shapes


class SceneShapes(unittest.TestCase):
    def test_cylinder_without_box_dimensions(self):
        rows=obstacle_shapes([dict(type='box',x=1,y=2,sx=.7,sy=.8),
                              dict(type='cylinder',x=3,y=4,radius=.2)])
        self.assertEqual(len(rows),2)
        self.assertNotIn('radius',rows[0])
        self.assertEqual(rows[1]['radius'],.2)
        self.assertEqual(rows[1]['sx'],.4)

    def test_one_bad_display_shape_does_not_break_other_cards(self):
        rows=obstacle_shapes([dict(type='cylinder',x=0,y=0,radius=-1),
                              dict(x=0,y=0,sx=float('nan'),sy=1),dict(x=0,y=0),
                              dict(x=0,y=0,sx=.7,sy=.7)])
        self.assertEqual(len(rows),1)


if __name__=='__main__':unittest.main()
