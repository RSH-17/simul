import math
import unittest

import numpy as np
from sensor_audit import CAMERA, CLOCK, epochs
from surface_audit import camera_poses, faces, mountable, opportunity
from exposure_opportunity import Blocker


class SurfaceAuditTests(unittest.TestCase):
    def setUp(self):
        self.k = dict(width=480, height=480, k=[333., 0., 240., 0., 333., 240., 0., 0., 1.],
                      translation=[0., 0., .32], quaternion=[-.5, .5, -.5, .5])
        self.points = [np.array(v) for v in ([2., 0., .32], [-1., 0., 0.], [0., 1., 0.])]
        self.block = Blocker('screen', np.array([1.,0.,.32]), np.array([.1,.4,.3]), np.eye(3))

    def view(self, times, solids=(), yaw=0., pixels=18.):
        track = np.array([(t, 0., 0., yaw) for t in np.arange(0, 4.1, .1)])
        camera = camera_poses(track, times, self.k)
        return opportunity(*self.points,.20,pixels,camera,self.k,solids)

    def test_repeated_visible_views_and_occlusion(self):
        self.assertEqual(self.view([.2,.4,.6]), 'observed')
        self.assertEqual(self.view([.2,.4,.6], [self.block]), 'line_of_sight_blocked')
        self.assertEqual(self.view([.2,.4,.6], yaw=math.pi), 'outside_readable_view')
        self.assertEqual(self.view([.2,.4,.6], pixels=40.), 'outside_readable_view')
        self.assertEqual(self.view([.2,.22,.25,2.]), 'insufficient_repeated_exposures')
        self.assertEqual(self.view([.2,.2,.4,2.]), 'insufficient_repeated_exposures')

    def test_rotated_object_and_hidden_join(self):
        body=dict(x=0.,y=0.,sx=1.5,sy=.32,yaw=.65,name='part',object_id='L',shape='elbow')
        scene=dict(room_size=[10.,8.],south_recess=dict(depth=.3,x_min=-1.,x_max=1.),
                   partitions=[],obstacles=[body])
        edges=[edge for edge in faces(scene) if edge[0]=='L']
        self.assertEqual(len(edges),4)
        for _,_,_,a,b,_ in edges:
            d=b-a
            self.assertGreater(np.dot((a+b)/2,[-d[1],d[0]]),0)
        self.assertFalse(mountable(self.block.center, self.points[2], .2, self.points[1], [self.block]))

    def test_clock_reset_keeps_two_runs_separate(self):
        marker=(480,480,tuple(self.k['k']))
        records=[(CLOCK,.05,None),(CAMERA,.1,marker),(CLOCK,12.,None),(CLOCK,11.99,None),
                 (CLOCK,.03,None),(CAMERA,.1,marker),(CLOCK,9.,None)]
        runs=list(epochs(records))
        self.assertEqual([r['last_clock'] for r in runs],[11.99,9.])
        self.assertEqual([len(r['camera']) for r in runs],[1,1])


if __name__=='__main__':
    unittest.main()
