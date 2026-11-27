import math

import numpy as np
import pytest

from audit_archived_marker_ranges import fixed_pose, rgb_rectangle, source_pixels


def test_archived_camera_rectangle_and_pixel_centers_match_frozen_compositor():
    assert rgb_rectangle(320,240)==(43,522,248,186)
    original=np.array([[10.,20.],[160.,120.],[300.,200.]])
    resized=(original+.5)*.775-.5
    np.testing.assert_allclose(source_pixels(resized,320,240),original,atol=1e-12)


def test_fixed_sensor_mount_uses_urdf_rotations_and_translations():
    xml=f'''<robot>
      <joint type="fixed"><parent link="base_link"/><child link="sensor"/>
        <origin xyz=".1 .2 .3" rpy="0 0 {math.pi/2}"/></joint>
      <joint type="fixed"><parent link="sensor"/><child link="optical"/>
        <origin xyz=".4 0 0" rpy="{-math.pi/2} 0 {-math.pi/2}"/></joint>
    </robot>'''
    pose=fixed_pose(xml,'optical')
    np.testing.assert_allclose(pose[:3,3],[.1,.6,.3],atol=1e-12)
    np.testing.assert_allclose(pose[:3,:3]@[0,0,1],[0,1,0],atol=1e-12)


@pytest.mark.parametrize('kind',['revolute','continuous'])
def test_moving_mount_is_not_assumed_to_be_a_fixed_calibration(kind):
    xml=f'<robot><joint type="{kind}"><parent link="base_link"/><child link="sensor"/></joint></robot>'
    with pytest.raises(ValueError,match='not fixed'):fixed_pose(xml,'sensor')
