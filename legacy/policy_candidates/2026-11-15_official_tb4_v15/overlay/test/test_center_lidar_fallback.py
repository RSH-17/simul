from types import SimpleNamespace as S
import math

import numpy as np
import pytest
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import LaserScan

from osp_mission.detector import Detector


def run(angles, ranges, bearing=0., scan_age=0.):
    scan = LaserScan()
    scan.header.frame_id = 'laser'
    scan.header.stamp = Time(seconds=10.-scan_age).to_msg()
    scan.angle_min = float(angles[0])
    scan.angle_increment = float(angles[1]-angles[0])
    scan.range_min, scan.range_max = .164, 12.
    scan.ranges = np.asarray(ranges, dtype=np.float32).tolist()
    mount = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
    rotation = Rotation.from_euler('z', bearing).as_matrix() @ mount
    q = Rotation.from_matrix(rotation).as_quat()
    tr = TransformStamped()
    tr.transform.rotation.x, tr.transform.rotation.y, tr.transform.rotation.z, tr.transform.rotation.w = map(float, q)
    owner = S(scan=scan, tf=S(lookup_transform=lambda *args, **kwargs: tr), diagnostics={})
    result = Detector.lidar_point(owner, np.array([0., 0., 1.]), 'camera',
                                 Time.from_msg(Time(seconds=10.).to_msg()))
    return result, owner.diagnostics.get('lidar_center_fallback_calls', 0)


def mixed(angles, bearing=0.):
    delta = np.arctan2(np.sin(angles-bearing), np.cos(angles-bearing))
    ranges = np.full(len(angles), np.inf)
    broad = np.abs(delta) < .071
    ranges[broad] = np.where(np.abs(delta[broad]) < .031, 2., 5.)/np.cos(delta[broad])
    return ranges


def test_valid_broad_plane_does_not_take_the_fallback():
    angles = np.arange(-13, 14)*.005
    point, calls = run(angles, 2./np.cos(angles))
    np.testing.assert_allclose(point, [0., 0., 2.], atol=1e-6)
    assert calls == 0


@pytest.mark.parametrize('step,phase', [(.005, 0.), (.01, .003), (.018, .009)])
def test_mixed_background_recovers_only_the_supported_central_plane(step, phase):
    angles = np.arange(-20, 21)*step + phase
    point, calls = run(angles, mixed(angles))
    np.testing.assert_allclose(point, [0., 0., 2.], atol=1e-6)
    assert calls == 1


def test_central_support_wraps_across_full_scan_boundary():
    angles = np.linspace(-math.pi, math.pi, 640, endpoint=False)
    point, calls = run(angles, mixed(angles, math.pi), bearing=math.pi)
    np.testing.assert_allclose(point, [0., 0., 2.], atol=1e-6)
    assert calls == 1


@pytest.mark.parametrize('condition', ['center_gap', 'one_sided', 'too_few', 'mixed_center', 'out_of_range', 'stale'])
def test_fallback_rejects_missing_or_ambiguous_support(condition):
    angles = np.arange(-13, 14)*.005
    ranges = mixed(angles)
    if condition == 'center_gap':
        ranges[np.abs(angles)<.012] = np.inf
    elif condition == 'one_sided':
        ranges[(angles<0) & (np.abs(angles)<.031)] = np.inf
    elif condition == 'too_few':
        ranges[(np.abs(angles)<.031) & (np.abs(angles)>.006)] = np.inf
    elif condition == 'mixed_center':
        ranges[(np.abs(angles)<.031) & (np.arange(len(angles)) % 2 == 0)] *= 3.
    elif condition == 'out_of_range':
        ranges[np.abs(angles)<.031] = 9./np.cos(angles[np.abs(angles)<.031])
    point, calls = run(angles, ranges, scan_age=.3 if condition == 'stale' else 0.)
    assert point is None and calls == 0
