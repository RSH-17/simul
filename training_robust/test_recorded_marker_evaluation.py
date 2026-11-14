import math

import numpy as np
import pytest

from evaluate_recorded_marker_ranges import world_point
from audit_imu_shadow import interpolate, ordered


def test_sensor_mount_and_heading_are_applied_before_world_translation():
    mount = np.eye(4)
    mount[:3, :3] = [[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]]
    mount[:3, 3] = [.1, -.2, .3]
    point = world_point([.5, -.4, 2.], mount, [10., 20., math.pi/2])
    np.testing.assert_allclose(point, [10.7, 22.1], atol=1e-12)


def test_truth_alignment_does_not_extrapolate_or_bridge_missing_segments():
    truth = ordered([[1., 0., 0., 3.1], [1.2, 1., 0., -3.1], [2., 9., 0., 0.]])
    assert interpolate(truth, 1.1)[2] == pytest.approx(math.pi)
    for stamp in [.9, 1.6, 2.1]:
        with pytest.raises(ValueError):
            interpolate(truth, stamp)
