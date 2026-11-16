import math
from types import SimpleNamespace

import pytest

from osp_mission.self_scan_filter import filter_ranges


def scan(ranges):
    return SimpleNamespace(ranges=ranges, range_min=.15, range_max=12.,
                           angle_min=0., angle_increment=math.pi/2)


def test_only_returns_inside_body_are_removed():
    values, count = filter_ranges(scan([.155, .30, .20, math.inf]), 0., 0., 0.)
    assert math.isinf(values[0]) and values[1:] == [.30, .20, math.inf]
    assert count == 1


def test_sensor_offset_and_rotation_use_base_frame():
    # The first return is 0.19 m from the laser, but 0.09 m from base_link.
    values, count = filter_ranges(scan([.19, .19]), -.10, 0., 0.)
    assert math.isinf(values[0]) and values[1] == .19 and count == 1


def test_real_nearby_obstacle_is_retained():
    values, count = filter_ranges(scan([.19, .25, .32]), 0., 0., 0.)
    assert values == [.19, .25, .32] and count == 0


def test_bad_geometry_and_oversized_mask_fail_closed():
    with pytest.raises(ValueError):
        filter_ranges(scan([.19]), 0., 0., 0., radius=.19)
    with pytest.raises(ValueError):
        filter_ranges(scan([.19]), math.nan, 0., 0.)
