import math
from pathlib import Path

import numpy as np
import pytest

from osp_mission.sim_mapping_scan import mapping_ranges


def test_only_positive_infinite_returns_are_normalized():
    original = [float('inf'), float('-inf'), float('nan'), 0., .1, .164, .3, 11.8, 12.]
    result = mapping_ranges(original, .164, 12.)
    assert 11.9 < result[0] < 12.  # no endpoint/feature at Karto threshold 11.9
    assert result[1] == float('-inf') and math.isnan(result[2])
    np.testing.assert_array_equal(result[3:], np.asarray(original[3:], dtype=np.float32))
    assert original[0] == float('inf')


@pytest.mark.parametrize('low,high', [(0.,12.), (.164,8.), (.164,float('inf')), (float('nan'),12.)])
def test_unexpected_sensor_metadata_fails_closed(low, high):
    with pytest.raises(ValueError):
        mapping_ranges([float('inf')], low, high)


def test_shared_hardware_launch_does_not_enable_sim_mapping():
    package = Path(__file__).resolve().parents[1]
    robot = (package/'launch/robot.launch.py').read_text()
    simulation = (package/'launch/simulation.launch.py').read_text()
    assert "'sim_no_return_mapping':'false'" in robot
    assert 'if no_return and not sim:' in robot
    assert "max_laser_range=11.9" in robot
    assert "sim_no_return_mapping='true'" in simulation
