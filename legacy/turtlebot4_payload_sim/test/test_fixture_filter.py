import math
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from turtlebot4_payload_sim.fixture_filter import filter_ranges, inside_fixture


CONFIG = yaml.safe_load((Path(__file__).resolve().parents[1]/'config/front_fixture_hypothesis.yaml').read_text())


def test_front_fixture_return_removed_but_farther_return_kept():
    assert inside_fixture([.145, 0., .193], CONFIG)
    # Official laser is at (-.04, 0, .193) and points forward at this beam.
    transform = SimpleNamespace(
        translation=SimpleNamespace(x=-.04, y=0., z=.193),
        rotation=SimpleNamespace(x=0., y=0., z=math.sin(math.pi/4),
                                 w=math.cos(math.pi/4)))
    scan = SimpleNamespace(ranges=[.185, .99], angle_min=-math.pi/2,
                           angle_increment=0., range_min=.164, range_max=12.)
    ranges, removed = filter_ranges(scan, transform, CONFIG)
    assert removed == 1
    assert math.isinf(ranges[0])
    assert ranges[1] == .99


def test_same_ray_occlusion_cannot_be_recovered_by_filter():
    # A LaserScan beam carries only the nearest physical hit. If that is the
    # fixture, the 0.99m external obstacle is absent from the input entirely.
    transform = SimpleNamespace(
        translation=SimpleNamespace(x=-.04, y=0., z=.193),
        rotation=SimpleNamespace(x=0., y=0., z=math.sin(math.pi/4),
                                 w=math.cos(math.pi/4)))
    scan = SimpleNamespace(ranges=[.185], angle_min=-math.pi/2,
                           angle_increment=0., range_min=.164, range_max=12.)
    filtered, removed = filter_ranges(scan, transform, CONFIG)
    assert removed == 1
    assert math.isinf(filtered[0])
