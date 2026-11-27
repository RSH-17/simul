import math

import numpy as np
from scipy.ndimage import distance_transform_edt

from osp_mission.start_waypoints import options
from osp_mission.exploration_memory import ExplorationMemory


def observed_corridor():
    data = np.full((17, 100), -1, dtype=np.int8)
    data[2:15, :70] = 100
    data[3:14, 1:70] = 0
    return data


def test_short_waypoint_approaches_distant_unobserved_exit_inside_free_body_path():
    data = observed_corridor()
    robot = (.625, .425)
    choices, previous = options(data, .05, (0., 0.), robot, return_paths=True)
    assert choices
    clearance = distance_transform_edt(np.pad(data == 0, 1))[1:-1, 1:-1] * .05
    for choice in choices:
        assert choice['start_escape'] and choice['scan_angle'] == 0.
        assert .45 <= choice['distance'] <= .70
        assert choice['x'] > robot[0]
        cell = choice['cell']
        while cell != (8, 12):
            assert data[cell] == 0 and clearance[cell] >= .189 + .05 / math.sqrt(2) + .01
            index = previous[cell[0] * data.shape[1] + cell[1]]
            assert index >= 0
            cell = divmod(int(index), data.shape[1])


def test_no_waypoint_through_unknown_or_when_body_does_not_fit():
    data = observed_corridor()
    data[:, 19:21] = -1  # Only 0.3 m ahead is known; cannot step 0.45 m.
    assert not options(data, .05, (0., 0.), (.625, .425))[0]
    data = observed_corridor()
    data[3:6, :70] = 100
    assert not options(data, .05, (0., 0.), (.625, .425))[0]


def test_no_escape_fallback_for_normal_room_or_known_closed_corridor():
    data = np.zeros((80, 80), dtype=np.int8)
    data[[0, -1], :] = 100
    data[:, [0, -1]] = 100
    assert not options(data, .05, (0., 0.), (2., 2.))[0]
    data = observed_corridor()
    data[data < 0] = 100
    assert not options(data, .05, (0., 0.), (.625, .425))[0]


def test_transit_never_claims_a_completed_inspection_but_failures_are_remembered():
    choices, _ = options(observed_corridor(), .05, (0., 0.), (.625, .425))
    memory = ExplorationMemory()
    memory.selected(choices[0], (0., 0., 0.))
    memory.completed(True, 10.)
    assert not memory.views
    assert memory.filter(choices, (0., 0., 0.), 11.)
    memory.selected(choices[0], (0., 0., 0.))
    memory.completed(False, 12.)
    assert memory.failures and not memory.filter(choices, (0., 0., 0.), 13.)
