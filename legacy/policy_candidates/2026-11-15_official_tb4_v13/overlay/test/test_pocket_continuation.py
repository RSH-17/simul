import math

import numpy as np
import pytest
from scipy.ndimage import distance_transform_edt

from osp_mission.edge_policy import options as edge_options
from osp_mission.start_waypoints import options


def pocket(resolution, width=.55, closed=False, unknown_gap=False):
    data = np.full((round(6 / resolution), round(9 / resolution)), 100, dtype=np.int8)
    def rectangle(x0, x1, y0, y1, value):
        data[round(y0 / resolution):round(y1 / resolution),
             round(x0 / resolution):round(x1 / resolution)] = value
    rectangle(.5, 3., 1.5, 4.5, 0)
    rectangle(2.5, 7.5, 3. - width / 2, 3. + width / 2, 0)
    if not closed:
        rectangle(7.5, 9., 3. - width / 2, 3. + width / 2, -1)
    if unknown_gap:
        rectangle(3., 3.2, 2., 4., -1)
    return data


def assert_body_path(data, resolution, origin, robot, goal, previous):
    clearance = distance_transform_edt(np.pad(data == 0, 1))[1:-1, 1:-1] * resolution
    minimum = .189 + resolution / math.sqrt(2) + .01
    x, y = np.floor((np.asarray(robot) - origin) / resolution).astype(int)
    start = (y, x)
    # Verify the entire path to the informative endpoint, not just the short goal.
    cell = goal['observed_frontier_cell']
    walked = set()
    while True:
        assert cell not in walked
        walked.add(cell)
        assert data[cell] == 0 and clearance[cell] >= minimum
        if cell == start:
            break
        index = previous[cell[0] * data.shape[1] + cell[1]]
        assert index >= 0
        parent = divmod(int(index), data.shape[1])
        dy, dx = parent[0] - cell[0], parent[1] - cell[1]
        if dy and dx:
            assert clearance[parent[0], cell[1]] >= minimum
            assert clearance[cell[0], parent[1]] >= minimum
        cell = parent
    assert goal['cell'] in walked


@pytest.mark.parametrize('resolution', [.025, .05])
@pytest.mark.parametrize('rotation', [0, 1, 2, 3])
def test_room_pocket_continues_through_passable_narrow_exit(resolution, rotation):
    data = pocket(resolution)
    start = np.floor(np.array([2.5, 3.]) / resolution).astype(int)[::-1]
    marker = np.zeros_like(data, dtype=bool)
    marker[tuple(start)] = True
    data = np.rot90(data, rotation).copy()
    y, x = np.argwhere(np.rot90(marker, rotation))[0]
    origin = (-8.3, 2.7)
    robot = (origin[0] + (x + .5) * resolution, origin[1] + (y + .5) * resolution)
    choices, previous = options(data, resolution, origin, robot, return_paths=True)
    assert choices
    for goal in choices:
        assert goal['pocket_continuation'] and goal['start_escape']
        assert goal['scan_angle'] == 0 and .45 <= goal['distance'] <= .70
        assert_body_path(data, resolution, origin, robot, goal, previous)
    integrated, _ = edge_options(data, resolution, origin, robot, inspection_focal=221.76)
    assert integrated and all(c['pocket_continuation'] for c in integrated)


@pytest.mark.parametrize('conditions', [dict(width=.35), dict(closed=True), dict(unknown_gap=True)])
def test_no_continuation_through_unsafe_or_uninformative_corridor(conditions):
    data = pocket(.025, **conditions)
    assert not options(data, .025, (0., 0.), (2.5, 3.))[0]


def test_normal_room_frontier_rim_does_not_activate_transit():
    data = np.full((200, 200), -1, dtype=np.int8)
    data[20:180, 20:180] = 0
    assert not options(data, .025, (0., 0.), (2.5, 2.5))[0]


def test_continuation_stops_when_wide_passage_joins_normal_components():
    data = pocket(.025, width=1.0)
    assert not options(data, .025, (0., 0.), (2.5, 3.))[0]
