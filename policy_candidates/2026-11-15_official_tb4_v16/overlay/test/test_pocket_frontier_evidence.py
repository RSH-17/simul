"""A room's own frontier must not attract transit back through its entry."""
import numpy as np
import pytest

from osp_mission.start_waypoints import options


def entry_room(resolution, unseen_behind_entry=False):
    data = np.full((round(8/resolution), round(10/resolution)), 100, dtype=np.int8)

    def rect(x0, x1, y0, y1, value):
        data[round(y0/resolution):round(y1/resolution),
             round(x0/resolution):round(x1/resolution)] = value

    rect(.5, 2., 3., 5., 0)
    rect(1.5, 4.5, 3.725, 4.275, 0)
    rect(4., 8., 2.8, 5.2, 0)
    rect(4., 8., 5.2, 6., -1)
    if unseen_behind_entry:
        rect(0., .5, 3., 5., -1)
    return data


@pytest.mark.parametrize('resolution', [.025, .05])
@pytest.mark.parametrize('rotation', [0, 1, 2, 3])
def test_reachable_room_frontier_does_not_pull_robot_back_to_closed_entry(resolution, rotation):
    data = entry_room(resolution)
    marker = np.zeros_like(data, dtype=bool)
    y, x = np.floor(np.array([4., 4.6])/resolution).astype(int)
    marker[y, x] = True
    data = np.rot90(data, rotation).copy()
    y, x = np.argwhere(np.rot90(marker, rotation))[0]
    origin = (-13.7, 6.2)
    robot = (origin[0]+(x+.5)*resolution, origin[1]+(y+.5)*resolution)
    assert not options(data, resolution, origin, robot)[0]


@pytest.mark.parametrize('resolution', [.025, .05])
def test_frontier_beyond_entry_still_allows_narrow_transit(resolution):
    # Returning is legitimate if the small room actually opens onto unseen
    # space. Room size and whether the corridor was visited are not vetoes.
    choices, _ = options(entry_room(resolution, True), resolution, (0., 0.), (4.6, 4.))
    assert choices
    assert all(c['start_escape'] and c['pocket_continuation'] for c in choices)
    assert all(c['x'] < 4.6 for c in choices)
