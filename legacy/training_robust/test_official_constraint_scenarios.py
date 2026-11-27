import copy
import math

import pytest

from official_constraint_scenarios import PROFILES, make_constraint_scene, physical_digest
from official_crowded_scenarios import dock_and_reverse_check


@pytest.mark.parametrize('index,profile', list(enumerate(PROFILES)))
def test_requested_constraints_remain_physical_and_reachable(index, profile):
    config = make_constraint_scene(1205000000+index*101, 'c_shape', profile)
    p = PROFILES[profile]
    assert len(config['obstacles']) == p['obstacles'] == config['actual_obstacles']
    assert len({t['id'] for t in config['targets']}) == 3
    assert all(t['mount']=='obstacle' and t['hidden_from_start'] and
               p['marker_size'][0] <= t['size'] <= p['marker_size'][1] for t in config['targets'])
    a, b = config['obstacles'][-3:-1]
    gap = math.hypot(a['x']-b['x'], a['y']-b['y'])-(a['sy']+b['sy'])/2
    assert gap == pytest.approx(p['width'])
    assert abs(config['constraint_profile']['dock_yaw_offset_degrees']) == pytest.approx(p['yaw_degrees'])
    assert config['constraint_profile']['native_validated'] is False
    # Independently recheck the constructed geometry at a third resolution.
    assert dock_and_reverse_check(config, .015) is not None
    blocked = copy.deepcopy(config)
    x, y, _ = blocked['start']
    blocked['obstacles'].append(dict(type='box', x=x, y=y, sx=.8, sy=.8, yaw=0., height=1.))
    assert dock_and_reverse_check(blocked, .025) is None
    assert physical_digest(blocked) != physical_digest(config)


def test_invalid_shape_and_unknown_profile_are_rejected():
    with pytest.raises(ValueError):
        make_constraint_scene(1, 'unsupported', 'narrow')
    with pytest.raises(KeyError):
        make_constraint_scene(1, 'u_shape', 'unbounded')
