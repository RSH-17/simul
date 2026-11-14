import copy

import pytest

from crowded_challenge_scenarios import body_accessibility, make_scene


@pytest.mark.parametrize('index,shape,count', [(0, 'c_shape', 8), (1, 'u_shape', 12), (2, 'zigzag', 16)])
def test_crowded_layouts_keep_body_exit_and_hidden_target_access(index, shape, count):
    config = make_scene(1197000000 + index * 101, count, shape=shape)
    evidence = config['crowded_start']
    assert len(config['obstacles']) == max(9, count)
    assert all(row['start_clearance_m'] < .32 for row in evidence['geometry_checks'])
    assert all(row['all_marker_approaches_reachable'] for row in evidence['geometry_checks'])
    assert all(target['mount'] == 'obstacle' and target['hidden_from_start'] for target in config['targets'])
    assert len({target['id'] for target in config['targets']}) == 3


def test_sealed_exit_is_rejected():
    config = make_scene(1197000000, 8, shape='c_shape')
    # Extend the rear cap along the corridor, making the starting footprint occupied.
    broken = copy.deepcopy(config)
    broken['obstacles'][-1]['sx'] = 2.
    assert body_accessibility(broken) is None
