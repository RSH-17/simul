import copy

import pytest

from official_crowded_scenarios import dock_and_reverse_check, make_official_scene


@pytest.mark.parametrize('index,shape,count', [(0, 'c_shape', 9), (1, 'u_shape', 12), (2, 'zigzag', 16)])
def test_hidden_targets_remain_accessible_after_official_undock(index, shape, count):
    cfg = make_official_scene(1198000000 + index*101, count, shape=shape)
    assert len(cfg['obstacles']) == count
    assert len({t['id'] for t in cfg['targets']}) == 3
    assert all(t['hidden_from_start'] and t['mount'] == 'obstacle' for t in cfg['targets'])
    assert all(r['dock_scene_clear'] and r['post_undock_marker_approaches_reachable']
               for r in cfg['official_start']['checks'])


def test_reverse_and_dock_collisions_are_rejected():
    cfg = make_official_scene(1198000000, 9, shape='c_shape')
    broken = copy.deepcopy(cfg)
    broken['start'][2] += 1.5707963267948966
    assert dock_and_reverse_check(broken) is None
    broken = copy.deepcopy(cfg)
    broken['obstacles'].append(dict(type='box', x=cfg['start'][0], y=cfg['start'][1],
                                  sx=1., sy=1., yaw=0., height=1.))
    assert dock_and_reverse_check(broken) is None
