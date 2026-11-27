import dataclasses

from osp import catalog
from osp.scenario import Mission


def test_every_catalog_set_resolves():
    for name in catalog.names():
        assert catalog.load(name), name


def test_mission_carries_no_ground_truth():
    scenario = catalog.load('robust/test')[0]
    mission = scenario.mission(3, 600)
    assert {f.name for f in dataclasses.fields(Mission)} == {'scenario_id', 'marker_count', 'duration_s', 'start'}
    assert mission.start.x == scenario.world()['start'][0]
    assert scenario.targets and len(scenario.targets) == 3


def test_world_is_a_copy():
    scenario = catalog.load('robust/test')[0]
    scenario.world()['targets'].clear()
    assert len(scenario.world()['targets']) == 3


def test_team_sim_seeds_keep_truth_with_the_judge():
    scenarios = catalog.load('exam/dev')
    assert [s.id for s in scenarios] == ['112601', '112602', '112603', '112604', '112605']
    assert all(s.targets == () and s.kind == 'team_sim_seed' for s in scenarios)
