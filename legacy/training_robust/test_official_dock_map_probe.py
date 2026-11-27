import pytest
from official_dock_map_probe import gate


def valid():
    return dict(parameters={k:'mission/navigation_map' for k in ['local_costmap','global_costmap']},
        subscribers=['/local_costmap/local_costmap','/global_costmap/global_costmap'],
        paired_maps=dict(same_metadata=True,only_dock_occupied_changes=True,dock_center_occupied=True,changed_cells=45),
        anchor_samples=3,anchor_constant=True,global_dock_cost=254,local_dock_in_bounds=True,local_dock_cost=254)


def test_requires_actual_costs_and_subscribers_not_just_config():
    assert gate(valid())['passed']
    for key,value in [('global_dock_cost',0),('global_dock_cost',255),('local_dock_cost',0),
                      ('subscribers',[]),('anchor_constant',False),('anchor_samples',1),('local_dock_in_bounds',None)]:
        r=valid();r[key]=value;assert not gate(r)['passed']


@pytest.mark.parametrize('field,value',[('same_metadata',False),('only_dock_occupied_changes',False),
    ('dock_center_occupied',False),('changed_cells',0)])
def test_map_cannot_be_replaced_or_left_unchanged(field,value):
    r=valid();r['paired_maps'][field]=value;assert not gate(r)['passed']
