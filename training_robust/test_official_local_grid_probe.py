import copy

from official_local_grid_probe import EXTRA, FOOTPRINT, navigation_grid_gate


def report():
    values=copy.deepcopy(EXTRA)
    values['local_costmap/local_costmap']['footprint']=str(FOOTPRINT)
    return dict(parameters=values,published_local_grid=dict(samples=2,resolution=.025,width=120,height=120,frame='odom'))


def test_actual_grid_is_required_not_only_declared_resolution():
    r=report();assert navigation_grid_gate(r)['passed']
    for field,value in [('samples',0),('resolution',.06),('width',50),('frame','map')]:
        altered=copy.deepcopy(r);altered['published_local_grid'][field]=value
        assert not navigation_grid_gate(altered)['passed']


def test_body_and_safety_cannot_be_shrunk_to_pass_narrow_grid():
    for node,key,value in [('local_costmap/local_costmap','footprint_padding',0.),
        ('local_costmap/local_costmap','footprint','[[.1,0],[0,.1],[-.1,0],[0,-.1]]'),
        ('local_costmap/local_costmap','voxel_layer.scan.marking',False),
        ('controller_server','FollowPath.CostCritic.consider_footprint',False),
        ('collision_monitor','FootprintApproach.enabled',False)]:
        r=report();r['parameters'][node][key]=value
        assert not navigation_grid_gate(r)['passed']
