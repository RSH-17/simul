from pathlib import Path
import ast
import yaml


def test_local_grid_precision_preserves_collision_and_body_constraints():
    config=yaml.safe_load((Path(__file__).parents[1]/'config/tb4_nav2.yaml').read_text())
    local=config['local_costmap']['local_costmap']['ros__parameters']
    assert local['resolution']==.025
    assert ast.literal_eval(local['footprint'])==[[.189,0.],[.134,-.134],[0.,-.189],[-.134,-.134],[-.189,0.],[-.134,.134],[0.,.189],[.134,.134]]
    assert local['inflation_layer']['inflation_radius']==.45
    assert local['voxel_layer']['scan']['marking'] is True
    assert config['controller_server']['ros__parameters']['FollowPath']['CostCritic']['consider_footprint'] is True
    assert config['collision_monitor']['ros__parameters']['FootprintApproach']['enabled'] is True
    assert config['collision_monitor']['ros__parameters']['FootprintApproach']['time_before_collision']==1.2
