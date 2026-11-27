import math
import pytest

from audit_departure_returns import metrics


@pytest.mark.parametrize('yaw',[0.,math.pi/2,math.pi,-math.pi/2])
def test_return_requires_whole_body_inside_after_complete_exit(yaw):
    scene=dict(start=[4.,-2.,0.],official_start=dict(exit_heading_rad=yaw),
        crowded_start=dict(exit_length_m=2.,corridor_width_m=.5))
    # Jitter near rail end is not a return; genuine exit and re-entry occur twice.
    positions=[0.,2.21,2.10,1.81,1.79,1.80,2.21,1.79]
    rows=[dict(sim_seconds=i,world_x=4+p*math.cos(yaw),world_y=-2+p*math.sin(yaw)) for i,p in enumerate(positions)]
    result=metrics(scene,rows)
    assert result['first_full_exit_seconds']==1
    assert result['reentry_seconds']==[4,7]
