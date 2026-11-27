import math
import pytest
from audit_official_crowded_escape import trace_metrics


def test_exit_requires_whole_body_and_uses_rotated_corridor_axis():
    scene=dict(start=[10.,10.,0.],official_start=dict(exit_heading_rad=math.pi/2),
               crowded_start=dict(exit_length_m=2.))
    rows=[dict(sim_seconds=t,world_x=10.,world_y=y) for t,y in [(5.,10.),(6.,12.18),(7.,12.21)]]
    assert trace_metrics(scene,rows[:2])['observed_exit_seconds'] is None
    report=trace_metrics(scene,rows)
    assert report['observed_exit_seconds']==2.
    assert report['actual_path_m']==pytest.approx(2.21)
    assert trace_metrics(scene,[])['observed_exit_seconds'] is None
