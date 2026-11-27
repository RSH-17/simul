import numpy as np

from osp_mission.edge_policy import options
from osp_mission.start_waypoints import narrow_start


def test_narrow_start_keeps_short_transit_when_distant_room_becomes_visible():
    data=np.full((120,170),100,dtype=np.int8)
    data[15:105,80:160]=0
    data[55:66,10:90]=0
    data[15:105,160:]=-1
    robot=(1.025,3.025)
    assert narrow_start(data,.05,(0.,0.),robot)
    choices,_=options(data,.05,(0.,0.),robot,inspection_focal=308.)
    assert choices
    assert all(c.get('start_escape') and .45<=c['distance']<=.70 and c['scan_angle']==0. for c in choices)


def test_initial_sweep_deferral_requires_known_body_clear_tight_footprint():
    data=np.full((40,80),100,dtype=np.int8);data[15:26,1:70]=0
    assert narrow_start(data,.05,(0.,0.),(.625,1.025))
    data[15:26,1:70]=-1
    assert not narrow_start(data,.05,(0.,0.),(.625,1.025))
    data[:]=0
    assert not narrow_start(data,.05,(0.,0.),(2.,1.))
