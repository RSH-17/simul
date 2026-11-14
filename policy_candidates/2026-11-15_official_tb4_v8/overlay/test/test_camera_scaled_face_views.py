import math

import numpy as np
import pytest
from scipy.ndimage import distance_transform_edt

from osp_mission.edge_policy import options, inspection_standoffs, readable_distance


def scene():
    data=np.zeros((100,100),dtype=np.int8)
    data[0,:]=data[-1,:]=data[:,0]=data[:,-1]=100
    data[40:60,40:60]=100
    viewed=np.ones((4,)+data.shape,dtype=bool)
    viewed[:,40:60,40:60]=False
    return data,viewed


def test_official_camera_generates_readable_body_safe_directed_views():
    data,viewed=scene();original=viewed.copy();resolution=.05;focal=221.765
    goals,_=options(data,resolution,(0,0),(1.,2.5),viewed=viewed,
                    inspection_focal=focal,collect_faces=True)
    directed=[g for g in goals if g.get('face_inspection')]
    assert len(directed)>=4
    clearance=distance_transform_edt(data<=50)*resolution
    assert all(clearance[g['cell']]>=.32 for g in goals)
    assert all(g['features'][0]>0 and len(g['visible_faces'])>0 for g in directed)
    assert all(g['scan_angle']==pytest.approx(math.pi/3) for g in directed)
    assert len({g['cell'] for g in goals})==len(goals)
    assert np.array_equal(viewed,original)  # A planned visit is not camera evidence.
    assert max(inspection_standoffs(focal))<readable_distance(focal)<1.2


@pytest.mark.parametrize('focal',[0.,-1.,float('nan'),float('inf'),60.])
def test_missing_or_insufficient_calibration_does_not_reduce_body_clearance(focal):
    assert inspection_standoffs(focal)==()


def test_high_resolution_and_legacy_callers_keep_bounded_standoffs():
    assert inspection_standoffs(None)==(1.2,1.8)
    assert inspection_standoffs(1000.)==(1.2,1.8)


def test_observed_faces_do_not_receive_reinspection_credit():
    data,viewed=scene();viewed[:]=True
    assert not options(data,.05,(0,0),(1.,2.5),viewed=viewed,inspection_focal=221.765)[0]
