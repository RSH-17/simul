import numpy as np
from replay_marker_range_inputs import fit_range


def fixture():
    angles=np.linspace(-.065,.065,31)
    return [np.array([0.,0.,1.]),np.zeros(3),np.array([[0,0,1],[-1,0,0],[0,-1,0]]),
            2/np.cos(angles),angles[0],angles[1]-angles[0],.164,12.,5.,4.99]


def test_planar_wall_retains_exact_range():
    result=fit_range(*fixture())
    assert result['reason']=='returned_range'
    assert np.allclose(result['camera_point'],[0,0,2])


def test_missing_or_stale_scans_cannot_provide_a_range():
    args=fixture();args[-1]=4.
    assert fit_range(*args)['reason']=='scan_age'
    args=fixture();args[3][3:]=np.inf
    assert fit_range(*args)['reason']=='too_few_points'


def test_disconnected_surfaces_are_exposed_as_plane_failure():
    args=fixture();args[3][::2]*=2
    result=fit_range(*args)
    assert result['reason']=='plane_residual' and result['max_plane_residual_m']>.06
