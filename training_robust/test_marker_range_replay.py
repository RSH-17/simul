import numpy as np
from replay_marker_range_inputs import fit_range, narrow_cone_ranges


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


def test_diagnostic_narrowing_preserves_input_and_rejects_insufficient_support():
    args=fixture();original=args[3].copy()
    restricted=narrow_cone_ranges(*args[:1],args[2],args[3],args[4],args[5],.004)
    np.testing.assert_array_equal(args[3],original)
    assert fit_range(*args[:3],restricted,*args[4:])['reason']=='too_few_points'


def test_narrow_cone_wraps_across_scan_angle_boundary():
    rotation=np.array([[0.,0.,-1.],[-1.,0.,0.],[0.,-1.,0.]])
    angles=np.linspace(-np.pi,np.pi,641)
    ranges=np.full(len(angles),2.)
    masked=narrow_cone_ranges(np.array([0.,0.,1.]),rotation,ranges,angles[0],angles[1]-angles[0],.025)
    assert np.flatnonzero(np.isfinite(masked)).tolist()==[0,1,2,638,639,640]
