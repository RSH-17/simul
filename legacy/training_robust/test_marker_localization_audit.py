import numpy as np
import pytest
from audit_marker_localization import bounded_pose,rotation,substitute_robot_pose


def test_pose_substitution_removes_translation_and_rotation_without_moving_landmark():
    actual=np.array([2.,3.,.7]);observed=np.array([4.,-2.,-.3]);relative=np.array([.8,-.1])
    point=observed[:2]+rotation(observed[2])@relative
    expected=actual[:2]+rotation(actual[2])@relative
    assert np.allclose(substitute_robot_pose(point,observed,actual),expected)


def test_measurement_error_survives_robot_pose_substitution():
    actual=np.array([2.,3.,.7]);observed=np.array([4.,-2.,-.3]);relative=np.array([.8,-.1]);error=np.array([.03,-.04])
    point=observed[:2]+rotation(observed[2])@(relative+error)
    expected=actual[:2]+rotation(actual[2])@relative
    assert np.linalg.norm(substitute_robot_pose(point,observed,actual)-expected)==pytest.approx(.05)


def test_missing_or_extrapolated_truth_cannot_create_good_residual():
    trace=np.array([[0.,0.,0.,0.],[.2,1.,0.,0.],[1.,2.,0.,0.]])
    assert np.allclose(bounded_pose(trace,.1),[.5,0.,0.])
    for t in [-1.,.5,1.,2.]:assert bounded_pose(trace,t) is None
