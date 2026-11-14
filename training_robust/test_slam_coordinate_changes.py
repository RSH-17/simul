import numpy as np
import pytest

from audit_slam_coordinate_changes import align_errors


def test_missing_reference_gap_and_extrapolation_are_not_scored():
    truth = [[0., 0., 0.], [.2, .2, 0.], [1., 1., 0.], [1.2, 1.2, 0.]]
    poses = [[-.1, -.1, 0.], [.1, .1, .3], [.6, 999., 0.], [1.1, 1.1, .4], [1.3, 1.3, 0.]]
    values, counts = align_errors(poses, truth)
    assert np.allclose(values, [[.1, .3], [1.1, .4]])
    assert counts['evaluated'] == 2 and counts['excluded'] == 3


def test_repeated_pose_is_counted_once_without_refitting_to_truth():
    values, counts = align_errors([[.1, 9., 8.], [.1, 3., 4.]], [[0., 0., 0.], [.2, 0., 0.]])
    assert np.allclose(values, [[.1, 5.]])
    assert counts['finite_unique_poses'] == 1


@pytest.mark.parametrize('truth', [[[0., 0., 0.], [0., 1., 1.]], [[0., 0., 0.], [.2, np.nan, 0.]]])
def test_invalid_truth_is_rejected(truth):
    with pytest.raises(ValueError):
        align_errors([[.1, 0., 0.]], truth)
