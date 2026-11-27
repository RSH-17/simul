import math

import numpy as np
import pytest

from audit_face_frame_drift import at_time, matched_transforms


def test_transform_alignment_discards_skew_and_unwraps_frame_rotation():
    rows = [dict(map_stamp=t,map_yaw=a,odom_stamp=o,odom_yaw=0.)
            for t, a, o in [(1.,3.1,1.01),(1.1,0.,.9),(1.2,-3.1,1.21)]]
    samples = matched_transforms(rows)
    assert samples.shape == (2,2)
    assert at_time(samples,1.1) == pytest.approx(math.pi)


@pytest.mark.parametrize('stamp',[.9,1.5,2.1])
def test_missing_or_extrapolated_transform_is_not_used(stamp):
    with pytest.raises(ValueError,match='bounded'):
        at_time(np.array([[1.,0.],[2.,.5]]),stamp)


def test_exact_event_sample_needs_no_interpolation_over_gap():
    assert at_time(np.array([[1.,0.],[2.,.5]]),2.) == .5
