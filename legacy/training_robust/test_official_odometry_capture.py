import os
import pytest

from capture_official_odometry_for_job import identity,stop_reason


def test_native_identity_is_actual_process_start():
    assert identity(os.getpid()) is not None
    assert identity(999999999) is None


@pytest.mark.parametrize('observed',[None,'reused-pid-new-start'])
def test_recording_cannot_cross_native_process_lifetime(observed):
    assert stop_reason('original',observed,False,5,100)=='original_native_process_ended'


def test_completed_episode_and_bounded_window_stop_without_restarting():
    assert stop_reason('original','original',True,5,100)=='episode_completed'
    assert stop_reason('original','original',False,100,100)=='bounded_recording_window'
    assert stop_reason('original','original',False,99,100) is None
