import numpy as np
import pytest

from pose_increment_gyro import integrate


def arc(times,v,w):
    if w==0:return np.column_stack([times,v*times,times*0,times*0])
    return np.column_stack([times,v/w*np.sin(w*times),v/w*(1-np.cos(w*times)),w*times])


@pytest.mark.parametrize('speed',[.2,-.2,0.])
@pytest.mark.parametrize('rate',[0.,.7,-.7])
def test_body_gyro_corrects_wheel_turn_bias_without_distance_scale_fit(speed,rate):
    t=np.arange(0,3,.01);wheel=arc(t,speed,rate*1.1);expected=arc(t,speed,rate)
    actual=integrate(wheel,np.column_stack([t,np.full(len(t),rate)]))
    assert np.allclose(actual,expected,atol=1e-12)


def test_initial_frame_change_does_not_change_relative_motion():
    t=np.arange(0,2,.01);wheel=arc(t,.2,.8);angle=1.2
    rotation=np.array([[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]])
    shifted=wheel.copy();shifted[:,1:3]=shifted[:,1:3]@rotation.T+[4.,-7.];shifted[:,3]+=angle
    gyro=np.column_stack([t,np.full(len(t),.7)])
    a=integrate(wheel,gyro);b=integrate(shifted,gyro)
    assert np.allclose(b[:,1:3],a[:,1:3]@rotation.T+[4.,-7.],atol=1e-12)
    assert np.allclose(b[:,3],a[:,3]+angle,atol=1e-12)


def test_gyro_gap_and_clock_reset_are_not_bridged():
    t=np.arange(0,2,.01);wheel=arc(t,.2,.8);gyro=np.column_stack([t,np.full(len(t),.7)])
    with pytest.raises(ValueError,match='Stale gyro'):integrate(wheel,gyro[(t<.5)|(t>1.)])
    reset=wheel.copy();reset[50:,0]-=.5
    with pytest.raises(ValueError,match='strictly ordered'):integrate(reset,gyro)


def test_future_gyro_sample_does_not_affect_earlier_increment():
    t=np.arange(0,1,.01);wheel=arc(t,.2,.7);gyro=np.column_stack([t,np.zeros(len(t))])
    original=integrate(wheel,gyro);gyro[gyro[:,0]>=.5,1]=1.
    changed=integrate(wheel,gyro)
    assert np.array_equal(changed[changed[:,0]<=.5],original[original[:,0]<=.5])
