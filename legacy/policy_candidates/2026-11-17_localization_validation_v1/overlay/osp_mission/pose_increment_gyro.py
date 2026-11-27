"""Offline planar integration of wheel pose increments and body-axis gyro.

No truth, map, absolute IMU orientation, fitted scale or target geometry enters
this function. This is an estimator experiment, not a ROS odometry publisher.
"""
import numpy as np


def checked(values,columns):
    values=np.asarray(values,dtype=float)
    if (values.ndim!=2 or values.shape[1]!=columns or len(values)<2
            or not np.isfinite(values).all() or not np.all(np.diff(values[:,0])>0)):
        raise ValueError('Require finite, strictly ordered sensor samples')
    return values.copy()


def integrate(wheel,gyro,max_wheel_gap=.1,max_gyro_gap=.05):
    """Return timestamp/x/y/yaw using wheel arc distance and past gyro samples.

    wheel columns: acquisition time, odom x, odom y, odom yaw.
    gyro columns: acquisition time, base-frame angular z in radians/second.
    Output uses the initial wheel frame. Both streams must cover the interval;
    gaps/reset timestamps are rejected instead of inventing intervening motion.
    """
    wheel=checked(wheel,4);gyro=checked(gyro,2)
    if min(max_wheel_gap,max_gyro_gap)<=0:raise ValueError('Require positive gap limits')
    wheel[:,3]=np.unwrap(wheel[:,3])
    wheel=wheel[(wheel[:,0]>=gyro[0,0])&(wheel[:,0]<=gyro[-1,0])]
    if len(wheel)<2:raise ValueError('No common sensor interval')
    if np.max(np.diff(wheel[:,0]))>max_wheel_gap+1e-9:raise ValueError('Wheel data gap')
    position=wheel[0,1:].copy();rows=[[wheel[0,0],*position]]
    for previous,current in zip(wheel[:-1],wheel[1:]):
        wheel_turn=current[3]-previous[3]
        if abs(wheel_turn)>=np.pi:raise ValueError('Ambiguous wheel rotation')
        mid_heading=(previous[3]+current[3])/2
        chord=np.dot(current[1:3]-previous[1:3],[np.cos(mid_heading),np.sin(mid_heading)])
        distance=chord/np.sinc(wheel_turn/(2*np.pi))
        left=np.searchsorted(gyro[:,0],previous[0],side='right')
        right=np.searchsorted(gyro[:,0],current[0],side='left')
        times=np.r_[previous[0],gyro[left:right,0],current[0]]
        indices=np.searchsorted(gyro[:,0],times[:-1],side='right')-1
        if np.any(indices<0) or np.any(times[1:]-gyro[indices,0]>max_gyro_gap+1e-9):
            raise ValueError('Stale gyro interval')
        turn=np.sum(gyro[indices,1]*np.diff(times))
        chord=distance*np.sinc(turn/(2*np.pi))
        position[:2]+=chord*np.array([np.cos(position[2]+turn/2),np.sin(position[2]+turn/2)])
        position[2]+=turn;rows.append([current[0],*position])
    return np.asarray(rows)
