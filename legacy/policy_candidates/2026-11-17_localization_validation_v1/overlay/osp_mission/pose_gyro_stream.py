"""Causal shadow integration with strict acquisition-order and gap checks."""
import numpy as np
from osp_mission.pose_increment_gyro import integrate


class PoseGyroStream:
    def __init__(self):
        self.gyro=[];self.wheels=[];self.previous=None;self.pose=None;self.last_wheel=None
        self.fault=None;self.duplicates=0

    def fail(self,reason):
        self.fault=reason;raise ValueError(reason)

    def imu(self,stamp,rate):
        if self.fault:raise ValueError(self.fault)
        row=np.array([stamp,rate],float)
        if not np.isfinite(row).all():self.fail('Nonfinite gyro')
        if self.gyro and stamp<=self.gyro[-1][0]:
            if np.array_equal(row,self.gyro[-1]):self.duplicates+=1;return []
            self.fail('Gyro reset or conflicting timestamp')
        if self.gyro and stamp-self.gyro[-1][0]>.05+1e-9:self.fail('Gyro gap')
        self.gyro.append(row)
        # A missing wheel stream must not grow memory indefinitely.
        if len(self.gyro)>2000:self.fail('Wheel stream absent')
        return self.drain()

    def wheel(self,row):
        if self.fault:raise ValueError(self.fault)
        row=np.array(row,float)
        if row.shape!=(4,) or not np.isfinite(row).all():self.fail('Invalid wheel sample')
        if self.last_wheel is not None and row[0]<=self.last_wheel[0]:
            if np.array_equal(row,self.last_wheel):self.duplicates+=1;return []
            self.fail('Wheel reset or conflicting timestamp')
        if self.last_wheel is not None and row[0]-self.last_wheel[0]>.1+1e-9:self.fail('Wheel gap')
        self.last_wheel=row;self.wheels.append(row)
        if len(self.wheels)>100:self.fail('Gyro stream absent')
        return self.drain()

    def drain(self):
        outputs=[]
        while self.wheels and len(self.gyro)>=2 and self.wheels[0][0]<=self.gyro[-1][0]:
            wheel=self.wheels.pop(0)
            if wheel[0]<self.gyro[0][0]:continue
            if self.previous is None:
                self.previous=wheel;self.pose=wheel[1:].copy();outputs.append([wheel[0],*self.pose])
            else:
                previous=self.previous
                try:relative=integrate([previous,wheel],self.gyro)
                except ValueError as exc:self.fail(str(exc))
                if len(relative)!=2:self.fail('Uncovered wheel interval')
                delta=relative[-1,1:3]-relative[0,1:3]
                angle=self.pose[2]-previous[3];c,s=np.cos(angle),np.sin(angle)
                self.pose[:2]+=np.array([[c,-s],[s,c]])@delta
                turn=(relative[-1,3]-relative[0,3]);self.pose[2]+=turn
                self.previous=wheel;outputs.append([wheel[0],*self.pose])
            before=[i for i,g in enumerate(self.gyro) if g[0]<=wheel[0]]
            if before:self.gyro=self.gyro[before[-1]:]
        return outputs
