"""Passive, strict simulation judge; independent pose is never sent to policy."""
import csv
import json
import math
import os
from pathlib import Path
import sys

import numpy as np
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import MarkerArray

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent/'2026-11-20_v12_camera_rate'))
from trial import PerformanceTrial, main


class StrictTrial(PerformanceTrial):
    def __init__(self,*args):
        super().__init__(*args)
        self.judge_tf=Buffer();self.judge_listener=TransformListener(self.judge_tf,self.node)
        self.landmarks={};self.world_alignment=None;self.anchor_sample=None
        self.external_near_at=None;self.detector_status={};self.scan_min=math.inf
        self.node.create_subscription(MarkerArray,'/targets',self.target_positions,10)
        from sensor_msgs.msg import LaserScan
        from rclpy.qos import qos_profile_sensor_data
        self.node.create_subscription(LaserScan,'/scan',self.scan_range,qos_profile_sensor_data)

    def target_positions(self,msg):
        self.landmarks={m.id:(m.pose.position.x,m.pose.position.y) for m in msg.markers if m.ns=='point'}

    def scan_range(self,msg):
        valid=[r for r in msg.ranges if math.isfinite(r) and msg.range_min<r<msg.range_max]
        if self.start_sim is not None and valid:self.scan_min=min(self.scan_min,min(valid))

    def on_detector(self,msg):
        self.detector_status=json.loads(msg.data)
        super().on_detector(msg)

    def truth(self,msg):
        super().truth(msg)
        if self.start_sim is None:return
        for tf in msg.transforms:
            if tf.child_frame_id!='turtlebot4':continue
            if self.anchor_sample is None:
                p,q=tf.transform.translation,tf.transform.rotation
                yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                self.anchor_sample=(tf.header.stamp,np.array([p.x,p.y]),yaw)
            if self.world_alignment is None:
                try:
                    stamp,position,world_yaw=self.anchor_sample
                    tr=self.judge_tf.lookup_transform('map','base_link',Time.from_msg(stamp)).transform
                    q=tr.rotation;map_yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                    angle=world_yaw-map_yaw;c,s=math.cos(angle),math.sin(angle)
                    rotation=np.array([[c,-s],[s,c]])
                    offset=position-rotation@np.array([tr.translation.x,tr.translation.y])
                    self.world_alignment=dict(rotation=rotation.tolist(),translation=offset.tolist(),
                                              exposure_s=stamp.sec+stamp.nanosec/1e9)
                except Exception:pass
        if self.pair_at is not None:
            if self.external_near_at is None:self.external_near_at=self.pair_at
            if not self.sensor_proof_complete():self.pair_at=None

    def sensor_proof_complete(self):
        return set(self.targets)==self.current and set(self.targets)<=set(self.status.get('approached_ids',[]))

    def run(self):
        result=super().run()
        expected=set(self.targets)
        # Report independent distance, sensor witness and fixed-frame map error
        # separately. Do not weaken the policy to make the evaluator pass.
        errors={}
        if self.world_alignment:
            rotation=np.array(self.world_alignment['rotation']);offset=np.array(self.world_alignment['translation'])
            errors={str(i):float(np.linalg.norm(rotation@np.array(p)+offset-np.array(self.targets[i])))
                    for i,p in self.landmarks.items() if i in expected}
        self.file.flush()
        with (self.path/'trajectory.csv').open() as stream:
            rows=list(csv.DictReader(stream))
        positions=np.array([[float(row['world_x']),float(row['world_y'])] for row in rows])
        times=np.array([float(row['elapsed_sim_s']) for row in rows])
        distances=np.linalg.norm(np.diff(positions,axis=0),axis=1);dt=np.diff(times)
        stopped=float(dt[(dt>0)&(distances/np.maximum(dt,1e-9)<.02)].sum())
        near=expected<=self.witness.keys();proof=self.sensor_proof_complete()
        map_ok=set(map(int,errors))==expected and max(errors.values(),default=math.inf)<=.25
        strict=bool(near and proof and map_ok and self.bumps==0 and self.current==expected)
        return dict(result,variant=os.environ['CAMERA_VARIANT'],profile=os.environ['CAMERA_PROFILE'],
            evaluation_protocol='3 correct IDs; independent <=1.5m; unchanged three-frame <=1.3m policy sensor proof; fixed-alignment map error <=0.25m; zero bumps',
            strict_success=strict,independent_approach_complete=near,policy_sensor_proof_complete=proof,
            independent_approach_time_s=self.external_near_at,marker_position_errors_m=errors,
            world_from_map_initial_alignment=self.world_alignment,map_accuracy_pass=map_ok,
            path_length_m=float(distances.sum()),stationary_time_s=stopped,
            minimum_valid_scan_range_m=self.scan_min if math.isfinite(self.scan_min) else None,
            unexpected_ids=sorted(self.current-expected),last_detector_status=self.detector_status)


if __name__=='__main__':main(StrictTrial,HERE/'camera.launch.py')
