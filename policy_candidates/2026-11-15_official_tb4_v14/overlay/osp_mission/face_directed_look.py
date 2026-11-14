"""Finish an observed-face sweep in the same map frame as its Nav2 goal."""
import json
import math
from pathlib import Path

from rclpy.time import Time


class FaceDirectedLook:
    def __init__(self,owner):
        self.owner=owner;self.pending=None;self.executed=0;self.unavailable=0

    def event(self,event,**values):
        try:
            with (Path(self.owner.p('output_dir'))/'face_directed_looks.jsonl').open('a') as stream:
                stream.write(json.dumps(dict(event=event,stamp=self.owner.now(),**values),allow_nan=False)+'\n')
        except (OSError,ValueError) as exc:self.owner.get_logger().warning(str(exc))

    def clear(self,reason):
        if self.pending is not None:self.event('cleared',reason=reason)
        self.pending=None

    def select(self,selected,map_to_odom):
        self.clear('new_selection')
        if not selected or not selected.get('face_inspection') or any(k in selected for k in ('approach_id','focus_id')):return
        self.pending=dict(end_map_yaw=None)
        try:
            requested=float(selected['scan_angle']);yaw=float(selected['yaw'])
            if not all(math.isfinite(v) for v in (requested,yaw)) or not 0<requested<=2*math.pi/3:
                raise ValueError('Invalid bounded inspection sweep')
            # Nav2 retains this map-frame goal during the trip. Fixing its
            # observation endpoint in odom instead would add all encoder drift
            # accumulated before arrival to the requested viewing direction.
            self.pending=dict(end_map_yaw=yaw+requested,planned_sweep_rad=requested)
        except (TypeError,ValueError,IndexError,KeyError):pass
        self.event('selected',**self.pending)

    def consume(self):
        task,self.pending=self.pending,None
        if task is None:return None
        try:
            if task['end_map_yaw'] is None:raise ValueError('Missing map inspection orientation')
            q=self.owner.tf.lookup_transform(self.owner.p('map_frame'),self.owner.inspection_camera_frame,Time()).transform.rotation
            heading=math.atan2(2*(q.y*q.z-q.w*q.x),2*(q.x*q.z+q.w*q.y))
            delta=task['end_map_yaw']-heading
            if not math.isfinite(delta):raise ValueError('Nonfinite camera orientation')
            turn=math.atan2(math.sin(delta),math.cos(delta))
            turn=max(-2*math.pi/3,min(2*math.pi/3,turn))
        except Exception as exc:
            self.unavailable+=1;self.event('unavailable',reason=str(exc));return 0.
        self.executed+=1;self.event('consumed',turn_rad=turn,**task)
        self.owner.get_logger().info(f'Planned face inspection turn: {math.degrees(turn):.2f} deg')
        return turn

    def report(self):
        return dict(version=2,frame='map',frame_parameter='map_frame',maximum_turn_rad=2*math.pi/3,executed=self.executed,
                    unavailable=self.unavailable,creates_coverage_or_recognition_credit=False,
                    assumption='Existing robot camera optical heading is aligned with body heading')
