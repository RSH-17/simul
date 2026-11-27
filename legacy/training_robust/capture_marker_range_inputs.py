"""Passive RGB/scan/TF evidence for marker range failures; no policy output.

Records actual observations only. Never instantiates a detector ROS node,
publishes hints, broadcasts TF, or reads scenario/evaluator target coordinates.
"""
import argparse
import json
from pathlib import Path
import time

import cv2
import numpy as np


def capture(domain,output,seconds,stop_file,max_frames):
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile,ReliabilityPolicy,qos_profile_sensor_data
    from rclpy.time import Time
    from cv_bridge import CvBridge
    from sensor_msgs.msg import CameraInfo,Image,LaserScan
    from std_msgs.msg import String
    from tf2_ros import Buffer,TransformListener
    output.mkdir(parents=True,exist_ok=False);cv2.setNumThreads(1)
    rclpy.init(domain_id=domain)
    node=rclpy.create_node('marker_range_passive_capture',parameter_overrides=[Parameter('use_sim_time',value=True)])
    buffer=Buffer(node=node);listener=TransformListener(buffer,node);bridge=CvBridge()
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    state=dict(info=None,scan=None,last_image=-1.,seen=0,saved=0,errors=[],cues=0)
    started=time.monotonic();latest_detector={};contracts=[]
    def stamp(m):return m.header.stamp.sec+m.header.stamp.nanosec/1e9
    def persist():
        result=dict(domain=domain,wall_seconds=time.monotonic()-started,seen_frames=state['seen'],
            saved_frames=state['saved'],provisional_cues=state['cues'],errors=state['errors'][-10:],
            read_only=True,commands_published=False,tf_published=False,ground_truth_used=False,
            latest_detector=latest_detector,platform_contract_samples=len(contracts),
            platform_contract_all_passed=bool(contracts) and all(contracts))
        temp=output/'progress.tmp';temp.write_text(json.dumps(result,indent=2)+'\n');temp.replace(output/'progress.json')
        return result
    def evidence(m):
        state['cues']+=1
        with (output/'provisional_cues.jsonl').open('a') as stream:stream.write(m.data+'\n')
    def status(m):
        latest_detector.clear();latest_detector.update(json.loads(m.data))
    def contract(m):
        d=json.loads(m.data);contracts.append(d.get('passed') is True)
    def image(m):
        state['seen']+=1;s=stamp(m)
        if s-state['last_image']<.2 or state['saved']>=max_frames:return
        state['last_image']=s;info=state['info'];scan=state['scan']
        if info is None or scan is None:return
        try:
            gray=bridge.imgmsg_to_cv2(m,'mono8');corners,ids,_=cv2.aruco.detectMarkers(gray,dictionary)
            if ids is None:return
            payload=dict(image=gray,ids=ids.flatten(),corners=np.asarray(corners).reshape(-1,4,2),
                camera_k=np.asarray(info.k),camera_d=np.asarray(info.d),camera_frame=m.header.frame_id,
                image_stamp=s,scan_stamp=stamp(scan),scan_ranges=np.asarray(scan.ranges),
                scan_angle_min=scan.angle_min,scan_angle_increment=scan.angle_increment,
                scan_range_min=scan.range_min,scan_range_max=scan.range_max,scan_frame=scan.header.frame_id)
            for key,parent in [('scan_from_camera',scan.header.frame_id),('map_from_camera','map')]:
                t=buffer.lookup_transform(parent,m.header.frame_id,Time.from_msg(m.header.stamp)).transform
                payload[key+'_translation']=np.array([t.translation.x,t.translation.y,t.translation.z])
                payload[key+'_rotation_xyzw']=np.array([t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w])
            file=output/f'{state["saved"]:04d}.npz';np.savez_compressed(file,**payload)
            state['saved']+=1
            with (output/'frames.jsonl').open('a') as stream:
                stream.write(json.dumps(dict(file=file.name,stamp=s,ids=ids.flatten().tolist(),scan_age_s=s-stamp(scan)))+'\n')
            persist()
        except Exception as exc:state['errors'].append(str(exc));del state['errors'][:-20]
    latest=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
    subscriptions=[
        node.create_subscription(CameraInfo,'oakd/rgb/preview/camera_info',lambda m:state.update(info=m),latest),
        node.create_subscription(LaserScan,'scan',lambda m:state.update(scan=m),qos_profile_sensor_data),
        node.create_subscription(Image,'oakd/rgb/preview/image_raw',image,latest),
        node.create_subscription(String,'mission/marker_candidates',evidence,10),
        node.create_subscription(String,'mission/detector_status',status,10),
        node.create_subscription(String,'mission/platform_status',contract,10)]
    timer=node.create_timer(2.,persist)
    try:
        while time.monotonic()-started<seconds:
            if stop_file.exists() or (contracts and not contracts[-1]):break
            rclpy.spin_once(node,timeout_sec=.1)
        report=persist();report['finished']=True
        report['stop_reason']=('platform_guard_failure' if contracts and not contracts[-1]
                               else 'episode_completed' if stop_file.exists() else 'bounded_wall_window')
        (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--domain',type=int,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--stop-file',type=Path,required=True)
    p.add_argument('--seconds',type=float,default=900.);p.add_argument('--max-frames',type=int,default=200)
    a=p.parse_args();capture(a.domain,a.output.resolve(),a.seconds,a.stop_file.resolve(),a.max_frames)
