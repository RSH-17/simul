"""Read-only paired detector experiment on the same live sensor receipts.

No Detector node is constructed. Production methods run on plain owners whose
outputs are local callbacks. The only ROS node owns subscribers and a TF
listener, never targets, hints, camera evidence, robot commands or TF outputs.
This is a shadow observation, not a native policy trial or accuracy validation.
"""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import time
from types import MethodType, SimpleNamespace as S


def detector_class(workspace, name):
    source=workspace/'src/osp_mission/osp_mission/detector.py'
    spec=importlib.util.spec_from_file_location(name,source)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module.Detector,hashlib.sha256(source.read_bytes()).hexdigest()


def owner(cls,node,buffer,output):
    import cv2
    from cv_bridge import CvBridge
    from osp_mission.landmark_reacquisition import ObservationConsensus
    from osp_mission.provisional_marker_cues import ProvisionalMarkerCues
    from osp_mission.exposure_queue import ExposureQueue
    parameters=dict(map_frame='map',odom_frame='odom',confirmations=3,min_marker_pixels=18)
    n=S(tf=buffer,bridge=CvBridge(),info=None,scan=None,depth=None,
        provisional=ProvisionalMarkerCues(),confirmation_guard=ObservationConsensus(),
        observations={},id_diagnostics={},exposures=ExposureQueue(),
        dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL),
        p=lambda key:parameters[key],get_clock=node.get_clock,get_logger=node.get_logger,
        diagnostics=dict(frames=0,decoded={},small=0,border=0,no_range=0,horizontal=0,
                         inconsistent=0,accepted=0,errors=0,last_error=''))
    n.processed_camera=S(publish=lambda m:output('processed_camera',dict(
        stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9)))
    n.candidate_pub=S(publish=lambda m:output('cue',json.loads(m.data)))
    n.observation_pub=S(get_subscription_count=lambda:1,
                        publish=lambda m:output('observation',json.loads(m.data)))
    for name in ('on_rgb','lidar_point','reject','queue_rgb','drain_rgb'):
        if hasattr(cls,name):setattr(n,name,MethodType(getattr(cls,name),n))
    return n


def owned_publishers(node):
    return sorted(name for name,_ in node.get_publisher_names_and_types_by_node(
        node.get_name(),node.get_namespace()))


def run(baseline,candidate,domain,output,seconds,stop_file):
    import cv2
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile,ReliabilityPolicy,qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import CameraInfo,Image,LaserScan
    from std_msgs.msg import String
    from tf2_ros import Buffer,TransformListener

    output.mkdir(parents=True,exist_ok=False);cv2.setNumThreads(1)
    sys.path.insert(0,str(candidate/'src/osp_mission'))
    old,old_hash=detector_class(baseline,'paired_original_detector')
    new,new_hash=detector_class(candidate,'paired_buffered_detector')
    rclpy.init(domain_id=domain)
    node=rclpy.create_node('exposure_pair_observer',enable_rosout=False,start_parameter_services=False,
        parameter_overrides=[Parameter('use_sim_time',value=True)])
    buffer=Buffer(node=node);listener=TransformListener(buffer,node)
    started=time.monotonic();contracts=[];received=0;last_receipt=None
    counts={role:dict(processed_camera=0,cue=0,observation=0) for role in ('original','buffered')}

    def local_output(role,kind,data):
        counts[role][kind]+=1
        if kind!='processed_camera':
            with (output/'observations.jsonl').open('a') as stream:
                stream.write(json.dumps(dict(role=role,kind=kind,**data))+'\n')

    original=owner(old,node,buffer,lambda kind,data:local_output('original',kind,data))
    buffered=owner(new,node,buffer,lambda kind,data:local_output('buffered',kind,data))
    owners={'original':original,'buffered':buffered}

    def sensor(name,message):
        for n in owners.values():setattr(n,name,message)

    def rgb(message):
        nonlocal received,last_receipt
        received+=1;stamp=Time.from_msg(message.header.stamp)
        last_receipt=stamp.nanoseconds/1e9
        ready=buffer.can_transform('map',message.header.frame_id,stamp)
        # Both see the same TF buffer state at receipt. Original processing may
        # spend its inherited 50 ms timeout before the queued path is called.
        # Receiver scheduling differs from the mission's own process.
        original.on_rgb(message);buffered.queue_rgb(message)
        with (output/'receipts.jsonl').open('a') as stream:
            stream.write(json.dumps(dict(stamp=last_receipt,map_tf_ready_at_receipt=ready,
                                         original_errors=original.diagnostics['errors']))+'\n')

    def contract(message):contracts.append(json.loads(message.data).get('passed') is True)

    def report(finished=False,stop_reason=None):
        publishers=owned_publishers(node)
        no_policy_outputs=all(p=='/parameter_events' for p in publishers)
        result=dict(evidence_type='Live paired shadow observations; not a new policy trajectory or accuracy result',
            domain=domain,wall_seconds=time.monotonic()-started,received_frames=received,
            last_receipt_sim_s=last_receipt,finished=finished,stop_reason=stop_reason,
            source_hashes=dict(original=old_hash,buffered=new_hash),
            observed_publishers=publishers,no_policy_outputs=no_policy_outputs,
            platform_samples=len(contracts),platform_all_passed=bool(contracts) and all(contracts),
            ground_truth_used=False,production_promoted=False,
            detectors={role:dict(diagnostics=n.diagnostics,local_outputs=counts[role],
                confirmation_samples={str(k):len(v) for k,v in n.observations.items()}) for role,n in owners.items()},
            limits=['Different receiver timing from the mission; no exact production error-count comparison',
                    'Original method runs first and can delay the queued callback by its inherited timeout',
                    'More observations do not prove correct IDs, coordinates or improved closed-loop success'])
        temporary=output/'progress.tmp';temporary.write_text(json.dumps(result,indent=2)+'\n')
        temporary.replace(output/'progress.json')
        return result

    latest=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
    subscriptions=[
        node.create_subscription(CameraInfo,'oakd/rgb/preview/camera_info',lambda m:sensor('info',m),latest),
        node.create_subscription(LaserScan,'scan',lambda m:sensor('scan',m),qos_profile_sensor_data),
        node.create_subscription(Image,'unused_depth',lambda m:sensor('depth',m),latest),
        node.create_subscription(Image,'oakd/rgb/preview/image_raw',rgb,latest),
        node.create_subscription(String,'mission/platform_status',contract,10)]
    timer=node.create_timer(.02,buffered.drain_rgb);last_report=0.
    try:
        stop='bounded_wall_window'
        while time.monotonic()-started<seconds:
            if stop_file.exists():stop='episode_completed';break
            if contracts and not contracts[-1]:stop='platform_guard_failure';break
            rclpy.spin_once(node,timeout_sec=.05)
            if time.monotonic()-last_report>=2.:
                state=report();last_report=time.monotonic()
                if not state['no_policy_outputs']:stop='unexpected_owned_publisher';break
        state=report(True,stop)
        (output/'report.json').write_text(json.dumps(state,indent=2)+'\n')
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('baseline','candidate','output','stop-file'):parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--domain',type=int,required=True);parser.add_argument('--seconds',type=float,default=1500.)
    args=parser.parse_args()
    run(args.baseline.resolve(),args.candidate.resolve(),args.domain,args.output.resolve(),args.seconds,args.stop_file.resolve())
