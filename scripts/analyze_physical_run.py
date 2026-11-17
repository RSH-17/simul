"""Analyze a finalized physical rosbag offline; requires the Jazzy Python environment."""
import argparse
import json,math,collections,bisect
from pathlib import Path
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--run',type=Path,required=True)
parser.add_argument('--output',type=Path,required=True)
args=parser.parse_args()
run=args.run.resolve()
result=json.loads(next((run/'artifacts').rglob('result.json')).read_text())
events=[json.loads(x) for x in (run/'artifacts/startup.jsonl').read_text().splitlines()]
start=next(x['wall_epoch_s'] for x in events if x['event']=='mission_started');end=start+result['elapsed_seconds']
reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(run/'rosbag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
types={x.name:x.type for x in reader.get_all_topics_and_types()}
selected=['/odom','/cmd_vel','/mission/status','/hazard_detection','/mission/detector_status','/mission/marker_observations','/mission/marker_candidates','/tf','/collision_monitor_state','/scan']
reader.set_filter(rosbag2_py.StorageFilter(topics=selected));classes={t:get_message(types[t]) for t in selected if t in types}
odom=[];cmd=[];statuses=[];tf=[];ages=collections.defaultdict(list);counts=collections.Counter();hazards=collections.Counter();hazard_events=[];last_hazard=None;detector={};obs=collections.defaultdict(list);candidates=collections.Counter();collision=[]
while reader.has_next():
 topic,data,ns=reader.read_next();t=ns/1e9
 if t<start or t>end:continue
 m=deserialize_message(data,classes[topic]);counts[topic]+=1
 if hasattr(m,'header'):
  stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9
  ages[topic].append(t-stamp)
 if topic=='/odom':odom.append((t,m.pose.pose.position.x,m.pose.pose.position.y,m.twist.twist.linear.x,m.twist.twist.angular.z))
 elif topic=='/cmd_vel':cmd.append((t,m.twist.linear.x,m.twist.angular.z))
 elif topic=='/mission/status':statuses.append((t,json.loads(m.data)))
 elif topic=='/mission/detector_status':detector=json.loads(m.data)
 elif topic=='/hazard_detection':
  current=tuple((int(x.type),x.header.frame_id) for x in m.detections)
  for k,_ in current:hazards[k]+=1
  if current!=last_hazard:hazard_events.append(dict(time_s=t-start,signals=current));last_hazard=current
 elif topic=='/mission/marker_observations':
  v=json.loads(m.data);obs[str(v['id'])].append(v)
 elif topic=='/mission/marker_candidates':candidates[str(json.loads(m.data)['id'])]+=1
 elif topic=='/collision_monitor_state':collision.append(dict(time_s=t-start,action=int(m.action_type),polygon=m.polygon_name))
 elif topic=='/tf':
  for v in m.transforms:
   if v.header.frame_id.strip('/')=='map' and v.child_frame_id.strip('/')=='odom':
    q=v.transform.rotation;tf.append((t,v.transform.translation.x,v.transform.translation.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))))
def stat(values):
 a=np.asarray(values,dtype=float)
 return dict(n=len(a),min=float(a.min()),median=float(np.median(a)),p95=float(np.quantile(a,.95)),max=float(a.max())) if len(a) else None
motion=collections.Counter();stops=[];active=None;path=0.;rotation=0.;speed_int=0.;odom_gaps=[];state_times=[x[0] for x in statuses];cmd_times=[x[0] for x in cmd]
for a,b in zip(odom,odom[1:]):
 dt=b[0]-a[0]
 if dt<=0:continue
 if dt>.5:odom_gaps.append(dt);continue
 v,w=abs(a[3]),abs(a[4]);phase='translation' if v>=.02 else 'rotation_only' if w>=.05 else 'stationary'
 motion[phase]+=dt;path+=math.hypot(b[1]-a[1],b[2]-a[2]);rotation+=w*dt;speed_int+=v*dt
 if phase=='stationary':
  if active is None:active={'start':a[0],'end':b[0],'nav':0.,'other':0.,'max_command_linear':0.}
  active['end']=b[0];i=bisect.bisect_right(state_times,a[0])-1
  if i>=0:active['nav' if statuses[i][1].get('active_action')=='nav' else 'other']+=dt
  j=bisect.bisect_right(cmd_times,a[0])-1
  if j>=0 and a[0]-cmd[j][0]<.5:active['max_command_linear']=max(active['max_command_linear'],abs(cmd[j][1]))
 elif active is not None:
  if active['end']-active['start']>=1:stops.append(active)
  active=None
if active and active['end']-active['start']>=1:stops.append(active)
for x in stops:x['duration_s']=x.pop('end')-x['start'];x['start_s']=x.pop('start')-start
jumps=[]
for a,b in zip(tf,tf[1:]):
 d=math.hypot(b[1]-a[1],b[2]-a[2]);yaw=abs(math.atan2(math.sin(b[3]-a[3]),math.cos(b[3]-a[3])))
 if d>.1 or yaw>.1:jumps.append(dict(time_s=b[0]-start,translation_m=d,yaw_rad=yaw))
summary=dict(run_id=run.name,scope='offline_recorded_ros_messages_no_ground_truth',interval=dict(start_epoch_s=start,end_epoch_s=end,duration_s=end-start),message_counts=dict(counts),motion=dict(duration_s=dict(motion),odom_position_path_m=path,integrated_abs_linear_speed_m=speed_int,integrated_abs_yaw_rad=rotation,translation_speed_m_s=stat([abs(v[3]) for v in odom if abs(v[3])>=.02]),all_linear_speed_m_s=stat([abs(v[3]) for v in odom]),odom_gap_over_half_second_s=odom_gaps,stationary_thresholds=dict(linear_m_s=.02,angular_rad_s=.05),stationary_episodes_over_1s=sorted(stops,key=lambda x:x['duration_s'],reverse=True)),hazard_sample_counts=dict(hazards),hazard_state_changes=hazard_events,collision_monitor_changes=collision,receive_minus_header_age_s={k:stat(v) for k,v in ages.items()},last_mission_status=statuses[-1][1] if statuses else {},last_detector_status=detector,observations={k:dict(frames=len(rows),first_s=rows[0]['stamp']-start,last_s=rows[-1]['stamp']-start,min_edge_px=stat([v['min_edge_pixels'] for v in rows]),range_m=stat([float(np.linalg.norm(v['camera_point'])) for v in rows if v.get('camera_point') is not None]),example=rows[0]) for k,rows in obs.items()},candidate_counts=dict(candidates),map_to_odom=dict(samples=len(tf),corrections_over_10cm_or_0_1rad=sorted(jumps,key=lambda x:x['translation_m'],reverse=True)[:20]))

phase_action=collections.Counter();speed_bins=collections.Counter();cmd_exceed=[]
for a,b in zip(odom,odom[1:]):
 dt=b[0]-a[0]
 if not 0<dt<=.5:continue
 v,w=abs(a[3]),abs(a[4]);phase='translation' if v>=.02 else 'rotation_only' if w>=.05 else 'stationary'
 j=bisect.bisect_right(state_times,a[0])-1
 action=statuses[j][1].get('active_action','none') if j>=0 else 'unknown'
 phase_action[phase+':'+(action or 'none')]+=dt
 for bound in [.12,.20,.25,.275,.30]:
  if v>bound:speed_bins[str(bound)]+=dt
ctimes=[x[0] for x in odom];effects=[]
for a,b in zip(tf,tf[1:]):
 origin=math.hypot(b[1]-a[1],b[2]-a[2]);j=bisect.bisect_right(ctimes,b[0])-1
 if j<0:continue
 x,y=odom[j][1:3]
 def transformed(tr,p):
  return np.array([tr[1]+math.cos(tr[3])*p[0]-math.sin(tr[3])*p[1],tr[2]+math.sin(tr[3])*p[0]+math.cos(tr[3])*p[1]])
 effect=float(np.linalg.norm(transformed(b,(x,y))-transformed(a,(x,y))))
 effects.append(dict(time_s=b[0]-start,origin_shift_m=origin,correction_at_robot_m=effect,yaw_rad=abs(math.atan2(math.sin(b[3]-a[3]),math.cos(b[3]-a[3])))))
summary['motion']['phase_by_active_action_s']=dict(phase_action)
summary['motion']['time_above_linear_speed_s']=dict(speed_bins)
summary['command_limits']=dict(linear_abs=stat([abs(v[1]) for v in cmd]),angular_abs=stat([abs(v[2]) for v in cmd]))
summary['map_to_odom']['effects_at_current_robot_top']=sorted(effects,key=lambda x:x['correction_at_robot_m'],reverse=True)[:10]
summary['map_to_odom']['largest_origin_shift_effect']=max(effects,key=lambda x:x['origin_shift_m']) if effects else None
summary['map_to_odom']['landmark_first_to_final_transform_difference_m']={}
for ident,rows in obs.items():
 p=rows[0].get('odom_point'); t=rows[0]['stamp'];times=[x[0] for x in tf];idx=bisect.bisect_right(times,t)-1
 if p is not None and idx>=0 and tf:summary['map_to_odom']['landmark_first_to_final_transform_difference_m'][ident]=float(np.linalg.norm(transformed(tf[-1],p)-transformed(tf[idx],p)))
summary['method_notes']=['Uses receipt-time mission interval; no ground-truth trajectory or fixed marker survey.', 'Translation is |v|>=0.02 m/s; otherwise rotation-only is |w|>=0.05 rad/s.', 'Header ages are receipt minus message header, not pure network latency.', 'map->odom origin translation is not robot pose jump; the actual current odom point is transformed under both transforms.', 'Re-expressing historical odom landmarks with the final transform is sensitivity analysis, not a valid ground-truth correction.']
assert abs(sum(motion.values())-(end-start))<1., 'Motion integration coverage gap'
assert sum(len(rows) for rows in obs.values())==counts['/mission/marker_observations']
args.output.parent.mkdir(parents=True,exist_ok=True)
args.output.write_text(json.dumps(summary,indent=2))
print(json.dumps(dict(output=str(args.output),motion_seconds=dict(motion),command_limits=summary['command_limits'],largest_map_correction_at_robot_m=max([x['correction_at_robot_m'] for x in effects],default=0))))
