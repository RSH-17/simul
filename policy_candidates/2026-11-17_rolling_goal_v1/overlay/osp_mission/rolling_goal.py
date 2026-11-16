"""Reconsider exploration destinations while Nav2 follows the current path.

Sensor-map data only. Cancellation must be acknowledged before ordinary mission
planning selects another destination; interrupted goals receive no failure or
coverage credit. Marker approaches and narrow-start transit remain protected.
"""
import copy
import json
import math
from pathlib import Path
import time

import numpy as np
from scipy.ndimage import distance_transform_edt

from osp_mission import dock_geometry,frontier_coverage
from osp_mission.exploration_memory import transform_pose
from osp_mission.policy import wrap


def utility(goal,weights,extra_seconds=0.):
    surface,frontier,distance,turn,revisit=goal['features']
    duration=distance/.20+turn/.45+abs(goal.get('scan_angle',0.))/.45+3.5+extra_seconds
    if not np.isfinite([surface,frontier,distance,turn,revisit,duration]).all() or duration<=0:return -math.inf
    gain=surface if goal.get('coverage_kind')=='surface' else frontier
    return gain/duration-weights[4]*revisit*.1


class RollingGoal:
    period_s=8.;minimum_hold_s=12.;cooldown_s=20.;switch_cost_s=5.
    improvement_ratio=1.4;minimum_advantage=.01;maximum_switches=6

    def __init__(self):
        self.assigned=None;self.assigned_at=-math.inf;self.last_check=-math.inf
        self.last_switch=-math.inf;self.switches=0;self.recent=[]

    def selected(self,goal,now):
        self.assigned=copy.deepcopy(goal);self.assigned_at=now

    def due(self,now):
        return (self.assigned is not None and now-self.last_check>=self.period_s
            and now-self.assigned_at>=self.minimum_hold_s and now-self.last_switch>=self.cooldown_s
            and self.switches<self.maximum_switches)

    def consider(self,choices,current,robot,weights,now,remaining,blocked=False):
        self.last_check=now
        if math.dist(robot,current[:2])<.65:return None  # Finish arrival and its observation.
        best=frontier_coverage.choose(choices,weights,remaining-self.switch_cost_s)
        if best is None or best.get('start_escape') or best.get('stationary'):return None
        if math.dist(current[:2],(best['x'],best['y']))<.8:return None
        anchor=best.get('_rolling_anchor')
        if anchor is not None and any(now-stamp<45. and math.dist(anchor,old)<.8 for old,stamp in self.recent):return None
        peers=[g for g in choices if g.get('coverage_kind')==self.assigned.get('coverage_kind')
            and math.dist(current[:2],(g['x'],g['y']))<.5 and abs(wrap(g['yaw']-current[2]))<math.pi/4]
        if not peers and not blocked:return None  # Missing sampled goal is not proof it became bad.
        old=max((utility(g,weights) for g in peers),default=0.)
        new=utility(best,weights,self.switch_cost_s)
        if not blocked and new<max(self.improvement_ratio*max(0.,old),old+self.minimum_advantage):return None
        if not math.isfinite(new) or new<=0:return None
        return dict(reason='destination_blocked' if blocked else 'substantially_better_destination',
            old_utility=old,new_utility_after_switch_cost=new,
            proposed=[best['x'],best['y'],best['yaw']],current=list(current))

    def acknowledged(self,anchor,now):
        self.last_switch=now;self.switches+=1;self.recent.append((tuple(anchor),now))
        self.recent=[r for r in self.recent if now-r[1]<45.]


class RollingGoalRuntime:
    def __init__(self,owner):
        self.owner=owner;self.policy=RollingGoal();self.pending=None
        self.last_wall=-math.inf;self.checks=0;self.errors=0;self.last_reason='not_checked'

    def event(self,event,**details):
        try:
            with (Path(self.owner.p('output_dir'))/'rolling_goal.jsonl').open('a') as stream:
                stream.write(json.dumps(dict(event=event,stamp=self.owner.now(),**details),allow_nan=False)+'\n')
        except OSError as exc:self.owner.get_logger().warning(str(exc))

    def selected(self,goal):
        self.pending=None
        self.policy.selected(goal,self.owner.now())
        self.event('selected',goal=[goal['x'],goal['y'],goal['yaw']],kind=goal.get('coverage_kind'),
            protected=any(goal.get(k) for k in ['start_escape','stationary']) or any(k in goal for k in ['focus_id','approach_id']))

    def abort(self,reason):
        if self.pending is not None:self.event('aborted',reason=reason)
        self.pending=None

    def tick(self):
        o=self.owner;now=o.now();assigned=self.policy.assigned
        if (not o.edge_mode or o.kind!='nav' or o.pending or o.goal_handle is None or o.cancel_reason
            or self.pending is not None or assigned is None
            or any(assigned.get(k) for k in ['start_escape','stationary'])
            or any(k in assigned for k in ['focus_id','approach_id'])
            or (getattr(o,'near_pass',None) is not None and o.near_pass.task is not None)
            or not self.policy.due(now) or time.monotonic()-self.last_wall<3.):return False
        self.last_wall=time.monotonic();self.policy.last_check=now
        try:
            from rclpy.time import Time
            grid=o.map;data=np.asarray(grid.data).reshape(grid.info.height,grid.info.width).copy()
            resolution=grid.info.resolution;origin=(grid.info.origin.position.x,grid.info.origin.position.y)
            dock=o.known_dock_pose(o.p('map_frame'))
            if dock is not None:dock_geometry.mark(data,resolution,origin,dock)
            base=o.tf.lookup_transform(o.p('map_frame'),o.p('base_frame'),Time()).transform
            q=base.rotation;heading=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            robot=(base.translation.x,base.translation.y)
            viewed=o.coverage.grid(data,resolution,origin)
            calibration=getattr(o,'inspection_camera',None)
            if calibration is None:return False
            choices,_=o.policy_options(data,resolution,origin,robot,heading,(),viewed,inspection_focal=calibration[0])
            mt=o.tf.lookup_transform(o.p('odom_frame'),o.p('map_frame'),Time()).transform;mq=mt.rotation
            transform=(mt.translation.x,mt.translation.y,math.atan2(2*(mq.w*mq.z+mq.x*mq.y),1-2*(mq.y*mq.y+mq.z*mq.z)))
            # Filtering a hypothetical decision must not mutate live mission memory.
            choices=copy.deepcopy(o.exploration_memory).filter(choices,transform,now)
            for goal in choices:goal['_rolling_anchor']=transform_pose(goal['x'],goal['y'],goal['yaw'],transform)[:2]
            target=o.active_goal.pose.pose;current=(target.position.x,target.position.y,
                math.atan2(2*target.orientation.w*target.orientation.z,1-2*target.orientation.z**2))
            x,y=np.floor((np.asarray(current[:2])-origin)/resolution).astype(int)
            inside=0<=y<data.shape[0] and 0<=x<data.shape[1]
            clearance=distance_transform_edt(np.pad(data==0,1))[1:-1,1:-1]*resolution
            # Occupied or unknown endpoints cannot remain navigation goals.
            blocked=not inside or data[y,x]!=0 or clearance[y,x]<.32
            choice=self.policy.consider(choices,current,robot,o.weights,now,o.p('duration')-(now-o.started),blocked)
            self.checks+=1;self.last_reason=choice['reason'] if choice else 'retain_current_destination'
            self.event('evaluated',decision=choice,choices=len(choices),blocked=bool(blocked))
            if choice is None:return False
            self.pending=dict(choice,old_anchor=transform_pose(*current,transform)[:2])
            self.event('cancel_requested',**self.pending)
            o.cancel_action('rolling_goal_reselect')
            return True
        except Exception as exc:
            self.errors+=1;self.last_reason='observation_unavailable'
            o.get_logger().debug('Rolling goal observation unavailable: '+str(exc))
            return False

    def completed(self,status,kind,reason):
        if self.pending is None or kind!='nav':return False
        task,self.pending=self.pending,None
        if status!=5 or reason!='rolling_goal_reselect':
            self.event('cancel_race',status=status,reason=reason);return False
        o=self.owner;self.policy.acknowledged(task['old_anchor'],o.now())
        o.exploration_memory.pending=None  # Neither a failed visit nor observed coverage.
        o.resume_action=None;o.scan_after=False;o.plan_after=o.now()+.25
        o.marker_look.clear('rolling_goal_reselect');o.face_look.clear('rolling_goal_reselect')
        self.event('cancel_acknowledged',switches=self.policy.switches)
        return True

    def report(self):
        return dict(enabled=True,checks=self.checks,switches=self.policy.switches,errors=self.errors,
            pending=self.pending is not None,last_reason=self.last_reason,period_s=self.policy.period_s,
            minimum_hold_s=self.policy.minimum_hold_s,cooldown_s=self.policy.cooldown_s,
            improvement_ratio=self.policy.improvement_ratio,maximum_switches=self.policy.maximum_switches)
