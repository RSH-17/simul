"""Passive live map and Nav2 plan display; no robot commands or evaluator data."""
import json
import math
import time

import cv2
import numpy as np

BLUE=(94,48,20)  # BGR: deep blue unknown space and panel background.


def render_map(grid,resolution,origin,plans,robot=None,trail=(),laser=(),targets=()):
    canvas=np.full((720,720,3),BLUE,np.uint8)
    height,width=grid.shape
    scale=min(680/width,620/height)
    size=(max(1,round(width*scale)),max(1,round(height*scale)))
    left=(720-size[0])//2;top=70+(620-size[1])//2
    colors=np.full((*grid.shape,3),BLUE,np.uint8)
    colors[(grid>=0)&(grid<=50)]=(228,215,192)
    colors[grid>50]=(39,27,15)
    canvas[top:top+size[1],left:left+size[0]]=cv2.resize(colors[::-1],size,interpolation=cv2.INTER_NEAREST)
    ox,oy,yaw=origin;c,s=math.cos(yaw),math.sin(yaw)
    def pixel(point):
        dx,dy=point[0]-ox,point[1]-oy
        gx=(c*dx+s*dy)/resolution;gy=(-s*dx+c*dy)/resolution
        return (round(left+gx*size[0]/width),round(top+(height-gy)*size[1]/height))
    def line(points,color,thickness):
        if len(points)>1:
            cv2.polylines(canvas,[np.array([pixel(p) for p in points],np.int32)],False,color,thickness,cv2.LINE_AA)
    line(trail,(104,203,93),2)
    for point in laser:cv2.circle(canvas,pixel(point),1,(255,235,45),-1)
    for kind,color in [('global',(0,215,255)),('local',(245,130,255))]:
        plan=plans.get(kind,{})
        points=plan.get('points',[])
        line(points,color,3 if kind=='global' else 2)
        if points:cv2.circle(canvas,pixel(points[-1]),6,color,2)
    if robot:
        point=pixel(robot);cv2.circle(canvas,point,6,(90,90,255),-1)
        cv2.arrowedLine(canvas,point,pixel((robot[0]+.4*math.cos(robot[2]),robot[1]+.4*math.sin(robot[2]))),(90,90,255),2)
    for ident,x,y in targets:
        p=pixel((x,y));cv2.circle(canvas,p,7,(20,160,255),2)
        cv2.putText(canvas,str(ident),(p[0]+9,p[1]),cv2.FONT_HERSHEY_SIMPLEX,.45,(20,160,255),1,cv2.LINE_AA)
    cv2.putText(canvas,'LIVE MAP / NAV2 PLANS',(20,28),cv2.FONT_HERSHEY_SIMPLEX,.65,(255,246,225),1,cv2.LINE_AA)
    counts='  '.join(f'{kind.upper()} #{plans.get(kind,{}).get("sequence",0)}' for kind in ['global','local'])
    cv2.putText(canvas,counts,(20,52),cv2.FONT_HERSHEY_SIMPLEX,.5,(255,246,225),1,cv2.LINE_AA)
    return canvas


class MapObserver:
    def __init__(self,node,output):
        from nav_msgs.msg import OccupancyGrid,Path
        from sensor_msgs.msg import LaserScan
        from visualization_msgs.msg import MarkerArray
        from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy,qos_profile_sensor_data
        from rclpy.clock import Clock,ClockType
        from tf2_ros import Buffer,TransformListener
        self.node=node;self.output=output;self.grid=None;self.plans={};self.counts={};self.trail=[];self.laser=[]
        self.errors={};self.targets=[];self.buffer=Buffer(node=node);self.listener=TransformListener(self.buffer,node)
        self.subscriptions=[node.create_subscription(OccupancyGrid,'/map',self.map,
            QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)),
            node.create_subscription(Path,'/plan',lambda m:self.plan(m,'global'),qos_profile_sensor_data),
            node.create_subscription(Path,'/local_plan',lambda m:self.plan(m,'local'),qos_profile_sensor_data),
            node.create_subscription(LaserScan,'/scan',self.scan,qos_profile_sensor_data),
            node.create_subscription(MarkerArray,'/targets',self.markers,10)]
        self.timer=node.create_timer(.5,self.render,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def transform(self,frame,stamp):
        from rclpy.time import Time
        if frame=='map':return 0.,0.,0.
        tr=self.buffer.lookup_transform('map',frame,Time.from_msg(stamp)).transform
        q=tr.rotation
        return tr.translation.x,tr.translation.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

    @staticmethod
    def project(points,transform):
        x,y,yaw=transform;c,s=math.cos(yaw),math.sin(yaw)
        return [[x+c*px-s*py,y+s*px+c*py] for px,py in points]

    def map(self,message):
        if message.header.frame_id!='map':return
        self.grid=message

    def markers(self,message):
        self.targets=[(m.id,m.pose.position.x,m.pose.position.y) for m in message.markers
            if m.ns=='point' and m.header.frame_id=='map' and m.action==0]

    def plan(self,message,kind):
        self.counts[kind]=self.counts.get(kind,0)+1
        stamp=message.header.stamp.sec+message.header.stamp.nanosec/1e9
        try:
            frames={pose.header.frame_id or message.header.frame_id for pose in message.poses}
            if frames and frames!={message.header.frame_id}:raise ValueError('Mixed plan frames')
            points=[[p.pose.position.x,p.pose.position.y] for p in message.poses]
            if points and not np.isfinite(points).all():raise ValueError('Nonfinite plan')
            if points:points=self.project(points,self.transform(message.header.frame_id,message.header.stamp))
            plan=dict(sequence=self.counts[kind],source_stamp=stamp,received_unix=time.time(),points=points,frame='map')
            self.plans[kind]=plan;self.errors.pop(kind,None)
            with (self.output/'plans.jsonl').open('a') as stream:
                stream.write(json.dumps(dict(kind=kind,**plan))+'\n')
            self.render()  # Every received recalculation updates the displayed path.
        except Exception as exc:
            self.plans.pop(kind,None);self.errors[kind]=str(exc)

    def scan(self,message):
        try:
            now=time.monotonic()
            if now-getattr(self,'last_scan',0)<.5:return
            self.last_scan=now
            points=[]
            for i,r in enumerate(message.ranges):
                if math.isfinite(r) and message.range_min<=r<=message.range_max:
                    a=message.angle_min+i*message.angle_increment;points.append([r*math.cos(a),r*math.sin(a)])
            self.laser=self.project(points,self.transform(message.header.frame_id,message.header.stamp))
        except Exception:self.laser=[]

    def render(self):
        if self.grid is None:return
        from rclpy.time import Time
        robot=None
        try:
            tr=self.buffer.lookup_transform('map','base_link',Time()).transform;q=tr.rotation
            robot=(tr.translation.x,tr.translation.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
            if not self.trail or math.dist(self.trail[-1],robot[:2])>.05:self.trail.append(robot[:2])
            self.trail=self.trail[-10000:]
        except Exception:pass
        info=self.grid.info;q=info.origin.orientation
        origin=(info.origin.position.x,info.origin.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
        image=render_map(np.asarray(self.grid.data).reshape(info.height,info.width),info.resolution,origin,self.plans,robot,self.trail,self.laser,self.targets)
        ok,data=cv2.imencode('.png',image)
        if not ok:return
        (self.output/'map.tmp').write_bytes(data.tobytes());(self.output/'map.tmp').replace(self.output/'map.png')
        metadata=dict(received_unix=time.time(),map_stamp=self.grid.header.stamp.sec+self.grid.header.stamp.nanosec/1e9,
            plans={k:{n:v for n,v in p.items() if n!='points'}|dict(point_count=len(p['points'])) for k,p in self.plans.items()},
            errors=self.errors,read_only=True)
        (self.output/'map_metadata.tmp').write_text(json.dumps(metadata));(self.output/'map_metadata.tmp').replace(self.output/'map_metadata.json')
