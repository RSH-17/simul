"""Camera callback timing and isolated autonomous v1.2 trial with recorded RGB."""
import importlib.util
import json
import math
import os
from pathlib import Path
import sys
import time

import numpy as np
import psutil
import cv2
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo

HELPERS=Path(os.environ['CAMERA_BENCH_HELPERS'])
sys.path.insert(0,str(HELPERS/'2026-11-19_pair_7_103'))
sys.path.insert(0,str(HELPERS/'2026-11-19_corridor_box'))
from measure_video import CameraTrial,main,seconds


class PerformanceTrial(CameraTrial):
    def __init__(self,*args):
        super().__init__(*args)
        self.input_times=[];self.processed_times=[];self.ages=[];self.cpu=[];self.camera_geometry=None
        self.processes={};self.last_wall=time.monotonic();self.previous_cpu={}
        self.node.create_subscription(CameraInfo,'/mission/processed_camera',self.processed,qos_profile_sensor_data)
        self.node.create_subscription(CameraInfo,'/oakd/rgb/preview/camera_info',self.calibration,qos_profile_sensor_data)
        self.node.create_timer(1.,self.resources)
        self.last_refresh=0.
        spec=importlib.util.spec_from_file_location('camera_board',HELPERS/'2026-11-19_corridor_box/compare_board.py')
        self.board=importlib.util.module_from_spec(spec);spec.loader.exec_module(self.board)
        self.board_cache={}
        self.board_scene=json.loads((self.path/'scene.json').read_text())
        layout=self.path/'run/simulation/demo_layout.json'
        if layout.is_file():self.board_scene['partitions']+=json.loads(layout.read_text())['tables']
        self.watchdog_started=time.monotonic();self.last_sim_time=None;self.last_sim_wall=time.monotonic()
        if os.environ.get('CAMERA_BENCH_LOSSLESS')=='1':(self.path/'raw_frames').mkdir()

    def calibration(self,msg):
        self.camera_geometry=dict(width=msg.width,height=msg.height,k=list(msg.k))

    def image(self,msg):
        if self.start_sim is not None and not self.finished:
            assert msg.width==msg.height==int(os.environ['CAMERA_BENCH_WIDTH'])
            self.input_times.append(seconds(msg.header.stamp)-self.start_sim)
        super().image(msg)
        if self.start_sim is not None and not self.finished and os.environ.get('CAMERA_BENCH_LOSSLESS')=='1':
            path=self.path/'raw_frames'/f'{self.count:05d}.png'
            if not cv2.imwrite(str(path),self.bridge.imgmsg_to_cv2(msg,'bgr8'),[cv2.IMWRITE_PNG_COMPRESSION,1]):raise OSError(path)

    def processed(self,msg):
        if self.start_sim is not None and self.sim is not None:
            self.processed_times.append(seconds(msg.header.stamp)-self.start_sim)
            self.ages.append(self.sim-seconds(msg.header.stamp))

    def resources(self):
        if self.finished:return
        now=time.monotonic()
        if self.sim!=self.last_sim_time:self.last_sim_time=self.sim;self.last_sim_wall=now
        if now-self.watchdog_started>30 and now-self.last_sim_wall>20:
            raise RuntimeError('Simulation clock stopped; exclude this trial from camera ranking')
        if self.start_sim is None:return
        now=time.monotonic();dt=now-self.last_wall;self.last_wall=now
        for process in psutil.process_iter(['pid','cmdline']):
            try:
                cmd=' '.join(process.info['cmdline'] or [])
                if not any(k in cmd for k in ('/lib/osp_mission/detector','/lib/nav2_controller/controller_server',
                    '/lib/slam_toolbox/async_slam_toolbox_node','gz sim')):continue
                if process.environ().get('ROS_DOMAIN_ID')!='174':continue
                kind=('detector' if '/lib/osp_mission/detector' in cmd else 'nav2' if 'controller_server' in cmd else 'slam' if 'slam_toolbox' in cmd else 'gazebo')
                t=process.cpu_times();total=t.user+t.system;previous=self.previous_cpu.get(process.pid,total)
                self.previous_cpu[process.pid]=total
                self.cpu.append(dict(t=self.sim-self.start_sim,kind=kind,cpu_percent_one_core=100*(total-previous)/max(dt,.01),rss_mib=process.memory_info().rss/2**20))
            except (psutil.Error,OSError):continue
        if now-self.last_refresh>=2:
            self.last_refresh=now
            self.file.flush();self.heading_log.flush();self.frame_log.flush()
            self.board.refresh(self.path,Path(os.environ['CAMERA_BENCH_CARDS'])/self.path.name,
                               self.board_scene,self.board_cache,origin=(0,0),duration=self.duration)

    def run(self):
        report=super().run()
        times=np.asarray(self.input_times);processed=np.asarray(self.processed_times);ages=np.asarray(self.ages)
        rate=lambda values:float((len(values)-1)/(values[-1]-values[0])) if len(values)>1 else None
        camera=dict(width=int(os.environ['CAMERA_BENCH_WIDTH']),height=int(os.environ['CAMERA_BENCH_HEIGHT']),
                    requested_fps=float(os.environ['CAMERA_BENCH_FPS']),camera_info=self.camera_geometry,
                    input_frames=len(times),input_fps_sim=rate(times),processed_frames=len(processed),processed_fps_sim=rate(processed),
                    processed_age_sim_p50_s=float(np.median(ages)) if len(ages) else None,
                    processed_age_sim_p95_s=float(np.percentile(ages,95)) if len(ages) else None,
                    input_max_gap_sim_s=float(np.diff(times).max()) if len(times)>1 else None,
                    real_time_factor=report['elapsed_sim_s']/report['elapsed_wall_s'],
                    limitations='Gazebo render/VM test only; excludes real camera exposure, USB and Create3 DDS overload')
        (self.path/'camera_metrics.json').write_text(json.dumps(camera,indent=2)+'\n')
        (self.path/'resource_samples.json').write_text(json.dumps(self.cpu)+'\n')
        return dict(report,camera=camera)


if __name__=='__main__':
    main(PerformanceTrial,Path(__file__).with_name('camera.launch.py'))
