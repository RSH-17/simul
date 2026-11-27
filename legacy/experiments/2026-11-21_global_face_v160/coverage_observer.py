"""Passive common observer, never supplies ground truth to either policy."""
import json
import os
from pathlib import Path
import sys
from action_msgs.msg import GoalStatusArray
from rclpy.time import Time
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from tf2_ros import TransformException

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent/'2026-11-21_camera_aware_policy'))
from strict_trial import StrictTrial,main
sys.path.insert(0,str(HERE.parent/'2026-11-21_wall_transit_v152'))
from board_atomic import put

class CoverageTrial(StrictTrial):
    def __init__(self,*args):
        super().__init__(*args)
        variant=os.environ['CAMERA_VARIANT']
        label='Mixed v1.6.0 · 벽·상자 통합 관찰' if variant=='global160' else 'Mixed v1.5.3 · 구역 전환'
        (self.path/'policy_revision.txt').write_text(label+'\n')
        board_link=Path.home()/'track-recordings'/self.path.name
        if not board_link.exists():board_link.symlink_to(self.path,target_is_directory=True)
        self.map_rate_client=None
        self.board.put=put
        self.mount=None
        self.terminal={}
        qos=QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.node.create_subscription(GoalStatusArray,'/navigate_to_pose/_action/status',self.statuses,qos)
    def statuses(self,msg):
        if self.start_sim is not None:
            for goal in msg.status_list:
                if goal.status in (4,5,6):
                    self.terminal[bytes(goal.goal_info.goal_id.uuid).hex()]=int(goal.status)
    def processed(self,msg):
        super().processed(msg)
        if self.mount is None:
            try:
                tr=self.judge_tf.lookup_transform('base_link',msg.header.frame_id,Time.from_msg(msg.header.stamp)).transform
            except TransformException:return
            t,q=tr.translation,tr.rotation
            self.mount=dict(frame=msg.header.frame_id,translation=[t.x,t.y,t.z],xyzw=[q.x,q.y,q.z,q.w])
    def run(self):
        result=super().run()
        data=dict(elapsed_s=self.processed_times,base_from_camera=self.mount,
                  camera=self.camera_geometry,navigate_terminal=self.terminal)
        (self.path/'processed_exposures.json').write_text(json.dumps(data)+'\n')
        return result

if __name__=='__main__':main(CoverageTrial,HERE.parent/'2026-11-21_wall_transit_v152/wall.launch.py')
