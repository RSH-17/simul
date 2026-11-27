"""Passive native observer for random IDs. The answer key never stops a mission."""
import os
from pathlib import Path
import sys
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

experiments=Path(os.environ['SURVEY_EXPERIMENTS'])
sys.path[:0]=[str(experiments/'2026-11-21_camera_aware_policy'),str(experiments/'2026-11-21_wall_transit_v152')]
from strict_trial import StrictTrial, main
from board_atomic import put


class PassiveExamTrial(StrictTrial):
    def __init__(self,*args):
        self.proof_at={}
        super().__init__(*args)
        self.map_rate_client=None
        self.board.put=put
        self.camera_gate=self.node.create_publisher(Image,'/exam/detector_rgb',qos_profile_sensor_data)
        self.forwarded_images=0
        self.first_exposure_s=None

    def image(self,msg):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1.e-9
        if not self.finished:
            if self.start_sim is not None and stamp>=self.start_sim:
                self.camera_gate.publish(msg)
                self.forwarded_images+=1
                if self.first_exposure_s is None:self.first_exposure_s=stamp-self.start_sim
            elif self.start_sim is None:
                # Keep detector heartbeat and camera calibration live during Nav2
                # startup without giving either policy a preview of the draw.
                blank=Image(header=msg.header,height=msg.height,width=msg.width,
                            encoding=msg.encoding,is_bigendian=msg.is_bigendian,
                            step=msg.step,data=bytes(len(msg.data)))
                self.camera_gate.publish(blank)
        super().image(msg)

    def truth(self,msg):
        super().truth(msg)
        if self.start_sim is not None:
            elapsed=self.sim-self.start_sim
            if 0<=elapsed<=self.duration:
                for ident in self.current.intersection(self.status.get('approached_ids',[])):
                    self.proof_at.setdefault(int(ident),float(elapsed))
        # The inherited observer normally ends a trial as soon as its private
        # answer key matches. Leave termination to the policy or the fixed clock.
        self.pair_at=None

    def run(self):
        report=super().run()
        if not self.forwarded_images:raise ValueError('No post-start RGB exposures reached the detector topic')
        report.update(sensor_verified_at_s=self.proof_at,judge_stopped_policy_on_truth=False,
                      forwarded_detector_images=self.forwarded_images,first_detector_exposure_s=self.first_exposure_s,
                      evaluation_protocol='Every enrolled Original ID; post-registration ground-truth 1.5 m approach, onboard near RGB evidence, saved location, false IDs, and bumper contacts.')
        return report


if __name__=='__main__':
    main(PassiveExamTrial,Path(__file__).with_name('exam_camera.launch.py'))
