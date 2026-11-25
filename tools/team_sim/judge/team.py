"""Team-runner judges, consolidated from the original experiment folders.

Origins (preserved unchanged under experiments/):
  WallTrial        <- experiments/2026-11-21_wall_transit_v152/trial.py
  PassiveExamTrial <- experiments/2026-11-22_random_exam/trial.py
The chain is Trial (base) -> CameraTrial (video) -> PerformanceTrial
(performance) -> StrictTrial (strict) -> the classes below. Ground truth is
only read by these judges and never sent to the policy.
"""
import json

from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from judge.board_atomic import put
from judge.camera_noise import CameraNoise
from judge.strict import StrictTrial, main

GATED_TOPIC = '/exam/detector_rgb'


class GatedCameraTrial(StrictTrial):
    """Forward RGB to the detector only after the mission start.

    Before the start the detector receives blank frames, which keep its
    heartbeat and calibration alive without a preview of the layout. After the
    start each frame passes through the optional CameraNoise stage.
    """
    gate = True

    def __init__(self, *args):
        super().__init__(*args)
        self.map_rate_client = None
        # The live board can refresh the same card concurrently with this
        # observer. Keep this observer's temporary files private to each write.
        self.board.put = put
        self.camera_noise = CameraNoise.from_environment()
        self.forwarded_images = 0
        self.first_exposure_s = None
        if self.gate:
            self.camera_gate = self.node.create_publisher(Image, GATED_TOPIC, qos_profile_sensor_data)
        # Exact confirmation times from runtimes that publish mission/marker_events
        # (e.g. 1.5.1-marker-events). registered_s keeps the 0.5 s status cadence.
        self.registered_event = {}
        self.event_publish_lag = []
        self.node.create_subscription(String, '/mission/marker_events', self.marker_event, 10)

    def marker_event(self, msg):
        event = json.loads(msg.data)
        if event.get('event') != 'confirmed' or self.start_sim is None or self.closed_at is not None:
            return
        self.registered_event.setdefault(int(event['id']), max(0., event['exposure_stamp']-self.start_sim))
        self.event_publish_lag.append(event['published_stamp']-event['exposure_stamp'])

    def image(self, msg):
        if self.gate and not self.finished:
            stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1.e-9
            if self.start_sim is not None and stamp >= self.start_sim:
                frame = self.camera_noise.apply(msg)
                if frame is not None:
                    self.camera_gate.publish(frame)
                    self.forwarded_images += 1
                    if self.first_exposure_s is None:
                        self.first_exposure_s = stamp-self.start_sim
            elif self.start_sim is None:
                blank = Image(header=msg.header, height=msg.height, width=msg.width,
                              encoding=msg.encoding, is_bigendian=msg.is_bigendian,
                              step=msg.step, data=bytes(len(msg.data)))
                self.camera_gate.publish(blank)
        super().image(msg)

    def run(self):
        report = super().run()
        if self.gate and not self.forwarded_images:
            raise ValueError('No post-start RGB exposures reached the detector topic')
        report.update(registered_event_s=self.registered_event or None,
                      marker_event_publish_lag_max_s=max(self.event_publish_lag, default=None),
                      camera_noise=self.camera_noise.report(), detector_camera_gated=self.gate,
                      forwarded_detector_images=self.forwarded_images if self.gate else None,
                      first_detector_exposure_s=self.first_exposure_s)
        return report


class WallTrial(GatedCameraTrial):
    """Legacy five-map judge. The detector reads the camera directly unless a
    camera-noise condition is requested, in which case frames are gated."""

    def __init__(self, *args):
        self.gate = CameraNoise.from_environment().enabled
        super().__init__(*args)


class PassiveExamTrial(GatedCameraTrial):
    def __init__(self, *args):
        self.proof_at = {}
        super().__init__(*args)

    def truth(self, msg):
        super().truth(msg)
        if self.start_sim is not None:
            elapsed = self.sim-self.start_sim
            if 0 <= elapsed <= self.duration:
                for ident in self.current.intersection(self.status.get('approached_ids', [])):
                    self.proof_at.setdefault(int(ident), float(elapsed))
        # The inherited observer normally ends a trial as soon as its private
        # answer key matches. Leave termination to the policy or the fixed clock.
        self.pair_at = None

    def run(self):
        report = super().run()
        report.update(sensor_verified_at_s=self.proof_at, judge_stopped_policy_on_truth=False,
                      evaluation_protocol='Every enrolled Original ID; post-registration ground-truth 1.5 m approach, onboard near RGB evidence, saved location, false IDs, and bumper contacts.')
        return report


def detector_topic_gated(environment, random_exam):
    """True when the detector must read GATED_TOPIC instead of the raw camera."""
    return random_exam or CameraNoise.from_environment(environment).enabled


__all__ = ['WallTrial', 'PassiveExamTrial', 'GatedCameraTrial', 'main', 'detector_topic_gated', 'GATED_TOPIC']
