"""Show marker confirmations on the TurtleBot 4: lightring flash and a short tone.

Subscribes to mission/marker_events (published by the detector at the moment
an ID is confirmed). It never reads ground truth and never commands motion.
Disable with the parameter enabled:=false.
"""
import json

import rclpy
from builtin_interfaces.msg import Duration
from irobot_create_msgs.msg import AudioNote, AudioNoteVector, LedColor, LightringLeds
from rclpy.node import Node
from std_msgs.msg import String

FLASH_S = 3.0
GREEN = (0, 255, 0)
TONE = ((880, 0.12), (1320, 0.18))  # Hz, seconds


def lightring(stamp, color, override):
    msg = LightringLeds()
    msg.header.stamp = stamp
    msg.override_system = override
    msg.leds = [LedColor(red=color[0], green=color[1], blue=color[2]) for _ in range(6)]
    return msg


def tone(stamp):
    msg = AudioNoteVector()
    msg.header.stamp = stamp
    msg.append = False
    for frequency, seconds in TONE:
        msg.notes.append(AudioNote(frequency=frequency,
                                   max_runtime=Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))))
    return msg


class MarkerSignal(Node):
    def __init__(self):
        super().__init__('marker_signal')
        self.declare_parameter('enabled', True)
        self.enabled = bool(self.get_parameter('enabled').value)
        self.light = self.create_publisher(LightringLeds, 'cmd_lightring', 10)
        self.audio = self.create_publisher(AudioNoteVector, 'cmd_audio', 10)
        self.create_subscription(String, 'mission/marker_events', self.on_event, 10)
        self.release_at = None
        self.signalled = []
        self.create_timer(0.1, self.tick)

    def on_event(self, msg):
        event = json.loads(msg.data)
        if not self.enabled or event.get('event') != 'confirmed':
            return
        now = self.get_clock().now()
        self.light.publish(lightring(now.to_msg(), GREEN, True))
        self.audio.publish(tone(now.to_msg()))
        self.release_at = now.nanoseconds / 1e9 + FLASH_S
        self.signalled.append(int(event['id']))
        self.get_logger().info(f"Marker ID {event['id']} confirmed: lightring + tone")

    def tick(self):
        if self.release_at is not None and self.get_clock().now().nanoseconds / 1e9 >= self.release_at:
            self.light.publish(lightring(self.get_clock().now().to_msg(), (0, 0, 0), False))
            self.release_at = None


def main(args=None):
    rclpy.init(args=args)
    node = MarkerSignal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.release_at is not None:
            node.light.publish(lightring(node.get_clock().now().to_msg(), (0, 0, 0), False))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
