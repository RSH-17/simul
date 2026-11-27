"""Exercise automatic start with synthetic ROS endpoints in domain 184 only.

No navigation launch is executed: its process is replaced with a sleeping
Python child. Fake odometry stays zero. Production domain 4 is never used.
"""
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from urllib.request import urlopen

import robot_session as session


@unittest.skipUnless(os.environ.get("ROS_DOMAIN_ID") == "184", "isolated ROS domain 184 required")
class ROSStartTest(unittest.TestCase):
    def test_board_sigterm_closes_http_without_starting_robot(self):
        with socket.socket() as sock:
            sock.bind(("127.0.0.1", 0))
            port = sock.getsockname()[1]
        root = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryFile() as log:
            process = subprocess.Popen([sys.executable, "-u", str(root / "dashboard/robot_board.py"),
                                        "--domain", "184", "--host", "127.0.0.1", "--port", str(port)],
                                       stdout=log, stderr=log)
            try:
                deadline = time.monotonic() + 5.
                ready = False
                while time.monotonic() < deadline and process.poll() is None:
                    try:
                        with urlopen(f"http://127.0.0.1:{port}/api/state", timeout=0.2) as result:
                            data = json.load(result)
                            self.assertTrue(data["read_only"])
                            self.assertFalse(data["control"]["enabled"])
                            ready = True
                            break
                    except OSError:
                        time.sleep(0.05)
                self.assertTrue(ready, "Read-only test board did not start")
                process.terminate()
                self.assertEqual(process.wait(timeout=3.), 0)
            finally:
                if process.poll() is None:
                    process.kill()
                    process.wait(timeout=3.)

    def test_preflight_is_read_only_and_service_starts_as_soon_as_ready(self):
        import rclpy
        from geometry_msgs.msg import TwistStamped
        from irobot_create_msgs.msg import DockStatus, HazardDetectionVector
        from nav_msgs.msg import Odometry
        from rclpy.executors import SingleThreadedExecutor
        from sensor_msgs.msg import CameraInfo, Image, LaserScan
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        io = session.RobotIO(184)
        source = rclpy.create_node("robot_session_test_source")
        counts = dict(start=0, stop=0, cmd=0)
        phase = dict(state="idle", started=None)
        publishers = [(source.create_publisher(kind, topic, 10), kind, name) for kind, topic, name in (
            (LaserScan, "/scan", "scan"), (Image, "/oakd/rgb/preview/image_raw", "camera"),
            (CameraInfo, "/oakd/rgb/preview/camera_info", "info"), (Odometry, "/odom", "odom"),
            (DockStatus, "/dock_status", "dock"), (HazardDetectionVector, "/hazard_detection", "hazards"))]
        status = source.create_publisher(String, "/mission/status", 10)
        def tick():
            for pub, kind, _ in publishers:
                msg = kind()
                msg.header.stamp = source.get_clock().now().to_msg()
                pub.publish(msg)
            if phase["started"] and time.monotonic() - phase["started"] > 0.5:
                phase["state"] = "done"
            status.publish(String(data=json.dumps(dict(state=phase["state"], active_action=""))))
        def on_start(req, res):
            counts["start"] += 1
            res.success = counts["start"] >= 3
            res.message = "Mission started" if res.success else "Waiting for map/TF"
            if res.success:
                phase.update(state="running", started=time.monotonic())
            return res
        def on_stop(req, res):
            counts["stop"] += 1
            res.success = True
            return res
        def on_cmd(msg):
            counts["cmd"] += 1
            self.assertEqual(msg.twist.linear.x, 0.)
            self.assertEqual(msg.twist.angular.z, 0.)
        resources = [source.create_timer(0.05, tick),
                     source.create_service(Trigger, "/mission/start", on_start),
                     source.create_service(Trigger, "/mission/stop", on_stop),
                     source.create_subscription(TwistStamped, "/cmd_vel", on_cmd, 10)]
        executor = SingleThreadedExecutor()
        executor.add_node(source)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        try:
            session.preflight(io, session.StopFlag(), 5.)
            self.assertEqual(counts["start"], 0)
            self.assertEqual(counts["cmd"], 0)
            with tempfile.TemporaryDirectory() as temp:
                root = Path(temp)
                args = SimpleNamespace(workspace=root, repo=root, duration=5., timeout=8., domain=184,
                                       ntp_server="unused_in_synthetic_test")
                real_popen = subprocess.Popen
                def stand_in(argv, **kwargs):
                    self.assertEqual(argv[:4], ["ros2", "launch", "osp_mission", "robot.launch.py"])
                    return real_popen([sys.executable, "-c", "import time; time.sleep(20)"], **kwargs)
                with patch.object(session, "runtime_check"), patch.object(session, "clock_check"), \
                     patch.object(session.subprocess, "Popen", side_effect=stand_in):
                    self.assertEqual(session.start(args, io, session.StopFlag(), root / "session.json"), 0)
                state = json.loads((root / "session.json").read_text())
                events = [json.loads(v) for v in (Path(state["output"]) / "startup.jsonl").read_text().splitlines()]
                by_name = {v["event"]: v for v in events}
                self.assertLess(by_name["mission_started"]["elapsed_s"] - by_name["launch_started"]["elapsed_s"], 2.)
                self.assertTrue(state["stationary_verified"])
                self.assertFalse(session.owned_alive(state, "launch"))
            self.assertEqual(counts["start"], 3)
            self.assertGreater(counts["stop"], 0)
            self.assertGreater(counts["cmd"], 0)
        finally:
            executor.shutdown(timeout_sec=2.)
            thread.join(timeout=2.)
            source.destroy_node()
            io.close()


if __name__ == "__main__":
    unittest.main()
