"""Install the instrumentation candidate and check its behavior without ROS nodes running.

Run with Jazzy sourced:  python3 -m pytest -q policy_candidates/2026-11-26_v151_marker_events
"""
import importlib
import json
import shutil
import sys
import tempfile
import types
import unittest
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
RELEASE = REPO / 'policy_releases/2026-11-21_mixed_camera_sync_v151'
sys.path.insert(0, str(HERE))
import install  # noqa: E402


class Stamp:
    def __init__(self, seconds):
        self.nanoseconds = int(seconds * 1e9)


class Clock:
    def now(self):
        return Stamp(12.5)


class Recorder:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(json.loads(msg.data))


class InstalledCandidate(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.mkdtemp()
        cls.ws = Path(cls.tmp) / 'ws'
        install.install(RELEASE, cls.ws)
        cls.package = cls.ws / 'src/osp_mission'
        sys.path.insert(0, str(cls.package))
        for name in [m for m in sys.modules if m == 'osp_mission' or m.startswith('osp_mission.')]:
            del sys.modules[name]
        cls.detector = importlib.import_module('osp_mission.detector')

    @classmethod
    def tearDownClass(cls):
        sys.path.remove(str(cls.package))
        shutil.rmtree(cls.tmp)

    def stub(self, observations):
        node = types.SimpleNamespace(observations=observations, announced=set(), event_pub=Recorder(),
                                     separate_markers=None, get_clock=Clock)
        node.p = lambda name: {'confirmations': 3}[name]
        return node

    def test_only_instrumentation_files_change(self):
        manifest = json.loads((self.ws / 'marker_events_manifest.json').read_text())
        self.assertEqual(set(manifest['changed_files']), {'osp_mission/detector.py', 'launch/robot.launch.py',
                                                          'setup.py', 'osp_mission/marker_signal.py'})
        frozen = json.loads((RELEASE / 'source_manifest.json').read_text())['source_sha256']
        for relative, expected in frozen.items():
            if relative not in manifest['changed_files']:
                self.assertEqual(install.digest(self.package / relative), expected, relative)
        for name in ('policy.json', 'nav2_hardware_profile.yaml'):
            self.assertEqual((self.ws / 'frozen-config' / name).read_bytes(), (RELEASE / name).read_bytes())

    def test_confirmation_is_announced_once_at_the_third_frame(self):
        row = lambda x: (0, np.array([x, 1., .3]))
        node = self.stub({7: [row(2.), row(2.02)]})
        announce = self.detector.Detector.announce_confirmations
        announce(node, Stamp(10.))
        self.assertEqual(node.event_pub.messages, [])
        node.observations[7].append(row(2.01))
        announce(node, Stamp(10.4))
        announce(node, Stamp(10.6))  # later frames of the same ID stay silent
        events = node.event_pub.messages
        self.assertEqual(len(events), 1)
        self.assertEqual((events[0]['event'], events[0]['id'], events[0]['frames']), ('confirmed', 7, 3))
        self.assertAlmostEqual(events[0]['exposure_stamp'], 10.4)
        self.assertAlmostEqual(events[0]['map_point'][0], 2.01)

    def test_retraction_is_reported_from_the_periodic_publish(self):
        node = self.stub({})
        node.announced = {5}
        node.diagnostic_pub = node.pub = types.SimpleNamespace(publish=lambda msg: None)
        node.diagnostics, node.id_diagnostics = {}, {}
        node.exposure_queue = types.SimpleNamespace(counts={}, rows=[])
        node.pipeline_timing = node.sensor_history = types.SimpleNamespace(report=lambda: {})
        node.get_logger = lambda: types.SimpleNamespace(debug=lambda *_: None)
        self.detector.Detector.publish(node)
        self.assertEqual([(e['event'], e['id']) for e in node.event_pub.messages], [('retracted', 5)])
        self.assertEqual(node.announced, set())

    def test_signal_messages(self):
        from builtin_interfaces.msg import Time
        signal = importlib.import_module('osp_mission.marker_signal')
        ring = signal.lightring(Time(), signal.GREEN, True)
        self.assertTrue(ring.override_system)
        self.assertEqual([(led.red, led.green, led.blue) for led in ring.leds], [signal.GREEN] * 6)
        release = signal.lightring(Time(), (0, 0, 0), False)
        self.assertFalse(release.override_system)
        notes = signal.tone(Time()).notes
        self.assertEqual([n.frequency for n in notes], [f for f, _ in signal.TONE])

    def test_launch_declares_the_signal_switch(self):
        text = (self.package / 'launch/robot.launch.py').read_text()
        self.assertIn("'marker_signal':'true'", text)
        self.assertIn("executable='marker_signal'", text)

    def test_modified_release_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            copy = Path(tmp) / 'release'
            shutil.copytree(RELEASE, copy, ignore=shutil.ignore_patterns('__pycache__'))
            target = copy / 'src/osp_mission/osp_mission/detector.py'
            target.write_text(target.read_text() + '\n# drift\n')
            with self.assertRaises(ValueError):
                install.install(copy, Path(tmp) / 'ws')


if __name__ == '__main__':
    unittest.main()
