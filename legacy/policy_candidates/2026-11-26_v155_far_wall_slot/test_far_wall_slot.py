"""Check the far-wall choice and the installer without ROS nodes.

Run:  python3 -m pytest -q policy_candidates/2026-11-26_v155_far_wall_slot
"""
import importlib.util
import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
RELEASE = REPO / 'policy_releases/2026-11-21_mixed_camera_sync_v151'


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


install = load('far_wall_slot_install', HERE / 'install.py')
slots = load('far_wall_slot_slots', HERE / 'slot_escape.py')


def wall(x, y, distance):
    return (distance, dict(surface_odom=[x, y], distance=distance))


class FarWallTest(unittest.TestCase):
    visited = [np.array([5., 0.]), np.array([7., 1.]), np.array([8., -2.])]

    def test_prefers_unvisited_end_of_room(self):
        items = [wall(9., 0., 2.), wall(-9., 4., 15.), wall(4., 5., 5.)]
        goal = slots.far_wall(items, self.visited)
        self.assertEqual(goal['surface_odom'], [-9., 4.])
        self.assertGreater(goal['far_wall_novelty_m'], 13.)

    def test_none_when_every_wall_is_near_visited(self):
        items = [wall(6., 1., 1.), wall(8., -1., 3.)]
        self.assertIsNone(slots.far_wall(items, self.visited))

    def test_toward_target(self):
        items = [wall(9., 0., 2.), wall(-8., 3., 14.), wall(-2., 0., 8.)]
        goal = slots.toward_target(items, np.array([-9., 4.]))
        self.assertEqual(goal['surface_odom'], [-8., 3.])

    def test_slot_limit_is_longer(self):
        slot = slots.MidWallSlot()
        self.assertTrue(slot.update(180., 600., 1, True))
        self.assertTrue(slot.update(299., 600., 1, True))
        self.assertFalse(slot.update(300., 600., 1, True))


class InstallTest(unittest.TestCase):
    def test_install_changes_only_listed_files(self):
        with tempfile.TemporaryDirectory() as tmp:
            target = Path(tmp) / 'ws'
            install.install(RELEASE, target)
            report = json.loads((target / 'far_wall_slot_manifest.json').read_text())
            names = {'center_wall.py', 'phased_hybrid.py', 'slot_escape.py'}
            self.assertEqual(set(report['changed_from_parent']), {'osp_mission/' + n for n in names})
            package = target / 'src/osp_mission/osp_mission'
            for name in names | {'mission.py'}:
                compile((package / name).read_text(), name, 'exec')
            parent = Path(tmp) / 'parent'
            install.parent_installer().install(RELEASE, parent)
            for path in (parent / 'src/osp_mission/osp_mission').glob('*.py'):
                if path.name not in names:
                    self.assertEqual(path.read_bytes(), (package / path.name).read_bytes(), path.name)


if __name__ == '__main__':
    unittest.main()
