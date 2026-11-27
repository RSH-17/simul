"""Check the candidate's installer and both state machines without ROS nodes.

Run:  python3 -m pytest -q policy_candidates/2026-11-26_v154_wall_slot_escape
"""
import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
RELEASE = REPO / 'policy_releases/2026-11-21_mixed_camera_sync_v151'


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


# Unique module names: other candidates' tests also import a module called install.
install = load('wall_slot_escape_install', HERE / 'install.py')
slots = load('wall_slot_escape_slots', HERE / 'slot_escape.py')
MidWallSlot, StuckEscape = slots.MidWallSlot, slots.StuckEscape


class MidWallSlotTest(unittest.TestCase):
    def test_waits_for_thirty_percent(self):
        slot = MidWallSlot()
        self.assertFalse(slot.update(179., 600., 1, True))
        self.assertTrue(slot.update(180., 600., 1, True))
        self.assertEqual(slot.started, 180.)

    def test_ends_after_ninety_seconds_and_never_restarts(self):
        slot = MidWallSlot()
        self.assertTrue(slot.update(200., 600., 1, True))
        self.assertTrue(slot.update(289., 600., 1, True))
        self.assertFalse(slot.update(290., 600., 1, True))
        self.assertEqual(slot.ended_reason, 'time_limit')
        self.assertFalse(slot.update(400., 600., 1, True))

    def test_view_limit(self):
        slot = MidWallSlot(max_views=2)
        self.assertTrue(slot.update(200., 600., 0, True))
        slot.dispatched('wall', 'wall_surface')
        slot.dispatched('wall', 'wall_surface')
        self.assertFalse(slot.update(210., 600., 0, True))
        self.assertEqual(slot.ended_reason, 'view_limit')

    def test_skipped_after_earlier_wall_phase(self):
        slot = MidWallSlot()
        slot.dispatched('wall', 'wall_surface')
        self.assertFalse(slot.update(200., 600., 1, True))
        self.assertEqual(slot.ended_reason, 'earlier_wall_phase')

    def test_needs_wall_views_and_missing_ids(self):
        slot = MidWallSlot()
        self.assertFalse(slot.update(200., 600., 3, True))
        self.assertFalse(slot.update(200., 600., 1, False))
        self.assertIsNone(slot.started)
        self.assertTrue(slot.update(201., 600., 2, True))
        self.assertFalse(slot.update(202., 600., 3, True))
        self.assertEqual(slot.ended_reason, 'all_ids_registered')


class StuckEscapeTest(unittest.TestCase):
    def test_three_failures_in_place(self):
        stuck = StuckEscape()
        self.assertFalse(stuck.quick_failure((1., 1.)))
        self.assertFalse(stuck.quick_failure((1.03, 1.)))
        self.assertTrue(stuck.quick_failure((1.05, 1.02)))
        self.assertEqual(stuck.triggers, 1)
        self.assertFalse(stuck.quick_failure((1.05, 1.02)))

    def test_movement_or_success_resets(self):
        stuck = StuckEscape()
        stuck.quick_failure((0., 0.))
        stuck.quick_failure((0., 0.))
        self.assertFalse(stuck.quick_failure((.5, 0.)))
        stuck.progress()
        self.assertFalse(stuck.quick_failure((.5, 0.)))
        self.assertFalse(stuck.quick_failure(None))


class InstallTest(unittest.TestCase):
    def test_install_changes_only_listed_files(self):
        with tempfile.TemporaryDirectory() as tmp:
            target = Path(tmp) / 'ws'
            install.install(RELEASE, target)
            report = json.loads((target / 'wall_slot_escape_manifest.json').read_text())
            self.assertEqual(set(report['changed_files']),
                             {'osp_mission/center_wall.py', 'osp_mission/phased_hybrid.py',
                              'osp_mission/mission.py', 'osp_mission/slot_escape.py'})
            package = target / 'src/osp_mission/osp_mission'
            for name in ('center_wall.py', 'phased_hybrid.py', 'mission.py', 'slot_escape.py'):
                compile((package / name).read_text(), name, 'exec')
            # Everything else equals the parent marker-events workspace.
            parent = Path(tmp) / 'parent'
            install.parent_installer().install(RELEASE, parent)
            changed = {'center_wall.py', 'phased_hybrid.py', 'mission.py', 'slot_escape.py'}
            for path in (parent / 'src/osp_mission/osp_mission').glob('*.py'):
                if path.name not in changed:
                    self.assertEqual(path.read_bytes(), (package / path.name).read_bytes(), path.name)


if __name__ == '__main__':
    unittest.main()
