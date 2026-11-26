"""Check the wall tour and the installer without ROS nodes.

Run:  python3 -m pytest -q policy_candidates/2026-11-27_v156_wall_tour
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


install = load('wall_tour_install', HERE / 'install.py')
slots = load('wall_tour_slots', HERE / 'slot_escape.py')


def wall(x, y, score):
    return (score, dict(surface_odom=[x, y], distance=score))


# Robot explored the middle of a long room; both ends and the south wall middle are unvisited.
VISITED = [np.array([x, 0.]) for x in np.arange(-2., 2.5, .5)]
HERE_XY = np.array([0., 0.])
ROOM = [wall(-9., 0., 9.), wall(-8.5, 1., 9.), wall(9., 0., 9.), wall(9., -1., 9.5),
        wall(0., -6., 6.), wall(1.5, 1.5, 1.5)]


class ClusterTest(unittest.TestCase):
    def test_clusters_cover_both_ends_and_wall_middle(self):
        clusters = slots.wall_clusters(ROOM, VISITED)
        targets = sorted(tuple(np.round(c[0], 1)) for c in clusters)
        self.assertEqual(len(clusters), 3)
        self.assertIn((0., -6.), targets)
        self.assertTrue(any(t[0] < -8 for t in targets) and any(t[0] > 8 for t in targets))

    def test_near_views_are_excluded(self):
        self.assertEqual(slots.wall_clusters([wall(1.5, 1.5, 1.5)], VISITED), [])


class TourTest(unittest.TestCase):
    def test_open_tour_is_shortest(self):
        points = [np.array(p, float) for p in [(10., 0.), (1., 0.), (5., 0.)]]
        self.assertEqual(slots.open_tour(np.zeros(2), points), [1, 2, 0])

    def test_heuristic_matches_exact_on_line(self):
        points = [np.array([float(x), 0.]) for x in (8, 3, 1, 6, 2, 9, 4, 7, 5)]
        order = slots.open_tour(np.zeros(2), points, exact_limit=0)
        self.assertEqual([points[i][0] for i in order], [1., 2., 3., 4., 5., 6., 7., 8., 9.])

    def test_plan_visits_both_far_ends(self):
        tour = slots.WallTour.plan(ROOM, VISITED, HERE_XY, budget=400.)
        xs = [t[0] for t in tour.targets]
        self.assertEqual(len(tour.targets), 3)
        self.assertTrue(min(xs) < -8 and max(xs) > 8)

    def test_budget_truncates_tour(self):
        tour = slots.WallTour.plan(ROOM, VISITED, HERE_XY, budget=60.)
        self.assertEqual(len(tour.targets), 1)

    def test_no_far_wall_gives_empty_tour(self):
        tour = slots.WallTour.plan([wall(1.5, 1.5, 1.5)], VISITED, HERE_XY, budget=180.)
        self.assertEqual(tour.targets, [])


class ChooseTest(unittest.TestCase):
    def setUp(self):
        self.tour = slots.WallTour([np.array([-9., 0.]), np.array([9., 0.])], [7., 7.], HERE_XY)

    def test_heads_to_current_cluster_then_advances_after_views(self):
        goal = self.tour.choose(ROOM, HERE_XY, 180.)
        self.assertLess(goal['surface_odom'][0], -8)
        self.assertEqual(goal['wall_tour_index'], 0)
        self.tour.dispatched(goal, 181.)
        self.tour.dispatched(goal, 200.)
        goal = self.tour.choose(ROOM, np.array([-8., 0.]), 210.)
        self.assertEqual(goal['wall_tour_index'], 1)
        self.assertGreater(goal['surface_odom'][0], 8)

    def test_skips_cluster_already_seen(self):
        remaining = [item for item in ROOM if item[1]['surface_odom'][0] > -5]
        goal = self.tour.choose(remaining, HERE_XY, 180.)
        self.assertEqual(goal['wall_tour_index'], 1)
        self.assertEqual(self.tour.skipped, [(0, 'seen')])

    def test_leg_timeout_moves_on(self):
        self.tour.choose(ROOM, HERE_XY, 180.)
        limit = 9./.18 + 30.
        goal = self.tour.choose(ROOM, HERE_XY, 180.+limit+1.)
        self.assertEqual(goal['wall_tour_index'], 1)
        self.assertEqual(self.tour.skipped[0][1], 'timeout')

    def test_exhausted_returns_none(self):
        self.assertIsNone(self.tour.choose([wall(1.5, 1.5, 1.5)], HERE_XY, 180.))
        self.assertTrue(self.tour.exhausted)


class SlotTest(unittest.TestCase):
    def test_slot_limit_and_finish(self):
        slot = slots.MidWallSlot()
        self.assertTrue(slot.update(180., 600., 1, True))
        self.assertTrue(slot.update(359., 600., 1, True))
        self.assertFalse(slot.update(360., 600., 1, True))
        slot = slots.MidWallSlot()
        slot.update(180., 600., 1, True)
        slot.finish('tour_complete')
        self.assertFalse(slot.update(181., 600., 1, True))
        self.assertEqual(slot.report()['ended'], 'tour_complete')


class InstallTest(unittest.TestCase):
    def test_install_changes_only_listed_files(self):
        with tempfile.TemporaryDirectory() as tmp:
            target = Path(tmp) / 'ws'
            install.install(RELEASE, target)
            report = json.loads((target / 'wall_tour_manifest.json').read_text())
            self.assertEqual(report['parent'], '1.5.5-far-wall-slot')
            names = {'center_wall.py', 'slot_escape.py'}
            self.assertEqual(set(report['changed_from_parent']), {'osp_mission/' + n for n in names})
            package = target / 'src/osp_mission/osp_mission'
            for name in names | {'mission.py', 'phased_hybrid.py'}:
                compile((package / name).read_text(), name, 'exec')
            self.assertIn('WallTour.plan', (package / 'center_wall.py').read_text())
            parent = Path(tmp) / 'parent'
            install.parent_installer().install(RELEASE, parent)
            for path in (parent / 'src/osp_mission/osp_mission').glob('*.py'):
                if path.name not in names:
                    self.assertEqual(path.read_bytes(), (package / path.name).read_bytes(), path.name)


if __name__ == '__main__':
    unittest.main()
