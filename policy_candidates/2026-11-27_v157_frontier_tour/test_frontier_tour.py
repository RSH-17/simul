"""Check the frontier tour and the installer without ROS nodes.

Run:  python3 -m pytest -q policy_candidates/2026-11-27_v157_frontier_tour
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


install = load('frontier_tour_install', HERE / 'install.py')
slots = load('frontier_tour_slots', HERE / 'slot_escape.py')


def wall(x, y, score):
    return (score, dict(surface_odom=[x, y], distance=score))


def frontier(x, y, gain=.5):
    return dict(x=x, y=y, tour_odom=[x, y], features=np.array([0., gain, 0., 0., 0.]), distance=abs(x)+abs(y),
                r32_kind='interior_frontier')


# Robot explored the middle of a long room; the west end is mapped, the east end is not.
VISITED = [np.array([x, 0.]) for x in np.arange(-2., 2.5, .5)]
HERE_XY = np.array([0., 0.])
WEST = [wall(-9., 0., 9.), wall(-8.5, 1., 9.)]
EAST_FRONTIER = [frontier(7., -4.)]


class Grid:
    """Identity costmap: every cell is a Nav2 cell."""
    def index(self, x, y):
        return (0, 0)

    def inside(self, *cell):
        return True


class OpenFrontierTest(unittest.TestCase):
    def test_filters_match_discovery_frontier(self):
        choices = [dict(x=1., y=2., features=np.array([0., .5, 0., 0., 0.])),
                   dict(x=3., y=4., features=np.array([0., .1, 0., 0., 0.])),
                   dict(x=5., y=6., features=np.array([0., .5, 0., 0., 0.]), stationary=True),
                   dict(x=7., y=8., features=np.array([0., .5, 0., 0., 0.]), vetoed=True)]
        out = slots.open_frontiers(choices, Grid(), {(0, 0): True}, lambda g: g.get('vetoed', False),
                                   lambda p: np.array(p)+10.)
        self.assertEqual(len(out), 1)
        self.assertEqual(out[0]['tour_odom'], [11., 12.])
        self.assertEqual(out[0]['r32_kind'], 'interior_frontier')


class PlanTest(unittest.TestCase):
    def test_far_frontier_joins_tour(self):
        tour = slots.WallTour.plan(WEST, VISITED, HERE_XY, 400., EAST_FRONTIER)
        self.assertEqual(sorted(tour.kinds), ['frontier', 'wall'])

    def test_near_frontier_is_ignored(self):
        tour = slots.WallTour.plan([], VISITED, HERE_XY, 400., [frontier(1., 2.)])
        self.assertEqual(tour.targets, [])

    def test_done_targets_are_excluded(self):
        tour = slots.WallTour.plan(WEST, VISITED, HERE_XY, 400., EAST_FRONTIER, done=[np.array([7., -4.])])
        self.assertEqual(tour.kinds, ['wall'])

    def test_wall_only_plan_matches_v156(self):
        v156 = load('wall_tour_slots_parent', REPO / 'policy_candidates/2026-11-27_v156_wall_tour/slot_escape.py')
        room = WEST + [wall(9., 0., 9.), wall(0., -6., 6.), wall(1.5, 1.5, 1.5)]
        for budget in (60., 400.):
            old = v156.WallTour.plan(room, VISITED, HERE_XY, budget)
            new = slots.WallTour.plan(room, VISITED, HERE_XY, budget)
            self.assertEqual([t.tolist() for t in old.targets], [t.tolist() for t in new.targets])


class ChooseTest(unittest.TestCase):
    def test_frontier_goal_then_reached(self):
        tour = slots.WallTour([np.array([7., -4.])], [7.], HERE_XY, kinds=['frontier'])
        goal = tour.choose([], HERE_XY, 180., EAST_FRONTIER)
        self.assertEqual(goal['wall_tour_kind'], 'frontier')
        self.assertEqual((goal['x'], goal['y']), (7., -4.))
        self.assertIsNone(tour.choose([], np.array([6.5, -3.5]), 200., EAST_FRONTIER))
        self.assertEqual(tour.skipped, [(0, 'reached')])

    def test_frontier_gone_counts_as_seen(self):
        tour = slots.WallTour([np.array([7., -4.])], [7.], HERE_XY, kinds=['frontier'])
        self.assertIsNone(tour.choose([], HERE_XY, 180., []))
        self.assertEqual(tour.skipped, [(0, 'seen')])


class RecedingTest(unittest.TestCase):
    def test_replans_with_wall_mapped_on_the_way(self):
        tour = slots.RecedingTour()
        goal = tour.choose([], EAST_FRONTIER, VISITED, HERE_XY, 180., 180.)
        self.assertEqual(goal['wall_tour_kind'], 'frontier')
        tour.dispatched(goal, 180.)
        # Arrived: the east corner is now mapped and shows a new wall view
        # (the first plan had no wall there at all).
        visited = VISITED + [np.array([6.5, -3.5])]
        goal = tour.choose([wall(9.3, -7.5, 3.)], [], visited, np.array([6.5, -3.5]), 220., 140.)
        self.assertEqual(goal['wall_tour_kind'], 'wall')
        self.assertEqual(tour.plans, 2)
        self.assertEqual(tour.ended[0][:2], ['frontier', 'reached'])

    def test_empty_first_plan_finishes(self):
        tour = slots.RecedingTour()
        self.assertIsNone(tour.choose([wall(1.5, 1.5, 1.5)], [], VISITED, HERE_XY, 180., 180.))
        self.assertEqual(tour.first_size, 0)
        self.assertTrue(tour.finished)

    def test_plan_limit(self):
        tour = slots.RecedingTour(max_plans=2)
        far = [frontier(float(x), 8.) for x in (-12, 0, 12)]
        for t in range(10):
            # Every frontier vanishes immediately: each plan ends at once.
            tour.choose([], far if t == 0 else [], VISITED, HERE_XY, 180.+t, 180.)
        self.assertLessEqual(tour.plans, 2)
        self.assertTrue(tour.finished)


class SlotTest(unittest.TestCase):
    def test_frontier_goal_reaches_tour_but_not_view_count(self):
        slot = slots.MidWallSlot()
        slot.update(180., 600., 1, True)
        slot.tour = slots.RecedingTour()
        goal = slot.tour.choose([], EAST_FRONTIER, VISITED, HERE_XY, 180., 180.)
        slot.dispatched('wall', 'interior_frontier', goal, 180.)
        self.assertEqual(slot.views, 0)
        self.assertEqual(slot.tour.current.views_here, 1)


class InstallTest(unittest.TestCase):
    def test_install_changes_only_listed_files(self):
        with tempfile.TemporaryDirectory() as tmp:
            target = Path(tmp) / 'ws'
            install.install(RELEASE, target)
            report = json.loads((target / 'frontier_tour_manifest.json').read_text())
            self.assertEqual(report['parent'], '1.5.6-wall-tour')
            names = {'center_wall.py', 'slot_escape.py'}
            self.assertEqual(set(report['changed_from_parent']), {'osp_mission/' + n for n in names})
            package = target / 'src/osp_mission/osp_mission'
            for name in names | {'mission.py', 'phased_hybrid.py'}:
                compile((package / name).read_text(), name, 'exec')
            self.assertIn('RecedingTour()', (package / 'center_wall.py').read_text())
            parent = Path(tmp) / 'parent'
            install.parent_installer().install(RELEASE, parent)
            for path in (parent / 'src/osp_mission').rglob('*'):
                if path.is_file() and path.name not in names | {'frontier_tour_manifest.json'}:
                    mine = package.parent / path.relative_to(parent / 'src/osp_mission')
                    self.assertEqual(path.read_bytes(), mine.read_bytes(), path.name)


if __name__ == '__main__':
    unittest.main()
