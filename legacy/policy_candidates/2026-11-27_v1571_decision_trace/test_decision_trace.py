"""Check the rule trace and the installer without ROS nodes.

Run:  python3 -m pytest -q policy_candidates/2026-11-27_v1571_decision_trace
"""
import difflib
import importlib.util
import json
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


install = load('decision_trace_install', HERE / 'install.py')
trace = load('decision_trace_rule_trace', HERE / 'rule_trace.py')


def face(x, y, path, **fields):
    return (path, dict(dict(x=x, y=y, distance=path*.8, previous_attempts=0, surface_map=[x+1, y],
                            completed_box=False, local_batch=False, standoff_m=.85, clearance_m=.6), **fields))


class Grid:
    def index(self, x, y):
        return (0, 0)

    def inside(self, *cell):
        return True


class RuleTraceTest(unittest.TestCase):
    def test_lists_keep_rule_order_and_mark_the_chosen_rank(self):
        near, far = face(1., 2., 1.5, fitted_face=1), face(4., 4., 5.)
        candidates = dict(interior=[near, far], wall=[(7., dict(x=9., y=0., distance=6.)), (3., dict(x=8., y=1.))])
        row = trace.inputs(30., 600., False, candidates, [], [near, far], [], [], None)
        self.assertEqual([r['x'] for r in row['interior']], [1., 4.])
        self.assertTrue(row['interior'][0]['fitted_face'])
        self.assertEqual([r['score'] for r in row['wall']], [3., 7.])  # lower score first
        trace.gates(row, [far], None, 30., [], Grid(), {}, lambda g: False)
        out = trace.record(row, far[1], 'unseen_reachable_object', 'interior', 30.2)
        self.assertEqual(out['chosen_from'], dict(list='fresh', rank=1))
        json.dumps(out, allow_nan=False)

    def test_frontiers_follow_discovery_score(self):
        def choice(x, gain, travel, novelty=0.):
            return dict(x=x, y=0., distance=travel, features=[0., gain, 0., 0., novelty])
        choices = [choice(1., .5, 1.), choice(2., .5, 4.), choice(3., .1, 4.), choice(4., .9, 4., novelty=5.)]
        row = trace.inputs(30., 600., False, dict(interior=[], wall=[]), [], [], [], [], None)
        trace.gates(row, [], dict(x=2., y=0.), 30., choices, Grid(), {(0, 0): True}, lambda g: False)
        self.assertEqual([r['x'] for r in row['frontiers']], [2., 1., 4.])  # gain .1 filtered
        self.assertEqual(row['frontier_chosen'], [2., 0.])

    def test_errors_drop_only_the_trace(self):
        row = trace.inputs(30., 600., False, dict(interior=[('bad',)]), [], [], [], [], None)
        self.assertIn('error', row)
        trace.gates(row, [], None, 0., [], None, None, None)  # ignored
        out = trace.record(None, dict(x=1., y=1.), 'mission_start', 'interior', 1.)
        self.assertNotIn('chosen_from', out)


class InstallTest(unittest.TestCase):
    def test_only_additions_on_top_of_frontier_tour(self):
        if not RELEASE.exists():
            self.skipTest('release archive not present')
        with tempfile.TemporaryDirectory() as tmp:
            parent, child = Path(tmp)/'parent', Path(tmp)/'child'
            install.parent_installer().install(RELEASE, parent)
            install.install(RELEASE, child)
            manifest = json.loads((child/'decision_trace_manifest.json').read_text())
            self.assertEqual(manifest['parent'], '1.5.7-frontier-tour')
            self.assertEqual(sorted(manifest['changed_from_parent']),
                             ['osp_mission/center_wall.py', 'osp_mission/phased_hybrid.py', 'osp_mission/rule_trace.py'])
            for name in ('center_wall.py', 'phased_hybrid.py'):
                old = (parent/'src/osp_mission/osp_mission'/name).read_text().splitlines()
                new = (child/'src/osp_mission/osp_mission'/name).read_text().splitlines()
                removed = [l for l in difflib.ndiff(old, new) if l.startswith('- ')]
                self.assertEqual(removed, [], name)
            compile((child/'src/osp_mission/osp_mission/center_wall.py').read_text(), 'center_wall.py', 'exec')
            compile((child/'src/osp_mission/osp_mission/phased_hybrid.py').read_text(), 'phased_hybrid.py', 'exec')
            files = sorted(p.relative_to(child/'src').as_posix() for p in (child/'src').rglob('*') if p.is_file())
            parent_files = sorted(p.relative_to(parent/'src').as_posix() for p in (parent/'src').rglob('*') if p.is_file())
            self.assertEqual(set(files)-set(parent_files), {'osp_mission/osp_mission/rule_trace.py'})


if __name__ == '__main__':
    unittest.main()
