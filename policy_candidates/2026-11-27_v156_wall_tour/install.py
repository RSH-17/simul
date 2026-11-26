"""v1.5.5 far-wall slot, with the slot visiting every far wall cluster in tour order.

Builds an exact `1.5.5-far-wall-slot` workspace first, then:
  slot_escape.py   replaced: adds wall_clusters(), open_tour() and WallTour;
                   slot limit 180 s and 10 views (several clusters to reach)
  center_wall.py   at the slot's first decision, groups every unviewed wall
                   view >= 3 m from the robot's visited positions into 3 m
                   clusters, orders them as the shortest open tour from the
                   robot, keeps the clusters that fit the slot budget, and
                   visits them in order (2 views each, skip when already seen
                   or the leg times out). The slot ends when the tour is done.
                   Without any far cluster it keeps v1.5.5's nearest-wall rule.
Everything else (the end-of-mission wall phase, stuck escape, Nav2 and the
detector) is as in v1.5.5. No ground truth is read.
"""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import tempfile

VERSION = '1.5.6-wall-tour'
HERE = Path(__file__).resolve().parent
PARENT = HERE.parent / '2026-11-26_v155_far_wall_slot'

EDITS = {
    'osp_mission/center_wall.py': [
        ("from osp_mission.slot_escape import MidWallSlot, far_wall, toward_target\n",
         "from osp_mission.slot_escape import MidWallSlot, WallTour\n"),
        ("            self.phase, self.reason = 'wall', 'midmission_wall_slot'\n"
         "            if self.slot.target is None:\n"
         "                far = far_wall(candidates['wall'], self.visited)\n"
         "                if far is not None:\n"
         "                    self.slot.target = np.asarray(far['surface_odom'], float)\n"
         "                    return far\n"
         "            elif np.linalg.norm(here-self.slot.target) > 3.:\n"
         "                return toward_target(candidates['wall'], self.slot.target)\n"
         "            return min(candidates['wall'], key=lambda x: x[0])[1]\n",
         "            if self.slot.tour is None:\n"
         "                budget = self.slot.max_seconds-(elapsed-self.slot.started)\n"
         "                self.slot.tour = WallTour.plan(candidates['wall'], self.visited, here, budget)\n"
         "            if not self.slot.tour.targets:\n"
         "                self.phase, self.reason = 'wall', 'midmission_wall_slot'\n"
         "                return min(candidates['wall'], key=lambda x: x[0])[1]\n"
         "            goal = self.slot.tour.choose(candidates['wall'], here, elapsed)\n"
         "            if goal is not None:\n"
         "                self.phase, self.reason = 'wall', 'midmission_wall_slot'\n"
         "                self.slot.target = self.slot.tour.targets[self.slot.tour.index]\n"
         "                return goal\n"
         "            self.slot.finish('tour_complete')\n"),
        ("        self.slot.dispatched(self.phase, goal.get('r32_kind'))\n",
         "        self.slot.dispatched(self.phase, goal.get('r32_kind'), goal, elapsed)\n"),
    ],
}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def parent_installer():
    spec = importlib.util.spec_from_file_location('far_wall_slot_install_parent', PARENT / 'install.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def install(release, destination):
    destination = destination.expanduser().resolve()
    if destination.exists():
        raise FileExistsError(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=destination.parent) as tmp:
        stage = Path(tmp) / 'workspace'
        parent_installer().install(Path(release), stage)
        package = stage / 'src/osp_mission'
        changed = {}
        for relative, edits in EDITS.items():
            path = package / relative
            text = path.read_text()
            for old, new in edits:
                if text.count(old) != 1:
                    raise ValueError(f'{relative}: expected exactly one match for an edit anchor')
                text = text.replace(old, new)
            path.write_text(text)
            changed[relative] = digest(path)
        shutil.copy2(HERE / 'slot_escape.py', package / 'osp_mission/slot_escape.py')
        changed['osp_mission/slot_escape.py'] = digest(package / 'osp_mission/slot_escape.py')
        parent = json.loads((stage / 'far_wall_slot_manifest.json').read_text())
        report = dict(version=VERSION, parent=parent['version'], base_release=parent['base_release'],
                      changed_from_parent=changed, nav2_unchanged=True,
                      changes=['slot plans an open tour over every far (>= 3 m) unviewed wall cluster',
                               'clusters kept while the estimated tour fits the slot budget',
                               '2 views per cluster; skip when seen or the leg times out',
                               'slot limit 180 s or 10 views; ends when the tour is done'])
        (stage / 'wall_tour_manifest.json').write_text(json.dumps(report, indent=2) + '\n')
        shutil.move(str(stage), str(destination))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('release', type=Path)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    install(args.release, args.destination)
