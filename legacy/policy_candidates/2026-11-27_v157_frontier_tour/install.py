"""v1.5.6 wall tour, with far frontiers in the tour and a replan after every target.

Builds an exact `1.5.6-wall-tour` workspace first, then:
  slot_escape.py   replaced: adds open_frontiers(), frontier_clusters() and
                   RecedingTour; WallTour targets carry a kind (wall/frontier);
                   the slot forwards every dispatched goal to the tour
  center_wall.py   the mid-mission slot collects frontier goals (same filters as
                   discovery_frontier) and may start when walls or frontiers
                   exist. RecedingTour plans the open tour over far wall clusters
                   (>= 3 m from visited positions) and far frontier clusters
                   (>= 4 m), follows its first target, and replans after it, so
                   walls mapped on the way join the next plan. Without any far
                   target on the first plan it keeps v1.5.5's nearest-wall rule.
Everything else (slot start time and 180 s / 10 wall views, the end-of-mission
wall phase, stuck escape, Nav2, SLAM and the detector) is as in v1.5.6.
No ground truth is read.
"""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import tempfile

VERSION = '1.5.7-frontier-tour'
HERE = Path(__file__).resolve().parent
PARENT = HERE.parent / '2026-11-27_v156_wall_tour'

EDITS = {
    'osp_mission/center_wall.py': [
        ("from osp_mission.slot_escape import MidWallSlot, WallTour\n",
         "from osp_mission.slot_escape import MidWallSlot, RecedingTour, open_frontiers\n"),
        ("        if self.slot.update(elapsed, duration, self.targets_found, bool(candidates['wall'])):\n"
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
         "            self.slot.finish('tour_complete')\n",
         "        frontiers = [] if self.slot.done else open_frontiers(\n"
         "            choices, costmap, nav, veto, lambda point: odom_point(point, transform))\n"
         "        if self.slot.update(elapsed, duration, self.targets_found, bool(candidates['wall']) or bool(frontiers)):\n"
         "            if self.slot.tour is None:\n"
         "                self.slot.tour = RecedingTour()\n"
         "            budget = self.slot.max_seconds-(elapsed-self.slot.started)\n"
         "            goal = self.slot.tour.choose(candidates['wall'], frontiers, self.visited, here, elapsed, budget)\n"
         "            if goal is None and self.slot.tour.first_size == 0 and candidates['wall']:\n"
         "                goal = min(candidates['wall'], key=lambda x: x[0])[1]\n"
         "            if goal is not None:\n"
         "                self.phase, self.reason = 'wall', 'midmission_wall_slot'\n"
         "                self.slot.target = self.slot.tour.target\n"
         "                return goal\n"
         "            self.slot.finish('tour_complete')\n"),
    ],
}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def parent_installer():
    spec = importlib.util.spec_from_file_location('wall_tour_install_parent', PARENT / 'install.py')
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
        parent = json.loads((stage / 'wall_tour_manifest.json').read_text())
        report = dict(version=VERSION, parent=parent['version'], base_release=parent['base_release'],
                      changed_from_parent=changed, nav2_unchanged=True, slam_unchanged=True,
                      changes=['far (>= 4 m from visited) frontier goals join the slot tour as targets',
                               'tour replanned after every finished target (receding horizon, max 12 plans)',
                               'slot may start when walls or far frontiers exist',
                               'frontier target done when reached (1.5 m), gone, 2 goals sent, or leg timeout'])
        (stage / 'frontier_tour_manifest.json').write_text(json.dumps(report, indent=2) + '\n')
        shutil.move(str(stage), str(destination))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('release', type=Path)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    install(args.release, args.destination)
