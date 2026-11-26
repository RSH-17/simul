"""v1.5.4 wall slot + stuck escape, with the slot sent to a far, unvisited wall.

Builds an exact `1.5.4-wall-slot-escape` workspace first, then:
  slot_escape.py   replaced: slot limit 120 s (travel to a far wall takes
                   longer), plus far_wall() and toward_target()
  center_wall.py   records the robot's own positions (odom, every 0.5 m).
                   The slot's first view is the unviewed wall segment
                   farthest (>= 3 m) from them; while the robot is more than
                   3 m from that segment, slot views stay at it. Afterwards
                   the baseline nearest/continuing wall choice applies.
  phased_hybrid.py logs the chosen segment's novelty distance
Everything else, including the end-of-mission wall phase and Nav2, is as in
v1.5.4. Experiment: experiments/2026-11-26_far_wall_slot.
"""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import tempfile

VERSION = '1.5.5-far-wall-slot'
HERE = Path(__file__).resolve().parent
PARENT = HERE.parent / '2026-11-26_v154_wall_slot_escape'

EDITS = {
    'osp_mission/center_wall.py': [
        ("from osp_mission.slot_escape import MidWallSlot\n",
         "from osp_mission.slot_escape import MidWallSlot, far_wall, toward_target\n"),
        ("        self.targets_found = 0\n",
         "        self.targets_found = 0\n"
         "        self.visited = []\n"),
        ("        samples = surface_cells(grid, pose)\n",
         "        here = odom_point(pose[:2], transform)\n"
         "        if not self.visited or np.linalg.norm(here-self.visited[-1]) > .5:\n"
         "            self.visited.append(here)\n"
         "        samples = surface_cells(grid, pose)\n"),
        ("            self.phase, self.reason = 'wall', 'midmission_wall_slot'\n"
         "            return min(candidates['wall'], key=lambda x: x[0])[1]\n",
         "            self.phase, self.reason = 'wall', 'midmission_wall_slot'\n"
         "            if self.slot.target is None:\n"
         "                far = far_wall(candidates['wall'], self.visited)\n"
         "                if far is not None:\n"
         "                    self.slot.target = np.asarray(far['surface_odom'], float)\n"
         "                    return far\n"
         "            elif np.linalg.norm(here-self.slot.target) > 3.:\n"
         "                return toward_target(candidates['wall'], self.slot.target)\n"
         "            return min(candidates['wall'], key=lambda x: x[0])[1]\n"),
    ],
    'osp_mission/phased_hybrid.py': [
        ("wall_slot=self.schedule.slot.report())",
         "wall_slot=self.schedule.slot.report(), far_wall_novelty_m=goal.get('far_wall_novelty_m'))"),
    ],
}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def parent_installer():
    spec = importlib.util.spec_from_file_location('wall_slot_escape_install_parent', PARENT / 'install.py')
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
        parent = json.loads((stage / 'wall_slot_escape_manifest.json').read_text())
        report = dict(version=VERSION, parent=parent['version'], base_release=parent['base_release'],
                      changed_from_parent=changed, nav2_unchanged=True,
                      changes=['slot first view: farthest unviewed wall from visited positions (>= 3 m)',
                               'slot stays at that segment until within 3 m', 'slot limit 120 s or 6 views'])
        (stage / 'far_wall_slot_manifest.json').write_text(json.dumps(report, indent=2) + '\n')
        shutil.move(str(stage), str(destination))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('release', type=Path)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    install(args.release, args.destination)
