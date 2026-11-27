"""v1.5.7 frontier tour, plus a trace of what the phased hybrid compared for each goal.

Builds an exact `1.5.7-frontier-tour` workspace first, then:
  rule_trace.py    new: read-only helpers that copy the rule's candidate lists
                   (unseen object faces in BoxBatch.rank order, wall views by
                   score, discovery frontiers by score, slot state) into JSON
  center_wall.py   decide() keeps those lists in `self.trace`; no comparison,
                   filter or return value changes
  phased_hybrid.py select() clears the trace first; dispatched() appends one row
                   per goal to native/rule_trace.jsonl (what was compared, which
                   list and rank the goal came from)
Decisions are meant to be identical to v1.5.7. Tracing errors are caught and only
drop the trace. No ground truth is read.
"""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import tempfile

VERSION = '1.5.7.1-decision-trace'
HERE = Path(__file__).resolve().parent
PARENT = HERE.parent / '2026-11-27_v157_frontier_tour'

EDITS = {
    'osp_mission/center_wall.py': [
        ("from osp_mission.slot_escape import MidWallSlot, RecedingTour, open_frontiers\n",
         "from osp_mission.slot_escape import MidWallSlot, RecedingTour, open_frontiers\n"
         "from osp_mission import rule_trace\n"),
        ("    def decide(self, grid, costmap, pose, transform, viewed, elapsed, duration, veto, choices=(), fitted=(), probe_only=False):\n",
         "    def decide(self, grid, costmap, pose, transform, viewed, elapsed, duration, veto, choices=(), fitted=(), probe_only=False):\n"
         "        self.trace = None\n"),
        ("        if probe_only:\n            if crowded:\n",
         "        self.trace = rule_trace.inputs(elapsed, duration, probe_only, candidates, deferred, fresh, boxes,\n"
         "                                       crowded, outside if crowded else None)\n"
         "        if probe_only:\n            if crowded:\n"),
        ("            choices, costmap, nav, veto, lambda point: odom_point(point, transform))\n",
         "            choices, costmap, nav, veto, lambda point: odom_point(point, transform))\n"
         "        rule_trace.slot(self.trace, frontiers, self.slot.report())\n"),
        ("        frontier = None if fresh else discovery_frontier(choices, costmap, nav, veto)\n",
         "        frontier = None if fresh else discovery_frontier(choices, costmap, nav, veto)\n"
         "        rule_trace.gates(self.trace, fresh, frontier, wall_reserve, choices, costmap, nav, veto,\n"
         "                         outside if crowded else None)\n"),
    ],
    'osp_mission/phased_hybrid.py': [
        ("from osp_mission import _r32_navigation_core as r32\n",
         "from osp_mission import _r32_navigation_core as r32\n"
         "from osp_mission import rule_trace\n"),
        ("               print_only=False, box_probe=False, choices=()):\n",
         "               print_only=False, box_probe=False, choices=()):\n"
         "        self.schedule.trace = None\n"),
        ("            owner.get_logger().warning(str(exc))\n",
         "            owner.get_logger().warning(str(exc))\n"
         "        try:\n"
         "            trace = getattr(self.schedule, 'trace', None)\n"
         "            if isinstance(trace, dict) and trace.get('elapsed') is not None and abs(trace['elapsed']-elapsed) > 1.:\n"
         "                trace = None  # left over from an earlier decision\n"
         "            row = rule_trace.record(trace, goal, self.schedule.reason, self.schedule.phase, elapsed)\n"
         "            with (Path(owner.p('output_dir'))/'rule_trace.jsonl').open('a') as out:\n"
         "                out.write(json.dumps(row, allow_nan=False, default=str)+'\\n')\n"
         "        except Exception as exc:\n"
         "            owner.get_logger().warning(f'Could not save rule trace: {exc}')\n"),
    ],
}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def parent_installer():
    spec = importlib.util.spec_from_file_location('frontier_tour_install_parent', PARENT / 'install.py')
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
        shutil.copy2(HERE / 'rule_trace.py', package / 'osp_mission/rule_trace.py')
        changed['osp_mission/rule_trace.py'] = digest(package / 'osp_mission/rule_trace.py')
        parent = json.loads((stage / 'frontier_tour_manifest.json').read_text())
        report = dict(version=VERSION, parent=parent['version'], base_release=parent['base_release'],
                      changed_from_parent=changed, nav2_unchanged=True, slam_unchanged=True,
                      decisions_unchanged_by_design=True,
                      changes=['phased hybrid candidate lists traced to native/rule_trace.jsonl, one row per goal'])
        (stage / 'decision_trace_manifest.json').write_text(json.dumps(report, indent=2) + '\n')
        shutil.move(str(stage), str(destination))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('release', type=Path)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    install(args.release, args.destination)
