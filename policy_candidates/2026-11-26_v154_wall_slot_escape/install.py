"""Marker-events baseline plus a mid-mission wall slot and a stuck escape.

Builds an exact `1.5.1-marker-events` workspace first (its own installer
checks the frozen release), then applies two policy changes proposed in
experiments/2026-11-26_baseline_repeat/REPORT.md section 8:
  center_wall.py   one bounded wall pass after 30 % of the mission when no
                   wall phase has run, fewer than three IDs are registered
                   and wall views exist (max 90 s or 6 wall views), then the
                   baseline interior-first rules again
  phased_hybrid.py passes the registered ID count and logs the slot state
  mission.py       three instant planner failures within 0.1 m trigger the
                   existing scan-checked backup, else a 90 degree turn, and
                   exclude the stuck spot
  slot_escape.py   the two small state machines (new)
Nav2 settings, recognition thresholds and wall choice inside a wall phase are
unchanged. The caller (tools/team_sim/setup.py) compiles helpers and builds.
"""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import tempfile

VERSION = '1.5.4-wall-slot-escape'
HERE = Path(__file__).resolve().parent
PARENT = HERE.parent / '2026-11-26_v151_marker_events'

EDITS = {
    'osp_mission/center_wall.py': [
        ("from osp_mission.wall_bearings import WallBearings\n",
         "from osp_mission.wall_bearings import WallBearings\n"
         "from osp_mission.slot_escape import MidWallSlot\n"),
        ("        self.counts = {}\n",
         "        self.counts = {}\n"
         "        self.slot = MidWallSlot()\n"
         "        self.targets_found = 0\n"),
        ("        if not boxes:\n            goal = self.weak.choose(",
         "        # One bounded wall pass mid-mission; wall choice itself is unchanged.\n"
         "        if self.slot.update(elapsed, duration, self.targets_found, bool(candidates['wall'])):\n"
         "            self.phase, self.reason = 'wall', 'midmission_wall_slot'\n"
         "            return min(candidates['wall'], key=lambda x: x[0])[1]\n"
         "        if self.slot.done and self.reason == 'midmission_wall_slot':\n"
         "            self.phase, self.reason, self.wall_anchor = 'interior', 'midmission_wall_slot_end', None\n"
         "        if not boxes:\n            goal = self.weak.choose("),
        ("        self.batch.dispatched(goal)\n",
         "        self.batch.dispatched(goal)\n"
         "        self.slot.dispatched(self.phase, goal.get('r32_kind'))\n"),
    ],
    'osp_mission/phased_hybrid.py': [
        ("        self.schedule.weak_clock = clock\n",
         "        self.schedule.weak_clock = clock\n"
         "        self.schedule.targets_found = len(owner.targets)\n"),
        ("clearance_m=goal.get('clearance_m'))",
         "clearance_m=goal.get('clearance_m'), wall_slot=self.schedule.slot.report())"),
    ],
    'osp_mission/mission.py': [
        ("from osp_mission.scan_safety import rear_clear\n",
         "from osp_mission.scan_safety import rear_clear\n"
         "from osp_mission.slot_escape import StuckEscape\n"),
        ("self.last_recovery=-100.; self.rear_clear=False\n",
         "self.last_recovery=-100.; self.rear_clear=False\n"
         "        self.stuck_escape=StuckEscape(); self.escape_pending=False\n"),
        ("        if guard is not None and self.kind=='nav' and result.status==6 and self.now()-self.action_start<.6:\n"
         "            self.plan_after=self.now()+2.\n",
         "        if guard is not None and self.kind=='nav' and result.status==6 and self.now()-self.action_start<.6:\n"
         "            self.plan_after=self.now()+2.\n"
         "            self.check_stuck()\n"
         "        elif self.kind=='nav' and result.status==4:\n"
         "            self.stuck_escape.progress()\n"),
        ("    def finish(self,reason):\n",
         "    def check_stuck(self):\n"
         "        try:\n"
         "            t=self.tf.lookup_transform(self.p('map_frame'),self.p('base_frame'),Time()).transform.translation\n"
         "            xy=(t.x,t.y)\n"
         "        except Exception:\n"
         "            xy=None\n"
         "        if not self.stuck_escape.quick_failure(xy):return\n"
         "        self.escape_pending=True;self.recovery_pending=True;self.scan_after=False\n"
         "        self.excluded.append(xy)\n"
         "        self.get_logger().warning(f'Planner failed instantly {self.stuck_escape.failures} times near ({xy[0]:.2f}, {xy[1]:.2f}); trying a scan-checked escape')\n"
         "        try:\n"
         "            with (Path(self.p('output_dir'))/'stuck_escape.jsonl').open('a') as out:\n"
         "                out.write(json.dumps(dict(elapsed=self.now()-self.started,position=list(xy),\n"
         "                    trigger=self.stuck_escape.triggers,rear_clear=bool(self.rear_clear)))+'\\n')\n"
         "        except OSError as exc:self.get_logger().warning(str(exc))\n"
         "    def finish(self,reason):\n"),
        ("                self.last_recovery=self.now(); goal=BackUp.Goal(); goal.target.x=.20; goal.speed=.08; goal.time_allowance.sec=8\n"
         "                self.send(self.backup,goal,'backup'); return\n",
         "                self.last_recovery=self.now(); self.escape_pending=False; goal=BackUp.Goal(); goal.target.x=.20; goal.speed=.08; goal.time_allowance.sec=8\n"
         "                self.send(self.backup,goal,'backup'); return\n"
         "            if self.escape_pending and self.spin.server_is_ready() and self.now()>=self.spin_blocked_until:\n"
         "                self.escape_pending=False\n"
         "                self.get_logger().warning('Backup unavailable; turning 90 degrees in place to escape')\n"
         "                goal=Spin.Goal(); goal.target_yaw=math.pi/2; goal.time_allowance.sec=10\n"
         "                self.send(self.spin,goal,'spin'); return\n"
         "            self.escape_pending=False\n"),
    ],
}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def parent_installer():
    spec = importlib.util.spec_from_file_location('marker_events_install', PARENT / 'install.py')
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
        parent = json.loads((stage / 'marker_events_manifest.json').read_text())
        report = dict(version=VERSION, parent=parent['version'], base_release=parent['base_release'],
                      changed_files=changed, nav2_unchanged=True,
                      changes=['midmission_wall_slot(start=30%, max=90s or 6 views)',
                               'stuck_escape(3 instant planner failures within 0.1 m)'])
        (stage / 'wall_slot_escape_manifest.json').write_text(json.dumps(report, indent=2) + '\n')
        shutil.move(str(stage), str(destination))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('release', type=Path)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    install(args.release, args.destination)
