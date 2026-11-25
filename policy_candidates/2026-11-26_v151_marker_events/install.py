"""Mixed v1.5.1-balanced plus confirmation events and an on-robot signal.

Copies the frozen release source unchanged and then adds instrumentation
only; target selection, Nav2 settings and recognition thresholds are the
release's:
  detector.py      publishes mission/marker_events once, at the frame that
                   confirms an ID (and 'retracted' if an ID is withdrawn)
  marker_signal.py lightring flash + tone on confirmation (enabled:=false
                   turns it off); never commands motion
  robot.launch.py  starts marker_signal (argument marker_signal, default true)
  setup.py         registers the marker_signal executable
The caller (tools/team_sim/setup.py) compiles the C++ helpers and builds.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

VERSION = '1.5.1-marker-events'

EDITS = {
    'osp_mission/detector.py': [
        ("        self.observation_pub=self.create_publisher(String,'mission/marker_observations',10)\n",
         "        self.observation_pub=self.create_publisher(String,'mission/marker_observations',10)\n"
         "        # Instrumentation: one event when an ID becomes confirmed (no polling delay).\n"
         "        self.event_pub=self.create_publisher(String,'mission/marker_events',10)\n"
         "        self.announced=set()\n"),
        ("                self.separate_markers.observe(stamp.nanoseconds,accepted_quads)\n",
         "                self.separate_markers.observe(stamp.nanoseconds,accepted_quads)\n"
         "                self.announce_confirmations(stamp)\n"),
        ("    def publish(self):\n"
         "        ids,conflicts=resolved_ids(self.observations,self.p('confirmations'),separate=getattr(self,'separate_markers',None))\n",
         "    def announce_confirmations(self,stamp):\n"
         "        ids,_=resolved_ids(self.observations,self.p('confirmations'),separate=getattr(self,'separate_markers',None))\n"
         "        for ident in sorted(set(ids)-self.announced):\n"
         "            point=np.median([v[1] for v in self.observations[ident]],axis=0)\n"
         "            self.event_pub.publish(String(data=json.dumps(dict(event='confirmed',id=int(ident),\n"
         "                exposure_stamp=stamp.nanoseconds/1e9,published_stamp=self.get_clock().now().nanoseconds/1e9,\n"
         "                frames=len(self.observations[ident]),map_point=[float(v) for v in point]))))\n"
         "        self.announced|=set(ids)\n"
         "\n"
         "    def publish(self):\n"
         "        ids,conflicts=resolved_ids(self.observations,self.p('confirmations'),separate=getattr(self,'separate_markers',None))\n"
         "        for ident in sorted(self.announced-set(ids)):\n"
         "            self.event_pub.publish(String(data=json.dumps(dict(event='retracted',id=int(ident),\n"
         "                published_stamp=self.get_clock().now().nanoseconds/1e9))))\n"
         "        self.announced&=set(ids)\n"),
    ],
    'launch/robot.launch.py': [
        ("        Node(package='osp_mission',executable='mission',output='screen',parameters=[{'use_sim_time':sim,\n",
         "        Node(package='osp_mission',executable='marker_signal',output='screen',parameters=[{'use_sim_time':sim,\n"
         "            'enabled':value('marker_signal')=='true'}]),\n"
         "        Node(package='osp_mission',executable='mission',output='screen',parameters=[{'use_sim_time':sim,\n"),
        ("'rviz':'true',", "'rviz':'true','marker_signal':'true',"),
    ],
    'setup.py': [
        ("'detector = osp_mission.detector:main',",
         "'detector = osp_mission.detector:main', 'marker_signal = osp_mission.marker_signal:main',"),
    ],
}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def install(release, destination):
    here = Path(__file__).resolve().parent
    release, destination = release.resolve(), destination.expanduser().resolve()
    if destination.exists():
        raise FileExistsError(destination)
    manifest = json.loads((release / 'source_manifest.json').read_text())
    for relative, expected in manifest['source_sha256'].items():
        if digest(release / 'src/osp_mission' / relative) != expected:
            raise ValueError('Frozen release source changed: ' + relative)
    shutil.copytree(release / 'src', destination / 'src', ignore=shutil.ignore_patterns('__pycache__', '.pytest_cache'))
    configs = destination / 'frozen-config'
    configs.mkdir()
    for name in manifest['external_config_sha256']:
        shutil.copy2(release / name, configs / name)
    package = destination / 'src/osp_mission'
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
    shutil.copy2(here / 'marker_signal.py', package / 'osp_mission/marker_signal.py')
    changed['osp_mission/marker_signal.py'] = digest(package / 'osp_mission/marker_signal.py')
    report = dict(version=VERSION, base_release=release.name, instrumentation_only=True,
                  changed_files=changed, policy_and_nav2_unchanged=True)
    (destination / 'marker_events_manifest.json').write_text(json.dumps(report, indent=2) + '\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('release', type=Path)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    install(args.release, args.destination)
