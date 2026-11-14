"""Replay raw RGB, calibration and scan through both detector state machines.

No ROS nodes, policy publishers or evaluator truth are used. Only transforms
actually present in each snapshot are available. This deliberately calls the
RGB processing function directly: TF delivery timing/queue recovery cannot be
reconstructed from decimated snapshots and is not simulated here.
"""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys
from types import SimpleNamespace as S

import numpy as np

from observe_exposure_pair import detector_class, owner


def replay(snapshots, baseline, candidate):
    sys.path.insert(0, str(candidate/'src/osp_mission'))
    import cv2
    from cv_bridge import CvBridge
    from geometry_msgs.msg import TransformStamped
    from sensor_msgs.msg import CameraInfo, LaserScan
    from rclpy.time import Time
    from osp_mission.landmark_identity import resolved_ids
    cv2.setNumThreads(1)
    classes = {role: detector_class(workspace, 'raw_state_'+role)
               for role, workspace in [('original', baseline), ('candidate', candidate)]}
    current = {}
    outputs = []

    def lookup(parent, child, when, **kwargs):
        d = current['data']
        if child != str(d['camera_frame']) or abs(when.nanoseconds/1e9-float(d['image_stamp'])) > 1e-7:
            raise ValueError('Unrecorded transform request')
        prefix = ('scan_from_camera' if parent == str(d['scan_frame'])
                  else 'map_from_camera' if parent == 'map' else None)
        if prefix is None or prefix+'_translation' not in d:
            raise ValueError('Transform absent from raw snapshot: '+parent)
        tr = TransformStamped()
        tr.header.frame_id = parent
        tr.child_frame_id = child
        tr.header.stamp = when.to_msg()
        tr.transform.translation.x, tr.transform.translation.y, tr.transform.translation.z = map(float, d[prefix+'_translation'])
        tr.transform.rotation.x, tr.transform.rotation.y, tr.transform.rotation.z, tr.transform.rotation.w = map(float, d[prefix+'_rotation_xyzw'])
        return tr

    def record(role, kind, data):
        if kind != 'processed_camera':
            outputs.append(dict(role=role, kind=kind, file=current['file'], **data))

    node = S(get_clock=lambda: None, get_logger=lambda: S(debug=lambda *args: None))
    owners = {role: owner(cls, node, S(lookup_transform=lookup),
        lambda kind, data, role=role: record(role, kind, data)) for role, (cls, _) in classes.items()}
    files = []
    source_hashes = {}
    for directory in snapshots:
        manifest = directory/'snapshot.json'
        source_hashes[str(manifest)] = hashlib.sha256(manifest.read_bytes()).hexdigest()
        for row in json.loads(manifest.read_text())['frames']:
            file = directory/'inputs'/row['file']
            if hashlib.sha256(file.read_bytes()).hexdigest() != row['sha256']:
                raise ValueError('Raw snapshot changed')
            files.append((row['stamp'], file, row['sha256']))
    files.sort()
    if len({stamp for stamp, _, _ in files}) != len(files):
        raise ValueError('Use nonoverlapping snapshots from one native trajectory')
    missing_map = 0
    bridge = CvBridge()
    for stamp, file, sha in files:
        with np.load(file, allow_pickle=False) as archive:
            d = {k: archive[k] for k in archive.files}
        current.update(data=d, file=str(file))
        missing_map += int('map_from_camera_translation' not in d)
        info = CameraInfo()
        info.height, info.width = map(int, d['image'].shape)
        info.k = d['camera_k'].tolist()
        info.d = d['camera_d'].tolist()
        rgb = bridge.cv2_to_imgmsg(d['image'], 'mono8')
        rgb.header.frame_id = str(d['camera_frame'])
        rgb.header.stamp = Time(seconds=float(d['image_stamp'])).to_msg()
        scan = LaserScan()
        scan.header.frame_id = str(d['scan_frame'])
        scan.header.stamp = Time(seconds=float(d['scan_stamp'])).to_msg()
        scan.ranges = d['scan_ranges'].tolist()
        for key in ['angle_min', 'angle_increment', 'range_min', 'range_max']:
            setattr(scan, key, float(d['scan_'+key]))
        for n in owners.values():
            n.info, n.scan, n.depth = info, scan, None
            n.on_rgb(rgb)
        source_hashes[str(file)] = sha
    branches = {}
    for role, n in owners.items():
        ids, conflicts = resolved_ids(n.observations, 3)
        branches[role] = dict(diagnostics=n.diagnostics, resolved_ids=sorted(ids), conflicts=conflicts,
            observation_history={str(k): len(v) for k, v in n.observations.items()},
            emitted_cues=dict(Counter(str(o['id']) for o in outputs if o['role']==role and o['kind']=='cue')))
    return dict(evidence_type='Decimated raw RGB detector-state replay, cold state, recorded transforms only',
        frames=len(files), frames_missing_map_transform=missing_map, branches=branches, outputs=outputs,
        detector_source_sha256={role: sha for role, (_, sha) in classes.items()},
        source_sha256=source_hashes, ground_truth_used=False, ros_node_constructed=False,
        policy_outputs_published=False, native_recognition_improvement_established=False,
        limits=['Cold-start state and decimated inputs differ from either native detector',
                'Direct RGB function replay isolates range/identity processing; it does not exercise the exposure queue',
                'Missing map/odom TF is reported rather than synthesized; later transform delivery is unrecorded',
                'Resolved IDs are processing outputs, not evaluator-qualified accuracy or approach success'])


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--snapshot', type=Path, action='append', required=True)
    for name in ['baseline', 'candidate', 'output']:
        p.add_argument('--'+name, type=Path, required=True)
    a = p.parse_args()
    if a.output.exists():
        raise FileExistsError('Preserve previous detector-state replay')
    r = replay([x.resolve() for x in a.snapshot], a.baseline.resolve(), a.candidate.resolve())
    a.output.write_text(json.dumps(r, indent=2)+'\n')
    print(json.dumps(dict(frames=r['frames'], missing_map_tf=r['frames_missing_map_transform'],
        branches={k: {key: v[key] for key in ['resolved_ids', 'observation_history', 'emitted_cues']}
                  for k, v in r['branches'].items()})))
