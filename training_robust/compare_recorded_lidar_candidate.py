"""Run an isolated detector candidate on previously sealed sensor-only inputs."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys
from types import SimpleNamespace as S

import numpy as np


def compare(replay_path, workspace):
    sys.path.insert(0, str(workspace/'src/osp_mission'))
    import cv2
    from geometry_msgs.msg import TransformStamped
    from sensor_msgs.msg import LaserScan
    from rclpy.time import Time
    from rclpy.clock import ClockType
    from osp_mission.detector import Detector
    source = Path(sys.modules[Detector.__module__].__file__).resolve()
    if source != (workspace/'src/osp_mission/osp_mission/detector.py').resolve():
        raise ValueError('Candidate imported from another workspace')
    result = json.loads(replay_path.read_text())
    if result.get('ground_truth_used') is not False:
        raise ValueError('Require independent sensor-only range predictions')
    for row in result['rows']:
        file = Path(row['file'])
        if hashlib.sha256(file.read_bytes()).hexdigest() != row['sha256']:
            raise ValueError('Raw input changed after original replay')
        with np.load(file, allow_pickle=False) as d:
            index = np.flatnonzero(d['ids'] == row['id'])
            if len(index) != 1:
                raise ValueError('Ambiguous ID occurrence in this frame')
            corners = d['corners'][index[0]]
            uv = corners.mean(axis=0)
            point = cv2.undistortPoints(np.array([[uv]], dtype=np.float64),
                d['camera_k'].reshape(3, 3), d['camera_d'])[0, 0]
            ray = np.array([*point, 1.])
            scan = LaserScan()
            scan.header.frame_id = str(d['scan_frame'])
            scan.header.stamp = Time(seconds=float(d['scan_stamp'])).to_msg()
            scan.ranges = d['scan_ranges'].tolist()
            for key in ['angle_min', 'angle_increment', 'range_min', 'range_max']:
                setattr(scan, key, float(d['scan_'+key]))
            tr = TransformStamped()
            tr.transform.translation.x, tr.transform.translation.y, tr.transform.translation.z = map(float, d['scan_from_camera_translation'])
            tr.transform.rotation.x, tr.transform.rotation.y, tr.transform.rotation.z, tr.transform.rotation.w = map(float, d['scan_from_camera_rotation_xyzw'])
            owner = S(scan=scan, tf=S(lookup_transform=lambda *a, **kw: tr), diagnostics={})
            value = Detector.lidar_point(owner, ray, str(d['camera_frame']),
                Time(seconds=float(d['image_stamp']), clock_type=ClockType.ROS_TIME))
            fallback = owner.diagnostics.get('lidar_center_fallback_calls', 0) == 1
            previous = row['original'].get('camera_point')
            if previous is not None:
                if value is None or fallback or not np.allclose(value, previous, atol=1e-5, rtol=0):
                    raise AssertionError('An originally successful range changed')
            elif value is not None and not fallback:
                raise AssertionError('Candidate changed behavior outside central fallback')
            row['candidate'] = dict(reason='returned_range' if value is not None else 'no_range',
                central_fallback_used=fallback, camera_point=None if value is None else value.tolist())
    attempted = [r for r in result['rows'] if r['production_would_attempt_range']]
    result.update(evidence_type='Same sealed raw sensor inputs evaluated by an unqueued range candidate',
        parent_replay=str(replay_path), parent_replay_sha256=hashlib.sha256(replay_path.read_bytes()).hexdigest(),
        candidate_source=str(source), candidate_source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        candidate_function_checks=len(result['rows']), originally_returned_ranges_preserved=True,
        candidate_attempt_reasons=dict(Counter(r['candidate']['reason'] for r in attempted)),
        candidate_fallback_attempts=sum(r['candidate']['central_fallback_used'] for r in attempted),
        native_accuracy_qualified=False, production_promoted=False)
    return result


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    for name in ['replay', 'workspace', 'output']:
        p.add_argument('--'+name, type=Path, required=True)
    a = p.parse_args()
    if a.output.exists():
        raise FileExistsError('Preserve the original candidate comparison')
    result = compare(a.replay.resolve(), a.workspace.resolve())
    a.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({k: result[k] for k in ['candidate_function_checks', 'originally_returned_ranges_preserved',
                                           'candidate_attempt_reasons', 'candidate_fallback_attempts']}))
