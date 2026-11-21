"""Estimate eligible edge-print exposures on unchanged JPEG paths, before full closed-loop testing."""
import argparse
from collections import Counter
import csv
import json
import math
from pathlib import Path
import sys

import cv2
import numpy as np

from osp_mission.edge_print import budget, edge_print
from osp_mission.marker_pair import decode

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'2026-11-22_v151_adaptive_survey'))
from vision_replay import project_targets


def audit(folder, calibration):
    scene = json.loads((folder/'scene.json').read_text())
    metrics = json.loads((folder/'metrics.json').read_text())
    registrations = {int(ident): t for ident,t in metrics.get('registered_s', {}).items() if t is not None}
    track = np.genfromtxt(folder/'headings.csv', names=True, delimiter=',')
    stamp = track['elapsed_sim_s']
    series = (track['x'], track['y'], np.unwrap(track['yaw_rad']))
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    fx, cx = calibration['k'][0], calibration['k'][2]
    shape = (calibration['height'],calibration['width'])
    camera_yaw = math.atan2(2*(calibration['quaternion'][1]*calibration['quaternion'][2]-
                              calibration['quaternion'][0]*calibration['quaternion'][3]),
                            2*(calibration['quaternion'][0]*calibration['quaternion'][2]+
                              calibration['quaternion'][1]*calibration['quaternion'][3]))
    counts = Counter()
    allowed, history = [], []
    seen_ids = {}
    with (folder/'frames.csv').open() as f:
        for row in csv.DictReader(f):
            t, n = float(row['elapsed_sim_s']), int(row['frame'])
            if not 0 <= t <= min(600., metrics['elapsed_sim_s']):
                continue
            image = cv2.imread(str(folder/'camera'/f'{n:05d}.jpg'), cv2.IMREAD_GRAYSCALE)
            if image is None or image.shape != shape:
                raise ValueError(f'Invalid camera frame {n} in {folder}')
            counts['frames'] += 1
            # An unscaled OpenCV decode is necessary for this policy. Avoid the
            # more expensive second pass on frames which cannot emit the cue.
            _, ids, _ = cv2.aruco.detectMarkers(image, dictionary)
            if ids is None:
                continue
            native = []
            corners, _ = decode(image,dictionary,[],native)
            hit = edge_print(native,corners,shape[1],shape[0],18)
            if hit is None:
                continue
            ident, u, v, px = hit
            counts['eligible_image_frames'] += 1
            if registrations.get(ident, math.inf) <= t:
                counts['already_registered_frames'] += 1
                continue
            pose = tuple(float(np.interp(t,stamp,feature)) for feature in series)
            x,y,a = pose
            dx,dy,_ = calibration['translation']
            origin = (x+math.cos(a)*dx-math.sin(a)*dy, y+math.sin(a)*dx+math.cos(a)*dy)
            bearing = a+camera_yaw-math.atan2(u-cx,fx)
            rectangle = project_targets(pose,scene,calibration).get(ident)
            matches = bool(rectangle and rectangle[0]-3 <= u <= rectangle[2]+3 and
                           rectangle[1]-3 <= v <= rectangle[3]+3)
            seen_ids.setdefault(ident,dict(first_s=t,matching_frames=0,unmatched_frames=0))
            seen_ids[ident]['matching_frames' if matches else 'unmatched_frames'] += 1
            cue = dict(id=ident,yaw_odom=bearing,origin_odom=origin)
            if budget(history, 8.*len(history), t, 600., 600.-t, cue):
                history.append(dict(source='edge_print',candidate_id=ident,started=t,
                                    odom_camera=(*origin,a+camera_yaw),odom_yaw=bearing))
                allowed.append(dict(frame=n,elapsed_s=t,id=ident,short_side_px=px,
                                    same_id_at_projected_location=matches,
                                    later_registration_s=registrations.get(ident)))
    return dict(run=folder.name,counts=dict(counts),ids=seen_ids,requests_if_navigation_and_safety_permit=allowed)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folders',type=Path,nargs='+')
    parser.add_argument('--calibration',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    cv2.setNumThreads(1)
    geometry = json.loads(args.calibration.read_text())
    output = {'camera': 'original recorded JPEG, mono8', 'cv2': cv2.__version__,
              'method': 'Navigation and hazard state are not replayed. Potential requests on a frozen trajectory get no approach or registration credit.',
              'runs': [audit(folder,geometry) for folder in args.folders]}
    args.output.write_text(json.dumps(output,indent=2,allow_nan=False)+'\n')
