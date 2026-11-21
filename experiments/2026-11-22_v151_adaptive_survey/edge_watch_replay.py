"""Replay the R2 two-exposure recommendation with passive recorded pose and GT evaluation.

Run with R2 on PYTHONPATH. The input can be the JPEG sample described in vision_replay.py.
No ROS goal, runtime hazard, busy state or counterfactual camera motion is reconstructed.
"""
import argparse
import csv
import json
import math
from pathlib import Path

import cv2
import numpy as np

from osp_mission.edge_watch import EdgeWatch
from osp_mission.marker_pair import decode
from vision_replay import rows, project_targets


def replay(directory, geometry):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    results = []
    for run in sorted(directory.glob('v151-boxes5-track-m11260?-trial?')):
        scene = json.loads((run/'scene.json').read_text())
        headings = np.genfromtxt(run/'headings.csv', delimiter=',', names=True)
        yaw = np.unwrap(headings['yaw_rad'])
        watch = EdgeWatch()
        triggers = []
        frames = negative_frames = eligible_pairs = 0
        last = -math.inf
        for frame in rows(run):
            i, t = int(frame['frame']), float(frame['elapsed_sim_s'])
            if .075 <= t-last <= .4:
                eligible_pairs += 1
            last = t
            frames += 1
            image = cv2.imread(str(run/'camera'/f'{i:05}.jpg'), cv2.IMREAD_GRAYSCALE)
            if image is None:
                raise ValueError(f'Missing JPEG for {run.name} {i}')
            tiny = []
            decode(image, dictionary, tiny)
            pose = [np.interp(t, headings['elapsed_sim_s'], v) for v in [headings['x'], headings['y'], yaw]]
            boxes = project_targets(pose, scene, geometry)
            negative = all(b[2] < -64 or b[0] > image.shape[1]+64 for b in boxes.values())
            negative_frames += int(negative)
            if not tiny:
                continue
            x, y, a = pose
            dx, dy, _ = geometry['translation']
            origin = [x+math.cos(a)*dx-math.sin(a)*dy,y+math.sin(a)*dx+math.cos(a)*dy]
            qx,qy,qz,qw = geometry['quaternion']
            camera_yaw = a+math.atan2(2*(qy*qz-qx*qw), 2*(qx*qz+qy*qw))
            fx, cx = geometry['k'][0],geometry['k'][2]
            h,w = image.shape
            hits = [(camera_yaw-math.atan2(u-cx,fx),size,2*u/(w-1)-1,2*v/(h-1)-1)
                    for u,v,size,_ in tiny]
            req = watch.offer(t,t,origin,camera_yaw,hits)
            if req:
                watch.commit(req)
                match = next(h for h in hits if h[0]==req[2])
                u,v = (match[2]+1)*(w-1)/2, (match[3]+1)*(h-1)/2
                ids = [index for index,b in boxes.items()
                       if b[0]-12 <= u <= b[2]+12 and b[1]-12 <= v <= b[3]+12]
                triggers.append(dict(frame=i, stamp=t, pixel=[u,v], turn_deg=math.degrees(req[3]),
                                     compatible_gt_ids=ids, conservative_negative=negative))
        results.append(dict(run=run.name,frames=frames,adjacent_sample_pairs=eligible_pairs,
                            conservative_negative_frames=negative_frames,triggers=triggers))
    if len(results)!=5:
        raise ValueError('expected five original recording directories')
    return dict(opencv=cv2.__version__,scope='JPEG sample, 5 consecutive exposures per 50 plus continuous '
                '978..1017 for 112601. Requests computed using truth-base pose interpolated to exposure and '
                'stored static camera calibration as passive TF proxy; all runtime guards assumed clear. '
                'IDs only identify compatibility with a projected rectangle enlarged by 12 pixels. '
                'No actual turns, reacquisition, registration, approach or collision outcomes inferred.',
                runs=results)


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('sample', type=Path)
    p.add_argument('geometry', type=Path)
    p.add_argument('output', type=Path)
    args=p.parse_args()
    cv2.setNumThreads(1)
    result=replay(args.sample,json.loads(args.geometry.read_text()))
    args.output.write_text(json.dumps(result,indent=2)+'\n')
