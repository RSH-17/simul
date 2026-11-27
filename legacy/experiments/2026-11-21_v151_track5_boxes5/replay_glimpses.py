"""Replay selected saved JPEGs with the unchanged installed OpenCV detector.

Set PYTHONPATH to the frozen runtime's src/osp_mission. This is a lossy-image
diagnostic, not a reconstruction of the native RGB input or ROS TF pipeline.
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np
from osp_mission.marker_pair import decode


def replay(run, frames):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    rows = []
    for frame in frames:
        image = cv2.imread(str(run/'camera'/f'{frame:05d}.jpg'), cv2.IMREAD_GRAYSCALE)
        assert image is not None
        small = []
        _, ids = decode(image, dictionary, small)
        _, _, rejected = cv2.aruco.detectMarkers(image, dictionary)
        proposals = []
        for quad in rejected:
            points = quad.reshape(4, 2)
            edge = np.linalg.norm(points-np.roll(points, 1, axis=0), axis=1)
            proposals.append(dict(center=points.mean(axis=0).tolist(), minimum_edge_px=float(edge.min())))
        rows.append(dict(frame=frame, decoded_ids=[] if ids is None else ids.flatten().tolist(),
                         small_candidates=small, native_rejected_quads=proposals))
    return dict(run=run.name, opencv=cv2.__version__, frames=rows,
                limitations='Saved JPEG replay only; compression and conversion differ from raw RGB. '
                'No ROS timestamps, TF, range association, temporal tracking or scheduling were replayed.')


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('run', type=Path)
    p.add_argument('output', type=Path)
    p.add_argument('frames', type=int, nargs='+')
    a = p.parse_args()
    result = replay(a.run, a.frames)
    a.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))
