"""Check an enrolled geometric witness against the actual print and the calibrated camera."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy import ndimage

from geometry import RESOLUTION, raster, table_parts
from surface_audit import aim_at, blockers, readable_indices
from exposure_opportunity import pose


def read(file):
    return json.loads(file.read_text())


def inspect(fixture, manifest, camera):
    sdf = fixture / 'world/osp_studio.sdf'
    if hashlib.sha256(sdf.read_bytes()).hexdigest() != manifest['files']['world/osp_studio.sdf']:
        raise ValueError('Enrolled and tested SDF differ')
    scene = read(fixture / 'scene.json')
    tables = read(fixture / 'run/simulation/demo_layout.json')['tables']
    world = ET.parse(sdf).find('world')
    occupied, origin = raster(scene, table_parts(tables, scene['furniture']))
    clearance = ndimage.distance_transform_edt(np.pad(~occupied, 1))[1:-1, 1:-1] * RESOLUTION
    labels, _ = ndimage.label(clearance >= .40)

    def cell(xy):
        return tuple(np.floor((np.asarray(xy) - origin) / RESOLUTION).astype(int)[::-1])

    start = labels[cell(manifest['start'][:2])]
    rows = []
    for target in scene['targets']:
        ident = target['id']
        viewer = np.array(manifest['geometric_witnesses_m'][str(ident)])
        marker = world.find(f"model[@name='target_{ident}']")
        frame = pose(marker)
        paper = pose(marker.find("model[@name='paper']"))
        ink = pose(next(m for m in marker.findall('model') if m.get('name').startswith('cell_')))
        axis = np.array([math.cos(frame[5]), math.sin(frame[5]), 0.])
        sign = float(np.sign(ink[0] - paper[0]))
        if sign == 0.:
            raise ValueError('Ambiguous print direction')
        center = frame[:3] + np.array([0., 0., paper[2]]) + ink[0] * axis
        normal = sign * axis
        tangent = np.array([-normal[1], normal[0], 0.])
        origins, axes = aim_at(viewer, center, camera)
        zlo = min(origins[0, 2], center[2] - target['size']/2)
        zhi = max(origins[0, 2], center[2] + target['size']/2)
        indices, reason = readable_indices(center, normal, tangent, target['size'], 18., origins,
                                           axes, camera, blockers(world, zlo, zhi))
        idx = cell(viewer)
        rows.append(dict(id=ident, host=target['attached_to'], witness_xy_m=viewer.tolist(),
                         base_distance_m=float(np.linalg.norm(viewer-frame[:2])),
                         grid_clearance_m=float(clearance[idx]),
                         same_connected_grid_region=bool(start and labels[idx] == start),
                         optical_witness_18px=bool(len(indices)), reason=reason))
    return dict(seed=scene['seed'], tables=manifest['tables'], targets=rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('inputs', nargs='+', type=Path, help='A paired campaign or an individual portable fixture')
    parser.add_argument('--calibration', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    camera = read(args.calibration)
    worlds = []
    for root in args.inputs:
        if (root/'fixture.json').is_file():
            worlds.append(inspect(root, read(root/'fixture.json'), camera))
        else:
            for entry in read(root/'protocol.json')['worlds']:
                worlds.append(inspect(root/'fixtures'/f"map-{entry['seed']}", entry, camera))
    if len({w['seed'] for w in worlds}) != len(worlds):
        raise ValueError('Duplicate seed')
    result = dict(scope='Geometric optical opportunity at the enrolled witness, aiming at the actual print. '
                        'Five static SDF sightlines, incidence ≤60 degrees, calibrated 480px camera. '
                        'Grid connectivity uses 0.40m clearance. No Nav2 plan or actual image is inferred.',
                  calibration_sha256=hashlib.sha256(args.calibration.read_bytes()).hexdigest(), worlds=worlds)
    args.output.write_text(json.dumps(result, ensure_ascii=False, allow_nan=False, indent=2)+'\n')


if __name__ == '__main__':
    main()
