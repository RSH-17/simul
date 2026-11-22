"""Locate the first binding triangulation gate during each recorded sighting."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path

import numpy as np
from osp_mission.camera_multiview import CameraVisualRange


def campaign_windows(rows, ident, min_pixels, true_ranges):
    model = CameraVisualRange()
    groups = []
    last = -math.inf
    for r in rows:
        if r['id'] != ident or r['pixels'] < min_pixels:
            continue
        t = r['t']
        if t-last > 45.:
            groups.append(dict(first_s=t, last_s=t, counts=Counter(), distances=[]))
        last = t
        group = groups[-1]
        group['last_s'] = t
        counts = group['counts']
        counts['decoded'] += 1
        group['distances'].append(true_ranges[r['frame']])
        origin, direction = np.array(r['origin']), np.array(r['direction'])
        point = model.observe(ident, t, origin, direction, r['pixels'])
        current = model.window(model.history[ident], t, origin)
        if not current:
            continue
        counts['three_recent'] += 1
        prior = [w for w in model.views.get(ident, []) if 1 <= t-w[-1].stamp <= 60
                 and np.linalg.norm(w[-1].origin-origin) >= .35]
        counts['another_position'] += bool(prior)
        prior = [w for w in prior if 12 <= math.degrees(math.acos(
                 np.clip(w[1].direction @ current[1].direction, -1., 1.))) <= 145]
        counts['parallax'] += bool(prior)
        depth_ok = False
        for w in prior:
            pair = w + current
            rays, origins = np.array([s.direction for s in pair]), np.array([s.origin for s in pair])
            p = np.eye(3) - rays[:,:,None]*rays[:,None,:]
            xy = np.linalg.solve(p.sum(axis=0), (p @ origins[:,:,None]).sum(axis=0)).ravel()
            depths = ((xy-origins)*rays).sum(axis=1)
            if depths.min() >= .3 and depths.max() <= 4.:
                depth_ok = True
                break
        counts['within_sensor_depth'] += depth_ok
        if point is not None:
            counts['valid_intersection'] += 1
            group.setdefault('first_valid_s', t)
    for group in groups:
        values = group.pop('distances')
        group['gazebo_camera_distance_m'] = [min(values), max(values)]
    return groups


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument('cache', type=Path)
    cli.add_argument('run', type=Path)
    cli.add_argument('calibration', type=Path)
    cli.add_argument('id', type=int)
    cli.add_argument('output', type=Path)
    args = cli.parse_args()
    cache = json.loads(args.cache.read_text())
    if args.run.name != cache['run']:
        raise ValueError('Mismatched recording')
    target = next(t for t in json.loads((args.run/'scene.json').read_text())['targets'] if t['id']==args.id)
    poses = np.genfromtxt(args.run/'headings.csv', names=True, delimiter=',')
    xyz = np.array([target[k] for k in ('x','y','z')])
    offset = np.array(json.loads(args.calibration.read_text())['translation'])
    def camera_distance(t):
        stamps = poses['elapsed_sim_s']
        x,y,yaw = [np.interp(t,stamps,r) for r in (poses['x'],poses['y'],angle)]
        c,s = math.cos(yaw),math.sin(yaw)
        return float(np.linalg.norm(xyz-[x+c*offset[0]-s*offset[1], y+s*offset[0]+c*offset[1], offset[2]]))
    angle = np.unwrap(poses['yaw_rad'])
    ranges = {r['frame']:camera_distance(r['t']) for r in cache['rows'] if r['id']==args.id}
    out = {str(gate):campaign_windows(cache['rows'], args.id, gate, ranges) for gate in (10.,18.)}
    args.output.write_text(json.dumps(dict(run=cache['run'], id=args.id, pixel_gate=out), indent=2)+'\n')


if __name__ == '__main__':
    main()
