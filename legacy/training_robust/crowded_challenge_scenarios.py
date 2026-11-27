"""Evaluator-only crowded starts for concave maps; never imported by a policy."""
import copy
import math
from pathlib import Path
import sys

import numpy as np
from scipy.ndimage import distance_transform_edt, label

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/osp_mission'))
from osp_mission.challenge_scenarios import make_challenge_scene
from osp_mission.space_geometry import rasterize_space


def body_accessibility(config, resolution=.05):
    """Check the real body exit independently of the policy's 0.32 m margin."""
    occupied = rasterize_space(config, resolution)
    clear = distance_transform_edt(np.pad(~occupied, 1, constant_values=False))[1:-1, 1:-1] * resolution
    minimum = .189 + resolution / math.sqrt(2) + .01
    groups, _ = label(clear >= minimum)
    x, y = np.floor(np.asarray(config['start'][:2]) / resolution).astype(int)
    if not (0 <= y < groups.shape[0] and 0 <= x < groups.shape[1]) or not groups[y, x]:
        return None
    reachable = groups == groups[y, x]
    wide = clear >= .34
    if (reachable & wide).sum() < max(45 / resolution**2, .97 * wide.sum()):
        return None
    for target in config['targets']:
        approach = False
        for distance in (.85, 1.1, 1.35):
            samples = np.linspace(.20, distance, 40)
            xs = np.floor((target['x'] + samples * math.cos(target['yaw'])) / resolution).astype(int)
            ys = np.floor((target['y'] + samples * math.sin(target['yaw'])) / resolution).astype(int)
            inside = (xs >= 0) & (ys >= 0) & (xs < occupied.shape[1]) & (ys < occupied.shape[0])
            if inside.all() and reachable[ys[-1], xs[-1]] and not occupied[ys, xs].any():
                approach = True
                break
        if not approach:
            return None
    return dict(start_clearance_m=float(clear[y, x]), minimum_body_clearance_m=minimum,
                resolution_m=resolution, all_marker_approaches_reachable=True,
                evidence='offline geometry only; no native or hardware success claim')


def make_scene(seed, count=12, profile='hidden_obstacles', shape='zigzag'):
    """Add a long narrow departure pocket without clearing occupied map cells."""
    rng = np.random.default_rng(seed + 1701)
    # The underlying generator requires six obstacles; the lowest tier is nine.
    actual_count = max(9, count)
    for attempt in range(60):
        generator_seed = seed + attempt * 100003
        config = make_challenge_scene(generator_seed, actual_count - 3, profile, shape)
        config = copy.deepcopy(config)
        start = np.asarray(config['start'][:2])
        heading = config['start'][2]
        forward = np.array([math.cos(heading), math.sin(heading)])
        side = np.array([-forward[1], forward[0]])
        width = float(rng.uniform(.53, .61))
        length = float(rng.uniform(2., 3.4))
        thickness = .18
        rails = []

        def box(center, sx, sy):
            return dict(type='box', kind='partition', x=float(center[0]), y=float(center[1]),
                        sx=sx, sy=sy, yaw=heading, height=.9)

        for sign in (-1, 1):
            center = start + forward * ((length - .55) / 2) + sign * side * ((width + thickness) / 2)
            rails.append(box(center, length + .55, thickness))
        rails.append(box(start - forward * .65, thickness, width + 2 * thickness))
        # Reject overlaps with existing geometry before checking connectivity.
        original = rasterize_space(config, .05)
        original_clear = distance_transform_edt(np.pad(~original, 1, constant_values=False))[1:-1, 1:-1] * .05
        yy, xx = np.indices(original.shape)
        overlap = False
        for rail in rails:
            dx, dy = (xx + .5) * .05 - rail['x'], (yy + .5) * .05 - rail['y']
            u, v = forward[0] * dx + forward[1] * dy, side[0] * dx + side[1] * dy
            mask = (abs(u) <= rail['sx'] / 2 + .05) & (abs(v) <= rail['sy'] / 2 + .05)
            if not mask.any() or original_clear[mask].min() < .15:
                overlap = True
                break
        if overlap:
            continue
        config['obstacles'].extend(rails)
        checks = [body_accessibility(config, res) for res in (.05, .025)]
        if not all(checks):
            continue
        if any((start[0] - t['x']) * math.cos(t['yaw']) + (start[1] - t['y']) * math.sin(t['yaw']) >= -.2
               for t in config['targets']):
            continue
        config.update(seed=seed, task='crowded_concave_hidden_search_v1', difficulty=count,
                      actual_obstacles=len(config['obstacles']))
        config['start'][2] = float(rng.uniform(-math.pi, math.pi))
        config['crowded_start'] = dict(version=1, requested_obstacles=count, actual_obstacles=actual_count,
            corridor_width_m=width, exit_length_m=length, generator_seed=generator_seed,
            geometry_checks=checks, policy_receives_ground_truth=False)
        return config
    raise RuntimeError(f'Could not produce a body-clear crowded scene: {seed}, {shape}')


if __name__ == '__main__':
    import argparse
    import hashlib
    import json
    import yaml

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seed', type=int, default=1197100000)
    parser.add_argument('--per-shape', type=int, default=4)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    rows = []
    for index, shape in enumerate(('c_shape', 'u_shape', 'zigzag')):
        for variation in range(args.per_shape):
            config = make_scene(args.seed + index * 10000 + variation * 101,
                                (9, 12, 16, 20)[variation % 4], shape=shape)
            path = args.output / f"{config['seed']}.yaml"
            path.write_text(yaml.safe_dump(config, sort_keys=False))
            rows.append(dict(file=str(path.resolve()), shape=shape, seed=config['seed'],
                             sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                             geometry=config['crowded_start']))
            print(f"Prepared {shape}, {config['actual_obstacles']} obstacles", flush=True)
    (args.output / 'manifest.json').write_text(json.dumps(dict(
        evidence_type='offline geometry only', native_validated=False, episodes=rows), indent=2) + '\n')
