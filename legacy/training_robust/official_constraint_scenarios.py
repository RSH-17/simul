"""Evaluator-only official TB4 scenario matrix, with explicit feasibility bounds.

This changes task worlds, never official robot assets or policy inputs. Existing
campaigns and their frozen generators are preserved. Sensor faults are NOT
injected by this generator and must not be reported as tested conditions.
"""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from scipy.ndimage import distance_transform_edt
import yaml

from official_crowded_scenarios import dock_and_reverse_check
from osp_mission.challenge_scenarios import make_challenge_scene
from osp_mission.space_geometry import rasterize_space

PROFILES = {
    'narrow': dict(width=.50, length=2.6, yaw_degrees=0., obstacles=9, marker_size=(.12, .24)),
    'angled_start': dict(width=.65, length=2.6, yaw_degrees=12., obstacles=12, marker_size=(.10, .22)),
    'long_pocket': dict(width=.55, length=4.2, yaw_degrees=3., obstacles=16, marker_size=(.10, .20)),
    'dense': dict(width=.80, length=2.6, yaw_degrees=20., obstacles=24, marker_size=(.10, .18)),
    'sideways_small': dict(width=1.60, length=2.4, yaw_degrees=75., obstacles=12, marker_size=(.08, .12)),
    'combined': dict(width=.55, length=3.2, yaw_degrees=6., obstacles=20, marker_size=(.08, .12)),
}
SHAPES = ('c_shape', 'u_shape', 'zigzag')
RUBRIC = dict(correct_ids=3, approach_radius_m=1.5, maximum_marker_error_m=.5,
              maximum_contacts=0, deadline_sim_seconds=600.)


def physical_digest(config):
    physical = {k: config[k] for k in ('arena', 'obstacles', 'targets', 'start')}
    return hashlib.sha256(json.dumps(physical, sort_keys=True).encode()).hexdigest()


def clear_rails(config, rails, resolution=.025):
    """Reject rail overlaps, including outside the arena and existing carriers."""
    base = copy.deepcopy(config)
    base['obstacles'] = config['obstacles'][:-3]
    occupied = rasterize_space(base, resolution)
    clearance = distance_transform_edt(np.pad(~occupied, 1, constant_values=False))[1:-1, 1:-1] * resolution
    yy, xx = np.indices(occupied.shape)
    for rail in rails:
        dx = (xx + .5) * resolution - rail['x']
        dy = (yy + .5) * resolution - rail['y']
        c, s = math.cos(rail['yaw']), math.sin(rail['yaw'])
        mask = ((abs(c*dx+s*dy) <= rail['sx']/2 + .05) &
                (abs(-s*dx+c*dy) <= rail['sy']/2 + .05))
        if not mask.any() or clearance[mask].min() < .15:
            return False
        # Raster masks clip at image boundaries; explicitly check all corners.
        for u in (-rail['sx']/2-.05, rail['sx']/2+.05):
            for v in (-rail['sy']/2-.05, rail['sy']/2+.05):
                x, y = rail['x']+c*u-s*v, rail['y']+s*u+c*v
                if not (0 <= x < occupied.shape[1]*resolution and 0 <= y < occupied.shape[0]*resolution):
                    return False
    return True


def make_constraint_scene(seed, shape, profile):
    p = PROFILES[profile]
    if shape not in SHAPES:
        raise ValueError('Unknown arena shape')
    for attempt in range(40):
        generator_seed = seed + attempt*190001
        config = make_challenge_scene(generator_seed, p['obstacles']-3, shape=shape)
        exit_yaw = config['start'][2]
        start = np.asarray(config['start'][:2])
        forward = np.array([math.cos(exit_yaw), math.sin(exit_yaw)])
        side = np.array([-forward[1], forward[0]])
        width, length, thickness = p['width'], p['length'], .18

        def box(center, sx, sy):
            return dict(type='box', kind='partition', x=float(center[0]), y=float(center[1]),
                        sx=sx, sy=sy, yaw=exit_yaw, height=.9)

        rails = [box(start+forward*((length-.55)/2)+sign*side*((width+thickness)/2),
                     length+.55, thickness) for sign in (-1, 1)]
        rails.append(box(start-forward*.65, thickness, width+2*thickness))
        config['obstacles'].extend(rails)
        if not clear_rails(config, rails):
            continue
        # Both signs occur across independently seeded worlds; no policy hint.
        rng = np.random.default_rng(seed)
        offset = math.radians(p['yaw_degrees']) * int(rng.choice([-1, 1]))
        config['start'][2] = (exit_yaw+math.pi+offset+math.pi) % (2*math.pi)-math.pi
        for target in config['targets']:
            target['size'] = float(rng.uniform(*p['marker_size']))
        checks = [dock_and_reverse_check(config, r) for r in (.025, .02)]
        if not all(checks):
            continue
        config.update(seed=seed, task='official_tb4_constraint_search_v1',
                      actual_obstacles=p['obstacles'], difficulty=p['obstacles'])
        config['crowded_start'] = dict(version=2, requested_obstacles=p['obstacles'],
            actual_obstacles=p['obstacles'], corridor_width_m=width, exit_length_m=length,
            geometry_checks=checks, policy_receives_ground_truth=False)
        config['official_start'] = dict(version=2, generator_seed=generator_seed, checks=checks,
            exit_heading_rad=exit_yaw, dock_spawned_by_upstream=True, policy_receives_truth=False,
            heading_scope='Profile-specific docked heading; independent reverse/approach geometry checks')
        config['constraint_profile'] = dict(name=profile, corridor_width_m=width, exit_length_m=length,
            dock_yaw_offset_degrees=math.degrees(offset), requested_obstacles=p['obstacles'],
            marker_size_range_m=list(p['marker_size']), marker_occlusion='obstacle back faces hidden from start',
            sensor_faults_injected=False, official_robot_files_modified=False,
            geometry_verified=True, native_validated=False, hardware_validated=False,
            policy_receives_truth=False)
        return config
    raise RuntimeError(f'No feasible {shape}/{profile} scene for seed {seed}; bounds were not relaxed')


def prepare(destination, seed=1205000000, holdout_repeats=1):
    if not 1 <= holdout_repeats <= 10:
        raise ValueError('Use 1..10 independent holdout worlds per condition')
    destination.mkdir(parents=True, exist_ok=False)
    rows, seen = [], set()
    for split, repeats in [('train', 1), ('holdout', holdout_repeats)]:
        (destination/split).mkdir()
        for si, shape in enumerate(SHAPES):
            for pi, profile in enumerate(PROFILES):
                for repeat in range(repeats):
                    scene_seed = seed + (1000000 if split == 'holdout' else 0) + si*10000 + pi*101 + repeat*10000000
                    config = make_constraint_scene(scene_seed, shape, profile)
                    fingerprint = physical_digest(config)
                    if fingerprint in seen:
                        raise ValueError('Duplicate geometry across matrix splits')
                    seen.add(fingerprint)
                    path = destination/split/f'{scene_seed}.yaml'
                    path.write_text(yaml.safe_dump(config, sort_keys=False))
                    rows.append(dict(split=split, shape=shape, profile=profile, seed=scene_seed,
                        file=str(path.resolve()), sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                        physical_sha256=fingerprint, obstacles=len(config['obstacles']),
                        constraints=config['constraint_profile']))
                    print(f'{split} {shape}/{profile}: {scene_seed}', flush=True)
    result = dict(evidence_type='Offline world geometry preparation only', rubric=RUBRIC,
        episodes=rows, train_scenes=18, holdout_scenes=18*holdout_repeats,
        holdout_used_for_training=False, native_started=False, production_promoted=False,
        pending_conditions=['timestamp-preserving sensor delay/dropout', 'wheel slip and odometry bias',
                            'dynamic obstacles', 'lighting and partial marker occlusion'],
        limits='Static flat indoor worlds; sensor faults are not covered by these prepared scenes')
    (destination/'manifest.json').write_text(json.dumps(result, indent=2)+'\n')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seed', type=int, default=1205000000)
    parser.add_argument('--holdout-repeats', type=int, default=1)
    args = parser.parse_args()
    prepare(args.output.resolve(), args.seed, args.holdout_repeats)
