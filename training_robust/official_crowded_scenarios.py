"""Evaluator-only concave worlds with space for the official dock and undocking."""
import copy
import math

import numpy as np
from scipy.ndimage import distance_transform_edt

from crowded_challenge_scenarios import make_scene, body_accessibility
from osp_mission.space_geometry import rasterize_space


def dock_and_reverse_check(config, resolution=.025):
    """Check static dock clearance and 55 cm of reverse travel against the scene.

    Dock geometry follows installed Create 3 standard_dock.urdf.xacro: box
    0.04 x 0.13 m at x=-0.045 m, spawned 0.157 m ahead and rotated pi.
    The robot starts touching its dock intentionally; only scene collisions are
    rejected here. The dock is added to connectivity checks after undocking.
    """
    occupied = rasterize_space(config, resolution)
    clearance = distance_transform_edt(np.pad(~occupied, 1, constant_values=False))[1:-1, 1:-1] * resolution
    x, y, yaw = config['start']
    direction = np.array([math.cos(yaw), math.sin(yaw)])
    side = np.array([-direction[1], direction[0]])
    samples = np.asarray([x, y]) - np.linspace(0., .55, 56)[:, None] * direction
    cells = np.floor(samples / resolution).astype(int)
    if (cells < 0).any() or (cells[:, 0] >= occupied.shape[1]).any() or (cells[:, 1] >= occupied.shape[0]).any():
        return None
    minimum = .189 + resolution / math.sqrt(2) + .01
    observed = clearance[cells[:, 1], cells[:, 0]]
    if observed.min() < minimum:
        return None
    center = np.array([x, y]) + .202 * direction
    u, v = np.meshgrid(np.linspace(-.05, .05, 9), np.linspace(-.095, .095, 15))
    dock_points = center + u.reshape(-1, 1)*direction + v.reshape(-1, 1)*side
    dock_cells = np.floor(dock_points/resolution).astype(int)
    if (dock_cells < 0).any() or (dock_cells[:, 0] >= occupied.shape[1]).any() or (dock_cells[:, 1] >= occupied.shape[0]).any():
        return None
    if occupied[dock_cells[:, 1], dock_cells[:, 0]].any():
        return None
    after = copy.deepcopy(config)
    after['start'] = [*samples[-1].tolist(), yaw]
    after['obstacles'].append(dict(type='box', kind='official_dock_check_only', x=float(center[0]),
        y=float(center[1]), sx=.04, sy=.13, yaw=yaw, height=.1))
    connectivity = body_accessibility(after, resolution)
    if not connectivity:
        return None
    return dict(resolution_m=resolution, reverse_distance_m=.55,
        minimum_reverse_clearance_m=float(observed.min()), required_clearance_m=minimum,
        dock_scene_clear=True, post_undock_marker_approaches_reachable=True,
        evidence='offline geometry only; no native success or firmware equivalence')


def make_official_scene(seed, count=12, profile='hidden_obstacles', shape='zigzag'):
    for attempt in range(20):
        config = make_scene(seed + attempt*170003, count, profile, shape)
        # The original narrow rails retain the departure heading. Orient the
        # official dock toward the rear cap so reverse undocking exits forward.
        exit_yaw = config['obstacles'][-1]['yaw']
        rng = np.random.default_rng(seed + 3101 + attempt)
        yaw = exit_yaw + math.pi + float(rng.uniform(-.10, .10))
        config['start'][2] = (yaw + math.pi) % (2*math.pi) - math.pi
        checks = [dock_and_reverse_check(config, r) for r in (.05, .025)]
        if not all(checks):
            continue
        config['seed'] = seed
        config['task'] = 'official_tb4_crowded_concave_hidden_search_v1'
        config['official_start'] = dict(version=1, dock_spawned_by_upstream=True,
            exit_heading_rad=exit_yaw, generator_seed=seed + attempt*170003,
            checks=checks, policy_receives_truth=False,
            heading_scope='Narrow docked starts within 0.1 rad of reverse exit; not arbitrary undocked headings')
        return config
    raise RuntimeError(f'No dock-compatible crowded world for {seed}, {shape}')
