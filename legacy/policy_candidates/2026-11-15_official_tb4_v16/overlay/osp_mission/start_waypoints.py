"""Short body-clear steps toward observed frontiers beyond a narrow passage."""
import math

import numpy as np
from scipy.ndimage import distance_transform_edt, label
from scipy.sparse.csgraph import dijkstra

from osp_mission.frontier import reachable_gain
from osp_mission.policy import graph, wrap


def narrow_start(data, resolution, origin, robot):
    """A body-clear tight footprint does not need an initial camera sweep."""
    if not np.isfinite([resolution, *origin, *robot]).all() or resolution <= 0:
        return False
    x, y = np.floor((np.asarray(robot) - origin) / resolution).astype(int)
    if not (0 <= y < data.shape[0] and 0 <= x < data.shape[1]) or data[y, x] != 0:
        return False
    clearance = distance_transform_edt(np.pad(data == 0, 1))[1:-1, 1:-1] * resolution
    return .189 + resolution / math.sqrt(2) + .01 <= clearance[y, x] < .32


def options(data, resolution, origin, robot, heading=0., return_paths=False):
    if not np.isfinite([resolution, *origin, *robot, heading]).all() or resolution <= 0:
        return [], None
    x, y = np.floor((np.asarray(robot) - origin) / resolution).astype(int)
    if not (0 <= y < data.shape[0] and 0 <= x < data.shape[1]) or data[y, x] != 0:
        return [], None
    free = data == 0
    ordinary = free & (distance_transform_edt(data <= 50) * resolution >= .32)
    regions, _ = label(ordinary)
    normal_region = regions == regions[y, x] if regions[y, x] else np.zeros_like(free)
    in_normal_pocket = np.count_nonzero(normal_region) * resolution**2 >= .5
    clearance = distance_transform_edt(np.pad(free, 1, constant_values=False))[1:-1, 1:-1] * resolution
    body = clearance >= .189 + resolution / math.sqrt(2) + .01
    regions, _ = label(body)
    if not regions[y, x]:
        return [], None
    body &= regions == regions[y, x]
    distance, previous = dijkstra(graph(body, resolution), directed=False,
        indices=y * data.shape[1] + x, return_predecessors=True)
    distance = distance.reshape(data.shape)
    # Frontier evidence comes from the connected observed free component.
    # The motion graph still requires the full body margin from unknown cells.
    free_regions, _ = label(free)
    evidence = free_regions == free_regions[y, x]
    beyond_pocket = None
    if in_normal_pocket:
        beyond_pocket = distance_transform_edt(~normal_region) * resolution >= .70
        # The frontier itself must require leaving this normal component.
        # Mask before the 3 m gain filter: otherwise a reachable room's edge
        # leaks gain into its old entry corridor and sends us back through it.
        evidence &= beyond_pocket
    gain = reachable_gain(data, resolution, evidence)
    # Aim toward an informative observed endpoint, then take only the first
    # 0.45--0.70 m of its body-clear path. The frontier may be farther away.
    valid = body & np.isfinite(distance) & (distance >= .45) & (gain > .015)
    if in_normal_pocket:
        # Having room to turn here does not mean the 0.32 m navigation graph
        # reaches the corridor outside. Only continue toward frontier evidence
        # well beyond this normal-clearance component. Its immediate wall rim
        # is not an escape destination. Every connecting cell still uses the
        # existing full-body margin, including distance from unknown space.
        valid &= beyond_pocket
    buckets = {}
    for cy, cx in zip(*np.where(valid)):
        px, py = origin[0] + (cx + .5) * resolution, origin[1] + (cy + .5) * resolution
        yaw = math.atan2(py - robot[1], px - robot[0])
        bucket = int(math.floor((yaw + math.pi) / (math.pi / 8)))
        score = (float(gain[cy, cx]), float(distance[cy, cx]), float(clearance[cy, cx]))
        if bucket not in buckets or score > buckets[bucket][0]:
            buckets[bucket] = score, (int(cy), int(cx))
    choices = {}
    for _, endpoint in buckets.values():
        cell = endpoint
        while distance[cell] > .70:
            predecessor = previous[cell[0] * data.shape[1] + cell[1]]
            if predecessor < 0:
                break
            cell = divmod(int(predecessor), data.shape[1])
        if not .45 <= distance[cell] <= .70:
            continue
        py, px = origin[1] + (cell[0] + .5) * resolution, origin[0] + (cell[1] + .5) * resolution
        yaw = math.atan2(py - robot[1], px - robot[0])
        candidate = dict(x=px, y=py, yaw=yaw, cell=cell, scan_angle=0., start_escape=True,
            pocket_continuation=bool(in_normal_pocket),
            observed_frontier_cell=endpoint, distance=float(distance[cell]),
            features=np.array([0., 5 * gain[endpoint], distance[cell], abs(wrap(yaw-heading)), 0.]))
        if cell not in choices or candidate['features'][1] > choices[cell]['features'][1]:
            choices[cell] = candidate
    return list(choices.values()), previous if return_paths else None
