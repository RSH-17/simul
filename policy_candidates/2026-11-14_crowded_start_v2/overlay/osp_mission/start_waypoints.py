"""Short observed-free transit goals when a narrow start has no normal exit yet."""
import math

import numpy as np
from scipy.ndimage import distance_transform_edt, label
from scipy.sparse.csgraph import dijkstra

from osp_mission.frontier import reachable_gain
from osp_mission.policy import graph, wrap


def options(data, resolution, origin, robot, heading=0., return_paths=False):
    if not np.isfinite([resolution, *origin, *robot, heading]).all() or resolution <= 0:
        return [], None
    x, y = np.floor((np.asarray(robot) - origin) / resolution).astype(int)
    if not (0 <= y < data.shape[0] and 0 <= x < data.shape[1]) or data[y, x] != 0:
        return [], None
    free = data == 0
    ordinary = free & (distance_transform_edt(data <= 50) * resolution >= .32)
    regions, _ = label(ordinary)
    if regions[y, x] and np.count_nonzero(regions == regions[y, x]) * resolution**2 >= .5:
        return [], None
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
    gain = reachable_gain(data, resolution, free_regions == free_regions[y, x])
    # Aim toward an informative observed endpoint, then take only the first
    # 0.45--0.70 m of its body-clear path. The frontier may be farther away.
    valid = body & np.isfinite(distance) & (distance >= .45) & (gain > .015)
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
            observed_frontier_cell=endpoint, distance=float(distance[cell]),
            features=np.array([0., 5 * gain[endpoint], distance[cell], abs(wrap(yaw-heading)), 0.]))
        if cell not in choices or candidate['features'][1] > choices[cell]['features'][1]:
            choices[cell] = candidate
    return list(choices.values()), previous if return_paths else None
