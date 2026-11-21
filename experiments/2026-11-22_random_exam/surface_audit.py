"""Post-run camera opportunities on mountable wall and object surfaces in the exam SDF."""
from collections import Counter
import math
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation

from geometry import polygon, wall_segments

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / '2026-11-22_v151_adaptive_survey/development'))
from exposure_opportunity import blockers, occluded


def faces(scene):
    for name, start, end in wall_segments(scene):
        offset = .002 if name.startswith('structure_') else .075
        yield 'wall', name, 'wall', start, end, offset
    for body in scene['obstacles']:
        # Clockwise around the solid puts free space on the left, as for the room segments.
        corners = polygon(body)[::-1]
        for index, start in enumerate(corners):
            name = body['name'] + '/' + str(index)
            yield body['object_id'], name, body['shape'], start, corners[(index + 1) % 4], 0.


def camera_poses(track, exposure_times, calibration):
    if len(track) < 2:
        raise ValueError('No matched pose and camera timestamps')
    times = np.unique(exposure_times)
    track = track[np.unique(track[:, 0], return_index=True)[1]]
    times = times[(times >= max(0., track[0, 0])) & (times <= min(600., track[-1, 0]))]
    if not len(times):
        raise ValueError('No matched pose and camera timestamps')
    j = np.clip(np.searchsorted(track[:, 0], times), 1, len(track) - 1)
    times = times[(track[j, 0] - track[j - 1, 0]) <= .5]
    if not len(times):
        raise ValueError('No exposures with an adjacent pose sample')
    x, y = (np.interp(times, track[:, 0], track[:, i]) for i in (1, 2))
    yaw = np.interp(times, track[:, 0], np.unwrap(track[:, 3]))
    base = Rotation.from_euler('z', yaw)
    axes = (base * Rotation.from_quat(calibration['quaternion'])).as_matrix()
    origins = base.apply(calibration['translation']) + np.column_stack((x, y, np.zeros_like(x)))
    return times, origins, axes


def mountable(center, tangent, size, normal, solids):
    # Test the print width just outside the support, including an L's internal seam.
    edge = .65 * size * tangent
    points = center[None, :] + .012 * normal + np.array([-edge, np.zeros(3), edge])
    for solid in solids:
        if np.any(np.all(np.abs((points - solid.center) @ solid.axes) < solid.half - 1.e-6, axis=1)):
            return False
    return True


def opportunity(center, normal, tangent, size, min_pixels, camera, calibration, solids):
    times, origins, axes = camera
    delta = origins - center
    toward = delta @ normal >= .5 * np.linalg.norm(delta, axis=1)
    selected = np.flatnonzero(toward)
    vertical = np.array([0., 0., size / 2])
    side = tangent * size / 2
    corners = center + np.array([-side-vertical, side-vertical, side+vertical, -side+vertical])
    k = calibration['k']
    relative = corners - origins[selected, None]
    optical = np.einsum('nki,nij->nkj', relative, axes[selected])
    depth = optical[:, :, 2]
    uv = optical[:, :, :2] / np.maximum(.1, depth[:, :, None])
    uv = uv * [k[0], k[4]] + [k[2], k[5]]
    lengths = np.linalg.norm(uv - np.roll(uv, 1, axis=1), axis=2)
    good = ((depth > .1).all(axis=1) & (lengths.min(axis=1) >= min_pixels)
            & (uv >= 4).all(axis=(1, 2))
            & (uv <= [calibration['width']-5, calibration['height']-5]).all(axis=(1, 2)))
    selected = selected[good]
    if not len(selected):
        return 'outside_readable_view'
    for point in (*corners, center):
        selected = selected[~occluded(origins[selected], point, solids)]
        if not len(selected):
            return 'line_of_sight_blocked'
    stamps = times[selected]
    # Three distinct useful views from the same pass. Do not combine isolated glimpses minutes apart.
    for first in np.flatnonzero(np.searchsorted(stamps, stamps + 1., side='right') >= np.arange(len(stamps)) + 3):
        middle = np.searchsorted(stamps, stamps[first] + .15 - 1.e-8)
        if middle < len(stamps):
            last = np.searchsorted(stamps, stamps[middle] + .15 - 1.e-8)
            if last < len(stamps) and stamps[last] <= stamps[first] + 1.:
                return 'observed'
    return 'insufficient_repeated_exposures'


def evaluate(scene, world, track, exposure_times, calibration, size=.20, height=.32, min_pixels=18.):
    camera = camera_poses(track, exposure_times, calibration)
    camera_z = camera[1][0, 2]
    solids = blockers(world, min(camera_z, height-size/2), max(camera_z, height+size/2))
    groups = {}
    totals = {'wall': Counter(), 'obstacle': Counter()}
    for host, name, shape, a, b, offset in faces(scene):
        length = float(np.linalg.norm(b - a))
        tan = np.r_[(b - a) / length, 0.]
        normal = np.array([-tan[1], tan[0], 0.])
        available = length - 1.3 * size
        result = Counter()
        if available > 0:
            count = math.ceil(available / .4)
            for i in range(count):
                along = .65 * size + available * (i + .5) / count
                center = np.r_[a, height] + tan * along + (offset + .004) * normal
                if not mountable(center, tan, size, normal, solids):
                    result['solid_at_mount'] += 1
                    continue
                outcome = opportunity(center, normal, tan, size, min_pixels, camera, calibration, solids)
                result[outcome] += 1
                totals['wall' if shape == 'wall' else 'obstacle'][outcome] += available / count
        samples = sum(result.values()) - result['solid_at_mount']
        fraction = result['observed'] / samples if samples else None
        group = groups.setdefault(host, dict(shape=shape, faces={}))
        group['faces'][name] = dict(fraction=fraction, sampled_centers=samples, counts=dict(result))
    boxes = [g for g in groups.values() if g['shape'] == 'box']
    complete = sum(all(f['fraction'] is not None and f['fraction'] >= .70 for f in g['faces'].values())
                   and len(g['faces']) == 4 for g in boxes)
    def summarize(values):
        length = sum(values.values())
        return dict(fraction=values['observed'] / length if length else None,
                    mountable_center_span_m=length, observed_span_m=values['observed'])
    return dict(exposures=len(camera[0]), wall=summarize(totals['wall']),
                obstacle=summarize(totals['obstacle']), boxes=len(boxes), boxes_all_four=complete,
                groups=groups, parameters=dict(print_m=size, height_m=height, minimum_pixels=min_pixels,
                    sample_interval_max_m=.4, maximum_incidence_deg=60, minimum_views=3,
                    window_s=1., face_completion_fraction=.70))
