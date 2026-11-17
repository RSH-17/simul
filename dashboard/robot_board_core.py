"""Small, ROS-independent calculations for the physical robot board."""
import math


def map_cell(grid, resolution, origin, robot):
    """Return the occupied-map value under base_link, honoring map origin yaw."""
    if grid is None or robot is None or not math.isfinite(resolution) or resolution <= 0:
        return {'reason': 'map_or_pose_unavailable'}
    height, width = grid.shape
    ox, oy, yaw = origin
    dx, dy = robot[0] - ox, robot[1] - oy
    c, s = math.cos(yaw), math.sin(yaw)
    x = math.floor((c*dx + s*dy) / resolution)
    y = math.floor((-s*dx + c*dy) / resolution)
    if not (0 <= x < width and 0 <= y < height):
        return {'x': x, 'y': y, 'reason': 'start_outside_map'}
    value = int(grid[y, x])
    return {'x': x, 'y': y, 'value': value,
            'reason': 'start_unknown' if value < 0 else
                      'start_not_free' if value > 0 else 'start_free'}


def scan_summary(scan, filtered=None):
    """Summarize raw returns and the matching filtered frame without guessing cause."""
    if scan is None:
        return {'available': False}
    finite = [float(r) for r in scan.ranges
              if math.isfinite(r) and scan.range_min < r < scan.range_max]
    result = {'available': True, 'frame': scan.header.frame_id,
              'samples': len(scan.ranges), 'valid_returns': len(finite),
              'min_range_m': min(finite) if finite else None,
              'within_25cm_from_laser': sum(r < .25 for r in finite)}
    if filtered is not None and len(scan.ranges) == len(filtered.ranges) and scan.header.stamp == filtered.header.stamp:
        result['filtered_frame_matched'] = True
        result['removed_returns'] = sum(
            math.isfinite(raw) and not math.isfinite(clean)
            for raw, clean in zip(scan.ranges, filtered.ranges))
    else:
        result['filtered_frame_matched'] = False
        result['removed_returns'] = None
    return result
