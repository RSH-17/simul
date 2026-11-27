"""Small, ROS-independent calculations for the physical robot board."""
import math


def mission_display(status):
    state = status.get('state', '')
    reason = status.get('termination_reason', '')
    labels = {'idle':'시작 대기', 'running':'탐색 · 주행 중'}
    reasons = {'operator_stop':'사용자·진단·안전 감시의 중지 요청', 'timeout':'제한 시간 도달',
               'all_targets_found':'마커 3개 확인 및 접근 완료', 'bump_contact':'범퍼 접촉으로 중지'}
    if state in ('saving', 'done'):
        success = status.get('task_success') is True and reason == 'all_targets_found'
        label = ('마커 임무 완료' if success else
                 '범퍼 감지 후 중지' if reason == 'operator_stop' and (status.get('collisions') or 0)>0 else
                 '주행 중지' if reason == 'operator_stop'
                 else '시간 초과' if reason == 'timeout' else '종료 · 미완료')
        if state == 'saving':
            label += ' · 저장 중'
        return {'label':label, 'detail':reasons.get(reason, reason or '종료 사유 미수신 · 성공 판정 없음'),
                'kind':'good' if success else 'warn'}
    return {'label':labels.get(state, state or '상태 대기'), 'detail':'', 'kind':''}


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
