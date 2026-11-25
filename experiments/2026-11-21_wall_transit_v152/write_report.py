"""Write a limited, source-grounded report of the completed development pair."""
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
data = json.loads((HERE/'result.json').read_text())
assert data['complete'] and len(data['comparisons']) == 1
pair = data['comparisons'][0]
old, new = pair['baseline'], pair['candidate']
looks = new['wall_looks']
wall = str(new['wall_marker_ids'][0])
summary = dict(version='1.5.2-wall-transit-r2', development_seed=pair['seed'],
    completed_native_runs=1, held_out_runs=0, baseline_strict_success=old['strict_saved_map_success'],
    candidate_strict_success=new['strict_saved_map_success'], wall_look_requests=looks['requests'],
    completed_fresh_frame_wall_looks=looks['completed_fresh_frame_dwells'],
    wall_registered_before=wall in {str(i) for i in old['registered_ids']},
    wall_registered_after=wall in {str(i) for i in new['registered_ids']},
    wall_near_rgb_verified_before=wall in {str(i) for i in old['sensor_verified_ids']},
    wall_near_rgb_verified_after=wall in {str(i) for i in new['sensor_verified_ids']},
    wall_closest_before_m=old['independent_closest_m'][wall],
    wall_closest_after_m=new['independent_closest_m'][wall],
    time_before_sim_s=old['elapsed_sim_s'], time_after_sim_s=new['elapsed_sim_s'],
    path_before_m=old['path_length_m'], path_after_m=new['path_length_m'],
    candidate_bumps=new['bumper_events'], source_tests_passed=29, recorder_test_passed=1,
    native_validation_complete=True, production_promoted=False, physical_drive=False)
(HERE/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
fmt = lambda x: f'{int(x)//60}:{x%60:04.1f}'
def error_label(row):
    error = max(row['map_errors_saved_m'].values(), default=None)
    return '등록 없음' if error is None else f'{100*error:.1f}cm'
lines = ['# v1.5.2 r2 · 이동 중 벽 관찰 · 동일 맵 개발 회귀 완료', '',
    f"벽 관찰 요청 **{looks['requests']}회**, 새 처리 영상3장 이상을 확인한 관찰 **{looks['completed_fresh_frame_dwells']}회**, "
    f"원래 이동 재개 예약 **{looks['requested_navigation_resumptions']}회**를 기록했다. "
    f"이번 한 회차의 엄격 결과는 **{'통과' if new['strict_saved_map_success'] else '미달'}**이며 범퍼는 **{new['bumper_events']}회**다.", '',
    '이미 원인을 분석한 seed112601 한 곳의 개발 회귀이다. 새 정책의 일반 완주율이나 미사용 맵 성능을 입증하지 않는다. '
    'v1.5.1·실물 기본값은 보존하며 새 후보는 명시적으로 선택한다.', '',
    '| 항목 | 동결 v1.5.1 | v1.5.2 r2 |', '|---|---:|---:|']
rows = [
    ('엄격 완주', old['strict_saved_map_success'], new['strict_saved_map_success']),
    ('등록 ID', old['registered_ids'], new['registered_ids']),
    ('근거리 RGB 확인 ID', old['sensor_verified_ids'], new['sensor_verified_ids']),
    ('벽 마커 실제 최소거리', f"{old['independent_closest_m'][wall]:.3f}m", f"{new['independent_closest_m'][wall]:.3f}m"),
    ('시뮬 시간', fmt(old['elapsed_sim_s']), fmt(new['elapsed_sim_s'])),
    ('실제 경과', fmt(old['elapsed_wall_s']), fmt(new['elapsed_wall_s'])),
    ('독립 실제 이동', f"{old['path_length_m']:.1f}m", f"{new['path_length_m']:.1f}m"),
    ('범퍼', old['bumper_events'], new['bumper_events']),
    ('정지 시간', f"{old['motion']['stopped_s']:.1f}s", f"{new['motion']['stopped_s']:.1f}s"),
    ('제자리 회전', f"{old['motion']['rotation_only_s']:.1f}s", f"{new['motion']['rotation_only_s']:.1f}s"),
    ('저장한 ID의 지도오차 최대', error_label(old), error_label(new))]
lines += [f'| {name} | {a} | {b} |' for name, a, b in rows]
lines += ['', '미등록 ID를 지도오차0으로 인정하지 않는다. 엄격 완주는 세 ID와 등록 후 각각 실제1.5m 접근, '
    '기존 세 장1.3m 영상 증거, 고정 초기 정렬의 live·저장 지도오차0.25m, 범퍼0을 모두 요구한다.', '',
    '![동일 맵 독립 경로와 벽 거리](comparison.png)', '',
    '벽 관찰을 요청한 실제 로봇 위치를 주황 점으로 표시했다. 책상은 상판 도식이며 실제 다리 충돌 형상과 다르다. '
    '바뀐 센서·제어 경로를 똑같은 궤적으로 재현한 실험은 아니다. 시뮬 시간으로 임무 비용을 비교하고 실제 경과는 VM 부하와 구분한다.', '',
    '## 실제 벽 관찰과 복귀', '',
    f"관찰에 직접 쓴 시간은 {looks['actual_observation_s']:.1f}초, 복귀 방향 전환까지 포함해 차감한 예산은 {looks['budget_charged_s']:.1f}초였다. "
    f"미션의 실제 navigation 재개 로그는 {looks['mission_navigation_resume_log_count']}건이다. "
    '정지·회전 시간 차이 전체가 이 기능 때문이라고 해석하지 않는다.', '',
    '두 회차 모두600초에 미완주했다. 거리 감소와 정지·회전 증가가 함께 나타났으므로 이동거리가 짧아졌다는 것만으로 탐색 효율이 개선됐다고 판단하지 않는다.', '',
    '| 미션 경과 | 회전 요청 | 새 처리 프레임 | 종료 이유 | 복귀 예약 | 이후10초 실제 이동 |',
    '|---:|---:|---:|---|---|---:|']
for l in looks['history']:
    frames = l['processed_frames'] if l['processed_frames'] is not None else '별도 기록 없음'
    lines.append(f"| {l['elapsed_mission_s']:.1f}s | {l['choice']['turn_rad']*180/3.141592653589793:.0f}° | {frames} | {l.get('reason','미완료')} | {l.get('resume_requested',False)} | {l.get('independent_motion_next_10_sim_s_m',0):.2f}m |")
lines += ['', '복귀 예약과 실제 이동은 별도 증거다. 새로운 마커 정보가 생기면 이전 목표를 재개하지 않고 정상 마커 계획으로 돌아간다. '
    '계획한 면의 길이를 실제 카메라 관찰 완료량으로 계산하지 않는다. ID·근거리 증거·기존 coverage는 실제 센서 경로에서만 갱신한다.', '']
preview = new.get('wall_look_preview')
if preview:
    lines += [f"![벽 관찰 중 저장 JPEG]({preview['file']})", '',
              f"성공한 첫 관찰의 저장 RGB JPEG, 미션 {preview['elapsed_sim_s']:.3f}초·프레임{preview['frame']}. 원시 입력과 동일하다는 뜻은 아니다.", '']
lines += ['## 적용과 검증 범위', '',
    '이동 중 미관찰 벽 유사 면을 짧게 확인하고 복귀하는 기능, 마지막 벽 시간 예산의 조기 반환 우회 수정을 별도 버전에 적용했다. '
    '한 번의 비교에서는 두 변경의 효과를 각각 분리할 수 없다. 작은 후보 검출 모델 자체는 바꾸지 않았다.', '',
    f"첫 벽 면 목표 선택은 {new['first_wall_surface_selection_s']}초, 마지막 예산 사유의 첫 선택은 {new['first_reserved_wall_selection_s']}초였다. "
    '예산 검사는 목표 재선택 시점에 적용된다. 진행 중인 긴 내부 이동은 벽 전환을 늦출 수 있으므로 마지막90초 전체를 벽 관찰에 보장했다고 해석하지 않는다.', '',
    '실제 조회한 MPPI·smoother·footprint·inflation·costmap 주기·18px/3프레임 판독 설정은 기준판과 같았다. '
    '480×480·5FPS를 유지한다. world SHA256·배치·예정 spawn을 그대로 사용하고 정답 위치는 평가기에만 제공했다. '
    '확인된 마커 접근·안전 복구를 보호하며 관찰 실패를 인식 성공으로 바꾸지 않는다.', '',
    '새 코드20개·기존 동작 제어9개,29개 검사와 빌드를 통과했다. 기록기의8개 동시 작성자 검사도 통과했다. '
    '옛 NearPassInspection 기하 검사3개는 원본 v1.5.1에서도 동일하게 실패했고 원본을 바꿔 숨기지 않았다. '
    '새 선택기는 해당 옛 choose를 호출하지 않는다.', '',
    '개발 r1은 일반 면 예약까지 보호하는 버그를 발견해88.6초에 중단·보존했다. 첫 r2는 약180초의 기록기 '
    'latest.tmp 동시 저장 충돌로 미완료됐다. 각각 [개발 중단](development_aborted_r1.json)·[인프라 실패](infrastructure_failure_first_r2.json)로 분리한다. '
    '완료 결과에 합산하지 않는다. 기록기를 고친 뒤 동일 r2 소스 해시로 이번 회차만 다시 실행했으며 성공할 때까지 정책 실패를 재시도하지 않았다. '
    '관측기의 원자적 저장 수정은 ROS 목표·센서·제어 파라미터를 바꾸지 않는다.', '',
    '## 남은 한계', '',
    ('이 한 곳에서는 완주가 개선됐지만 다른 네 배치와 실물에서는 아직 검증하지 않았다.' if new['strict_saved_map_success'] else
     '벽 관찰 기능은 발동했지만 이번 회차의 전체 완주 향상은 확인하지 못했다. 결과를 기본 정책 승격의 근거로 사용하지 않는다.'), '',
    f"벽{wall}의 실제 최소거리는 {new['independent_closest_m'][wall]:.3f}m였고 판독기 ID 기록은 "
    f"{'있었다' if wall in new['detector'].get('by_id',{}) else '없었다'}. "
    '벽을 향한 관찰 목표가 생겨도 남은 시간에 미관찰 벽 구간까지 이동하지 못하면 탐색을 완료할 수 없다. '
    '이 실행에서는 이동 중 벽 정보 수집과 남은 벽 구간을 고르는 경로 선택을 구분해야 한다.', '',
    f"후보 취소 사유 {new['cancellations']}, 스케줄 단계 {new['scheduler_phases']}, 마지막 벽 예산 우회 기록 {len(new['late_fresh_interior_wall_budget_bypass'])}건. "
    '마지막 값0은 이 실행에서 해당 선택이 기록되지 않았다는 뜻이며, 모든 마커를 볼 수 있었다는 증명은 아니다.', '',
    '[정책 설명·재현](../../policy_candidates/2026-11-21_mixed_wall_transit_v152/README.md) · [상세 비교](result.json) · '
    '[집계](summary.json) · [실제 설정](runtime_parameters.json) · [관측기 수정](observer_revision.json) · '
    '[주행 영상](videos.md) · [보드](http://192.168.252.2:8780/)', '']
(HERE/'README.md').write_text('\n'.join(lines))
print(json.dumps(summary, indent=2))
