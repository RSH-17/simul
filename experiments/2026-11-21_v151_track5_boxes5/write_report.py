"""Summarize the five completed large-track layouts without changing scores."""
import json
from pathlib import Path
import statistics

HERE=Path(__file__).resolve().parent
data=json.loads((HERE/'result.json').read_text());rows=data['runs']
assert data['complete'] and len(rows)==5 and data['boxes_per_layout']==5
success=sum(r['strict_saved_map_success'] for r in rows)
fmt=lambda s:f'{int(s)//60}:{s%60:04.1f}'
summary=dict(native_runs=5,boxes=5,successes=success,bumps=sum(r['bumper_events'] for r in rows),
    sensor_interruptions=sum(r['sensor_interruptions'] or 0 for r in rows),
    mean_sim_s=statistics.mean(r['elapsed_sim_s'] for r in rows),mean_wall_s=statistics.mean(r['elapsed_wall_s'] for r in rows),
    mean_path_m=statistics.mean(r['path_length_m'] for r in rows),policy_changed=False,physical_drive=False)
for host,total in (('box',10),('wall',5)):
    summary[host+'_marker_registered']=sum(len(set(r[host+'_marker_ids'])&set(r['registered_ids'])) for r in rows)
    summary[host+'_marker_rgb_verified']=sum(len(set(r[host+'_marker_ids'])&set(r['sensor_verified_ids'])) for r in rows)
    summary[host+'_marker_total']=total
(HERE/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
lines=['# v1.5.1 · 사진의 대형 트랙 · 상자 총 5개 · 5회 완료','',
    f"엄격 통과 **{success}/5회**, 범퍼 **{summary['bumps']}회**, 센서·준비도 일시 중단 **{summary['sensor_interruptions']}회**다. "
    '각 회차는 하나의 새 무작위 배치이며 다른 정책과의 직접 비교가 아니다.','',
    f"상자 부착 마커는 등록 {summary['box_marker_registered']}/10·근거리 영상 확인 {summary['box_marker_rgb_verified']}/10, "
    f"벽 마커는 등록 {summary['wall_marker_registered']}/5·근거리 영상 확인 {summary['wall_marker_rgb_verified']}/5다. "
    '이 부분 집계는 ID 등록·RGB 증거만 나타내며, 위 엄격 완주 판정은 독립 실제 접근과 지도 오차도 요구한다.','',
    '| 초기 seed | 결과 | 등록 ID | 근거리 영상 확인 ID | 시뮬 시간 | 실제 시간 | 실제 이동 m | 범퍼 | 저장 오차 최대 |',
    '|---|---|---|---|---:|---:|---:|---:|---:|']
for r in rows:
    worst=max(r['map_errors_saved_m'].values(),default=None)
    error=f'{worst*100:.1f}cm' if worst is not None else '없음'
    lines.append(f"| {r['seed']} | {'통과' if r['strict_saved_map_success'] else '미달'} | {r['registered_ids']} | {r['sensor_verified_ids']} | {fmt(r['elapsed_sim_s'])} | {fmt(r['elapsed_wall_s'])} | {r['path_length_m']:.1f} | {r['bumper_events']} | {error} |")
lines+=['','![독립 실제 주행 경로](routes.png)','',
    '![벽 마커까지의 실제 거리](wall_distance.png)','',
    '## 조건과 판정','',
    '사용자가 지정한 20.15×11.35m demo room의 트랙·책상·기둥·남쪽 확장 공간을 사용했다. '
    '원본 4개 상자에 하나를 추가해 **총 5개**를 무작위 배치했고 매 회차 실제 생성 개수를 검사했다. '
    '마커는 벽 1개·서로 다른 상자 2개이며 나머지 상자 3개에는 없다. '
    '사진 회차의 정확한 좌표를 역산한 복제는 아니며, 같은 구조의 다섯 다른 배치다.','',
    '원본 크기(7번20cm·32번25cm·103번30cm)를 유지했다. 벽에 붙는 ID가 달라지므로 벽/상자 효과와 마커 크기의 효과를 분리한 대조 실험은 아니다. '
    '경로 그림의 책상은 상판을 위에서 본 표시이며, 실제 충돌 형상인 다리와 달라 로봇 경로가 상판 영역 아래를 통과할 수 있다.','',
    '초기 seed112601~112605를 미리 정했다. 가구 겹침을 피하는 생성기의 재추첨·격자 조정이 있을 수 있어 '
    '실제 생성 seed와 최종 spawn을 상세 기록에 남겼다. 정책은 정답 장면을 받지 않는다. '
    '실행 도중 정책·가중치·속도·관찰 기준을 튜닝하지 않았고 실패를 성공할 때까지 재시도하지 않았다.','',
    '동결 v1.5.1-balanced, 480×480·5FPS, MPPI 전진0.25m/s·회전0.5rad/s·inflation0.45m·global costmap1Hz를 유지했다. '
    '원래 트랙 recorder의 4Hz 변경은 사용하지 않았다. 실제 파라미터는 [조회 기록](runtime_parameters.json)을 따른다. '
    '하나의 Gazebo씩 domain174·localhost·두 공통 lock으로 실행했고 실물 로봇은 사용하지 않았다.','',
    '각 회차 최대600초(시뮬 시간). 세 ID, 등록 후 독립 실제 거리 각각1.5m 이내, 기존 세 장1.3m 근거리 영상 증거, '
    '초기 좌표 정렬을 고정한 live·저장 지도 오차0.25m 이내, 범퍼0을 함께 요구한다. '
    '등록만 했거나 단순히 목표 근처를 지나간 것을 완주로 처리하지 않는다.','',
    '시뮬 속도가 실시간보다 느려 바깥 감독기의 1200초 실제 시간 한도를2400초로 늘렸다. '
    '첫 관측기와 ROS·Gazebo는 같은 PID로 계속 실행했고 나머지 네 회차도 같은 바깥 한도를 적용했다. '
    '시뮬600초 제한과 정책은 바꾸지 않았다. [인계 증거](supervisor_revision.json).','',
    '처음 시작한 4상자 조건은 첫 회차만 완료하고 두 번째 도중 사용자 요청으로 중단·보존했다. '
    '그 중단을 정책 실패로 세거나 위 다섯 회차에 합산하지 않는다.','',
    '## 남은 문제와 해석','']
for r in rows:
    missing=sorted({7,32,103}-set(r['registered_ids']))
    bad={k:round(v,3) for k,v in r['map_errors_saved_m'].items() if v>.25}
    lines.append(f"- {r['seed']}: 미등록 {missing}, 독립 접근 완료 {r['independent_approach_complete']}, "
        f"저장 오차 초과 {bad}, 정지 {r['motion']['stopped_s']:.1f}초, 제자리 회전 {r['motion']['rotation_only_s']:.1f}초, "
        f"취소 사유 {r['cancellations']}.")
    lines.append(f"  스케줄 단계 {r.get('scheduler_phases',{})}, 마지막 후보 수 {r.get('scheduler_last_available',{})}, "
                 f"선택된 목표 종류 {r['selected_goal_counts']}.")
    lines.append(f"  약한 벽 단서 목표 선택 {r.get('weak_wall_goal_selections',0)}회, "
        f"첫 벽 면 선택 {r.get('first_wall_surface_selection_s')}초, "
        f"벽 예산 구간의 내부 새 면 우선 선택 {len(r.get('late_interior_before_wall_budget',[]))}회.")
lines+=['','정지·회전에는 필요한 관찰도 포함된다. 목표 선택 횟수는 실제 Action 전송 횟수와 구분한다. '
    'sensor_interruptions는 카메라뿐 아니라 Nav2 등 준비도 검사 취소도 포함하는 정책 카운터다. '
    '3회차에는 nav2_unavailable로 목표를 보존했다가 자동 재개한 기록이 있으며, 이를 카메라 프레임 유실로 해석하지 않는다. '
    '보드의 진행 중 거리 표시가 독립 기록보다 크게 나오는 구간이 있어 위 거리와 경로는 시뮬레이터 위치로 다시 계산했다. '
    '상자 추가의 인과 효과나 모든 밀집 환경에서의 최적성을 입증하는 비교는 아니다. 실물의 케이블·탑재 중량·OAK-D blur·RPi/Create3 부하는 검증하지 않았다.','',
    '[벽 우선순위 결함·실제 영상 근거](WALL_FINDING.md) · [상세 결과](result.json) · [집계](summary.json) · [다섯 영상](videos.md) · [보드](http://192.168.252.2:8780/)','',
    '보드 카드: `v151-boxes5-track-m112601-trial1`~`v151-boxes5-track-m112605-trial5`. '
    '영상은 실제 Gazebo RGB이며 시청용MP4만5FPS로 재표본화했다. 원래 수신 프레임으로 검출·타이밍을 평가했다. '
    '지도·ROS 원본·영상 바이너리는 Git에 넣지 않고 VM과 Mac 아티팩트에 보존한다.','']
(HERE/'README.md').write_text('\n'.join(lines))
print(json.dumps(summary,indent=2))
