# Mixed v1.5.7.1 · 상위 규칙 비교 기록 (`1.5.7.1-decision-trace`)

[v1.5.7 새 바닥까지 도는 순회 슬롯](../2026-11-27_v157_frontier_tour/README.md)에 **기록만** 더한 후보다. 목표를 고르는 방식은 바꾸지 않았다. **운영 정책이 아니다.** 아직 Gazebo에서 돌려 보지 않았다.

```sh
bash team-sim.sh setup ~/simul-ws/decision-trace --policy decision-trace
TEAM_SIM_RENDER=system bash team-sim.sh run ~/simul-ws/decision-trace 112601
```

## 왜 필요한가

팀 보드의 결정 타임라인에서 v1.5.7 회차들을 보면, 결정 181개 가운데 156개가 상위 규칙(phased hybrid)이 따로 만든 목표였다. 점수식 상위 3개 후보로 간 경우는 4개뿐이었다(frontier-tour 8회차). 그런데 기존 로그(`center_wall.jsonl`)에는 규칙의 이유 이름과 대상만 남는다. 규칙이 어떤 후보들을 비교했는지, 왜 그 지점이 1위였는지는 남지 않는다.

## 바꾼 점 (v1.5.7 대비)

| 파일 | 내용 |
|---|---|
| `rule_trace.py` (새 파일) | 규칙이 비교한 목록을 JSON으로 옮기는 읽기 전용 함수들. 각 목록은 규칙 자신의 순서로 앞 10개까지 저장한다 |
| `center_wall.py` | `decide()`가 비교한 목록을 `self.trace`에 담는다. 줄을 추가하기만 했다 |
| `phased_hybrid.py` | `select()` 시작 때 기록을 비운다. `dispatched()`가 목표마다 `native/rule_trace.jsonl`에 한 줄씩 쓴다. 줄을 추가하기만 했다 |

`rule_trace.jsonl` 한 줄에 담기는 내용은 다음과 같다.

| 항목 | 내용 | 규칙의 순서 |
|---|---|---|
| `fresh` | 아직 시도하지 않은 물체 면(복잡한 상자 무리를 정리한 최종 목록) | 끝난 상자 아님 → 같은 상자 이어서 → 형태를 맞춘 면 → 걸어서 가까운 순 (`BoxBatch.rank`) |
| `interior`, `deferred` | 전체 물체 면, 끝난 상자의 면 | 위와 같음 |
| `wall` | 벽 관측 지점 | 점수가 낮은 순. 점수 = 경로 거리 + 서는 거리·옆 이동 벌점 + 시도 횟수×2 + 벽 이어짐 벌점 |
| `frontiers` | `discovery_frontier`와 같은 조건을 통과한 프런티어 | 점수가 높은 순. 점수 = 이득 ÷ (1 + 0.18×이동 + 2 m 미만 벌점 + 재방문) |
| `slot`, `slot_frontiers` | 벽 구간 순회의 상태와 순회용 프런티어 수 | |
| `counts`, `crowded_centers`, `remaining_s`, `wall_reserve_s` | 어느 분기로 갔는지 판단한 입력 | |
| `chosen_from` | 실제 목표가 어느 목록의 몇 위였는지 | |

좌표는 지도(map) 좌표다. 팀 보드는 시작 자세로 월드 좌표로 바꿔 그린다.

## 결정이 바뀌지 않는 근거

- 설치 결과를 v1.5.7과 비교하면 두 파일 모두 줄이 **추가만** 되었다. 바뀌거나 지워진 줄은 없다(테스트로 확인).
- 추가한 코드는 목록을 읽기만 한다. 정렬은 복사본으로 하고, 원래 목록이나 목표를 고치지 않는다. 오류는 모두 잡아서 기록만 버린다.
- 프런티어 순위를 매기려고 `veto`를 한 번 더 부른다. `veto`가 쓰는 `NavigationFlow.blocked()`는 같은 시각으로 다시 불러도 결과와 상태가 같다(만료된 실패 기록만 지운다).
- 남는 위험은 계산 시간이다. 결정마다 후보 200개 정도를 한 번 더 훑는다.

## 한계

- 마커 접근, 기억된 마커 단서처럼 규칙(`decide()`)을 거치지 않은 목표에는 비교 기록이 없다(`trace: null`).
- `discover_beyond_inspected_boxes`(상자 무리 밖 프런티어)는 별도 프런티어 목록에서 고르므로, 그 목록은 남지 않는다.
- 벽 구간 순회는 순회 계획의 다음 목표로 간다. 그래서 `wall` 목록의 순위와 실제 목표가 다를 수 있다. 순회 계획 자체는 `slot.tour`에 남는다.

## 검증

- `test_decision_trace.py` 4개 통과.
  - 목록이 규칙의 순서대로 저장되고, 고른 목표의 순위가 맞게 표시된다.
  - 프런티어 순위가 `discovery_frontier` 점수와 같다.
  - 오류가 나도 기록만 빠진다.
  - 설치 결과가 v1.5.7에 줄을 추가만 하고, 새 파일은 `rule_trace.py` 하나다.
- `~/simul-ws/decision-trace` 작업공간의 설치·빌드와 ROS 환경 import를 확인했다.
- **아직 하지 않은 것:** Gazebo 주행. 이 후보를 만들 때 v1.5.7 반복 캠페인이 동시 실행 한도(2개)를 쓰고 있었다. 첫 주행에서 `rule_trace.jsonl`이 결정마다 써지는지, 경고(`Could not save rule trace`)가 없는지, 결정 간격이 v1.5.7과 비슷한지 확인해야 한다.
