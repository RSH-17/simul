# Mixed v1.5.5 · 먼 벽 슬롯 (`1.5.5-far-wall-slot`)

[v1.5.4 벽 슬롯·갇힘 탈출](../2026-11-26_v154_wall_slot_escape/README.md)에서 슬롯이 벽을 고르는 방식만 바꾼 후보다. **운영 정책이 아니다.**

같은 조건 20회 결과는 기준판보다 좋았다. 개발 맵은 10/15(기준판 7/15), 무작위 배치는 4/5(기준판 3/5)였다. 그러나 미리 정한 판정 기준인 12/15에는 못 미쳤다. 그래서 맵당 반복을 늘린 확인이 남아 있다. [결과](../../experiments/2026-11-26_far_wall_slot/README.md)


## 실물 실행 패키지

[`policy-v1.5.5` 릴리스](https://github.com/RSH-17/simul/releases/tag/policy-v1.5.5)의 `osp-policy-v1.5.5.tar.gz`(SHA-256 `60426c7533e9…159a`). 커밋 0146ff4에서 `scripts/package_robot_policy.py`로 만들었고, `runtime/`은 v1.5.2.57 패키지의 것을 그대로 넣었다. 로봇에 유선 연결한 Ubuntu 24.04 + Jazzy 컴퓨터에서 `./run.sh build`, `check`, `start`, `stop`으로 실행한다. 이 PC(x86_64)에서 압축 해제·체크섬·빌드만 확인했다. 실물 주행 결과는 없다([상태](robot_package_status.md)).

```sh
bash team-sim.sh setup ~/simul-ws/far-wall-slot --policy far-wall-slot
```

| 변경 (v1.5.4 대비) | 내용 |
|---|---|
| `center_wall.py` | 로봇이 지나간 위치를 0.5 m 간격으로 기록한다(odom 좌표). 슬롯의 첫 벽 관찰 지점은 아직 보지 않은 벽 구간 중 이 기록에서 가장 먼 곳(3 m 이상)으로 고른다. 로봇이 그 구간에서 3 m 안에 들어올 때까지는 다음 슬롯 지점도 그 구간에서 고른다. 그 뒤로는 기존 규칙대로 가까운 지점을 고르며 벽을 따라간다 |
| `slot_escape.py` | `far_wall()`와 `toward_target()`를 추가했다. 슬롯 상한을 90초에서 120초로 늘렸다. 먼 벽까지 이동하는 시간을 고려한 값이다 |
| `phased_hybrid.py` | `center_wall.jsonl`에 고른 먼 벽까지의 거리(`far_wall_novelty_m`)와 슬롯 목표를 기록한다 |

슬롯 시작 조건은 v1.5.4와 같다. 30%가 지났고, 벽 단계가 아직 없었고, 등록한 ID가 3개 미만이고, 벽 후보가 있어야 한다. 다음은 v1.5.4와 같다.

- 갇힘 탈출
- 마지막 순간의 벽 단계
- Nav2 설정
- 판독 기준

정답 데이터는 읽지 않는다.

## 검증

- `test_far_wall_slot.py` 5개 통과. 확인한 항목은 다음과 같다.
  - 지나가지 않은 방 끝을 고른다.
  - 모든 벽이 지나간 위치에서 가까우면 먼 벽을 고르지 않는다.
  - 슬롯이 고른 구간 쪽 지점을 계속 고른다.
  - 120초 상한이 적용된다.
  - 설치 시 바뀐 파일이 3개뿐이다.
- Gazebo 20회 결과는 [실험 기록](../../experiments/2026-11-26_far_wall_slot/README.md)에 있다.
