# simul

TurtleBot4가 처음 보는 공간을 SLAM으로 탐색해서 ArUco 마커 3개를 찾는 프로젝트입니다.
각 마커에 1.5 m 이내로 접근하고, 지도 좌표를 정확히 기록하고, 충돌 없이 빨리 끝내는 것이 목표입니다.

## 저장소 구조 (2026-11-27 정리)

| 경로 | 내용 |
|---|---|
| `legacy/` | 2026-11-27까지의 정책, 실험, 로그, 시나리오, 도구 전부. 내부 상대 경로를 그대로 유지했으므로 frozen 릴리스와 후보의 sha256 검증, 재빌드가 그대로 동작합니다. **읽기 전용 보관소**입니다. |
| `robot.sh`, `start_robot.command`, `stop_robot.command` | 실물 로봇 시작과 정지 진입점. VM 쪽 체크아웃이 기존 평면 구조인지 `legacy/` 구조인지 자동으로 판별합니다. |
| `AGENTS.md` | 저장소 유지 규칙 |

새 작업은 `legacy/` 밖의 새 구조에서 시작합니다.

## 기존 작업 이어 보기

- 현재 상태 요약과 날짜순 진행 기록: [`legacy/README.md`](legacy/README.md)
- 기계 판독용 상태: [`legacy/repository_snapshot/status.json`](legacy/repository_snapshot/status.json)
- 팀 실험 안내: [`legacy/docs/team-policy-lab/README.md`](legacy/docs/team-policy-lab/README.md)

기존 도구는 `legacy/` 안에서 예전처럼 실행합니다.

```sh
cd legacy
bash team-sim.sh setup ~/simul-ws/events-baseline --policy marker-events
TEAM_SIM_RENDER=system bash team-sim.sh run ~/simul-ws/events-baseline 112601
```

## 실물 로봇

```sh
./robot.sh check    # 읽기 전용 준비 상태 확인
./robot.sh start
./robot.sh stop
```

## GitHub Release

시뮬레이션 영상, 실행 로그, run-package는 이전과 같이 `RSH-17/simul`의 GitHub Release에 올립니다.
게시 도구는 `legacy/scripts/publish_simulation_videos.py`이고, 태그 규칙은 바뀌지 않았습니다.
