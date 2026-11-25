# 팀 실행기 평가기 (`judge/`)

`team-sim.sh run`의 감시·채점 코드다. 예전에는 실험 폴더 다섯 곳에 흩어진 클래스를 상속해 썼는데, 팀 실행기가 쓰는 사본을 이 패키지 하나로 모았다. 원본 실험 폴더(`experiments/…`)는 기록 보존을 위해 그대로 두었다.

| 모듈 | 클래스 | 원본 |
|---|---|---|
| `base.py` | `Trial` — 시작 요청, 실제 위치·범퍼·등록 ID 기록, 종료, 지도 저장 대기 | `tools/team_sim/helpers/2026-11-19_pair_7_103/measure.py` |
| `video.py` | `CameraTrial` — RGB 녹화와 방향 기록 | `…/2026-11-19_corridor_box/measure_video.py` |
| `performance.py` | `PerformanceTrial` — 카메라 처리율, 자원 사용, 실시간 보드 | `experiments/2026-11-20_v12_camera_rate/trial.py` |
| `strict.py` | `StrictTrial` — 지도 오차, 근거리 증거, 엄격 판정 | `experiments/2026-11-21_camera_aware_policy/strict_trial.py` |
| `team.py` | `GatedCameraTrial`, `WallTrial`, `PassiveExamTrial` | `experiments/2026-11-21_wall_transit_v152/trial.py`, `experiments/2026-11-22_random_exam/trial.py` |
| `camera_noise.py` | 선택적 RGB 잡음·흐림·손실 (`TEAM_SIM_CAMERA_NOISE`) | 신규 |
| `compare_board.py`, `board_atomic.py`, `finish_trial.py`, `diagnostics.py`, `refresh_final.py` | 보드 카드와 종료 후 검증 | corridor_box, random_track, wall_transit, v12 |

원본과 달라진 점은 다음과 같다.

- **지도 저장 대기:** `/mission/stop` 뒤 상태가 `done`이 될 때까지 최대 45초 기다린다. 예전에는 3초 만에 종료해서 지도가 저장되지 않았다.
- **종료 직후 위치:** 종료 시각을 고정한 뒤 0.25초 동안 실제 위치를 더 받는다. 이 구간의 새 등록과 범퍼는 집계하지 않는다.
- **추가 기록:** `start_sim_gazebo_s`를 기록하고, `/mission/marker_events`가 있으면 `registered_event_s`를 기록한다.
- **카메라 잡음:** 잡음 조건이면 기존 5맵 모드에서도 검출기 영상을 감시 프로그램이 중계한다(`/exam/detector_rgb`).
- **경로 정리:** `CAMERA_BENCH_HELPERS` 환경 변수와 실험 폴더 경로 의존을 없앴다. DDS 설정 파일은 `tools/team_sim/cyclone.xml`이다.

정답 위치는 이 패키지만 읽고, 정책에는 보내지 않는다.
