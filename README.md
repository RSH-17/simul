# simul

TurtleBot4가 처음 보는 공간을 SLAM으로 탐색해서 ArUco 마커 3개를 찾는 프로젝트입니다.
각 마커에 1.5 m 이내로 접근하고, 지도 좌표를 정확히 기록하고, 충돌 없이 빨리 끝내는 것이 목표입니다.

## 저장소 구조

| 경로 | 내용 |
|---|---|
| `osp/` | 새 공통 패키지입니다. ROS에 의존하지 않습니다. 시나리오, 설정, 정책, 백엔드, 평가기, CLI가 들어 있습니다. |
| `configs/` | 계층 설정입니다. 아래 "설정 계층" 참고. |
| `tests/` | 새 구조의 테스트입니다. 기존 결과를 그대로 재현하는지 확인하는 동등성 테스트도 포함합니다. |
| `runs/` | 실행 결과가 쌓이는 곳입니다. Git에는 올리지 않습니다. |
| `legacy/` | 2026-11-27까지의 정책, 실험, 로그, 시나리오, 도구 전부입니다. **읽기 전용 보관소**이며, 내부 상대 경로를 그대로 두었으므로 frozen 해시 검증과 재빌드가 그대로 동작합니다. |
| `robot.sh`, `*.command` | 실물 로봇 시작과 정지 진입점입니다. |

### 구성 요소

```
Scenario ──mission()──▶ Policy          정책이 받는 것: 시작 자세, 마커 수(3), 제한 시간
   │  └──world()─────▶ Backend         fast_sim(2-D 대리 모델) | gazebo(team_sim)
   └──targets────────▶ Evaluator       정답은 평가기만 봄
Backend ──EpisodeRecord──▶ Evaluator ──▶ runs/<id>/{config,plan,summary}.json, episodes.jsonl
```

- **평가기**(`osp/evaluator.py`)는 필수 조건을 하나씩 따로 판정합니다. 조건은 ID 정확성, 1.5 m 접근, 지도 위치 0.25 m 이내, 충돌 0회, 시간 내 완료입니다. 백엔드가 측정하지 못한 조건은 통과가 아니라 **미측정**으로 남깁니다. 예를 들어 fast_sim은 지도 위치를 추정하지 않으므로 strict 성공이 나오지 않습니다.
- **정책**: 새 2-D 결정 규칙은 `osp.policies.SurrogatePolicy`를 상속해서 `options` / `choose`만 바꾸면 됩니다. 이 두 함수는 관측된 지도만 받습니다.

### 설정 계층

`configs/base.yaml` → `configs/backends/<b>.yaml` → `configs/policies/<p>.yaml` → `--set key.path=value` 순서로 겹칩니다. 뒤에 오는 값이 앞의 값을 덮어씁니다.
정책 설정은 스스로를 `production`으로 지정할 수 없습니다. 쓸 수 있는 상태는 `legacy_release`, `legacy_candidate`, `experimental`입니다.

## 사용법

```sh
python3 -m osp list policies          # 정책별 지원 백엔드
python3 -m osp list scenarios         # 시나리오 세트 (robust/test, exam/dev, ...)
python3 -m osp config --backend fast_sim --policy face_intent_v2 --set mission.duration_s=300

# 2-D 대리 모델 (부작용 없음, 즉시 실행)
python3 -m osp run --backend fast_sim --policy face_intent_v2 --scenarios robust/test

# Gazebo: 기본은 계획만 출력합니다. --execute를 붙여야 실행합니다(호스트 동시 실행 최대 2개).
python3 -m osp prepare --policy frontier_tour --execute     # 작업공간이 없을 때만
python3 -m osp run --backend gazebo --policy frontier_tour --scenarios exam/dev --repeats 3 --execute

# 이미 끝난 team_sim 시도를 새 평가기로 다시 채점합니다(작업공간과 동시 실행 수별로 집계).
python3 -m osp import-team-sim

python3 -m pytest                     # 새 구조 테스트
```

검증한 내용:
- `fast_sim` 결과가 `legacy/training_robust/evaluate.py`와 시드별로 정확히 같습니다.
- 로컬 team_sim 시도 157개를 새 평가기로 다시 채점한 결과가 기존 strict 판정과 모두 일치합니다.

## 기존 작업

- 현재 상태와 날짜순 진행 기록: [`legacy/README.md`](legacy/README.md), [`legacy/repository_snapshot/status.json`](legacy/repository_snapshot/status.json)
- 팀 실험 안내: [`legacy/docs/team-policy-lab/README.md`](legacy/docs/team-policy-lab/README.md)
- 기존 도구는 `legacy/` 안에서 예전처럼 실행합니다. 예: `cd legacy && bash team-sim.sh run ~/simul-ws/baseline 112601`

## 실물 로봇

```sh
./robot.sh check    # 읽기 전용 준비 상태 확인
./robot.sh start
./robot.sh stop
```

`osp` CLI는 실물 로봇을 구동하지 않습니다.

## GitHub Release

시뮬레이션 영상, 실행 로그, run-package는 이전과 같이 `RSH-17/simul`의 GitHub Release에 올립니다.
게시 도구는 `legacy/scripts/publish_simulation_videos.py`이고, 태그 규칙은 바뀌지 않았습니다.
