# Mixed v1.5.1 · 확정 이벤트와 실물 알림 (`1.5.1-marker-events`)

v1.5.1-balanced(`policy_releases/2026-11-21_mixed_camera_sync_v151`)의 소스를 그대로 복사한 뒤 **계측 기능만** 더한 작업공간이다. 목표 선택, Nav2 설정, 판독 기준(18픽셀, 3프레임 확정)은 바꾸지 않았다.

| 변경 | 내용 |
|---|---|
| `detector.py` | ID가 확정되는 프레임에서 `/mission/marker_events`에 `confirmed`를 한 번 발행한다. 내용은 ID, 확정 프레임의 촬영 시각, 발행 시각, 프레임 수, 지도 좌표 중앙값이다. 0.5초 주기 발행에서 확정이 취소된 ID가 있으면 `retracted`를 보낸다 |
| `marker_signal.py` (새 노드) | `confirmed`를 받으면 TurtleBot4 LED 링 전체를 3초 동안 초록으로 켜고, 짧은 소리 두 음(880Hz, 1320Hz)을 낸다. 이동 명령은 내지 않고 정답 데이터도 읽지 않는다 |
| `robot.launch.py`, `setup.py` | `marker_signal` 실행 파일을 등록하고 실행한다. `marker_signal:=false`로 끄면 노드는 뜨지만 LED·소리를 내지 않는다 |

```sh
bash team-sim.sh setup ~/simul-ws/events-baseline --policy marker-events
```

## 검증

- `test_marker_events.py` 6개 통과(Jazzy). 확인한 항목은 다음과 같다.
  - 바뀐 파일이 위 4개뿐이고 나머지 소스·설정이 동결본과 같다.
  - 세 번째 프레임에서 이벤트가 한 번만 나간다.
  - 확정 취소를 보고한다.
  - LED·소리 메시지 형식이 맞다.
  - 수정된 릴리스는 설치를 거부한다.
- Gazebo 112601 120초: 103번 확정 이벤트가 촬영 0.02초 뒤에 발행됐다. 평가기가 이 이벤트로 기록한 `registered_event_s`는 69.4초로, 기존 주기 방식 `registered_s`(69.9초)보다 0.5초 정확하다. 시뮬 안에서 `marker_signal`이 LED·소리 명령을 발행했다.

## 한계

- **실물 LED·소리는 확인하지 않았다.** 시뮬의 Create3는 모의 노드라 LED와 소리가 실제로 나지 않는다. 첫 실물 사용 때 확인이 필요하다.
- 판단은 v1.5.1-balanced와 같지만 소스 해시는 다르다. 이 작업공간 회차와 기존 v1.5.1 회차는 따로 집계한다.
