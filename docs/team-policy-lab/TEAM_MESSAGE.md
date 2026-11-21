팀원들, 각자 Ubuntu 24.04·ROS2 Jazzy에서 우리 시뮬을 돌리고 정책을 같이 개선할 수 있게 README랑 실행 도구를 정리했어.

기준판은 Mixed v1.5.1-balanced야. 같은 대형 트랙 다섯 배치에서 완주3/5, 벽 접근3/5, 상자 접근9/10이었어. 최근 rescue360은 완주3/5, 벽 접근3/5, 상자 접근10/10이라 벽을 더 잘 찾는 문제는 아직 남아 있어. 실물 성능은 별도 확인이 필요해.

README대로 저장소를 받고 `team-sim.sh setup`으로 기준판과 개인 수정판 작업공간을 따로 만들면 돼. `team-sim.sh board`로 자기 컴퓨터의 보드를 열고, `team-sim.sh run`으로 같은 장면을 실행하면서 RGB·이동 궤적·마커 등록·접근 결과를 볼 수 있어. 기존 Mac이나 VM에 접속하지 않아도 되는 방식이야.

나처럼 계속 시뮬 영상과 로그를 보면서 더 좋은 정책을 찾아줬으면 해. 벽을 건너뛰는 문제, 상자 주변을 맴도는 문제, 멀리 보인 마커를 놓치는 문제, 막힌 목표에서 복구하지 못하는 문제를 나눠서 맡아보자. 기존판은 보존하고 개인 브랜치에서 수정한 뒤 같은 다섯 맵으로 비교해줘. 실패도 같이 남기고, 마커를 읽은 것과 실제1.5m 접근에 성공한 걸 구분해서 공유해줘.

README: https://github.com/peter010905/simul/blob/codex/mixed-camera-aware/docs/team-policy-lab/README.md
코드: https://github.com/peter010905/simul/tree/codex/mixed-camera-aware
기존 영상: https://github.com/peter010905/simul/releases/tag/simulation-videos-2026-11

설치나 실행이 막히면 사용한 Ubuntu·ROS 버전, 명령, 실행 ID와 에러 로그를 남겨줘. 수정안은 영상 시각·로그 근거와 전후 수치를 붙여 PR로 올리면 돼. 실물 주행은 이번 시뮬 실험과 분리해서 진행하자.
