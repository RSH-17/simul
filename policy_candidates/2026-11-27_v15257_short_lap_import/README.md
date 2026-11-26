# Mixed v1.5.2.57 · 짧은 벽 순회 · team-sim 비교용 가져오기

팀원 저장소(`peter010905/simul`)의 prerelease [`policy-v1.5.2.57`](https://github.com/peter010905/simul/releases/tag/policy-v1.5.2.57)을 **소스 수정 없이** team-sim 작업공간으로 가져온다. 같은 track 맵·seed에서 기준판, v1.5.4, v1.5.5와 비교하는 것이 목적이다. 운영 정책이 아니며 실물 구동에는 쓰지 않는다.

```bash
bash team-sim.sh setup ~/simul-ws/v15257-short-lap --policy v15257-short-lap
```

- 배포 묶음 SHA-256 `f959bde5…0e88`와 묶음 안의 `SHA256SUMS` 589개 항목을 모두 확인한다. 링크·절대경로·`..`가 포함된 묶음은 거부한다.
- 묶음에 들어 있는 미리 빌드된 `_raycast.so`, `export_slam_graph_poses`는 지우고 setup.py가 소스에서 다시 빌드한다.
- 정책 소스·`frozen-config`(policy.json, Nav2 프로필)는 배포본 그대로다. Nav2 프로필과 policy.json은 v1.5.5 작업공간과 바이트 단위로 같다. 다른 점은 SLAM `max_laser_range` 5 m(v1.5.5는 12 m)와 `.57`까지 누적된 v1.5.2 계열 정책 코드다.
- `robot.launch.py`의 실물 전용 분기(속도 상한·voxel 높이)는 `use_sim_time`일 때 꺼진다.
- `marker_signal` 노드가 없으므로 `registered_event_s`는 기록되지 않는다. 판정에는 `registered_s`를 쓴다.
- `.57`은 원래 실물 교실 재현 시뮬(250×250·10 fps 카메라)에 맞춘 후보다. 여기서는 team-sim 기본 조건(480×480·5 fps)으로 돌리므로, 팀원 보고 수치와 직접 비교하지 않는다.
