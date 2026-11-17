# 정책 실행 로그 — simulation / robot

정책을 실행한 **컴퓨터**에서 수집하고 종료 시 자동 업로드합니다. 원시 파일은 GitHub Release 첨부파일이며, 이 디렉터리에는 작은 실행별 JSON 색인만 저장합니다.

| 실행 분류 | 로컬 원본 | GitHub 색인 | 월별 Release |
|---|---|---|---|
| Gazebo 시뮬레이션 | `run_logs/simulation/<run-id>/` | [simulation](simulation) | `policy-logs-simulation-YYYY-MM` |
| 실제 TurtleBot4 | `run_logs/robot/<run-id>/` | [robot](robot) | `policy-logs-robot-YYYY-MM` |

`mode=robot`은 사용자가 지정한 실행 분류입니다. 하드웨어 연결이나 주행 성공을 자동 인증하지 않습니다. `evidence_type=logging_smoke_test`는 업로드/기록 기능 시험이며 실제 로봇이나 Gazebo 성능 결과가 아닙니다. 현재 저장소가 공개이므로 첨부 로그도 공개됩니다.

## 한 번 설정

각 실행 PC에서 GitHub CLI 로그인과 저장소 쓰기 권한이 필요합니다. 로봇 PC의 인증은 이 컴퓨터에서 복사하지 않습니다.

```bash
gh auth login
bash scripts/install_policy_logging.sh
systemctl --user status osp-policy-log-sync.timer
```

종료 직후 업로드하고, 실패분과 종료된 native Gazebo 작업은 사용자 systemd 타이머가 **2분 간격**으로 재시도합니다. 원본은 자동 삭제하지 않습니다. 네트워크 실패로 정책 종료 코드를 성공으로 바꾸지 않습니다. 설치기는 사용자 systemd linger도 켜도록 시도합니다. 해당 PC의 권한으로 허용되지 않으면 안내에 따라 별도 설정해야 사용자 세션 종료 후에도 재시도가 계속됩니다.

## 기본 실행

다음 기존 진입점에는 자동 기록이 연결돼 있습니다. `--dry-run`, `--check`, `--build-only`는 주행 기록을 만들지 않습니다. 기록 기능이 임무 시작 권한을 추가하지 않으며 실물은 기존처럼 `--start`가 있어야 자동 출발합니다.

```bash
# 실물: 기존 빌드/센서 bringup 및 ROS discovery 설정 필요
bash run_robot_policy.sh --domain 7 --start

# 시뮬레이션: 기존 준비된 장면과 정책 사용
bash run_gazebo_runtime_probe.sh --scene-index 0
bash run_gazebo_benchmark.sh --limit 1
```

일반 ROS 명령은 아래 진입점으로 감쌉니다. 직접 `ros2 launch`만 실행하면 자동 수집되지 않습니다.

```bash
bash run_policy.sh --mode robot --workspace "$PWD/deployment/robot_v13" \
  --policy "$PWD/deployment/robot_v13/install/osp_mission/share/osp_mission/config/edge_policy.json" \
  -- ros2 launch osp_mission robot.launch.py use_sim_time:=false

bash run_policy.sh --mode simulation --workspace "$PWD" \
  -- ros2 launch osp_mission simulation.launch.py use_sim_time:=true
```

일반 실행의 ROS domain은 환경의 `ROS_DOMAIN_ID`를 따릅니다. 기존 실물 진입점은 `--domain`, Gazebo 진입점은 `--domain`(기본 84)을 기록기에도 적용합니다. `--namespace robot1`을 기록기에 지정하면 해당 네임스페이스와 공통 `/tf`, `/tf_static`, `/clock`, `/rosout`, `/parameter_events`로 제한합니다. 네임스페이스가 없으면 발견된 모든 토픽과 숨김 토픽을 기록합니다.

## 수집 범위

- `run.json`: 실행 분류·시각·종료 코드·중단 신호·명령·Git 버전·로거 해시·정책 해시·ROS domain.
- `console.log`, `recorder.log`, `ros_logs/`: 콘솔/ROS/기록기 출력. 별도 네이티브 서비스 출력은 원래의 `service.log`에 포함됩니다.
- `rosbag/`: MCAP+zstd, 256MiB 단위 분할. 발행된 odom·IMU·LiDAR·카메라·TF·지도·Nav2 계획경로·cmd_vel·진단 등 발견 가능한 ROS 토픽. 서비스/액션 호출 자체는 해당 노드의 introspection 이벤트 발행이 있어야 기록됩니다.
- `policy.json`, `config/`, `artifacts/`: 사용한 정책과 설정, 실행이 생성한 지도 PGM/YAML·pose graph·마커/경로/평가 결과. `osp_mission` 직접 launch에는 격리된 `output_dir`가 기본 주입됩니다. 별도 지정한 `output_dir`와 `--artifact /출력/폴더`도 묶습니다. Gazebo 기본 진입점은 실행별 `artifacts/`를 사용합니다.

`capture_complete`는 기록 프로세스의 정상 종료 여부이며 센서의 모든 메시지 수신이나 과제 성공을 뜻하지 않습니다. ROS discovery/QoS, 센서 발행 중단, 부하로 메시지가 누락될 수 있으므로 센서 품질은 rosbag metadata와 원래 평가 결과로 판단합니다. 기록기는 로봇 명령/TF/추정 pose를 발행하지 않습니다. 토픽 보존을 위해 여유 디스크가 필요하며 원본 보존 기간은 자동으로 줄이지 않습니다.

실행 중/대기 중인 frozen native 캠페인은 수정하지 않습니다. 타이머 설정 이후 종료된 `training_robust/native90/*/jobs/*`는 서비스 종료와 출력 소유 프로세스 부재를 확인해 기존 결과·센서/ROS 로그를 별도 수집합니다. 이 경우 **소급 rosbag은 없고** `bag_enabled=false`와 수집 범위를 명시합니다. 실패 작업도 남기며 완료 평가가 없으면 `capture_complete=false`입니다. 평가 정답은 정책에 전달하지 않습니다. 기존 영상 게시기는 유지합니다.

## 업로드 및 복구

아카이브는 256MiB 이하의 `*.tar.gz.part0001` 파일로 나눕니다. 각 첨부파일의 GitHub 크기와 SHA-256을 확인한 뒤에만 `upload.json`을 `uploaded`로 표시하고 GitHub `main`에 색인 JSON을 커밋합니다. 기존 첨부파일과 색인이 다르면 덮어쓰지 않습니다. 비밀키·`.env`·빌드 산출물·원본 소스 복사본을 제외하고 전체 환경 변수를 덤프하지 않습니다. 영상은 기존 `scripts/publish_simulation_videos.py`를 통해 별도로 게시하며 로그 묶음/Git에는 넣지 않습니다.

```bash
# 특정 실행 재시도
python3 scripts/policy_run_logs.py upload run_logs/robot/RUN_ID
# 전체 대기열 확인/재시도 (로그 기록 자체는 ROS 환경을 설정한 run_policy.sh 사용)
python3 scripts/policy_run_logs.py sync
# 오래된 native 작업까지 명시적으로 포함하려면 Unix timestamp 지정
python3 scripts/policy_run_logs.py sync --since 0 --limit 1
# 다운로드한 동일 실행의 part 파일을 번호순으로 결합
cat RUN_ID.tar.gz.part* | tar -xz
```

SIGINT/SIGTERM 때 자식 프로세스에 신호를 전달하고 rosbag을 정리한 뒤 기록을 남깁니다. wrapper가 강제 종료되면 sync가 프로세스/부팅 ID를 확인합니다. 정책이 살아 있는 동안 업로드하거나 종료시키지 않습니다. 정책이 끝난 뒤 미완료로 복구하며 깨진 bag은 별도 reindex가 필요할 수 있습니다. `--no-upload`, `--no-bag`는 명시적 예외 옵션입니다.
