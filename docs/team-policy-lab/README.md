# 각자 Jazzy에서 정책을 개선하는 팀 실험실

팀원 각자의 Ubuntu 24.04·ROS2 Jazzy에서 시험장 시뮬을 실행하고 대시보드로 결과를 확인할 수 있습니다. 새 배치를 추첨해 연습하고, 두 정책을 비교할 때는 같은 시드를 사용해 주세요. 원래 비교에 사용한 다섯 배치도 재현할 수 있습니다.

목표는 제한 시간 10분 안에 벽·상자에 붙은 Original ArUco 마커 세 ID를 등록하고 각각 실제 1.5m 안으로 접근하는 것이다. 지도에 나온 공간과 카메라로 확인한 표면은 구분한다.

## 지금까지의 진행 상황

기준판은 `Mixed v1.5.1-balanced`다. 새 후보는 별도로 남기며 기준판을 덮어쓰지 않는다. 같은 대형 트랙 다섯 배치의 결과는 다음과 같다.

| 지표 | v1.5.1-balanced | v1.5.1-rescue360 |
|---|---:|---:|
| 엄격 완주 | 3/5 | 3/5 |
| 벽 등록·실제 근거리 확인 | 3/5 | 3/5 |
| 상자 실제 근거리 확인 | 9/10 | 10/10 |
| 평균 시뮬 임무 시간, 미완주 포함 | 489.6초 | 458.0초 |
| 평균 독립 이동거리 | 83.8m | 73.3m |
| 범퍼 접촉 | 0 | 0 |

`rescue360`은 처음 360초에 기존 주행을 사용한다. 이후 카메라 미관찰 표면과 미지 공간 이득으로 이동할 위치를 고르고 실제 한 바퀴를 관찰한다. 이동 시간에 회전 가감속·영상 대기·여유를 더해 예산을 계산한다. 회전 상한 0.5rad/s에서 이동을 제외한 예산은 약 17.6초다. 기존 마커 판독·접근과 안전 검사는 유지했다.

벽 성공률과 완주율은 같았다. 후보의 두 성공은 전환 전인 360초 안에 종료돼 회전의 효과로 해석할 수 없다. 아직 가장 좋은 정책으로 확정하지 않았고 실물 기본판으로도 교체하지 않았다. [다섯 맵의 경로·수치·영상](../../experiments/2026-11-21_v151_rescue360/README.md)을 먼저 확인한다.

[최신 원본 R32 비교](../../experiments/2026-11-22_latest_r32_compare/README.md)에서는 세 마커를 모두 등록한 회차가 R32 4/5, balanced·rescue360 각각 3/5였습니다. 접근까지 입증된 완주는 R32 2회, 종료 위치 샘플 부족으로 판정 보류 2회였습니다. [Adaptive Survey](../../policy_candidates/2026-11-22_v151_adaptive_survey/README.md)는 벽과 장애물의 관찰 차례, 경로 비용 및 가려진 마커 후보 감지를 개선하는 별도 실험입니다. 아래 setup에서 `--policy adaptive-survey`를 선택하면 독립 작업공간에 설치됩니다.

## 1. Ubuntu와 Jazzy 준비

Ubuntu 24.04와 ROS2 Jazzy가 설치된 PC 또는 VM에서 실행한다. Mac 사용자는 Ubuntu VM 안에서 아래 명령을 실행하고 그 VM의 보드를 브라우저로 연다. ROS 설치가 처음이면 [Jazzy 공식 Ubuntu 설치 안내](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)를 따른다.

Jazzy 패키지 저장소를 설정한 뒤 필요한 패키지를 설치한다.

```sh
sudo apt update
sudo apt install -y build-essential libeigen3-dev \
  libboost-serialization-dev libboost-thread-dev \
  python3-colcon-common-extensions python3-numpy python3-scipy \
  python3-opencv python3-yaml python3-matplotlib python3-psutil python3-pytest \
  ros-jazzy-turtlebot4-simulator ros-jazzy-turtlebot4-navigation \
  ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox ros-jazzy-robot-localization \
  ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-cv-bridge
source /opt/ros/jazzy/setup.bash
```

전용 Ubuntu VM을 사용할 때 자원은 호스트의 여유 메모리에 맞춘다. 소프트웨어 렌더링 때문에 실시간보다 느릴 수 있다. 시뮬 600초와 실제 600초는 다르며 RTF 0.5라면 시뮬 10분에 대략 실제 20분이 걸린다. 같은 기기에서 여러 Gazebo를 동시에 띄우면 결과의 지연 조건도 달라진다.

## 2. 저장소와 개인 작업공간 만들기

```sh
git clone --branch codex/mixed-camera-aware --single-branch \
  https://github.com/peter010905/simul.git
cd simul
git switch -c codex/team-my-policy

bash team-sim.sh setup ~/simul-ws/baseline --policy balanced
bash team-sim.sh setup ~/simul-ws/my-policy --policy balanced
```

브랜치의 `team-my-policy` 이름은 팀원별로 다르게 정한다. 쓰기 권한이 없으면 포크에 푸시하고 PR을 만든다.

첫 `setup`은 같은 트랙 다섯 장면을 Release에서 받고 SHA-256을 검사한다. 장면은 상자5개, 벽 마커1개·상자 마커2개다. 방·책상·장애물·마커 배치는 기존 비교를 재사용하고, 바닥 이미지 참조만 각자 설치 경로로 바꾼다. 정답은 평가기에만 사용한다. 설치된 ROS 파일이나 기존 작업공간은 덮어쓰지 않는다.

`baseline`은 그대로 보존하고 `my-policy`에서 수정한다. 두 작업공간은 동결 v1.5.1 소스·동일 Nav2 설정과 가중치에서 출발한다. 기존 폴더에 다시 설치하면 오류가 나므로 새 버전을 만들 때 새 경로를 고른다. `rescue360`에서 시작하려면 다음과 같이 별도 작업공간을 만든다.

```sh
bash team-sim.sh setup ~/simul-ws/my-rescue --policy rescue360
```

장면 다운로드가 안 되는 환경에서는 [장면 묶음](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/team-jazzy-track5-fixtures-20261122-r2.tar.gz)을 먼저 받아 설치에 `--archive /받은/파일.tar.gz`를 추가한다. 코드·빌드 결과와 장면 캐시는 분리된다.

## 3. 대시보드 열고 주행하기

### 시험장 안을 매번 새로 추첨하는 주행

벽의 외곽·기둥·바닥과 물리는 공유 8780 시험장과 같습니다. 시드를 생략하면 책상 14개의 위치와 방향, 추가 장애물 0~9개의 형태·자리, 출발 자세, 마커 번호·크기·높이·부착면을 새로 뽑습니다. 기본 무작위 모드에서는 마커 1~5개를 뽑으며 전체 개수도 정책에 알려주지 않습니다. 번호와 배치를 처음 보는 상태에서 3개 과제만 비교할 때는 `--markers three`를 사용해 주세요.

```sh
bash team-sim.sh setup ~/simul-ws/print-relook --policy balanced-print-relook
bash team-sim.sh run ~/simul-ws/print-relook
```

같은 배치로 다른 정책을 비교하려면 첫 실행에서 출력한 시드를 넣어 주세요. 아래 두 명령은 책상·장애물·출발점·인쇄물이 동일한 배치에서 각각의 정책을 실행합니다.

```sh
bash team-sim.sh run ~/simul-ws/print-relook 3818821658 --random-layout --markers three
bash team-sim.sh run ~/simul-ws/baseline 3818821658 --random-layout --markers three
```

기존 비교용 시드 112601~112605를 단독으로 지정하면 이전 다섯 배치를 그대로 엽니다. 원하는 시드에 `--random-layout`를 붙이면 해당 시드를 새 추첨기로 재생성합니다. `--check`를 추가하면 주행 전에 장면 지문과 설치를 확인할 수 있습니다. 스친 마커 재관찰판과 R5는 개수를 숨긴 경우 셋을 발견한 뒤에도 탐색을 이어갑니다. 원본 Balanced는 자체 종료 조건으로 세 마커에서 끝날 수 있습니다. 이 차이를 비교할 때는 마커 수 공개 여부도 결과에 적어 주세요.

무작위 주행의 최종 점수와 보드의 초록 마커에는 등록 후 실제 1.5m 접근과 로봇의 근거리 카메라 증거가 함께 필요합니다. 감독용 평가기는 정답을 다 확인해도 주행을 종료시키지 않습니다. [8780 무작위 시험 조건과 결과](../../experiments/2026-11-22_random_exam/README.md)

### 보드 접속

저장소 루트에서 첫 터미널을 열어 보드를 실행한다.

```sh
bash team-sim.sh board
```

같은 Ubuntu의 브라우저에서 `http://127.0.0.1:8780/`을 연다. 화면은 이 컴퓨터에서 실행한 회차만 표시한다. 기존에 쓰던 `192.168.252.2:8780`은 공유 Mac의 VM 주소다. 팀원 각자의 설치에서는 각자의 localhost 보드를 사용한다.

Ubuntu가 Mac의 VM이라면 VM 내부 브라우저를 쓰거나 다음과 같이 전달한다. VM 접속 계정과 주소는 자신의 환경에 맞춘다.

```sh
ssh -N -L 18780:127.0.0.1:8780 계정@Ubuntu접속주소
```

Mac에서는 `http://127.0.0.1:18780/`을 연다. 이 보드는 localhost에만 바인딩하며 ROS를 구독하거나 주행을 시작하는 버튼을 제공하지 않는다. 아래 실행 명령으로 시뮬을 시작하면 카드를 자동 표시한다.

두 번째 터미널에서 먼저 의존성·선택 작업공간·장면 무결성을 확인한다.

```sh
cd simul
bash team-sim.sh run ~/simul-ws/baseline 112601 --check
```

통과하면 기준판과 개인 수정판을 차례로 실행한다.

```sh
bash team-sim.sh run ~/simul-ws/baseline 112601
bash team-sim.sh run ~/simul-ws/my-policy 112601
```

실행기는 선택한 작업공간을 source하고 실제 설치 경로를 검사한다. 별도 ROS 도메인·localhost DDS에서 Gazebo, SLAM, Nav2와 기존 탐지기를 띄운 뒤 준비 상태를 확인하고 미션을 시작한다. 기본은 domain174다. 이 호스트에서 다른 벤치가 실행 중이면 도메인이 달라도 거부한다. 실물 domain4는 사용하지 않는다.

각 실행의 출력 경로는 다르다. 같은 설치에서는 한 번에 한 회차를 실행한다. 현재 호스트의 다른 native 벤치도 검사해 동시 부하로 비교 조건이 달라지는 일을 막는다. 공유 호스트에서 진행 중인 다른 실험을 종료하거나 동결 스테이지를 수정하지 않는다. 기본 제한은 시뮬600초다. `--duration 60`은 연결·기록 확인용 짧은 검사이며 정식 600초 비교와 합산하지 않는다. 중간에 멈추면 `Ctrl-C`를 사용하고 중단 기록을 보존한다.

보드에서는 실행 선택, 평가용 장면 위 실제 이동 궤적, 실제 Gazebo RGB, 등록 ID와 완료 후 실제 1.5m 접근, 엄격 판정·범퍼 횟수를 확인한다. 배경은 평가용 정답 배치이며 SLAM 지도 커버리지 화면은 아니다. SLAM 지도를 직접 보고 싶으면 별도 터미널에서 같은 작업공간과 `ROS_DOMAIN_ID=174`를 source한 뒤 RViz에서 `/map`을 구독한다.


새 무작위 배치는 벽 몰딩·기둥·책상까지 포함한 실제 시뮬 도형에서 마커를 볼 수 있는 자리가 있어야 사용합니다. 같은 Jazzy 환경에서 비교할 때는 정책 두 개에 같은 시드와 마커 수 모드를 주세요. [새 배치의 마커 가림 검사](../../experiments/2026-11-22_random_exam/fixture-visibility.json)

## 4. 어디를 고치면 되는가

수정할 실제 소스는 저장소 루트의 `src`가 아니라 선택한 작업공간의 아래 경로다.

```text
~/simul-ws/my-policy/src/osp_mission/osp_mission/
```

| 확인할 기능 | 실제 모듈 |
|---|---|
| 미션 시작·Nav2 액션·실패 처리·종료 | `mission.py` |
| Mixed 전체 선택 흐름과 마커 접근 연결 | `phased_hybrid.py` |
| 내부 물체와 벽 목표의 선택 | `center_wall.py` |
| 관찰 위치·미관찰 표면·frontier 후보 | `edge_policy.py` |
| 카메라 표면 관찰 메모리 | `surface_coverage.py` |
| 상자 면과 관찰 이력 | `box_views.py`, `tracked_faces.py` |
| 약한 벽 마커 단서 | `small_wall_cues.py` |
| ArUco 판독·시간/TF 정합 | `detector.py`, `camera_work.py` |
| 6분 후 전 방향 관찰 후보 | rescue 작업공간의 `rescue360.py` |

먼저 해당 함수가 실제 실행 경로에 연결돼 있는지 launch와 호출부를 추적한다. 파일이 있다는 이유만으로 실행 중이라고 판단하지 않는다. Python은 symlink 설치지만 변경 반영을 명확히 하려면 아래처럼 빌드하고 새 회차에서 실행한다.

```sh
cd ~/simul-ws/my-policy
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select osp_mission
```

실행 중인 회차에는 코드를 바꾸지 않는다. 수정 전 원본·패치와 버전을 저장하고, 재실행 후 `protocol.json`의 작업공간·소스 해시를 확인한다. 실험이 끝나면 변경한 파일을 저장소의 별도 `policy_candidates/`에 재현 가능한 패치·설정으로 정리한다. 실행 로그와 영상에 어떤 커밋·패치를 사용했는지도 남긴다.

## 5. 같은 다섯 맵 비교

시드는 112601,112602,112603,112604,112605다. 정식 비교에서는 각 seed에 기준판과 후보판을 같은 600초·카메라480×480·5FPS·출발 자세로 차례로 실행한다. 예를 들어 Ubuntu 저장소 루트에서:

```sh
for seed in 112601 112602 112603 112604 112605; do
  bash team-sim.sh run ~/simul-ws/baseline "$seed"
  bash team-sim.sh run ~/simul-ws/my-policy "$seed"
done
```

실패한 회차를 지우고 성공할 때까지 다시 돌리지 않는다. 여러 가설을 한꺼번에 바꾸기보다 “카메라가 안 본 서쪽 벽이 왜 선택에서 빠지는가”처럼 한 질문부터 검증한다. 이 다섯 장면은 이미 본 개발 배치다. 여기에서 튜닝한 뒤에는 사용하지 않은 장면에서 추가 검증해야 한다.

## 6. 결과와 팀 공유

기본 저장 위치는 `~/.local/share/simul-team/results/`다. 실행할 때 `RUN`으로 정확한 새 경로를 출력한다. 다른 디스크를 쓰려면 보드·setup·run 세 터미널 모두 같은 `TEAM_SIM_HOME` 값을 지정한다.

| 파일 | 확인할 내용 |
|---|---|
| `protocol.json`, `policy.patch` | 실제 작업공간·소스/설정 해시·장면 지문·카메라·시간 제한과 기준판 대비 수정 내용 |
| `metrics.json`, `audit.json` | ID 등록, 등록 후 독립 실제1.5m 접근, 근거리 촬영 증거, 지도 오차·엄격 성공 |
| `headings.csv`, `trajectory.csv` | 독립 실제 이동 궤적·시각·그때 등록된 ID |
| `camera/`, `frames.csv`, `mission.mp4` | 실제 RGB와 시각, 완료 후 영상 |
| `native/` | 정책 결정·관찰 메모리·SLAM 지도·마커 저장 결과 |
| `launch.log`, `observer.log`, `interruption.json` | 실행 실패·준비 지연·중단 원인 |

엄격 성공은 정확한 세 ID, 등록 후 각각 실제1.5m 접근, 기존 세 장1.3m 근거리 촬영 증거, 지도 오차0.25m 이내, 범퍼0을 함께 요구한다. 등록만 했거나 목표 근처를 지나간 것을 완주로 세지 않는다. 미완주도 평균 시간에 포함하고 시뮬 시간과 실제 시간을 나눠 보고한다. 기본 실행은 Nav2 안전 설정·footprint·inflation을 보존한다.

PR에는 다음을 남긴다.

- 실패 영상의 실행 ID·시각과 확인한 원인
- 수정 버전·커밋·패치와 변경한 설정
- 같은 맵의 벽/상자 ID 등록·실제 접근·엄격 완주·시간·거리·범퍼 비교
- 아직 확인하지 못한 조건과 기존보다 나빠진 회차

큰 영상·생성 지도·ROS 로그·build/install은 Git에 넣지 않는다. 코드·설정·요약·경로 그림을 PR에 남기고 영상은 허용된 Release 자산으로 공유한다. 실물은 별도 준비·구동 지시가 있을 때만 사용한다.

## 7. 팀원별로 맡을 질문

- 벽 담당은 정보량이 적은 미관찰 벽이 후보 생성·필터·점수·취소 중 어디에서 빠지는지 본다.
- 상자 담당은 미관찰 면을 건너뛰거나 관찰한 상자에 계속 재방문하는 경우를 본다.
- 마커 담당은 먼 거리·화면 가장자리·스쳐가는 영상에서 후보, ID 판독, 위치 추정, 접근이 어디에서 끊기는지 본다.
- 복구 담당은 막힌 목표를 반복하거나 이동 가능한 후보를 잘못 거부하는 경우를 본다. 실제 Nav2 경로를 확인하기 전에는 도달 가능하다고 단정하지 않는다.

## Codex에 이어 맡길 요청

> docs/team-policy-lab/README.md와 AGENTS.md를 읽고 내 Ubuntu 24.04·ROS2 Jazzy에서 team-sim.sh로 현재 시뮬을 실행해줘. Mixed v1.5.1-balanced는 기준판으로 보존하고 개인 작업공간의 별도 버전에서 개선해줘. 공유 Mac 경로에 의존하지 말고 실제 source·launch·설치 모듈과 로그를 확인해줘. 실행 중인 스테이지는 수정하지 말고 새 회차에 적용해줘. 카메라 미관찰 표면과 지도 정보량을 구분하고, 같은 seed 다섯 장면에서 벽·상자별 마커 등록과 실제1.5m 접근, 완주·시간·거리·범퍼를 비교해줘. 안전 검사는 유지하고 정답을 정책에 넣지 마. 실패도 보존하고 결과가 나오기 전에 개선됐다고 단정하지 마. 실물은 구동하지 마.

## 확인한 범위

2026-11-22 기존 VM 작업공간과 다른 빈 폴더에 기준판을 새로 설치·빌드했다. 소스 지문과 두 C++ 산출물의 동결 VM 일치, 장면 무결성, 선택 작업공간·공식 ROS 의존성 확인, 관측기 import와 로컬 보드 HTTP 검사를 확인했다. 다른 native 벤치가 없는 상태의 새 실행기로 시뮬60초 주행·8.36m 이동·480×480 5FPS 영상300장·완료 MP4와 로그 저장·범퍼0을 확인했다. [실측 설치 검증](../../tools/team_sim/validation.json). 이 60초 검사는 세 마커 완주에는 미달했으며 600초 정책 비교에 넣지 않는다. 이 설치 검증을 새 정책의 성능이나 팀원 각 기기의 설치 성공으로 확대하지 않는다.

[단독 설치 검사의 실제 RGB 영상](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/team-jazzy-installation-check-1122__team-team-portable-balanced1122-track-m112601-1795277031081303114.mp4). 제한60초의 설치 검사이며 세 마커 완주에는 미달했다. 첫 동시 부하 검사와 단독 재검사는 별도 파일로 보존했다.
