# 실물 TurtleBot4 주행 보드

기존 `dashboard/server.py`는 Gazebo 캠페인용입니다. 이 보드는 실물 로봇의
ROS 토픽을 직접 구독하며 주행 명령이나 `/mission/start` 호출을 하지 않습니다.
미션 상태, 출발 지도 셀, 충돌 감시, 원본/필터 LiDAR, RGB, 지도와 Nav2 경로를
1초마다 표시합니다. 데이터가 끊기면 마지막 수신 시각과 경고를 보여줍니다.

로봇과 같은 ROS 네트워크에 있는 Ubuntu/Jazzy 노트북에서:

```bash
cd /path/to/simul
source /opt/ros/jazzy/setup.bash
source /path/to/osp-v21/install/setup.bash
python3 dashboard/robot_board.py --domain 4 --host 0.0.0.0 --port 8766
```

Mac 브라우저에서 `http://<노트북-LAN-IP>:8766`을 엽니다. ROS 도메인이 4가
아니면 `--domain`을 바꾸고, 로봇 네임스페이스가 있으면 `--namespace tb4`를
추가합니다. 원본 스캔과 RGB 토픽이 다르면 `--scan`과 `--rgb`를 지정합니다.
기본 바인딩은 `127.0.0.1`이며, 외부 접근이 필요할 때만 같은 신뢰할 수 있는
LAN에서 `--host 0.0.0.0`을 사용합니다. 인터넷에는 공개하지 마세요.

`osp_mission` 설치 경로가 화면 아래에 표시됩니다. v21 필터를 확인하려면
`scan_self_filtered`의 마지막 수신 시각과 **몸체 내부 제거** 수를 확인하세요.
원본 스캔의 19cm 반사가 로봇 중심 기준 16cm 영역 밖이면 필터는 제거하지
않습니다. 출발 지도 셀이 계속 `start_not_free`라면 보드의 근접 LiDAR 화면과
지도/TF를 함께 살펴야 합니다. 저장된 점유 흔적을 제외하려면 새 SLAM 세션을
사용하세요.

보드는 ROS 2 Jazzy, `rclpy`, `nav2_msgs`, `cv_bridge`, OpenCV, NumPy가 필요합니다.
`/map`, `/scan`, `/scan_self_filtered`, `/mission/status`,
`/collision_monitor_state`, RGB 이미지, `/plan`, `/local_plan`, TF를 구독합니다.
실물에서 확인한 값과 시뮬레이션 평가 점수는 별개입니다.
