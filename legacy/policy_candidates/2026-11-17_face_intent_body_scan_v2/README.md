# face_intent_v2 hardware adapter: body scan and mission outcomes

Parent: `2026-11-17_face_intent_start30_v1`. This adapter retains the 30 cm
exploration start exception and the original 32 cm destination clearance.

## Changes

- Raw `/scan` is preserved. A separate `/scan_self_filtered` removes a return
  only when its exact-time TF endpoint is within an 18 cm disk **and** more
  than 5 mm inside the configured 18.9 cm base footprint polygon. The observed
  points near `(-0.115, -0.134)` m satisfy both conditions. Outside points,
  invalid ranges, and no-return measurements are not converted to free space.
- SLAM, perception, mission recovery, and Nav2 costmaps use the filtered scan.
  The independent collision monitor and physical bump input retain the raw
  sensors. A footprint mismatch refuses the adapter rather than enlarging its
  mask. Missing TF withholds the filtered scan, allowing freshness checks to
  stop planning/control while the raw stream remains available.
- Mission status publishes `termination_reason` and `task_success`. A stopped
  or timed-out run cannot be displayed as marker-task success. Elapsed time is
  frozen at `finish()`, including while results are saved.

The runtime passed 366 tests on the Jazzy VM. Dashboard outcome presentation
has separate tests. These checks do not establish physical task completion.

## Build and run

```bash
python3 policy_candidates/2026-11-17_face_intent_body_scan_v2/build.py \
  --workspace /home/ubuntu/face_intent_body_scan_v2_ws --test
source /home/ubuntu/face_intent_body_scan_v2_ws/install/setup.bash
candidate="$PWD/policy_candidates/2026-11-17_face_intent_body_scan_v2"
python3 "$candidate/deployment/prepare_hardware_profile.py" \
  --input /home/ubuntu/face_intent_body_scan_v2_ws/install/osp_mission/share/osp_mission/config/nav2_robot_policy.yaml \
  --output /home/ubuntu/face_intent_body_scan_v2_ws/nav2_hardware_profile.yaml
export ROS_DOMAIN_ID=4 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$candidate/deployment/fastdds_tb4_wired.xml"
ros2 launch osp_mission robot.launch.py use_sim_time:=false rviz:=false \
  planner:=edge planner_start_ignore_radius_m:=0.30 \
  nav2_params:=/home/ubuntu/face_intent_body_scan_v2_ws/nav2_hardware_profile.yaml \
  policy_file:="$PWD/policy_releases/2026-11-13_face_intent_v2/policy.json"
```

This Ethernet profile explicitly advertises VM address `192.168.185.6`; adjust
it for a different host. The sysctl template was installed on the VM before
restarting its ROS processes. Kernel and Fast DDS receive buffers were raised
after ongoing UDP receive-buffer errors were observed. The template preserves
the collision monitor and both costmap geometries; forward speed is capped at
0.25 m/s and rotation at 0.5 rad/s. The mission remains idle until its start
service is called. Check active sensors and a clear bump signal before start.

## Camera and physical observations

The robot's OAK-D override at `/etc/turtlebot4/oakd_osp.yaml` uses 10 FPS and
`rgb.i_update_ros_base_time_on_ros_msg: true`. The camera include in
`/etc/ros/jazzy/turtlebot4.d/standard.launch.py` points to that override; its
original was backed up with suffix `.osp_backup_20261117`. Runtime parameters
were applied and the camera restarted. Persistence is configured but has not
been re-tested through a full robot reboot.

In the earlier 0.15 m/s trial, 648 straight-command samples had median request
0.15 m/s and median measured odometry speed 0.146 m/s. This explains the slow
straight segments. It is distinct from navigation recovery stalls, which
also occurred. One subsequent start stopped after 1.7 s because the independent
trial supervisor observed a bump; this was not marker completion or VM exit.
After the operator reported leaving the dock, `is_docked=false` and a clear
bump signal were verified before restarting.

A fresh map is required after self-reflection changes. Physical markers have
not been validated or completed by these trials. The 18.9 cm footprint is a
configured base model, not a new measurement of the laptop/payload envelope.
Recovery failure memory and global-path quality checks remain separate work.
