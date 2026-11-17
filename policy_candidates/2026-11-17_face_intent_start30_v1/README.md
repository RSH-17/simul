# face_intent_v2: 30 cm exploration start exception

This hardware trial adapter changes only the exploration planner's connectivity
mask around the current robot pose. Set `planner_start_ignore_radius_m:=0.30`
to ignore occupied/unknown cells within that disk while connecting the current
pose to observation goals. Zero retains the frozen release behavior; values
above 0.30 m are rejected.

The published SLAM map, raw LiDAR, Nav2 costmaps, collision monitor, bump input,
and marker recognition remain unchanged. Observation destinations still require
known free cells with the original 0.32 m obstacle clearance. A proposed route
is not a physical clearance certificate: Nav2 can reject it or stop when the
original sensors show a real obstacle. The exception cannot guarantee a start
inside a truly blocked footprint.

This candidate uses the frozen `2026-11-13_face_intent_v2` weights and source,
plus the checked transform in `runtime_changes.py`. It does not include the
unrelated v20/v21 runtime. It is not promoted as a validated policy.

```bash
python3 policy_candidates/2026-11-17_face_intent_start30_v1/build.py \
  --workspace /home/ubuntu/face_intent_start30_ws --test
source /home/ubuntu/face_intent_start30_ws/install/setup.bash
export ROS_DOMAIN_ID=4 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch osp_mission robot.launch.py use_sim_time:=false rviz:=false \
  planner:=edge planner_start_ignore_radius_m:=0.30 \
  nav2_params:=/home/ubuntu/face_intent_start30_ws/install/osp_mission/share/osp_mission/config/nav2_robot_policy.yaml \
  policy_file:=/home/ubuntu/simul/policy_releases/2026-11-13_face_intent_v2/policy.json
```

The mission stays idle until `/mission/start`. All previous SLAM/Nav2/mission
instances in the robot domain must be stopped before this launch. Use a fresh
map after moving robot-mounted cables out of the LiDAR plane.

## Verification and physical trial

The complete staged runtime passed 356 tests on the Jazzy VM. Replaying three
recorded real-robot planning inputs at the actual 250 px camera calibration
changed candidate counts from 0 to 5, 0 to 3, and 0 to 2. The replay checks also
verified that the input maps were not changed and every destination retained
the original 0.32 m clearance. These are offline input replays, not new physical
completion results.

The physical trial additionally limits forward speed to 0.15 m/s, rotation to
0.5 rad/s, and uses a separate supervisor that requests `/mission/stop` on the
first bump. This speed profile changes no costmap or collision-monitor settings.
See `repository_snapshot/physical_robot_trial_20261117.json` for timestamped
partial observations. The three-marker task has not been verified on hardware.
