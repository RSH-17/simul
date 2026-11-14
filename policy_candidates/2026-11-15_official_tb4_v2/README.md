# Official simulator sensor-frame integration

Retains the official robot and the [v1 task wrapper](../2026-11-15_official_tb4_v1). Adds the identity transform from the official IMU link to its Gazebo sensor header frame, and recognizes the official `dock_state_publisher` in the simulation interface profile. The odometry publisher/TF ownership and fresh actuator-stream checks remain required.

394 runtime tests passed. The native integration screen completed with the official spawn, 8 obstacles, hidden markers, and independent actual-position evaluation: 2.749 m actual path, zero contacts, successful undocking, and saved map/pose graph. All input configuration, floor and official-description checks passed. It is a 60-second integration test, not full-task training or hardware validation. The previous failed screen is preserved.

```bash
python3 policy_candidates/2026-11-15_official_tb4_v2/build.py --workspace /absolute/new/workspace --test
source /absolute/new/workspace/install/setup.bash
ros2 launch osp_mission simulation.launch.py scenario:=/absolute/scenario.yaml
```

The default view starts Gazebo GUI and RViz. Batch evaluation uses `headless:=true rviz:=false`; robot and sensors remain the official ones. The official model always spawns its dock, so scene generation must leave physical room for the dock and undocking motion. The built-in simulator's sensor rates and Create 3 encoder estimator are not identical to physical firmware; use hardware `CameraInfo`, odom, and TF when transferring the policy.
