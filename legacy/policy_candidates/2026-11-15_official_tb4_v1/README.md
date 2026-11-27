# Official TurtleBot4 simulator integration

This replaces the task's separate robot spawning, model editing, control configuration, and sensor bridges with the installed official `turtlebot4_gz_bringup/launch/turtlebot4_spawn.launch.py`. The official robot URDF, sensors, wheels, controllers, docking, and bridges are unmodified. A live canonical XML comparison confirmed that the published robot description exactly matches the official Xacro output.

The task still supplies its generated world, obstacles, hidden markers, mission policy, SLAM/Nav2 parameters, recording, and independent evaluator. Gazebo server rendering is selected through the standard `--render-engine-server ogre2` option and software rendering for this host. The world uses the standard IMU system and forwards the official IMU topic; stock bringup does not bridge it to ROS. This is an application wrapper around official robot bringup, not an unchanged top-level official launch.

The simulation input contract uses the actual official camera (320x240), 640-beam LiDAR, and nominal 62 Hz odom/IMU. The existing hardware input profile is retained separately. Mission start reads fresh `dock_status` and only requests undocking when the robot reports being docked. This prevents an already-undocked response from terminating the mission. Missing/stale docking state still blocks start.

394 runtime tests passed. The first native screen confirmed the exact official robot description but could not pass preparation: it required the IMU sensor-frame transform and recognition of the official dock state publisher. Those changes are staged separately in [v2](../2026-11-15_official_tb4_v2). No task success, hardware equivalence, or production promotion is claimed. [Transition evidence](../../repository_snapshot/official_tb4_transition.json).

```bash
python3 policy_candidates/2026-11-15_official_tb4_v1/build.py --workspace /absolute/new/workspace --test
```
