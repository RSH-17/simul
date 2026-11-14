# Crowded start v4

The preceding native comparison escaped the 60 cm sideways-start corridor in 26.724 simulation seconds with zero contacts, but stopped after 1.13 m in the 55 cm corridor. Those are two development cases, not full-task or hardware qualification.

This candidate defers the initial camera sweep when the observed map shows a body-clear narrow footprint. It continues 45–70 cm observed-free transit even when a distant normal-clearance goal first becomes visible. Unknown cells stay blocked and the body clearance remains unchanged. Marker IDs and scenario truth are never supplied to the policy.

Simulation readiness now requires the actual `motion_control` subscriber, its actuator command publisher, the `diffdrive_controller` subscriber, and recent command receipts. An evaluator-only subscriber cannot make the robot ready. The simulated motion-control process uses UDPv4, as the simulated Nav2 stack already does. Hardware launch and firmware odometry remain unchanged. This is not proof that shared memory caused every earlier startup failure.

The built runtime passed 390 tests. Four supervisor negative controls and one live ROS hardware-mode sensor-loss test also passed. A fresh paired native comparison is running; neither native v4 acceptance nor production promotion has been claimed.

```bash
python3 policy_candidates/2026-11-14_crowded_start_v4/build.py --workspace /absolute/new/workspace --test
python3 training_robust/run_crowded_start_regression_v4.py --output /absolute/new/results --candidate /absolute/candidate/workspace --baseline /absolute/baseline/workspace
```

The evaluator permits at most two native runs with the existing 5 GiB memory-high, 6 GiB memory-max, and 512 MiB swap limits. Every trial must pass the independent actuator probe in addition to sensor, configuration, floor, and transport checks. It uses domains 220/221, outside this host's ephemeral port range 32768–60999, following the [ROS 2 domain guidance](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Concepts/Intermediate/About-Domain-ID.rst). This avoids a port-overlap risk; it does not establish the cause of previous failures.

Original experiments and queued runtimes remain frozen. Results must distinguish corridor escape from all three correct IDs, actual 1.5 m approaches, correct map locations, zero contacts, and completion within 600 seconds.
