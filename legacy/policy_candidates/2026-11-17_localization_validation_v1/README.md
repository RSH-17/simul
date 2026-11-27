# Odometry observation and final-coordinate validation candidate

Parent: official v20 (fixed exploration policy). This candidate changes the final landmark postprocessor and adds a **file-only** odometry/IMU observer in simulation. It does not feed fused odometry to SLAM, Nav2 or the robot, and publishes no TF or commands. The upstream official robot and existing live odometry authority stay intact.

The observer integrates signed wheel-pose increments with past base-axis IMU rates. IMU angular velocity is transformed using observed base-to-sensor TF; absolute IMU orientation and reported wheel twist are unused. Timestamp reversals, conflicting duplicates, missing streams and acquisition gaps latch a fault rather than fabricating movement or resetting position. An explicit gyro bias may be supplied, but no hardware bias calibration is claimed. The streaming estimator agrees with the batch implementation within 5e-13 on two approximately 600-second U-shaped recordings. Those captures retained `complete=false` after a known shutdown exception; the independent auditor preserves that status and checks frozen input hashes, sensor schema, sequence, frame axes, passivity and final episode evidence before quarantined replay.

The final graph postprocessor retains the existing readable-frame and distinct-scan-node consensus checks. It additionally refuses to automatically replace a marker coordinate when the graph would move it by more than **0.25m**. It records the proposed coordinate, retains the original value, and marks it **requiring reobservation**. Retention does not establish agreement with the corrected final map. Smaller accepted changes pass only an internal temporal/consistency guard, not an absolute-accuracy proof; independent evaluation still decides correctness. Any successful native outcome with unresolved coordinate verification is rejected by the new supervisor. Legitimate large loop corrections may also need reobservation under this conservative development guard.

Offline original-U replay stops two large overwrites (1.28m and 1.51m shifts): retained coordinate errors are 0.056m/0.141m instead of 1.333m/1.649m, but both remain flagged unverified. C-shaped candidate errors of 0.658m/0.564m remain because their original localization was already wrong. This does not repair that localization or claim a new task success.

Fresh installed runtime: **503 tests passed** (490 inherited plus 13 new); **8 supervisor checks passed**, including exact v2 map-frame output-profile compatibility, full two-native resource limits, and rejection of unresolved coordinates. Six 600-second paired development trials are prepared on U/C/zigzag scenes with fixed weights. Baseline and candidate U trials are actually running. Native localization-control improvement, unseen-scene performance and physical hardware remain unverified.

```bash
python3 policy_candidates/2026-11-17_localization_validation_v1/build.py --workspace /absolute/new/workspace --test
```

See [the report](../../repository_snapshot/localization_validation_progress.json). Original failed frontier and queued rolling-goal experiments remain preserved; their frozen source and input files have not been edited. The new supervisor fixes output-profile compatibility only within its own process.
