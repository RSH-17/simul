# Keep simulator no-return rays in SLAM

The official C-shape run still stalled after enabling rotation updates: its live graph contained 17 scans, but an unknown wedge blocked the forward body-clear path. A live raw scan contained 28 positive-infinite returns out of 640. Karto excludes those rays when rasterizing. Normalizing the 455 positive-infinite returns in that retained graph, with fixed recorded poses and the dock keepout preserved, produced one 65 cm forward goal. This is an offline counterfactual, not actual movement or a completed task.

This candidate inherits official v4 and adds a **simulation-only SLAM input adapter**. It maps positive infinity to 11.95 m and sets Karto's mapping threshold to 11.9 m, below the substituted value and below the sensor's 12 m maximum. These rays clear space up to the threshold; they are not obstacle endpoints or scan-matching features. NaNs, negative infinity and finite measurements remain unchanged. The raw LiDAR, detector, Nav2 collision inputs, known dock keepout, body margin, official URDF and policy weights stay unchanged.

`robot.launch.py` defaults to no adapter and rejects enabling it without simulated time. The official simulation launch enables it explicitly. Physical RPLIDAR dropout/no-return semantics have not been verified; this candidate does not claim to fix real hardware.

References: [REP-117 distance semantics](https://github.com/ros-infrastructure/rep/blob/master/rep-0117.rst), [Karto AddScan and point filtering](https://github.com/SteveMacenski/slam_toolbox/blob/jazzy/lib/karto_sdk/include/karto_sdk/Karto.h).

Build: `python3 policy_candidates/2026-11-15_official_tb4_v5/build.py --workspace /absolute/new/workspace --test`.

Validation: 400 runtime tests and 4 effective-input probe tests passed. The 90-second native diagnostic on the same physical C-shape completed with 0.353 m of undocking movement, zero contacts and zero IDs; it did not escape. Its live scan adapter and all six input gates passed. A separate 2.5 cm successor is being tested; this 5 cm stage is preserved. No production, full-task or hardware qualification is claimed. Original running and queued source stages are preserved.
