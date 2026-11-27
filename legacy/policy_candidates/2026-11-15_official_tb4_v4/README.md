# Admit rotational SLAM updates at crowded starts

A new official C-shape run undocked 35 cm and rotated nearly 3 rad, but its exported SLAM graph retained only two scans. The policy then repeatedly found no body-clear path. Finer rasterization of those two scans did not produce a goal, so resolution alone did not repair this input.

This candidate retains the official robot and the 5 cm map. It enables the installed SLAM Toolbox `check_min_dist_and_heading_precisely` option so sufficient rotation can admit a scan even without sufficient translation. All distance/heading thresholds, collision checks and policy weights remain unchanged. See the [upstream implementation](https://github.com/SteveMacenski/slam_toolbox/blob/jazzy/src/slam_toolbox_common.cpp) and [upstream change](https://github.com/SteveMacenski/slam_toolbox/pull/808).

This changes SLAM scan selection and may affect rotational localization and map quality. It requires new native validation, including independent pose error and full marker results. It is not a production or hardware-qualified fix. Original running/queued workspaces remain frozen.

Build with `python3 policy_candidates/2026-11-15_official_tb4_v4/build.py --workspace /absolute/new/workspace --test`.

Completed 90-second native result: zero marker IDs, zero contacts, approximately 35 cm of undocking motion and no subsequent navigation goal. Rotational updates increased the live retained scan count to 17 but did not resolve the start deadlock. The next diagnostic isolates missing positive-infinite no-return rays; see the repository diagnostic snapshots.
