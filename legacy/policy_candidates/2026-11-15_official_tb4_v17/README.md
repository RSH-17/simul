# Official TurtleBot4 integrated candidate v17

This candidate combines v16 departure/dock repairs with camera-calibrated inspection distances (v8), exposure-time TF buffering (v11), the opt-in deadline selector (v12), map-frame observation turns (v14), and the supported central LiDAR fallback (v15). `mission.py` merges the dock and deadline branches, retaining both status fields and the dock map behavior. All source layers are checked against their recorded hashes before a fresh build.

The accompanying `policy.json` retains the five existing weights and enables `settings.deadline_aware`. This is source integration, not retraining. Official robot files, odometry, footprint margins, Nav2 configuration and SLAM loop thresholds remain those of v16. No ground truth enters the policy.

Validation: **478 runtime tests passed** in a fresh installed workspace. The installed mission loads the unchanged weights with the deadline flag. All **13 recorded early-transit candidate sets** are identical to v16 in separate planner processes. These checks do not establish combined mission success, later trajectory behavior, recognition accuracy or hardware transfer.

Built in `training_robust/native90/official_tb4_v19/runtime`; **not queued, not production, not hardware validated**. The isolated v16 native departure tests remain queued after the existing zigzag pair. SLAM loop-gate comparisons are a separate diagnostic and are not silently incorporated.
