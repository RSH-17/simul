# Map rasterization candidate, held for official simulator transition

Two failed 55 cm native runs were reconstructed from their saved SLAM datasets. At 5 cm resolution, every map cell matched the recorded final planning input and both inputs produced zero goals. Rebuilding the same corrected scans at 2.5 cm resolution produced 25 cm start clearance and one 67.5 cm body-clear transit goal in both cases. No robot radius, obstacle occupancy threshold, or physical safety margin was reduced. See [offline evidence](../../repository_snapshot/crowded_map_resolution_replay.json).

This candidate changes only the shared SLAM map resolution to 2.5 cm, including the hardware launch default. It built and passed 390 tests. **It has not run a native benchmark or been validated on hardware.** The user requested prioritizing the official TurtleBot4 simulator base; this candidate remains preserved and unqueued. Offline rasterization is not a counterfactual mission result.

Build with `python3 policy_candidates/2026-11-14_crowded_start_v5/build.py --workspace /absolute/new/workspace --test`. The prepared supervisor source is `training_robust/run_crowded_start_regression_v5.py`; no campaign was created with it.
