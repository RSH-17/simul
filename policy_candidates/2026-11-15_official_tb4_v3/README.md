# Official TurtleBot4: finer observed occupancy grid

This candidate retains the unmodified installed official robot description, sensor and control stack from v2. Only the shared SLAM map resolution and launch default change from 5 cm to 2.5 cm. Robot radius, occupancy threshold, collision monitor, and physical clearance margin are unchanged.

Two earlier 55 cm failures produced no goals on the original 5 cm maps. Offline reconstruction of the same retained scans at 2.5 cm produced a body-clear short transit goal. That is evidence for this comparison, not successful counterfactual movement.

The new paired native diagnostic uses a 55 cm corridor with the official dock oriented toward the closed end. This leaves room for the standard reverse undock toward the exit. It is a new case (1196100000), not the old sideways-start case. Both arms use identical scene and policy weights, the official robot, and 90 simulation seconds. Independent truth evaluates escape and contacts; the policy receives only its sensors and map.

No full-task, hardware or production qualification is claimed. Build into a new workspace with `python3 policy_candidates/2026-11-15_official_tb4_v3/build.py --workspace /absolute/new/workspace --test`.

Validation: 394 runtime tests and 15 supervisor/resolution tests passed. Both native comparison services started; results are pending.
