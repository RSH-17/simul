# Observed corridor continuation candidate

The completed sideways departure screen stayed inside a normal-clearance pocket, despite a body-clear observed corridor extending outside it. The original transit helper returned early for any starting normal component of at least 0.5 m².

This candidate branches from official v10 (dock map repair). It changes only `start_waypoints.py` and adds regression tests. In a normal-clearance pocket, informative endpoints must be at least 0.70 m outside that component. The full path still requires observed free space, the original 0.189 m body radius, cell-extent allowance and 1 cm margin. Waypoints remain 0.45–0.70 m long and do not claim camera inspection credit. Wide connected rooms and immediate wall fringes retain normal observation planning.

Fresh runtime: `training_robust/native90/official_tb4_v15/runtime`. All 425 runtime tests passed, including two resolutions and four corridor orientations. Six supervisor checks passed. An offline replay of six actual SLAM inputs generated transit candidates in five previously suppressed decisions; every path to its informative endpoint satisfied the unchanged recorded-map body margin. No future motion, success rate or physical contact result is predicted by this replay.

Two 90-second native regressions are prepared in `official_pocket_screen_v1`, waiting for the existing U pair without interrupting it. The coordinator resumes the remaining full-task comparison afterward. Neither these departure checks nor this candidate have established three-marker task success or hardware transfer.

Camera-distance, frontier-area, exposure-buffer and deadline candidates are not merged. The original policy weights, official robot model, odometry, Nav2 settings and task rubric remain those of the dock-repair parent.
