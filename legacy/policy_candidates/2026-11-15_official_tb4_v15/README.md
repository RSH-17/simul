# Central LiDAR surface fallback candidate

Recorded official TurtleBot4 U-shape RGB/LiDAR samples expose broad-cone plane rejection when the observed surface and background enter the same fit. This candidate branches from official v11, retaining its bounded exposure-time buffer and the v8 camera-distance parent.

Successful original +/-0.07 rad fits remain unchanged. Only a plane residual above 6 cm enables a +/-0.03 rad retry. At least four consecutive valid beams, two on either side of the viewing direction, are required; a central gap, one-sided support, mixed center, stale scan, nearly parallel plane, or invalid range is rejected. Existing identity, pixel, map-frame and approach rules are preserved. `lidar_center_fallback_calls` counts function calls, including audit rechecks, not frames or recognized IDs.

Fresh workspace: `training_robust/native90/official_tb4_v17/runtime`. Build and 429 runtime tests passed, including angular resolution/phase variants, scan-boundary wrapping, gaps, stale data and preserved nominal ranges. The evaluator/replay checks passed 11 tests.

Two nonoverlapping segments contain 64 frames and 65 decoded observations from one U-shape development run. All 65 range-function comparisons passed. Among 54 size-eligible range attempts, the original returned 31 ranges and this candidate returned 54; all originally successful values were preserved. The 23 recovered ranges have a maximum planar sensor-geometry error of 1.46 cm in an independent evaluator. Nine recovered observations reach 18 pixels; a returned range does not itself confirm an ID. The remaining recovered observations can only be provisional cues under the existing detector rules.

Truth is read only by the separate evaluator after the predictions are sealed. It is never passed to the detector or policy. The evaluator uses bounded interpolation of base XY/yaw and the verified official mount, not SLAM map coordinates. These correlated development frames do not establish generalization, native completion, collision-free approach, or hardware accuracy.

Native execution is not queued. Preserve the active U comparisons and queued pocket departure screens. Dock-map, pocket-continuation, frontier-area, deadline and map-frame face-sweep changes are not merged. No official robot files, odometry or policy weights change.
