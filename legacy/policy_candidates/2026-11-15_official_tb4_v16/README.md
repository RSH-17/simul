# Official TurtleBot4 candidate v16

Parent: `2026-11-15_official_tb4_v13`. The completed parent escaped both 90-second departure scenes without contacts (29.172s sideways, 23.868s narrow), but the sideways trajectory repeatedly returned through its entry after reaching the larger room. Neither run completed the marker task.

Only `start_waypoints.py` changes: frontier evidence, not only its candidate endpoint, must lie outside the current normal-clearance component before applying the existing 3m spatial gain filter. This removes gain leaked from already reachable room edges. It does not impose a room-size cap or forbid useful backtracking into smaller rooms. Body clearance, unknown-space margins, official robot, odometry, map resolution, Nav2, weights and marker processing remain the parent versions.

Validation: 435 runtime tests and 7 supervisor checks passed. Synthetic rotated maps at 2.5cm/5cm resolution reproduce the old return-to-entry bug and preserve return when there is actual frontier evidence beyond the corridor. Replaying 34 recorded decisions reproduces the original generation/ranking. All 13 initial transit candidate sets are unchanged; seven sideways post-exit decisions switch from forced transit to normal viewpoints. This is not a new trajectory or success-rate result.

Two same-scene 90-second native trials are queued after the ongoing zigzag pair in `official_frontier_evidence_screen_v1`, using the existing input gates, independent evaluator and two-native limit. The supervisor then resumes the preserved original campaign. This candidate is not production or hardware validated. Sensor-range, map-frame look, deadline and supported-unknown candidates remain separate.
