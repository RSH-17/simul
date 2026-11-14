# Frontier reward supported by unknown-region extent

A recorded post-departure decision returned toward a previously passed area.
Its local free/unknown boundary contained many small raster gaps between scan
rays. The old frontier feature saturated at 5.0; filtering those gaps reduced
that goal's feature to 1.5. Re-ranking the original eligible candidates with
the same poses, costs, revisit penalties and weights instead selected a goal
ahead of the robot. Two of three reproduced ordinary selections changed.
This is offline evidence; it does not establish better native performance.

Only `edge_policy` post-departure information gain uses the new helper. Unknown
components must have area at least `pi * 0.2² m²` and contain a point at least
0.1 m inside the component. A narrow entrance connected to a larger unknown
room remains eligible. These fixed heuristics are smaller than a robot body
and are not claims of traversability. Unknown cells remain unknown; the
observed map, collision checks and body clearance are unchanged. Crowded-start
waypoints still use the original gain function. No hidden targets enter policy.

The candidate inherits calibrated face viewpoints from official v8. All other
runtime files and fixed weights match v8; the comparison preparer rejects
unrelated changes. A final run of all 416 runtime tests passed, plus five
supervisor tests covering full duration, frozen paired inputs, contamination
exclusion and the two-native concurrency limit.

```bash
python3 policy_candidates/2026-11-15_official_tb4_v9/build.py \
  --workspace /absolute/new/workspace --test
```

`run_official_frontier_comparison.py` prepares six full 600-second trials:
baseline/candidate on C narrow, U dense and zigzag combined training scenes.
All three correct IDs, actual <=1.5 m approaches, <=0.5 m map errors, zero
contacts and <=600 seconds remain required. Current departure screens finish
unchanged; the preserved matrix resumes after these finite trials. This is
fixed-weight algorithm development, not coefficient training, holdout testing
or promotion to production. The new candidate had no native qualification
before the experimental comparison; its parent passed the departure screen.
