# Nav2 frontier and surface coverage candidate

This task policy combines online connected-frontier exploration with greedy camera-surface coverage. Nav2 executes each selected `NavigateToPose` goal. This is a custom task planner, not an installation of `opennav_coverage` or Fields2Cover.

Frontiers are free/unknown interfaces with substantial connected unknown support. Goals stand 0.35–0.9m inside observed free space, retain 0.32m clearance and a free sightline, and require a finite collision-grid path. A bent cluster uses spread boundary anchors rather than an unsafe centroid. Camera coverage uses feasible unobserved boundary faces, including opposite sides of thin obstacles. Nearby readable surface opportunities are handled before leaving for another frontier; the remaining-time selector returns to coverage near the deadline. Planned goals never become sensor coverage credit.

The candidate combines v20's cumulative navigation budget with v19's feasible face proposals. Narrow-start body-clear steps, marker follow-up and approach overrides, official robot/sensor models, odometry, SLAM loop settings and collision margins remain unchanged. No target identity, placement, scenario geometry or evaluator truth enters this policy. Fixed weights are retained; this is an algorithm comparison rather than coefficient training.

Validation: **513 installed runtime tests passed** and **6 new supervisor tests passed**. Six selected stored C/U/zigzag maps exercised frontier and surface candidate generation, including a narrow passage that retained its transit action. The native comparison has six 600-second development trials, v20 versus this candidate on U/C/zigzag scenes, with at most two Gazebo processes and the existing 5GiB/6GiB/512MiB resource limits. Full success still requires three accurate IDs, actual approach within 1.5m, each map location within 0.5m, zero contacts and completion within 600 seconds. Improvement, holdout generalization and hardware performance are unverified.

Build in a fresh directory:

```bash
python3 policy_candidates/2026-11-17_frontier_coverage_v1/build.py --workspace /absolute/new/workspace --test
```

See [the evidence and experiment report](../../repository_snapshot/frontier_coverage_candidate.json). Existing experiments are preserved. A new supervisor handles missing historical systemd units without changing frozen older drivers.
