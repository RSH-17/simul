# Rolling exploration destination experiment

Parent: `2026-11-17_frontier_coverage_v1`. Only mission wiring, the new rolling-goal module and its tests differ. Frontier/coverage generation and ranking, fixed weights, odometry, Nav2/SLAM settings, sensors and collision margins stay identical between arms.

While an ordinary exploration navigation is active, reconsider destinations every **8 simulation seconds**, after a **12-second minimum target hold**. A new goal must improve utility by **40%** and at least **0.01**, including **5 seconds** of switch cost. Known occupied/unknown/body-unsafe destinations may be abandoned for a positive-utility reachable alternative. Missing a sampled incumbent alone is insufficient evidence to switch. Do not interrupt within 0.65m of arrival, narrow-start transit, QR follow-up/approach, or an active near-pass inspection. Enforce 20 seconds between switches, a 45-second cooldown around abandoned odometry anchors, at least 0.8m destination separation and at most six switches per episode. Parameters are predeclared development settings, not an optimal-policy claim.

The existing Nav2 action is cancelled first. Only its cancellation acknowledgement returns control to the normal planner, which selects from the newest map. No second navigation goal is sent before acknowledgement. A completion race, contact or sensor cancellation stays with the original handlers. Interrupted goals receive neither successful coverage credit nor failed-view blacklisting. Hypothetical filtering uses a copy of mission memory. `rolling_goal.jsonl` records evaluations, cancellation requests/acknowledgements and subsequent goal selection; live status exposes checks, errors and switch count.

**530 runtime tests passed** in a fresh installed workspace, `training_robust/native90/rolling_goal_v2/runtime` (513 inherited plus 17 new checks). New checks cover gains, cooldowns, clock scheduling, missing candidate samples, near-arrival protection, observed blockage, marker/escape protection, action acknowledgement races and a real runtime reassessment with mocked sensor/TF inputs. The first prototype build remains preserved separately in `rolling_goal_v1`. Five supervisor checks confirm preservation of ongoing native work and the extra live-configuration gate.

Six full **600-second paired trials** are queued after all six existing frontier/coverage trials and their coordinator have actually ended: fixed versus rolling destinations on U/C/zigzag development scenes. The existing two-native host limit and 5GiB/6GiB/512MiB resource limits are retained. The independent full-task rubric still requires three correct IDs, actual approach within 1.5m of each marker, map error at most 0.5m, zero contacts and completion within 600 seconds. Native improvement and hardware behavior are unverified.

The five adjacent decision-snapshot checks from the ongoing U trial retained the preceding destination. These snapshots were recorded at mission decisions, not mid-navigation ticks, so this is a conservative-filter diagnostic only. It does not establish a trajectory improvement or time saved.

```bash
python3 policy_candidates/2026-11-17_rolling_goal_v1/build.py --workspace /absolute/new/workspace --test
```

See [the experiment status and evidence](../../repository_snapshot/rolling_goal_experiment.json).
