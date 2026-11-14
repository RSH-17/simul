# Official TurtleBot4 cumulative navigation budget candidate v20

The U-shaped integrated trials exposed an action lifetime bug: `Mission.send()` reset the 65-second timer every time a nearby-surface inspection or sensor recovery resumed the same destination. A target could therefore consume substantially more navigation time while Nav2 repeatedly recovered.

This candidate retains the existing **65 seconds of active navigation per selected target**. Cancellation acknowledgement pauses accounting; inspection turns, image dwell and sensor recovery waits do not consume navigation time. Resuming preserves the unused allowance. An exhausted resume sends no new action, gives no successful observation credit, and returns through the existing failed-view retry/replanning rules. Explicitly selecting a new target starts a new allowance. Non-navigation action timeouts stay unchanged.

Parent: integrated v17. Only `mission.py`, the new `navigation_budget.py` and its tests differ. Official robot files, odometry, SLAM/Nav2 parameters, geometry clearance, marker handling, candidate generation and the five fixed policy weights are unchanged. The candidate does not consume ground truth or scenario geometry.

A fresh installed workspace, `training_robust/native90/official_tb4_v23/runtime`, passed **490 runtime tests** (478 inherited plus 12 new). New checks exercise the actual mission callbacks through sensor cancellation and inspection cancellation/turn/dwell/resume, accumulated expiry, late cancellation, exhausted resume without publication, normal arrival, non-navigation timeouts and clock faults. These are code-level results, not native performance qualification.

Six 600-second development trials are queued after the preserved v17/v18 integrated campaign: v17 versus this candidate on the same U-shaped, C-shaped and zigzag scenes. The existing two-native limit, resource limits and full independent success rubric remain in force. Correct IDs for three markers, actual approach within 1.5m each, map error at most 0.5m each, zero contacts and completion within 600 seconds are still required. No holdout has been consumed and no production or hardware promotion is claimed.

```bash
python3 policy_candidates/2026-11-15_official_tb4_v20/build.py --workspace /absolute/new/workspace --test
```

See [the evidence and queue report](../../repository_snapshot/official_navigation_budget_candidate.json). A cancelled target can alter the subsequent observation sequence, so the earlier timeout cannot be counted as saved mission time or improved success before native evaluation.
