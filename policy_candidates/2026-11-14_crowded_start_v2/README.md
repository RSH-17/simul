# Crowded start v2: incremental observed-free transit

The first native 55 cm corridor trial exposed a second failure. Removing the old 0.8 m connector bound was insufficient: the initial SLAM map had no sufficiently large normal-clearance destination and had unknown gaps beyond the observed corridor. Both original and v1 produced zero goals in the captured input.

This candidate retains the normal planner and its 0.32 m destination clearance. Only when the starting region cannot provide that connection, it proposes a short **0.45–0.70 m transit waypoint** along a known-free path toward an observed frontier. Every path cell and endpoint require the 0.189 m body radius, grid cell extent, and a 1 cm margin from occupied/unknown space. The planner does not clear unknown cells or use scenario truth. Nav2, the footprint and collision monitor retain control.

Transit goals do not create completed-viewpoint, camera coverage, marker recognition or approach credit. Successful transit triggers a fresh planning pass without an observation spin. Failed transit retains the existing failure cooldown. Ordinary inspection and marker behavior remain separate.

## Evidence and scope

- Fresh build and **382 runtime tests passed** (v1: 378).
- Replay of the actual v1 initial SLAM input at 8.048 simulation seconds changed zero goals to one 0.65 m transit goal. This is offline replay, not a native success.
- Four 60-second native runs are prepared on two paired scenes (55 cm straight and 60 cm sideways starts). They use the same policy weights and shared nominal TB4 configuration; compare v2 with the original platform baseline.
- Previous runs and their queued workspaces remain preserved. At most two native benchmarks run, with unchanged service memory limits and independent truth evaluation.
- **Native validation pending. No production promotion or hardware claim.** The encoder drift, C-shape SLAM jump and incomplete marker inspection remain separate issues. This screen does not qualify the 600-second full task.

```bash
python3 policy_candidates/2026-11-14_crowded_start_v2/build.py --workspace /absolute/new/workspace --test
python3 training_robust/run_crowded_start_regression_v2.py --output /absolute/new/results --candidate /absolute/candidate/workspace --baseline /absolute/baseline/workspace
```
