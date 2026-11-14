# Combine simulator no-return mapping with a 2.5 cm grid

The 5 cm no-return candidate still has no start goal in its live native input: the start clearance is 20 cm, below its body-plus-cell requirement of 23.44 cm. Reconstructing the same retained sensor graph at 2.5 cm gives 23.72 cm clearance against a 21.67 cm requirement and a 69 cm forward goal, with the known dock cells conservatively preserved. This replay holds the original poses fixed; it is not new robot motion or task success.

This inherits official v5's simulation-only no-return adapter and v4's rotational SLAM updates. Only the shared map resolution changes from 5 cm to 2.5 cm. The physical body radius and 1 cm margin are unchanged; the cell extent term decreases with the finer grid. The default hardware launch still does not enable the no-return adapter. Official robot files, raw sensor streams and policy weights remain unchanged.

Build: `python3 policy_candidates/2026-11-15_official_tb4_v6/build.py --workspace /absolute/new/workspace --test`.

All 400 runtime tests passed. The same-scene 90-second native verification has started; results are pending. Three effective-resolution checks and one rotated-corridor evaluator test also passed. This candidate is not production- or hardware-qualified. See [the recorded diagnostic sequence](../../repository_snapshot/official_no_return_diagnosis.json).

Interim native observation: the body fully crossed the corridor exit at 20.4 s (timed from the first independent ground-truth sample including undock). Zero contacts were recorded at the snapshot. The 90-second trial is still running; this is not finalized escape qualification, full-task success or hardware evidence.
