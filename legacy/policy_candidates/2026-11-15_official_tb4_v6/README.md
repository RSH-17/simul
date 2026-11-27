# Combine simulator no-return mapping with a 2.5 cm grid

The 5 cm no-return candidate still has no start goal in its live native input: the start clearance is 20 cm, below its body-plus-cell requirement of 23.44 cm. Reconstructing the same retained sensor graph at 2.5 cm gives 23.72 cm clearance against a 21.67 cm requirement and a 69 cm forward goal, with the known dock cells conservatively preserved. This replay holds the original poses fixed; it is not new robot motion or task success.

This inherits official v5's simulation-only no-return adapter and v4's rotational SLAM updates. Only the shared map resolution changes from 5 cm to 2.5 cm. The physical body radius and 1 cm margin are unchanged; the cell extent term decreases with the finer grid. The default hardware launch still does not enable the no-return adapter. Official robot files, raw sensor streams and policy weights remain unchanged.

Build: `python3 policy_candidates/2026-11-15_official_tb4_v6/build.py --workspace /absolute/new/workspace --test`.

All 400 runtime tests passed, plus three effective-resolution checks and one rotated-corridor evaluator test. The same-scene native screen completed at 90.392 simulation seconds: the body fully crossed the corridor exit at 20.4 s (from first independent truth sample, including undock), traveled 13.291 m and recorded zero contacts. All seven input gates passed, and map PGM/YAML plus pose graph/data were saved. It found zero marker IDs: this is a single-scene start improvement, not full-task success.

The independent pose audit found a maximum map position error of 0.686 m and raw-odom position error of 1.064 m. Localization remains an unresolved limitation. This candidate is not production- or hardware-qualified. See [the recorded diagnostic sequence](../../repository_snapshot/official_no_return_diagnosis.json).
