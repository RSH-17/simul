# Project maintenance

- Layout (since 2026-11-27): every pre-existing policy, experiment, log,
  scenario set and tool lives in `legacy/` with its internal relative layout
  unchanged, so frozen sha256 manifests and `parents[N]` paths stay valid.
  Treat `legacy/` as a read-only archive: never edit hash-locked release or
  candidate files there. New work starts in a new structure at the repository
  root. Only the physical robot entry points (`robot.sh`, `*.command`) stay at
  the root; `robot.sh` resolves either the flat or `legacy/` layout on the VM.
- New structure: `osp/` (ROS-free core), `configs/` (base -> backend ->
  policy -> `--set`), `tests/`, `runs/` (git-ignored). Use `python3 -m osp`.
  Score every run with `osp/evaluation/evaluator.py`; an unmeasured requirement is
  "not measured", never a pass. Policies see only `Scenario.mission()`;
  `world()` is for backends and `targets` for the evaluator. Gazebo runs go
  through `legacy/team-sim.sh` (slot locks, host limit two) and need
  `--execute`. Keep `tests/test_fast_sim_equivalence.py` and the team_sim
  re-scoring test passing; run `python3 -m pytest` at the root before commits.

- Physical robot commands on this Mac use `./robot.sh start` and
  `./robot.sh stop` with the prepared `face_intent_flow_v4_ws` in `osp-jazzy`.
  The physical board also supports explicit start/stop buttons when launched
  with `--enable-control`. Both use `legacy/scripts/robot_session.py` and the same
  readiness checks. Do not rebuild, redeploy, restart the board, or manually
  call `/mission/start` for an ordinary drive request. Execute the prepared
  entry point first; investigate only a reported failure. `check` and `status`
  are read-only. A stop remains in force until the user explicitly requests
  driving or presses the board's start button. Never start for a UI/code test.

- The user authorized ongoing maintenance of the private GitHub repository
  `RSH-17/simul` (git remote `mine`). After completing a project change, run
  relevant checks, review the intended diff, commit it, and push to `main` on
  `mine`. No repeated upload approval is needed. Unless the user explicitly
  says otherwise, push only to `mine`, which is the repository of record.
  The old teammate remote `origin` (`peter010905/simul`) is no longer
  followed: do not fetch, merge or push it. Do not claim that unattended code review or scheduled
  maintenance exists unless it has actually been configured.
- Update the README and status file of the area the milestone belongs to
  (for legacy work: `legacy/README.md` and
  `legacy/repository_snapshot/status.json`) when reporting a new experiment
  milestone. Distinguish original benchmark results, offline replay,
  and new Gazebo results. Never promote an unverified candidate as production.
- Publish finished simulation videos as private GitHub Release assets using
  `legacy/scripts/publish_simulation_videos.py` (repo `RSH-17/simul`, same
  release tags as before); commit the generated video index.
  Do not add video binaries, generated maps, ROS logs, or build output to Git.
- Preserve original experiments. Do not edit source in a frozen runtime stage
  while its campaign is running or queued. Create a new staged candidate.
- Run at most two native Gazebo benchmarks concurrently on this host. Preserve
  the existing service memory limits and independent evaluation ground truth.
- Required task outcome: three accurate marker IDs, actual approach within
  1.5 m of each marker, correct map locations, and fast collision-free completion.
  Ground-truth scenario data belongs only to the evaluator, never the policy.
