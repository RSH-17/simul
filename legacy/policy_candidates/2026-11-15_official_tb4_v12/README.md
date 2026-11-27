# Opt-in deadline handling for boundary-face exploration

This candidate adds deadline support to official v11 (the exposure buffering
candidate). Neither candidate has completed a native trial. The shipped
`policy.json` keeps the original five weights and opts in with
`settings.deadline_aware=true`; policies without the setting retain their
original behavior. The separate dock and supported-frontier candidates are
not included.

In a recorded official v8 baseline decision at 434.479 simulation seconds,
the selected goal had a 36.269 m planned path. The existing conservative
motion estimate required 193.232 s while 165.521 s remained. The `edge`
mission loader previously ignored settings, so the existing deadline selector
could not be enabled for this planner. This estimate is a planning heuristic,
not a proof that the actual robot could never arrive sooner.

The edge loader now validates the opt-in setting, and mission status exposes
the loaded planning settings. Deadline selection uses the existing conservative
travel/turn/sweep estimate. The ranking helper returns the original candidate
after scoring its temporary copy: this preserves the original frontier gain
in exploration memory, face-inspection metadata, and the selected index in
saved policy audits. Clearance, collision maps and controller limits do not
change. Marker focus/confirmed-target approach overrides retain their existing
precedence; they are not newly subjected to this normal-exploration filter.

Validation:

- All **425 runtime tests passed**. Tests cover configuration validation, legacy
  defaults, the long-trip regression, retained feature values and audit identity.
- Offline replay reproduced 23/24 baseline and 24/25 frontier-candidate choices.
  Each unmatched choice is excluded from ordinary-score interpretation.
- Of the reproduced decisions, one changed: the baseline's 36.269 m goal became
  a 4.699 m surface visit. No reproduced frontier-candidate choice changed in
  this captured window. All returned choices preserved original object identity.

This is not a replayed trajectory, successful mission, learned policy or
hardware validation. Native qualification is pending; no running or queued
experiment was changed. The combined candidate needs a full 600-second trial
against its official v8 ancestor before any performance claim.

```bash
python3 policy_candidates/2026-11-15_official_tb4_v12/build.py \
  --workspace /absolute/new/workspace --test
```
