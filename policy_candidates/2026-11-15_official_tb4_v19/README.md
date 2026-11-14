# Official TurtleBot4 feasible face proposal candidate v19

The fixed 48-face budget previously sampled every observed boundary before checking reachability and sightlines. On seven recorded C/U maps this left only 13–39 useful samples. This candidate retains those useful samples in their original order and fills wasted slots with feasible faces spread across map position and outward normal. It preserves separate sides of a thin partition and does not mark planned visits as observed camera evidence.

Parent: integrated v17. The only production source changes are `edge_policy.py` and the new `face_proposals.py`. The official robot, odometry, body clearance, Nav2/SLAM configuration, camera thresholds, fixed weights and deadline setting remain unchanged. Candidate generation uses observed occupancy and path distances only; hidden target geometry is never read.

Validation in a fresh installed workspace: **491 tests passed**, including rotated clutter at 2.5/5cm resolution, inaccessible destinations, unknown sightlines, retained legacy proposals and equivalence to the existing line-of-sight samples. Offline replay on eight fixed maps and thirteen early departure decisions preserved **all 21 original candidate sets**. Seven normal-inspection maps produced 48 feasible face samples each, adding 3.85–15.55m of geometrically observable boundary with zero lost baseline boundary. One transit map and all thirteen early departure decisions stayed identical.

These are proposal-generation results, not new navigation, marker recognition or complete-task successes. Memory filtering, deadline selection and subsequent camera/SLAM feedback still require native verification. The earlier simple replacement prototype remains preserved in runtime v21; the candidate described here is runtime **v22**. It is **not queued, not production and not hardware validated**. Existing frozen v16 departure and v17/v18 full-task campaigns are unchanged.

Build in a new workspace:

```bash
python3 policy_candidates/2026-11-15_official_tb4_v19/build.py --workspace /absolute/new/workspace --test
```

Recorded-input comparison: `training_robust/audit_face_proposals.py --help`. Complete input/source hashes, individual results and limitations are in [the candidate report](../../repository_snapshot/official_face_proposal_candidate.json).
