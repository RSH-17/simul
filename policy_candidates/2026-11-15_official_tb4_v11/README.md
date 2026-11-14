# Bounded exposure-time buffering for marker detection

This candidate branches from official v8, the exact parent of the running
baseline with the observed timing error. It does not include the separate
frontier v9 or dock v10 changes. It is built and tested, but has **not run a
native Gazebo benchmark**, has not been queued, and is not a production policy.

At 390.979 simulation seconds the original baseline reported one marker and
57 detector exceptions. The last exception requested map-to-camera TF at
340.593 s while the newest available transform was at 340.580 s. These are
interim native observations, not a completed task result or 57 proven lost IDs.

The RGB subscription now keeps at most eight exposure bundles, each holding
the original RGB, CameraInfo, scan and depth messages. A nonblocking check
retries the required transforms for up to 250 ms of ROS time. Processing uses
the original exposure timestamp and original sensor bundle, then restores the
latest received sensor references. No latest-pose substitution, timestamp
rewrite, synthetic landmark, recognition-threshold change or ground truth is
introduced. The node's default mutually exclusive callback group prevents a
sensor callback from changing the bundle during processing.

Expired, duplicate, out-of-order, zero-stamped and excessively future-stamped
frames are discarded explicitly. Clock rollback clears pending frames from
the previous session. Queue counters appear in `mission/detector_status`.
Missing required transforms can still prevent recognition; this bounded queue
does not manufacture a location when calibration or localization is absent.

Validation: **418 runtime tests passed** in a fresh build. The new tests cover
0/13/50/200/300 ms transform delay, missing transforms, bounded memory, queue
expiry, duplicate timestamps and clock reset. An integration fixture uses the
actual tf2 Buffer and detector, with both LiDAR and registered depth, to check
interpolated exposure-time positions and retention of the original sensor
messages. These are synthetic-input tests, not improved native success rates.

Native qualification remains pending behind the existing full-task comparison
and priority dock regression. No running or queued runtime was edited. The
candidate does not fix low-pixel marker identity, mixed-surface LiDAR range
failures or exploration weighting by itself.

```bash
python3 policy_candidates/2026-11-15_official_tb4_v11/build.py \
  --workspace /absolute/new/workspace --test
```
