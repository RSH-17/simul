# TurtleBot4 mounted-equipment self-scan candidate v21

Parent: official v20. The real-robot launch now relays raw LiDAR to
`scan_self_filtered`. Only returns whose TF-transformed endpoint lies strictly
inside a 0.16 m disc around `base_link` are removed. This is inside the
0.189 m assumed robot footprint. The same relay is consumed by SLAM, marker
perception, mission recovery, Nav2 costmaps, and collision monitor. The raw
`scan` remains available for inspection and the platform contract. Gazebo
launches are unchanged. A missing TF passes the raw scan through unchanged.

This targets the observed `start_not_free` failure when a fixture mounted on
the robot makes its own map cell occupied. It does not assume that every
19 cm reading is self-reflection: a reading measured from the laser can lie
inside or outside the robot-body disc depending on the laser TF. A genuine
obstacle outside the disc remains visible to all navigation consumers. The
hardware bump hazard input remains active. Existing saved maps must not be
reused for this test because their occupied cells cannot be undone by a new
scan filter; use `session:=new`.

No physical scan, fixture dimensions, or hardware trial are available yet.
The candidate is not hardware validated or production promoted. In particular,
returns from a cable or router outside the inner disc remain and may still
cause `start_not_free`. Compare raw and filtered scans plus the start-map cell
before any wider robot-specific exclusion is considered.

Build on a Jazzy host in a fresh workspace:

```bash
python3 policy_candidates/2026-11-17_tb4_self_scan_v21/build.py \
  --workspace /absolute/new/workspace --test
```

Run on the robot with the new installed workspace sourced, using the existing
`robot.launch.py` arguments and `session:=new`. Do not use the repo-root
`run_robot_policy.sh` for this candidate; it still builds v13.
