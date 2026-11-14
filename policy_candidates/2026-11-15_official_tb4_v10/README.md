# Fixed map anchor for the known charging station

This experimental candidate branches from official v7, the exact parent of the
failed sideways-start trial. The camera/frontier candidates remain separate so
this comparison changes only known-dock handling and its Nav2 map connection.
It does not modify the official robot description, odometry, footprint, safety
margins, controller settings, exploration weights or hidden-target inputs.

The completed native baseline moved 11.297 m without leaving the start pocket
and recorded four contacts. Body localization error in map peaked at 5.08 cm,
while the odom-fixed dock representation shifted up to 37.9 cm at saved goal
selection times. The original saved global costmap marked the physical dock
center free. Its official-profile static layers consumed only the SLAM map;
the existing known-obstacle point cloud was not connected to those layers.

Before requesting undock, the mission now captures the observed map-to-dock TF.
That pose stays fixed in map for the mission. Once undock succeeds, a separate
`mission/navigation_map` adds only the known dock footprint to each received
SLAM map. Both actual Nav2 static layers consume this stream. Original `/map`,
PGM/YAML/pose-graph saving and unknown cells outside the station are preserved.
Jazzy costmap topic remaps also support a robot namespace on the shared hardware
launch. The compatibility point cloud uses the same fixed map pose.

With `undock=true` and a reported docked robot, an unavailable dock TF produces
an explicit waiting response before wheel motion. A dock position is never
inferred from scenario truth. Already-undocked starts and `undock=false` retain
the observed-map path without adding an assumed dock. The known station must
remain stationary, and its initial transform must be valid in the active map.
External map resets/relocalization during a mission and physical hardware are
not validated by this candidate.

Validation completed:

- All 412 runtime tests passed; 11 observer/supervisor tests passed.
- A ROS-only fixture drove the installed local and global Nav2 costmaps in
  `/dock_test_tb4`. Both changed the dock cell from free (0) to lethal (254).
  After a synthetic 0.35 m map-to-odom translation, global dock position stayed
  fixed and the local lethal cell followed the corrected coordinates. No robot
  commands or Gazebo simulation were involved in this wiring test.
- Offline overlay on one recorded raw SLAM map added 40 occupied cells and
  preserved every other cell. Six of eleven historical goal positions lie
  within the existing 0.32 m clearance threshold on that particular map.
  This is not closed-loop replay or a new native success.

Two new 90-second native trials are queued: the unchanged sideways failure
scene and the unchanged 50 cm narrow-start regression. Existing 600-second
native runs finish unchanged first, with a maximum of two simultaneous Gazebo
processes. The priority supervisor resumes the preserved full-task comparison
after both trials finish. Actual map subscriptions, stable anchor, paired map
content and lethal Nav2 costs are additional mandatory input gates. Departure
success does not establish the full three-marker task or production readiness.

Build into a new workspace:

```bash
python3 policy_candidates/2026-11-15_official_tb4_v10/build.py \
  --workspace /absolute/new/workspace --test
```
