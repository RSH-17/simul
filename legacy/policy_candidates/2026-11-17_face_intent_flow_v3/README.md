# face_intent flow v3: fewer optional stops

Parent: `2026-11-17_face_intent_body_scan_v2`. This is a separate experimental
runtime; previous releases and physical runs are preserved.

The latest recorded 273.811 s physical run had zero bump reports and zero
sensor interruptions, but six mid-route wall-inspection cancellations. Those
inspections consumed 24.303 s including their turn/dwell phases. It reached ten
navigation goals and found no confirmed markers before operator stop. These
are baseline observations, not measurements of this candidate.

## Changes

- Ordinary wall inspection no longer cancels an active navigation goal by
  default. Arrival inspections and marker evidence interruptions remain.
  `allow_mid_route_wall_inspection:=true` restores the older optional behavior.
- A generic zero-angle observation proceeds to planning in the same tick,
  without publishing another stop or adding a fixed 1.5 s wait. A pending marker
  or explicit face inspection still waits for actual processed camera frames.
- Successful observation turns wait at least 0.5 s and three fresh processed
  frames, with the original 1.5 s upper wait bound, rather than always 1.5 s.
  Existing sensor freshness gates still apply; this creates no recognition or
  surface-coverage credit.
- If navigation makes less than 6 cm net odometry progress for eight seconds,
  the goal is canceled. The threshold is longer than a nominal 180-degree
  alignment at the trial's 0.5 rad/s angular limit.
- Failed/rejected/stalled goals and their 0.45 m neighborhood are excluded for
  60 s. Goals are anchored in odom when TF is available, so a map correction
  does not erase the failure. Sensor pauses, marker confirmation, successful
  arrivals, and intentional observation cancellations do not create failures.
- A failed backup is not repeated for 60 s. This is only a cooldown; a retreat
  still requires the existing rear scan check and Nav2 collision checking.

The 30 cm exploration-start exception, original destination clearance, body
reflection filter, raw collision monitor, hardware bump input, three-frame
marker verification and close-approach proof remain in the adapter chain.
Navigation status exposes the flow settings/counters for diagnosis.

## Verification

The complete runtime passed **383 tests** on the Jazzy VM in isolated ROS
domain 184. Tests exercise actual mission callbacks/ticks for skipped wall
interruptions, immediate zero-turn planning, retained marker dwell, stalled
goal cancellation, sensor pause resumption and failed-goal cooldown. The
separate dashboard suite passed eight tests.

No physical run or new Gazebo episode of v3 was started. Reduced physical stop
frequency, marker recall and full-task completion remain unverified. Generic
arrival sweeps that genuinely reveal unseen surfaces can still stop the robot.

## Build and prepare

```bash
python3 policy_candidates/2026-11-17_face_intent_flow_v3/build.py \
  --workspace /absolute/new/workspace --test
```

The prepared workspace on this Mac's VM is
`/home/ubuntu/face_intent_flow_v3_ws`. Use the inherited 0.25 m/s hardware
profile and wired Fast DDS settings from the parent candidate. Launch arguments
include `planner:=edge planner_start_ignore_radius_m:=0.30` and
`navigation_flow_enabled:=true allow_mid_route_wall_inspection:=false`.

The robot remains stopped pending an explicit drive command. If docked, use
the Create 3 undock action while the exploration mission is idle, wait for
successful undocking and a clear bump signal, and only then start exploration.
