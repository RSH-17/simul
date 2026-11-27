# Camera-calibrated face viewpoints

The official RGB CameraInfo supplies focal length 221.765 px. The existing
8 cm minimum marker prior and 18 pixel threshold imply a conservative readable
distance of 0.838 m, but explicit face visits used fixed 1.2/1.8 m offsets.
Ordinary sampled viewpoints could still yield useful views; the old policy did
not have zero readable candidates everywhere.

This candidate proposes offsets at 65% and 85% of the calibrated range, capped
at the old distances and omitted below 0.4 m. Every destination still needs
the existing 0.32 m body clearance, reachability, known free line of sight and
revisit checks. Duplicate tile candidates retain a directed 60-degree face
look. Proposed views never count as actual inspection or marker recognition.
No hidden marker ID, pose or size enters the policy. Official robot assets,
odometry, SLAM, Nav2, collision checks and policy weights remain unchanged.

Build in a new workspace:

```bash
python3 policy_candidates/2026-11-15_official_tb4_v8/build.py \
  --workspace /absolute/new/workspace --test
```

The new runtime passed 409 tests. Recorded policy observations replayed with
the original CameraInfo produced 19–42 directed readable candidates in five
post-departure snapshots, versus 0–3 previously. The seven early departure
snapshots were unchanged. This is candidate-generation evidence, not native
motion or detection success. A paired 90-second native screen is running with all eight input gates passed, using
`run_official_camera_view_screen.py`; the preserved 18-condition matrix also resumes once this single candidate
is running and cannot dispatch another job. A persistent user service makes
subsequent resumption possible after the transient registration is removed. No production or hardware qualification is claimed.

The separate IMU shadow experiment was not applied: short-interval heading
improved but translation error increased. See the repository snapshots for
the observation-tool interference, invalidated U run and isolated second
recording. Estimator configuration references:
[Nav2 Jazzy](https://docs.nav2.org/jazzy/configuration_and_development/first_time_robot_setup_guide/odom/setup_robot_localization/)
and [ROS 2 controller TF interface](https://control.ros.org/jazzy/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html).
