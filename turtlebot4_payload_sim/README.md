# TurtleBot4 + MacBook payload replication (experimental)

This ROS 2 Jazzy package extends the **installed official TurtleBot4 Standard
URDF** at launch time. It does not edit the installed TurtleBot4, Create3,
Gazebo, or Nav2 packages. It models a 1.5 kg MacBook as a physical link with
box inertia, and models an adapter and cable as fixed links with visual,
collision and inertial geometry. All positions except the official robot
geometry and the stated 1.5 kg laptop mass are **assumptions** pending robot
measurements. The two layouts in `config/` are hypotheses, not observations of
the actual user's laptop and cable.

## Confirmed observations on the `osp-jazzy` VM

Environment: Ubuntu 24.04 aarch64, ROS 2 Jazzy, Gazebo Harmonic, official
TurtleBot4 Standard description and bringup, Cyclone DDS. The official URDF
has 5.39 kg total link mass, base-link COM z=0.130 m at zero joint position,
RPLIDAR minimum range 0.164 m, and sensor center approximately (-0.04, 0,
0.193) m in `base_link`. The mass sum includes existing plate, supports, and
weight blocks. Adding 1.5 kg laptop, 0.03 kg adapter, and 0.01 kg cable gives
6.93 kg and analytic COM z=0.180 m in the assumed side layout. These are URDF
calculations, not measurements of the physical robot.
The laptop contributes 14.715 N at 9.81 m/s². The zero-joint-position system
inertia about its own COM, expressed in `base_link`, rises from approximately
(Ixx, Iyy, Izz) = (0.0873, 0.0779, 0.0741) to
(0.1543, 0.1508, 0.0928) kg·m² with the full side payload. This calculation
rotates each link's inertial tensor and applies the parallel-axis theorem;
moving joints or a flexible cable would change the values.

All conditions used the same 4.9 m square walled room, start (0, 0), and a
physical external obstacle centered at (1, 0). The raw Gazebo ray sensor was
bridged to ROS `/scan`. Each static row below contains 12 actual scan frames.

| Layout / condition | Minimum `/scan` return | Returns within 40 cm | Result |
| --- | ---: | ---: | --- |
| Side, baseline | 0.990 m | 0 | External obstacle visible |
| Side, 1.5 kg laptop only | 0.990 m | 0 | Load alone did not make a short scan |
| Side, adapter only | 0.990 m | 0 | Assumed adapter is inside the sensor's minimum range |
| Side, cable only | 0.165 m | 1 | Cable is detected |
| Side, full payload | 0.165 m | 1 | Cable is detected |
| Side, repositioned | 0.990 m | 0 | Raising cable/adapter clears the scan |
| Front adapter hypothesis, baseline / mass only | 0.990 m | 0 | No self return |
| Front adapter hypothesis, occlusion only / adapter only / full | 0.185 m | 19 | Adapter is detected |
| Front adapter hypothesis, cable only | 0.217 m | 11 | Cable is detected |
| Front adapter hypothesis, repositioned | 0.990 m | 0 | Raising hardware clears the scan; the laptop moves 5 cm forward in the final variant |

Nav2 used the official parameter file with only MPPI batch size 200 (from
2000) and time steps 28 (from 56) to keep the ARM VM responsive. Costmap
geometry and Collision Monitor were unchanged. The same goal was (0.5, 0) m;
Nav2's 0.25 m XY tolerance means `SUCCEEDED` can occur after about 0.25 m
physical travel. The front-layout comparison below was rerun with the robot
spawn height corrected and a 75 s wall-time limit for stalled runs.

| Condition | Nav2 observation |
| --- | --- |
| Front baseline (A) | Goal accepted, movement observed, `SUCCEEDED` |
| Front 1.5 kg mass only (B) | Movement observed; 75 s wall timeout |
| Front near-massless adapter/cable occlusion only (C) | Goal accepted; no linear motion; 75 s wall timeout; Collision Monitor `APPROACH` |
| Front full payload (D) | Goal accepted; no linear motion; 75 s wall timeout; Collision Monitor `APPROACH` |
| Front adapter only | Same no-motion and `APPROACH` result without laptop |
| Front repositioned (fixture raised, laptop 5 cm forward) | Goal accepted, movement observed, `SUCCEEDED` |
| Front full with selective fixture filter | Movement observed, but 75 s wall timeout; **unsafe same-ray blind spot** |

The front fixture condition reproduces a *failure to start*, but **does not
reproduce the exact `start_not_free` diagnostic**: the observed raw SLAM map
cell at the simulated robot center was 0 (free), while the local inflated
costmap cell was 99. The v20 diagnostic uses the raw start map cell. The
physical robot's map, fixture placement and TF are still needed to establish
why its own start cell is occupied. A single VM trial is not a success rate.
Raising only the fixture while leaving the laptop centered cleared the LiDAR
but moved about 0.08 m and timed out in a diagnostic run. Moving
the assumed laptop 5 cm forward as well completed the goal in one trial. The
shift requires a rigid mount that has not been measured on the real robot.

### Physical response sweep (one trial per condition)

After a two simulated second settling period, the stock Create3 motion
controller received the same 0.2 m/s straight command for two seconds and
then a stop. The added laptop positions and masses remain hypothetical.

| Added mass / placement | Travel in three simulated seconds | Peak pitch | Create3 hazard |
| --- | ---: | ---: | --- |
| 0 kg baseline | 0.401 m | 0.005 rad | none |
| 1.0 kg centered | 0.400 m | 0.046 rad | none |
| 1.5 kg centered | 0.112 m | 0.091 rad | CLIFF (type 2) |
| 2.0 kg centered | 0.118 m | 0.091 rad | CLIFF (type 2) |
| 1.5 kg, 5 cm higher | 0.111 m | 0.091 rad | CLIFF (type 2) |
| 1.5 kg, 5 cm forward | 0.401 m | 0.004 rad | none |
| 1.5 kg, 5 cm backward | 0.093 m | 0.095 rad | CLIFF (type 2) |
| 1.5 kg, 5 cm left/right | 0.114 / 0.117 m | 0.091 / 0.091 rad | CLIFF (type 2) |

These are simulator responses to a fixed velocity command, not motor torque
or hardware load limits. The 1.5 kg model **does start moving**. The later
cliff event can explain its short travel in this assumed geometry, while the
front-adapter-only Nav2 failure occurs without the laptop mass. A lower laptop
position was excluded because it intersects the official plate. The off-center
positions assume an unmeasured rigid mount.
The 0.6 rad/s two-second turn test yielded 1.199 rad for baseline and
1.197 rad for the centered 1.5 kg model, with no Create3 hazard in either.
The 5 cm backward offset yielded 0.991 rad and a CLIFF hazard; the remaining
turn conditions are recorded in the CSV. Each condition was run once, so
these differences have no uncertainty estimate.

### Selective filter safety result

`fixture_filter.py` removes only returns whose transformed endpoints overlap
the configured adapter/cable geometry. It retains raw `/scan` and publishes
`/scan_payload_filtered`; the optional launch mode routes SLAM, Nav2 costmaps,
and Collision Monitor to that filtered topic. The checked node subscriptions
all used `/scan_payload_filtered`. In the front fixture condition the filter
removed beams 151–169. Beam 160 changed from 0.185 m to no return, even though
the external obstacle at (1, 0) lies behind the fixture along the same beam.
The physical ray never measured that farther obstacle, so no scan filter can
recover it. **Do not deploy this fixture filter on a real robot.** Move the
adapter and cable out of the scan plane first, then retest with an external
obstacle in the same direction. The existing v21 inner-body filter is a
separate, unvalidated policy candidate.

## Build and run in the VM

```bash
cd /home/ubuntu
mkdir -p payload_ws/src
ln -s /home/ubuntu/simul/turtlebot4_payload_sim payload_ws/src/turtlebot4_payload_sim
source /opt/ros/jazzy/setup.bash
cd payload_ws
colcon build --symlink-install --packages-select turtlebot4_payload_sim
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/ubuntu/cyclonedds.xml
export ROS_DOMAIN_ID=57
ros2 launch turtlebot4_payload_sim payload_sim.launch.py variant:=full \
  payload_config:=/home/ubuntu/simul/turtlebot4_payload_sim/config/front_fixture_hypothesis.yaml \
  navigation:=true
```

The launch defaults to a headless Gazebo room and `baseline` variant. Replace
`variant` with `mass_only`, `occlusion_only`, `adapter_only`, `cable_only`,
`full`, or `repositioned`. `navigation:=true` starts official SLAM and Nav2.
`fixture_filter:=true` is **simulation only** and has the safety failure above.
The launch prints calculated mass/COM values and keeps the stock LiDAR pose,
FOV, resolution, and range. It uses a unique ROS domain and Gazebo partition
per automated experiment. Use only one VM Gazebo run at a time.

Reproduce the scan comparisons:

```bash
python3 -m turtlebot4_payload_sim.scan_probe --variant full --domain 66 \
  --output-dir /home/ubuntu/payload_results/side/full
python3 -m turtlebot4_payload_sim.nav_probe --variant full --domain 88 \
  --payload-config /home/ubuntu/simul/turtlebot4_payload_sim/config/front_fixture_hypothesis.yaml \
  --output-dir /home/ubuntu/payload_results/front/nav_full
python3 -m turtlebot4_payload_sim.filter_probe --domain 92 \
  --payload-config /home/ubuntu/simul/turtlebot4_payload_sim/config/front_fixture_hypothesis.yaml \
  --output-dir /home/ubuntu/payload_results/front/filter
```

Generate analytical mass/COM configurations and open-loop motion sweeps with
`sweep.py` and `motion_probe.py`. The lower mounting position is excluded
because its laptop box intersects the existing official top plate; running
that geometry would be physically meaningless. The other offset positions
assume a rigid, unmeasured mounting bracket. The stock Create3 velocity
interface, wheel friction and software acceleration limits are present, but
the URDF does not specify a measured motor torque curve. Do not infer real
motor load capacity from these simulation velocities.

```bash
python3 -m turtlebot4_payload_sim.sweep \
  --official-xacro /opt/ros/jazzy/share/turtlebot4_description/urdf/standard/turtlebot4.urdf.xacro \
  --base-config /home/ubuntu/simul/turtlebot4_payload_sim/config/payload.yaml \
  --output-dir /home/ubuntu/payload_results/sweep/configs
python3 -m turtlebot4_payload_sim.motion_probe --variant mass_only \
  --profile straight --domain 121 \
  --payload-config /home/ubuntu/payload_results/sweep/configs/mass_1_5.yaml \
  --output-dir /home/ubuntu/payload_results/sweep/motion/mass_1_5
python3 -m turtlebot4_payload_sim.motion_probe --variant mass_only \
  --profile turn --domain 122 \
  --payload-config /home/ubuntu/payload_results/sweep/configs/mass_1_5.yaml \
  --output-dir /home/ubuntu/payload_results/sweep/turn/mass_1_5
python3 -m turtlebot4_payload_sim.analyze_sweep /home/ubuntu/payload_results/sweep
python3 -m turtlebot4_payload_sim.analyze_navigation \
  /home/ubuntu/simul/turtlebot4_payload_sim/results/20261117/front_fixture/navigation
```

Run the conditions serially with distinct ROS domains and Gazebo partitions.
Use ROS domain IDs from 0 to 230; higher values can exceed Cyclone DDS's
UDP port range.
The motion probe waits two simulated seconds for the robot to settle, then
commands 0.2 m/s straight or 0.6 rad/s turning for two simulated seconds,
followed by one second of stop. It records odometry, IMU and Create3 hazard
messages. It does not provide goal tracking or a motor torque measurement.

The tracked data and figures are under
`repository_snapshot/payload_replication_20261117/` at the repository root.
Additional working files are under this package's ignored `results/20261117/`.
Gazebo launch logs and generated worlds remain in the VM, outside Git. The figures compare
actual ROS/Gazebo observations; they are not physical robot validation.

## Outstanding physical measurements

- Measure laptop dimensions, center/height, adapter size and pose, cable path
  and width, and the sensor TF on the actual robot. Update `config/payload.yaml`.
- Record raw `/scan`, `/map`, Nav2 costmap, Collision Monitor state, TF and
  `/cmd_vel` on the robot when `start_not_free` occurs. Without these, the
  simulated failure does not establish the real cause.
- Repeat Nav2 trials for a success-rate estimate and test the same-direction
  external obstacle after moving the hardware. The filtered same-ray case
  already fails the safety criterion.
- The small room is a causal start experiment. It does not test the separate
  three-marker policy task or full 600-second mission.
