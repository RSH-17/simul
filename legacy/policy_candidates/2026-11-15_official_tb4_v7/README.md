# Finer local navigation grid for narrow official TB4 corridors

The completed 50 cm departure screen generated valid goals but stopped after 1.272 m, with zero contacts and zero IDs. The observed SLAM/global grid was 2.5 cm while the rolling local costmap was 6 cm. On the unchanged captured grids, the installed Nav2 C++ footprint checker classified 37/61 samples along a 60 cm forward probe as lethal on the local map versus 0/61 on the global map. Rotation probes were lethal at 100/361 angles locally versus 0/361 globally. Snapshots are asynchronous and layers differ; these component results do not isolate resolution or prove a successful new drive.

This candidate changes only `local_costmap.local_costmap.ros__parameters.resolution` from 0.06 to 0.025 m. It retains the exact footprint, padding default, inflation, obstacle layers, collision monitor, official robot assets, sensors, SLAM parameters and policy weights. It inherits v6's simulator-only SLAM no-return handling. Hardware launch is not enabled for that adapter.

Build: `python3 policy_candidates/2026-11-15_official_tb4_v7/build.py --workspace /absolute/new/workspace --test`.

All 401 runtime tests passed, plus four input, preparation and installed C++ component checks. The same-scene native screen completed at 90.248 s with all eight input gates verified. The body exited at 23.46 s, traveled 12.905 m and recorded zero contacts. It found one marker, whose independent closest approach was 0.514 m. Map PGM/YAML and pose graph/data were saved.

The marker coordinate error was 0.556 m, above the 0.5 m rubric limit; maximum map pose error was 0.631 m and maximum raw-odom position error was 5.847 m. This is a single-scene start improvement, not full-task or hardware qualification. A separate frozen campaign repeats all 18 training constraint cells and records localization errors after each completed run. Reserved holdout scenes remain unused. The original 17 queued baseline screens are preserved.
