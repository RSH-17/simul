# Map-frame face inspection candidate

The completed official C-shape comparisons contain seven face-sweep arrivals. Between selecting a face goal and consuming its sweep, the observed map/odom rotation changed by up to 22.736 degrees. The prior helper stored the endpoint in odom at selection, whereas Nav2 retained the navigation goal in map. Encoder drift during the trip could therefore rotate the camera's requested endpoint away from that map goal.

This candidate branches from official v8 (camera-scaled observation distances). It changes only `face_directed_look.py` and its runtime tests. The sweep endpoint stays in the configured map frame; arrival uses the current optical camera transform in that frame. Missing TF yields a stationary dwell, marker tasks retain precedence, and the existing 120-degree cap remains. No coverage, marker or approach credit is created by the turn.

Fresh build: `training_robust/native90/official_tb4_v16/runtime`, 417 runtime checks passed. Eight additional audit/tf2 checks passed, including real tf2 composition with changing odom and a custom map frame. Offline replay of seven recorded events used at most 40 ms map/odom skew and 350 ms interpolation gaps. A 60-second trip changed the requested turn from 50.187 to 72.923 degrees. This does not predict alternate images, correct marker detections, completion or hardware accuracy.

Native execution is not scheduled. Preserve the active U pair and queued v13 departure checks. Camera-distance changes come from the v8 parent; deadline, exposure-buffer, dock-map and pocket-continuation changes are not merged. The official robot, odometry, policy weights, marker-directed look and visit memory remain unchanged. Physical face association after a large SLAM loop closure still requires validation.
