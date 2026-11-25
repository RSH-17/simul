#!/usr/bin/env bash
set -eo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
source /opt/ros/jazzy/setup.bash
source "$HOME/flow_v151_exposure_sync_r2_ws/install/setup.bash"
export ROS_DOMAIN_ID=174 ROS_LOCALHOST_ONLY=1 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CAMERA_BENCH_HELPERS="$HOME/track-resume-5e95333"
export CAMERA_BENCH_CARDS="$HOME/corridor-board-73bbb05/corridor-comparison"
export CYCLONEDDS_URI="file://$CAMERA_BENCH_HELPERS/2026-11-19_pair_7_103/cyclone.xml"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
unset ROS_STATIC_PEERS ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE
exec python3 -u "$here/run.py" "$@"
