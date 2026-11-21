#!/usr/bin/env bash
set -eo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
source /opt/ros/jazzy/setup.bash
source "$HOME/flow_v151_exposure_sync_r2_ws/install/setup.bash"
export CAMERA_BENCH_HELPERS="$HOME/track-resume-5e95333"
export SURVEY_EXPERIMENTS="$HOME/camera-aware-tools/experiments"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
exec python3 -B "$here/screen_audit.py" "$@"
