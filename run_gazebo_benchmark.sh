#!/usr/bin/env bash
# Run the balanced native Gazebo suite once; each invocation has fresh outputs.
set -eo pipefail
osp_repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
osp_args=("$@")
osp_policy="$osp_repo_dir/training_robust/gazebo_runtime_calibration/policy.json"
export ROS_DOMAIN_ID=84
for ((osp_i=0;osp_i<${#osp_args[@]};osp_i++)); do
    if [[ "${osp_args[osp_i]}" == --domain ]]; then export ROS_DOMAIN_ID="${osp_args[osp_i+1]}"; fi
    if [[ "${osp_args[osp_i]}" == --domain=* ]]; then export ROS_DOMAIN_ID="${osp_args[osp_i]#*=}"; fi
    if [[ "${osp_args[osp_i]}" == --policy ]]; then osp_policy="${osp_args[osp_i+1]}"; fi
    if [[ "${osp_args[osp_i]}" == --policy=* ]]; then osp_policy="${osp_args[osp_i]#*=}"; fi
done
if [[ "${OSP_LOGGED_RUN:-}" != 1 ]]; then
    exec bash "$osp_repo_dir/run_policy.sh" --mode simulation --workspace "$osp_repo_dir" \
        --policy "$osp_policy" \
        -- bash "$osp_repo_dir/run_gazebo_benchmark.sh" "$@"
fi
cd /home/peter/simul
source /opt/ros/jazzy/setup.bash
export ROS_LOG_DIR="${ROS_LOG_DIR:-/home/peter/simul/training_robust/ros_logs}"
export OPENBLAS_NUM_THREADS=1
colcon build --symlink-install --packages-select osp_mission
source install/setup.bash
osp_benchmark_output="${OSP_RUN_DIR:-/home/peter/simul/training_robust/gazebo_campaigns/$(date -u +%Y%m%dT%H%M%S_%NZ)}/artifacts"
bash run_gazebo_guarded.sh python3 training_robust/gazebo_benchmark.py \
    --scenarios training_robust/gazebo_runtime_calibration/scenarios \
    --policy training_robust/gazebo_runtime_calibration/policy.json \
    --limit 20 --record-video "$@" --output "$osp_benchmark_output"
