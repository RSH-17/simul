#!/usr/bin/env bash
set -eo pipefail
osp_repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
osp_args=("$@")
osp_prepared="$osp_repo_dir/training_robust/gazebo_runtime_calibration"
export ROS_DOMAIN_ID=84
for ((osp_i=0;osp_i<${#osp_args[@]};osp_i++)); do
    if [[ "${osp_args[osp_i]}" == --domain ]]; then export ROS_DOMAIN_ID="${osp_args[osp_i+1]}"; fi
    if [[ "${osp_args[osp_i]}" == --domain=* ]]; then export ROS_DOMAIN_ID="${osp_args[osp_i]#*=}"; fi
    if [[ "${osp_args[osp_i]}" == --prepared ]]; then osp_prepared="${osp_args[osp_i+1]}"; fi
    if [[ "${osp_args[osp_i]}" == --prepared=* ]]; then osp_prepared="${osp_args[osp_i]#*=}"; fi
done
if [[ "${OSP_LOGGED_RUN:-}" != 1 ]]; then
    exec bash "$osp_repo_dir/run_policy.sh" --mode simulation --workspace "$osp_repo_dir" \
        --policy "$osp_prepared/policy.json" \
        -- bash "$osp_repo_dir/run_gazebo_runtime_probe.sh" "$@"
fi
cd /home/peter/simul
source /opt/ros/jazzy/setup.bash
export ROS_LOG_DIR="${ROS_LOG_DIR:-/home/peter/simul/training_robust/ros_logs}"
colcon build --symlink-install --packages-select osp_mission
source install/setup.bash
bash run_gazebo_guarded.sh python3 training_robust/run_runtime_probe.py "$@" --output "$OSP_RUN_DIR/artifacts"
