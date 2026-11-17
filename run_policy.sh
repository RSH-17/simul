#!/usr/bin/env bash
# Unified logged entry point. Recording/upload never grants mission start.
set -eo pipefail
osp_repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
osp_arguments=("$@")
osp_workspace=""
for ((osp_i=0;osp_i<${#osp_arguments[@]};osp_i++)); do
    [[ "${osp_arguments[osp_i]}" != '--' ]] || break
    if [[ "${osp_arguments[osp_i]}" == '--workspace' ]]; then
        osp_workspace="${osp_arguments[osp_i+1]:-}"
    fi
done
if [[ -f /opt/ros/jazzy/setup.bash ]]; then source /opt/ros/jazzy/setup.bash; fi
if [[ -n "$osp_workspace" && -f "$osp_workspace/install/setup.bash" ]]; then source "$osp_workspace/install/setup.bash"; fi
exec /usr/bin/python3 "$osp_repo_dir/scripts/policy_run_logs.py" run "${osp_arguments[@]}"
