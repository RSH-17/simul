#!/usr/bin/env bash
set -eo pipefail
team_repo="$(cd "$(dirname "$0")" && pwd)"
team_action="${1:-help}"
if [[ "$team_action" == board ]]; then
  shift
  exec python3 "$team_repo/tools/team_sim/board.py" "$@"
fi
if [[ "$team_action" == help ]]; then
  echo 'setup WORKSPACE [--policy balanced|rescue360|adaptive-survey|object-followthrough] [--archive PATH]'
  echo 'run WORKSPACE SEED [--duration SECONDS] [--check]'
  echo 'board [--port 8780]'
  exit 0
fi
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo 'Ubuntu 24.04 + ROS 2 Jazzy is required. See docs/team-policy-lab/README.md.' >&2
  exit 2
fi
shift
source /opt/ros/jazzy/setup.bash
if [[ "$team_action" == setup ]]; then
  exec python3 "$team_repo/tools/team_sim/setup.py" "$@"
elif [[ "$team_action" == run ]]; then
  team_workspace="${1:?Pass a prepared workspace}"; shift
  team_workspace="$(cd "$team_workspace" && pwd)"
  source "$team_workspace/install/setup.bash"
  exec python3 "$team_repo/tools/team_sim/run.py" --workspace "$team_workspace" "$@"
else
  echo 'Unknown action. Use bash team-sim.sh help.' >&2
  exit 2
fi
