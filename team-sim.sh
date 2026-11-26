#!/usr/bin/env bash
set -eo pipefail
team_repo="$(cd "$(dirname "$0")" && pwd)"
team_action="${1:-help}"
if [[ "$team_action" == board ]]; then
  shift
  exec python3 "$team_repo/tools/team_sim/board.py" "$@"
fi
if [[ "$team_action" == gui || "$team_action" == rviz ]]; then
  exec bash "$team_repo/tools/team_sim/viewer.sh" "$@"
fi
if [[ "$team_action" == help ]]; then
  echo 'setup WORKSPACE [--policy balanced|rescue360|adaptive-survey|object-followthrough|tiny-named-relook|surface-followthrough|balanced-print-relook|visual-survey|visual-survey-focus|marker-events|wall-slot-escape] [--archive PATH]'
  echo 'run WORKSPACE [SEED] [--random-layout] [--markers three|unknown] [--tables fixed|shuffle] [--duration SECONDS] [--check]'
  echo 'board [--port 8780]'
  echo 'gui                    attach the Gazebo GUI to the running simulation'
  echo 'rviz [WORKSPACE]       attach RViz (/map, /scan, /targets) to the running simulation'
  echo 'env: TEAM_SIM_RENDER=software|system  TEAM_SIM_RVIZ=1  TEAM_SIM_CAMERA_NOISE="sigma=6,blur=1,drop=0.05,seed=1"'
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
