#!/usr/bin/env bash
# Attach a viewer to the team simulation that is already running on this host.
#   viewer.sh gui                Gazebo GUI client (world, robot, sensors)
#   viewer.sh rviz [WORKSPACE]   RViz with the package view (/map, /scan, /targets)
# Viewers only subscribe; they never start, stop or command the simulation.
set -eo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
kind="${1:?gui or rviz}"
source /opt/ros/jazzy/setup.bash
export DISPLAY="${DISPLAY:-:1}"

# The trial sets GZ_PARTITION and GZ_SIM_RESOURCE_PATH in the Gazebo server
# environment. Read them from that process instead of matching command lines.
find_sim_env() {
  local e
  for _ in $(seq 1 60); do
    for e in /proc/[0-9]*/environ; do
      [[ -r "$e" ]] || continue
      if { tr '\0' '\n' < "$e"; } 2>/dev/null | grep -q '^GZ_PARTITION=camera_aware_' &&
         { [[ -z "${TEAM_SIM_DOMAIN:-}" ]] || { tr '\0' '\n' < "$e"; } 2>/dev/null | grep -qx "ROS_DOMAIN_ID=$TEAM_SIM_DOMAIN"; }; then
        echo "$e"; return 0
      fi
    done
    sleep 0.5
  done
  return 1
}

case "$kind" in
  gui)
    src="$(find_sim_env)" || { echo 'No running team simulation found.' >&2; exit 3; }
    export GZ_PARTITION="$(tr '\0' '\n' < "$src" | grep -m1 '^GZ_PARTITION=' | cut -d= -f2-)"
    export GZ_SIM_RESOURCE_PATH="$(tr '\0' '\n' < "$src" | grep -m1 '^GZ_SIM_RESOURCE_PATH=' | cut -d= -f2-)"
    # The GUI renders with the desktop GPU driver regardless of TEAM_SIM_RENDER.
    unset LIBGL_ALWAYS_SOFTWARE
    echo "Gazebo GUI on partition $GZ_PARTITION (close the window to detach)"
    exec gz sim -g -v 2 ;;
  rviz)
    workspace="${2:-$HOME/simul-ws/baseline}"
    source "$workspace/install/setup.bash"
    # Same DDS settings as tools/team_sim/run.py; with other settings RViz
    # sees topic names but receives no data.
    export ROS_DOMAIN_ID="${TEAM_SIM_DOMAIN:-174}" ROS_LOCALHOST_ONLY=1 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI="file://$here/cyclone.xml"
    exec rviz2 -d "$workspace/install/osp_mission/share/osp_mission/rviz/mission.rviz" --ros-args -p use_sim_time:=true ;;
  *) echo "Unknown viewer: $kind (use gui or rviz)" >&2; exit 2 ;;
esac
