#!/bin/bash
# Double-click in Finder, or run with bash. Only this explicit action starts motion.
set -euo pipefail
export PATH="/opt/homebrew/bin:/usr/local/bin:$PATH"
robot_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == --dry-run ]]; then
  exec bash "$robot_root/robot.sh" dry-run
fi
printf 'TurtleBot4 구동 · Flow v3\n시간·센서 확인 후 자동 시작합니다. 정지는 stop_robot.command 또는 Ctrl+C입니다.\n\n'
exec bash "$robot_root/robot.sh" start "$@"
