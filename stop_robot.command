#!/bin/bash
# Independent stop entry point; usable while the start terminal is occupied.
set -euo pipefail
export PATH="/opt/homebrew/bin:/usr/local/bin:$PATH"
robot_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
printf 'TurtleBot4 정지 · 준비 중인 구동도 취소하고 실제 속도를 확인합니다.\n\n'
exec bash "$robot_root/robot.sh" stop "$@"
