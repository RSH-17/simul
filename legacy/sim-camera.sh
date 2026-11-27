#!/usr/bin/env bash
set -euo pipefail
camera_repo="$(cd "$(dirname "$0")" && pwd)"
exec bash "$camera_repo/policy_candidates/2026-11-21_mixed_camera_aware/simulate.sh" "$@"
