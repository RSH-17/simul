#!/usr/bin/env bash
# Mac entry point: simulation only, with the same locks as the comparison.
set -euo pipefail
version="${1:-v1.5.0-balanced}"
seed="${2:-112101}"
case "$version" in
  v1.2-balanced|v1.2-glance|v1.2-detail|v1.5.0-balanced|v1.5.0-glance|v1.5.0-detail) ;;
  *) echo 'Choose v1.2 or v1.5.0 with -balanced, -glance or -detail' >&2; exit 2 ;;
esac
case "$seed" in 112021|112022|112101|112102|112103) ;; *) echo 'Unknown frozen map seed' >&2; exit 2 ;; esac
exec multipass exec osp-jazzy -- bash /home/ubuntu/camera-aware-tools/experiments/2026-11-21_camera_aware_policy/single.sh "$version" "$seed"
