#!/usr/bin/env bash
# Run one optional 8780 dashboard trial, or repeat the frozen five-seed campaign.
set -euo pipefail
selection="${1:-112601}"
case "$selection" in
  all) seeds=(112601 112602 112603 112604 112605) ;;
  112601|112602|112603|112604|112605) seeds=("$selection") ;;
  *) echo 'Choose 112601..112605 or all' >&2; exit 2 ;;
esac
exec multipass exec osp-jazzy -- bash /home/ubuntu/camera-aware-tools/experiments/2026-11-21_region_wall_v153/run.sh /home/ubuntu/v151-track-five-boxes5-1795247134501974510 --seeds "${seeds[@]}"
