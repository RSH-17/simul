#!/usr/bin/env bash
# Opt-in v1.5.2 large-track regression; leaves the v1.5.1 default intact.
set -euo pipefail
seed="${1:-112601}"
case "$seed" in 112601|112602|112603|112604|112605) ;; *) echo 'Choose seed 112601..112605' >&2; exit 2 ;; esac
exec multipass exec osp-jazzy -- bash /home/ubuntu/camera-aware-tools/experiments/2026-11-21_wall_transit_v152/run.sh /home/ubuntu/v151-track-five-boxes5-1795247134501974510 --seeds "$seed"
