#!/bin/bash
RESULT_PATHS=(artifacts/tracking-performance-20260719_134544/orb-only-2x/tracking_benchmark.json artifacts/tracking-performance-20260719_134544/orb-only-3x/tracking_benchmark.json artifacts/tracking-performance-20260719_134544/orb-only-4x/tracking_benchmark.json)
ARGS=()
for p in "${RESULT_PATHS[@]}"; do
  ARGS+=("--result" "$p")
done
export PYTHONPATH=./orb_slam_bringup
python3 -m orb_slam_bringup.tracking_benchmark select --source-camera-hz 15 "${ARGS[@]}"
echo $?
