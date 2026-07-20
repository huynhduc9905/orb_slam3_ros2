#!/bin/bash
RESULT_PATHS=(artifacts/tracking-performance-20260719_134544/orb-only-2x/tracking_benchmark.json artifacts/tracking-performance-20260719_134544/orb-only-3x/tracking_benchmark.json)
SOURCE_CAMERA_HZ=15
export PYTHONPATH=./orb_slam_bringup
ARGS=()
for p in "${RESULT_PATHS[@]}"; do
  ARGS+=("--result" "$p")
done
python3 -m orb_slam_bringup.tracking_benchmark select --source-camera-hz "${SOURCE_CAMERA_HZ}" "${ARGS[@]}"
echo $?
