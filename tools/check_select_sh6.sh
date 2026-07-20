#!/bin/bash
RESULT_PATHS=(artifacts/tracking-performance-20260719_134544/orb-only-2x/tracking_benchmark.json artifacts/tracking-performance-20260719_134544/orb-only-3x/tracking_benchmark.json artifacts/tracking-performance-20260719_134544/orb-only-4x/tracking_benchmark.json)
SOURCE_CAMERA_HZ=15
export PYTHONPATH=./orb_slam_bringup

# Wait, --result is appended to the array incorrectly?
# It should be: python3 -m orb_slam_bringup.tracking_benchmark select --source-camera-hz 15 --result A --result B --result C
# BUT: --result "${RESULT_PATHS[@]}" in bash will result in:
# --result A B C  <-- only A is associated with --result!
