#!/bin/bash
RESULT_PATHS=(artifacts/tracking-performance-20260719_134544/orb-only-2x/tracking_benchmark.json artifacts/tracking-performance-20260719_134544/orb-only-3x/tracking_benchmark.json)
python3 -m orb_slam_bringup.tracking_benchmark select --source-camera-hz 15 --result "${RESULT_PATHS[@]}"
