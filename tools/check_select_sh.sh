#!/bin/bash
RESULT_PATHS=(artifacts/tracking-performance-20260719_134544/orb-only-2x/tracking_benchmark.json artifacts/tracking-performance-20260719_134544/orb-only-3x/tracking_benchmark.json)
SOURCE_CAMERA_HZ=15
export PYTHONPATH=./orb_slam_bringup
SELECT_OUT=$(python3 -m orb_slam_bringup.tracking_benchmark select --source-camera-hz "${SOURCE_CAMERA_HZ}" --result "${RESULT_PATHS[@]}")
echo "Output:"
echo "$SELECT_OUT"
if echo "$SELECT_OUT" | grep -q '"playback_rate":'; then
  SELECTED_RATE=$(echo "$SELECT_OUT" | python3 -c 'import sys, json; print(json.load(sys.stdin)["playback_rate"])')
  echo "Selection successful: Rate ${SELECTED_RATE}x chosen."
else
  echo "No saturation yet, continuing..."
fi
