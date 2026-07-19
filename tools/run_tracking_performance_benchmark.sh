#!/usr/bin/env bash
set -euo pipefail

# Tools path discovery
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"

# Default Arguments
BAG_PATH="/home/duc/robot/20260713_152907"
OUTPUT_DIR="artifacts/tracking-performance-$(date +%Y%m%d_%H%M%S)"
DOMAIN=95
SOURCE_CAMERA_HZ=30
START_RATE=2
MAX_RATE=8
MIN_DURATION_S=10
ROS_SETUP=""

# Argument parsing
while [[ $# -gt 0 ]]; do
  case $1 in
    --bag) BAG_PATH="$2"; shift 2 ;;
    --output) OUTPUT_DIR="$2"; shift 2 ;;
    --domain) DOMAIN="$2"; shift 2 ;;
    --source-camera-hz) SOURCE_CAMERA_HZ="$2"; shift 2 ;;
    --start-rate) START_RATE="$2"; shift 2 ;;
    --max-rate) MAX_RATE="$2"; shift 2 ;;
    --min-duration-s) MIN_DURATION_S="$2"; shift 2 ;;
    --ros-setup) ROS_SETUP="$2"; shift 2 ;;
    *) echo "Unknown parameter: $1"; exit 1 ;;
  esac
done

# Validation
if awk -v val="$SOURCE_CAMERA_HZ" 'BEGIN { if (val <= 0) exit 0; else exit 1 }'; then
  echo "Error: --source-camera-hz must be positive."
  exit 1
fi
if awk -v val="$MIN_DURATION_S" 'BEGIN { if (val <= 0) exit 0; else exit 1 }'; then
  echo "Error: --min-duration-s must be positive."
  exit 1
fi
if [ "$START_RATE" -lt 1 ]; then
  echo "Error: --start-rate must be at least 1."
  exit 1
fi
if [ "$MAX_RATE" -lt "$START_RATE" ]; then
  echo "Error: --max-rate must be at least --start-rate."
  exit 1
fi

if [ ! -d "$BAG_PATH" ] && [ ! -f "$BAG_PATH" ]; then
  echo "Error: Bag path does not exist: $BAG_PATH"
  exit 1
fi

if [ -n "$ROS_SETUP" ]; then
    if [ ! -f "$ROS_SETUP" ]; then
        echo "Error: Provided ros-setup does not exist: $ROS_SETUP"
        exit 1
    fi
    source "$ROS_SETUP"
fi

# Ensure commands are available
if ! command -v ros2 >/dev/null 2>&1; then
    echo "Error: ros2 command not found"
    exit 1
fi

if ! command -v python3 >/dev/null 2>&1; then
    echo "Error: python3 command not found"
    exit 1
fi

if [ ! -f "install/setup.bash" ]; then
    echo "Error: install/setup.bash not found. Please build the workspace."
    exit 1
fi
source install/setup.bash


# Execution
RESULT_PATHS=()
SELECTED_RATE=-1

for RATE in $(seq "$START_RATE" "$MAX_RATE"); do
    CURRENT_DOMAIN=$((DOMAIN + RATE))
    RUN_DIR="${OUTPUT_DIR}/orb-only-${RATE}x"
    
    echo "Running orb_only benchmark at ${RATE}x (Domain: ${CURRENT_DOMAIN})"
    
    ros2 launch orb_slam_bringup bag_replay.launch.py \
      "bag_path:=${BAG_PATH}" \
      "artifact_dir:=${RUN_DIR}" \
      "rate:=${RATE}" \
      "ros_domain_id:=${CURRENT_DOMAIN}" \
      "start_dashboard:=false" \
      "benchmark_mode:=orb_only" \
      "benchmark_min_duration_s:=${MIN_DURATION_S}"
      
    RESULT_FILE="${RUN_DIR}/tracking_benchmark.json"
    if [ ! -f "$RESULT_FILE" ]; then
        echo "Error: Expected output file missing: $RESULT_FILE"
        exit 1
    fi
    
    RESULT_PATHS+=("$RESULT_FILE")
    
    # Try selecting
    SELECT_OUT=$(python3 -m orb_slam_bringup.tracking_benchmark select \
      --source-camera-hz "${SOURCE_CAMERA_HZ}" \
      --result "${RESULT_PATHS[@]}")
      
    # If the tool prints out valid JSON, grab the playback rate
    if echo "$SELECT_OUT" | grep -q '"playback_rate":'; then
        SELECTED_RATE=$(echo "$SELECT_OUT" | python3 -c 'import sys, json; print(json.load(sys.stdin)["playback_rate"])')
        echo "Selection successful: Rate ${SELECTED_RATE}x chosen."
        break
    else
        echo "No saturation yet, continuing..."
    fi
done

if [ "$SELECTED_RATE" -eq -1 ]; then
    echo "Error: Reached --max-rate ($MAX_RATE) without selecting a rate."
    exit 1
fi

FULL_STACK_DIR="${OUTPUT_DIR}/full-stack-${SELECTED_RATE}x"
FULL_STACK_DOMAIN=$((DOMAIN + MAX_RATE + 1))

echo "Running full_stack benchmark at ${SELECTED_RATE}x (Domain: ${FULL_STACK_DOMAIN})"

ros2 launch orb_slam_bringup bag_replay.launch.py \
  "bag_path:=${BAG_PATH}" \
  "artifact_dir:=${FULL_STACK_DIR}" \
  "rate:=${SELECTED_RATE}" \
  "ros_domain_id:=${FULL_STACK_DOMAIN}" \
  "start_dashboard:=false" \
  "benchmark_mode:=full_stack" \
  "benchmark_min_duration_s:=${MIN_DURATION_S}"

FULL_STACK_RESULT_FILE="${FULL_STACK_DIR}/tracking_benchmark.json"
if [ ! -f "$FULL_STACK_RESULT_FILE" ]; then
    echo "Error: Expected full stack output file missing: $FULL_STACK_RESULT_FILE"
    exit 1
fi

COMPARISON_FILE="${OUTPUT_DIR}/comparison.json"
mkdir -p "$OUTPUT_DIR"

echo "Running comparison..."

set +e
python3 -m orb_slam_bringup.tracking_benchmark compare \
    --orb-only "$RESULT_FILE" \
    --full-stack "$FULL_STACK_RESULT_FILE" \
    --output "$COMPARISON_FILE"
COMP_STATUS=$?
set -e

echo "=== Benchmark Complete ==="
echo "ORB-only Artifacts: ${RUN_DIR}"
echo "Full-stack Artifacts: ${FULL_STACK_DIR}"
echo "Comparison Result: ${COMPARISON_FILE}"
echo "Selected Rate: ${SELECTED_RATE}x"

exit $COMP_STATUS
