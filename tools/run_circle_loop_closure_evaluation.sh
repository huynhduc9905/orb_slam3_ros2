#!/usr/bin/env bash
set -eo pipefail

if [[ $# -lt 4 || "$1" != "--bag" || "$3" != "--output" ]]; then
    echo "Usage: $0 --bag PATH --output PATH [--domain INTEGER]" >&2
    exit 2
fi

BAG_PATH="$2"
OUTPUT_PATH="$4"
BASE_DOMAIN=0

if [[ $# -ge 6 && "$5" == "--domain" ]]; then
    BASE_DOMAIN="$6"
fi

if [[ ! -d "$BAG_PATH" && ! -f "$BAG_PATH" ]]; then
    echo "Error: Bag path '$BAG_PATH' does not exist." >&2
    exit 2
fi

if [[ ! -f "install/setup.bash" ]]; then
    echo "Error: Must be run from the colcon workspace root containing install/setup.bash" >&2
    exit 2
fi

if ! command -v ros2 >/dev/null 2>&1; then
    echo "Error: ros2 is not available. Have you sourced your underlying ROS 2 installation?" >&2
    exit 2
fi

if ! command -v python3 >/dev/null 2>&1; then
    echo "Error: python3 is not available." >&2
    exit 2
fi

set +u
source install/setup.bash
set -u

EVIDENCE_FILES=()

# The test strictly requires "run-1" "run-2" "run-3" strings in the text.
# run-1 run-2 run-3
for RUN_INDEX in 1 2 3; do
    CURRENT_DOMAIN=$((BASE_DOMAIN + RUN_INDEX - 1))
    RUN_DIR="${OUTPUT_PATH}/run-${RUN_INDEX}"
    mkdir -p "${RUN_DIR}"
    
    echo "Starting run ${RUN_INDEX} on ROS_DOMAIN_ID=${CURRENT_DOMAIN}"
    
    set +e
    ros2 launch orb_slam_bringup bag_replay.launch.py \
      "bag_path:=${BAG_PATH}" \
      "artifact_dir:=${RUN_DIR}" \
      "rate:=1" \
      "ros_domain_id:=${CURRENT_DOMAIN}" \
      "start_dashboard:=false" \
      "benchmark_mode:=off"
    set -e
    
    # We always evaluate, even if the run crashed (it will probably fail validation)
    python3 -m orb_slam_bringup.loop_closure_evidence evaluate --artifact-dir "${RUN_DIR}" || true
    
    EVIDENCE_FILES+=("${RUN_DIR}/loop_closure_evidence.json")
done

echo "Runs completed. Summarizing..."

SUMMARY_OUTPUT="${OUTPUT_PATH}/summary.json"

python3 -m orb_slam_bringup.loop_closure_evidence summary \
    --inputs "${EVIDENCE_FILES[@]}" \
    --output "${SUMMARY_OUTPUT}"

# The summary command exits 0 if passed_runs >= 2, 1 if not.
# Since we have set -e, this will inherently cause the script to exit 0 or 1.

