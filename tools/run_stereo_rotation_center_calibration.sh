#!/usr/bin/env bash
# Offline stereo ORB rotation-center calibration.
# Sources the local workspace and writes report.html under tools/.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

BAG_PATH="${BAG_PATH:-/home/duc/robot/bag/inplace-rotate}"
OUTPUT_DIR="${OUTPUT_DIR:-${SCRIPT_DIR}/stereo-rotation-center-report}"
OVERWRITE="${OVERWRITE:-1}"

usage() {
  cat <<EOF
Usage: $(basename "$0") [options]

Runs stereo_rotation_center_check with workspace sourcing and writes:
  ${OUTPUT_DIR}/report.html
  ${OUTPUT_DIR}/calibration.json
  ${OUTPUT_DIR}/centers.csv
  ${OUTPUT_DIR}/trajectory.csv

The tool prints stage percentages while reading the bag, tracking with
ORB-SLAM3 stereo, estimating the rotation center, and writing the report.

Options:
  --bag PATH              Bag directory or MCAP (default: ${BAG_PATH})
  --output DIR            Output directory (default: ${OUTPUT_DIR})
  --no-overwrite          Fail if output already exists
  --vocab PATH            Forwarded to the tool (ORB vocabulary)
  --settings PATH         Forwarded to the tool (stereo settings YAML)
  -h, --help              Show this help

Environment:
  BAG_PATH, OUTPUT_DIR, OVERWRITE (0/1)
  ROS_SETUP                Optional distro setup.bash (auto-detected if unset)
EOF
}

EXTRA_ARGS=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag)
      BAG_PATH="${2:?--bag requires a path}"
      shift 2
      ;;
    --output)
      OUTPUT_DIR="${2:?--output requires a path}"
      shift 2
      ;;
    --no-overwrite)
      OVERWRITE=0
      shift
      ;;
    --vocab|--settings)
      EXTRA_ARGS+=("$1" "${2:?$1 requires a value}")
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "error: unknown option: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
done

if [[ ! -e "${BAG_PATH}" ]]; then
  echo "error: bag not found: ${BAG_PATH}" >&2
  exit 1
fi

if [[ ! -f "${REPO_ROOT}/install/setup.bash" ]]; then
  echo "error: workspace not built. Expected ${REPO_ROOT}/install/setup.bash" >&2
  echo "hint:  cd ${REPO_ROOT} && colcon build --packages-select orb_slam3_wrapper" >&2
  exit 1
fi

# Source ROS distro if needed, then the local overlay.
# Colcon/ROS setup scripts read optional vars (e.g. COLCON_TRACE) without defaults,
# so nounset must be off while sourcing them.
set +u
if [[ -n "${ROS_SETUP:-}" ]]; then
  # shellcheck disable=SC1090
  source "${ROS_SETUP}"
elif [[ -z "${ROS_DISTRO:-}" ]]; then
  for candidate in \
    /opt/ros/kilted/setup.bash \
    /opt/ros/jazzy/setup.bash \
    /opt/ros/humble/setup.bash \
    /opt/ros/iron/setup.bash; do
    if [[ -f "${candidate}" ]]; then
      # shellcheck disable=SC1090
      source "${candidate}"
      break
    fi
  done
fi

# shellcheck disable=SC1091
source "${REPO_ROOT}/install/setup.bash"
set -u

if ! command -v ros2 >/dev/null 2>&1; then
  echo "error: ros2 not found after sourcing. Set ROS_SETUP to your distro setup.bash." >&2
  exit 1
fi

mkdir -p "${OUTPUT_DIR}"

CMD=(
  ros2 run orb_slam3_wrapper stereo_rotation_center_check
  --bag "${BAG_PATH}"
  --output "${OUTPUT_DIR}"
)
if [[ "${OVERWRITE}" == "1" ]]; then
  CMD+=(--overwrite)
fi
if [[ ${#EXTRA_ARGS[@]} -gt 0 ]]; then
  CMD+=("${EXTRA_ARGS[@]}")
fi

echo "repo:   ${REPO_ROOT}"
echo "bag:    ${BAG_PATH}"
echo "output: ${OUTPUT_DIR}"
echo "run:    ${CMD[*]}"
echo "note:   progress prints as [  N%] stages on stderr"
echo

start_ts=$(date +%s)
set +e
# Unbuffered stderr so progress updates live under pipes/ros2.
if command -v stdbuf >/dev/null 2>&1; then
  stdbuf -oL -eL "${CMD[@]}"
else
  "${CMD[@]}"
fi
status=$?
set -e
elapsed=$(( $(date +%s) - start_ts ))

report="${OUTPUT_DIR}/report.html"
echo
if [[ -f "${report}" ]]; then
  echo "elapsed: ${elapsed}s"
  echo "report:  ${report}"
  echo "open:    xdg-open ${report}"
else
  echo "error: report.html was not written (exit ${status}, elapsed ${elapsed}s)" >&2
fi

exit "${status}"
