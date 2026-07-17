#!/usr/bin/env bash
# Offline stereo ORB-SLAM rotation-center calibration.
# Sources the local workspace and writes reports under tools/, same layout as
# run_lidar_rotation_center_calibration.sh.
#
# Default output:
#   tools/stereo-rotation-center-report/<bag-basename>/
# e.g. bag /home/duc/robot/bag/inplace-rotate-1rev
#   -> tools/stereo-rotation-center-report/inplace-rotate-1rev/{report.html,...}
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPORT_ROOT="${SCRIPT_DIR}/stereo-rotation-center-report"

BAG_PATH="${BAG_PATH:-/home/duc/robot/bag/inplace-rotate}"
# Empty means "derive from bag basename under REPORT_ROOT"
OUTPUT_DIR="${OUTPUT_DIR:-}"
OVERWRITE="${OVERWRITE:-1}"

usage() {
  cat <<EOF
Usage: $(basename "$0") [options]

Runs stereo_rotation_center_check (ORB-SLAM3 stereo → planar center →
base_link→camera_link xy vs bag /tf_static).

Default bag:
  ${BAG_PATH}

Default output (under tools/, gitignored):
  ${REPORT_ROOT}/<bag-basename>/
    report.html
    calibration.json
    centers.csv
    trajectory.csv

Examples:
  $(basename "$0")
  $(basename "$0") --bag /home/duc/robot/bag/inplace-rotate-1rev
  $(basename "$0") --bag /path/to/bag --output ${REPORT_ROOT}/my-run

Options:
  --bag PATH              Bag directory or MCAP (default: ${BAG_PATH})
  --output DIR            Output directory (default: ${REPORT_ROOT}/<bag-name>)
  --no-overwrite          Fail if output already exists
  --vocab PATH            ORB vocabulary (default: ament share orb_slam3_vendor)
  --settings PATH         Stereo settings YAML (default: tasterobot_stereo.yaml)
  -h, --help              Show this help

Environment:
  BAG_PATH, OUTPUT_DIR, OVERWRITE (0/1)
  ROS_SETUP                Optional distro setup.bash (auto-detected if unset)

Exit codes (from tool):
  0  CONSISTENT
  1  operational failure
  2  LIKELY_OFFSET_ERROR
  3  INCONCLUSIVE
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

# Resolve bag path and default output dir from bag basename (like lidar multi-bag runs).
BAG_PATH="$(cd "$(dirname "${BAG_PATH}")" && pwd)/$(basename "${BAG_PATH}")"
BAG_BASENAME="$(basename "${BAG_PATH}")"
# Strip trailing .mcap if someone passed the file directly.
if [[ "${BAG_BASENAME}" == *.mcap ]]; then
  BAG_BASENAME="$(basename "$(dirname "${BAG_PATH}")")"
  BAG_PATH="$(dirname "${BAG_PATH}")"
fi

if [[ -z "${OUTPUT_DIR}" ]]; then
  OUTPUT_DIR="${REPORT_ROOT}/${BAG_BASENAME}"
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

if ! ros2 pkg prefix orb_slam3_wrapper >/dev/null 2>&1; then
  echo "error: package orb_slam3_wrapper not found after sourcing." >&2
  echo "hint:  cd ${REPO_ROOT} && colcon build --packages-select orb_slam3_wrapper && source install/setup.bash" >&2
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
echo "note:   progress prints as [  N%] stages + bag read bar on stderr"
echo "note:   ORB stereo pass can take several minutes on multi-turn bags"
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
json="${OUTPUT_DIR}/calibration.json"
echo
if [[ -f "${report}" ]]; then
  echo "elapsed: ${elapsed}s"
  echo "report:  ${report}"
  echo "json:    ${json}"
  echo "open:    xdg-open ${report}"
  if [[ -f "${json}" ]] && command -v python3 >/dev/null 2>&1; then
    python3 - "${json}" <<'PY'
import json, sys
d = json.load(open(sys.argv[1]))
cls = d.get("result_class", "?")
rec = d.get("recorded_camera_link_xy", {})
est = d.get("estimated_camera_link_xy", {})
print(
    f"result:  {cls}  "
    f"recorded_xy=({rec.get('x_m', float('nan')):.4f}, {rec.get('y_m', float('nan')):.4f})  "
    f"estimated_xy=({est.get('x_m', float('nan')):.4f}, {est.get('y_m', float('nan')):.4f})"
)
PY
  fi
else
  echo "error: report.html was not written (exit ${status}, elapsed ${elapsed}s)" >&2
fi

exit "${status}"
