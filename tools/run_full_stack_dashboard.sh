#!/usr/bin/env bash
# Full-stack bag replay: ORB-SLAM3 + lidar mapper + metrics + dashboard.
# Sources the local workspace, writes report artifacts under tools/, and runs
# orb_slam_report_check when the bag finishes (same layout spirit as lidar/stereo
# calib scripts).
#
# Default output:
#   tools/full-stack-report/<bag-basename>-<timestamp>/
#     report.html
#     metrics.json
#     events.jsonl
#     final-map.pgm / final-map.yaml
#     map-revision-*.png
#     orb_trajectory.csv / wheel_trajectory.csv / corrected_trajectory.csv
#     tf_audit.json
#
# Live dashboard (while running):
#   http://<dashboard_host>:51871/
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPORT_ROOT="${SCRIPT_DIR}/full-stack-report"

# Prefer bag under /home/duc/robot/bag; fall back to profile default path.
DEFAULT_BAG="/home/duc/robot/bag/20260713_152907"
if [[ ! -e "${DEFAULT_BAG}" ]]; then
  DEFAULT_BAG="/home/duc/robot/20260713_152907"
fi

DEFAULT_RATE="1.0"
DEFAULT_DOMAIN="42"
DEFAULT_DASHBOARD_HOST="127.0.0.1"

BAG_PATH="${BAG_PATH:-${DEFAULT_BAG}}"
ARTIFACT_DIR="${ARTIFACT_DIR:-}"
RATE="${RATE:-${DEFAULT_RATE}}"
# Prefer explicit --domain; env ROS_DOMAIN_ID only if already set by caller.
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-${DEFAULT_DOMAIN}}"
DASHBOARD_HOST="${DASHBOARD_HOST:-${DEFAULT_DASHBOARD_HOST}}"
START_DASHBOARD="${START_DASHBOARD:-1}"
PUBLISH_ODOM_TF="${PUBLISH_ODOM_TF:-true}"
RUN_REPORT_CHECK="${RUN_REPORT_CHECK:-1}"
OPEN_BROWSER="${OPEN_BROWSER:-0}"
EXTRA_LAUNCH_ARGS=()

usage() {
  cat <<EOF
Usage: $(basename "$0") [options]

Full ORB-SLAM3 + lidar mapper stack on a bag, with the read-only HTTP dashboard
and metrics/report artifacts under tools/ (gitignored).

Default bag:
  ${DEFAULT_BAG}

Default artifacts:
  ${REPORT_ROOT}/<bag-name>-<YYYYMMDD-HHMMSS>/

While running, open the dashboard:
  http://<host>:51871/
  (default host ${DEFAULT_DASHBOARD_HOST}; use --dashboard-host 0.0.0.0 for LAN/Tailscale)

After the bag ends, launch shuts down, metrics flush, and (unless disabled)
  orb_slam_report_check <artifact_dir>/metrics.json
  is run. report.html is written by the metrics recorder regardless.

Examples:
  $(basename "$0")
  $(basename "$0") --bag /home/duc/robot/bag/20260713_152907
  # Forward/back bag with Handsfree 200 Hz /imu (mapper IMU deskew default on):
  $(basename "$0") --bag /home/duc/robot/bag/forward-and-back-origin --domain 91
  $(basename "$0") --rate 2.0 --domain 91 --dashboard-host 0.0.0.0
  $(basename "$0") --no-dashboard --output ${REPORT_ROOT}/ci-run
  $(basename "$0") --open   # open dashboard URL if xdg-open exists

Options:
  --bag PATH              Bag directory (default: ${DEFAULT_BAG})
  --output DIR            Artifact directory (default: ${REPORT_ROOT}/<bag>-<ts>)
  --rate N                Playback rate (default: ${DEFAULT_RATE})
  --domain N              ROS_DOMAIN_ID (default: ${DEFAULT_DOMAIN})
  --dashboard-host HOST   Bind host for dashboard (default: ${DEFAULT_DASHBOARD_HOST})
  --no-dashboard          Do not start the HTTP dashboard
  --no-report-check       Skip orb_slam_report_check after exit
  --no-odom-tf            Do not publish odom→base_link from /odom_wheel
  --open                  Try xdg-open on the dashboard URL after launch starts
  -h, --help              Show this help

Environment:
  BAG_PATH, ARTIFACT_DIR, RATE, ROS_DOMAIN_ID, DASHBOARD_HOST
  START_DASHBOARD (0/1), RUN_REPORT_CHECK (0/1), OPEN_BROWSER (0/1)
  ROS_SETUP                Optional distro setup.bash (auto-detected if unset)

Artifacts written under the output directory:
  report.html  metrics.json  events.jsonl  final-map.*  map-revision-*.png
  *trajectory.csv  tf_audit.json
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag)
      BAG_PATH="${2:?--bag requires a path}"
      shift 2
      ;;
    --output)
      ARTIFACT_DIR="${2:?--output requires a path}"
      shift 2
      ;;
    --rate)
      RATE="${2:?--rate requires a value}"
      shift 2
      ;;
    --domain)
      ROS_DOMAIN_ID="${2:?--domain requires a value}"
      shift 2
      ;;
    --dashboard-host)
      DASHBOARD_HOST="${2:?--dashboard-host requires a host}"
      shift 2
      ;;
    --no-dashboard)
      START_DASHBOARD=0
      shift
      ;;
    --no-report-check)
      RUN_REPORT_CHECK=0
      shift
      ;;
    --no-odom-tf)
      PUBLISH_ODOM_TF=false
      shift
      ;;
    --open)
      OPEN_BROWSER=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      # Forward unknown args to ros2 launch (e.g. future launch args).
      EXTRA_LAUNCH_ARGS+=("$1")
      shift
      ;;
  esac
done

if [[ ! -e "${BAG_PATH}" ]]; then
  echo "error: bag not found: ${BAG_PATH}" >&2
  echo "hint:  place bag at ${DEFAULT_BAG} or pass --bag PATH" >&2
  exit 1
fi

BAG_PATH="$(cd "$(dirname "${BAG_PATH}")" && pwd)/$(basename "${BAG_PATH}")"
BAG_BASENAME="$(basename "${BAG_PATH}")"
if [[ "${BAG_BASENAME}" == *.mcap ]]; then
  BAG_BASENAME="$(basename "$(dirname "${BAG_PATH}")")"
  BAG_PATH="$(dirname "${BAG_PATH}")"
fi

if [[ -z "${ARTIFACT_DIR}" ]]; then
  TS="$(date +%Y%m%d-%H%M%S)"
  ARTIFACT_DIR="${REPORT_ROOT}/${BAG_BASENAME}-${TS}"
fi

if [[ ! -f "${REPO_ROOT}/install/setup.bash" ]]; then
  echo "error: workspace not built. Expected ${REPO_ROOT}/install/setup.bash" >&2
  echo "hint:  cd ${REPO_ROOT} && colcon build --packages-select \\" >&2
  echo "         orb_slam3_wrapper orb_lidar_mapper orb_slam_dashboard orb_slam_bringup" >&2
  exit 1
fi

# Source ROS distro if needed, then the local overlay.
# Colcon/ROS setup scripts read optional vars without defaults under nounset.
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

for pkg in orb_slam_bringup orb_slam3_wrapper orb_lidar_mapper; do
  if ! ros2 pkg prefix "${pkg}" >/dev/null 2>&1; then
    echo "error: package ${pkg} not found after sourcing." >&2
    echo "hint:  cd ${REPO_ROOT} && colcon build --packages-select \\" >&2
    echo "         orb_slam3_wrapper orb_lidar_mapper orb_slam_dashboard orb_slam_bringup \\" >&2
    echo "       && source install/setup.bash" >&2
    exit 1
  fi
done

mkdir -p "${ARTIFACT_DIR}"
ARTIFACT_DIR="$(cd "${ARTIFACT_DIR}" && pwd)"

START_DASHBOARD_ARG="false"
if [[ "${START_DASHBOARD}" == "1" || "${START_DASHBOARD}" == "true" ]]; then
  START_DASHBOARD_ARG="true"
fi

export ROS_DOMAIN_ID

URL_HOST="${DASHBOARD_HOST}"
if [[ "${DASHBOARD_HOST}" == "0.0.0.0" || "${DASHBOARD_HOST}" == "::" || -z "${DASHBOARD_HOST}" ]]; then
  URL_HOST="127.0.0.1"
fi
DASHBOARD_URL="http://${URL_HOST}:51871/"

CMD=(
  ros2 launch orb_slam_bringup bag_replay.launch.py
  "bag_path:=${BAG_PATH}"
  "artifact_dir:=${ARTIFACT_DIR}"
  "rate:=${RATE}"
  "ros_domain_id:=${ROS_DOMAIN_ID}"
  "publish_odom_tf:=${PUBLISH_ODOM_TF}"
  "start_dashboard:=${START_DASHBOARD_ARG}"
  "dashboard_host:=${DASHBOARD_HOST}"
)
if [[ ${#EXTRA_LAUNCH_ARGS[@]} -gt 0 ]]; then
  CMD+=("${EXTRA_LAUNCH_ARGS[@]}")
fi

echo "repo:      ${REPO_ROOT}"
echo "bag:       ${BAG_PATH}"
echo "artifacts: ${ARTIFACT_DIR}"
echo "domain:    ${ROS_DOMAIN_ID}"
echo "rate:      ${RATE}"
echo "dashboard: ${START_DASHBOARD_ARG}  host=${DASHBOARD_HOST}"
if [[ "${START_DASHBOARD_ARG}" == "true" ]]; then
  echo "open:      ${DASHBOARD_URL}"
  echo "state:     curl -sf ${DASHBOARD_URL}state"
fi
echo "run:       ${CMD[*]}"
echo "note:      bag end → launch shutdown → metrics flush → report.html"
echo "note:      Ctrl+C also stops the stack; report may still flush"
echo

if [[ "${OPEN_BROWSER}" == "1" && "${START_DASHBOARD_ARG}" == "true" ]]; then
  if command -v xdg-open >/dev/null 2>&1; then
    # Delay so the server can bind before the browser hits it.
    ( sleep 3; xdg-open "${DASHBOARD_URL}" >/dev/null 2>&1 || true ) &
  fi
fi

start_ts=$(date +%s)
set +e
if command -v stdbuf >/dev/null 2>&1; then
  stdbuf -oL -eL "${CMD[@]}"
else
  "${CMD[@]}"
fi
status=$?
set -e
elapsed=$(( $(date +%s) - start_ts ))

report="${ARTIFACT_DIR}/report.html"
metrics="${ARTIFACT_DIR}/metrics.json"

echo
echo "elapsed:   ${elapsed}s"
echo "artifacts: ${ARTIFACT_DIR}"

if [[ -f "${report}" ]]; then
  echo "report:    ${report}"
  echo "open:      xdg-open ${report}"
else
  echo "warn:      report.html not written yet (exit ${status})" >&2
fi

if [[ -f "${metrics}" ]]; then
  echo "metrics:   ${metrics}"
  if [[ "${RUN_REPORT_CHECK}" == "1" || "${RUN_REPORT_CHECK}" == "true" ]]; then
    echo
    echo "=== orb_slam_report_check ==="
    set +e
    if command -v orb_slam_report_check >/dev/null 2>&1; then
      orb_slam_report_check "${metrics}"
      check_status=$?
    else
      # Fallback when entry point not on PATH but package is sourced.
      python3 -c "
from orb_slam_bringup.report import check_main
import sys
sys.exit(check_main(['orb_slam_report_check', '${metrics}']))
" 2>/dev/null || ros2 run orb_slam_bringup orb_slam_report_check "${metrics}"
      check_status=$?
    fi
    set -e
    if [[ ${check_status} -eq 0 ]]; then
      echo "check:     PASS (exit 0)"
    else
      echo "check:     FAIL (exit ${check_status}) — see gates above" >&2
      # Prefer check status if launch already succeeded.
      if [[ ${status} -eq 0 ]]; then
        status=${check_status}
      fi
    fi
  fi
else
  echo "warn:      metrics.json missing — bag may have been interrupted early" >&2
fi

if [[ "${START_DASHBOARD_ARG}" == "true" ]]; then
  echo
  echo "dashboard was: ${DASHBOARD_URL}"
fi

echo
echo "ls:        ls -la ${ARTIFACT_DIR}"

exit "${status}"
