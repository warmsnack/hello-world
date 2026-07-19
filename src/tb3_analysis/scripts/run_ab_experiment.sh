#!/usr/bin/env bash
# =============================================================================
# End-to-end A/B experiment driver.
#
# Runs Mode B (EKF) then Mode A (ground truth), each for a fixed duration with
# rosbag recording, then produces the Phase-5 analysis artifacts (evo ATE/RPE
# per run + a combined cmd_vel jerk comparison plot).
#
# Usage:
#   ./run_ab_experiment.sh [run_seconds] [eval_dir]
#
# Defaults: run_seconds=60  eval_dir=~/tb3_eval
#
# Prereqs: workspace built + sourced, TURTLEBOT3_MODEL=waffle exported,
#          `pip install evo` for trajectory analysis.
# =============================================================================
set -uo pipefail

RUN_SECONDS="${1:-60}"
EVAL_DIR="${2:-$HOME/tb3_eval}"
mkdir -p "${EVAL_DIR}"

run_mode() {
  local use_gt="$1" label="$2"
  echo "================ Running Mode ${label} (use_gt=${use_gt}) ================"
  ros2 launch tb3_experiment experiment.launch.py \
      use_gt:="${use_gt}" gui:=false record:=true csv_dir:="${EVAL_DIR}" &
  local lpid=$!
  sleep "${RUN_SECONDS}"
  echo "Stopping Mode ${label}..."
  kill -INT "${lpid}" 2>/dev/null || true
  wait "${lpid}" 2>/dev/null || true
  sleep 3
}

run_mode false B
run_mode true  A

echo "================ Analysis ================"
BAG_A="${EVAL_DIR}/rosbag_mode_A"
BAG_B="${EVAL_DIR}/rosbag_mode_B"

if command -v evo_ape >/dev/null 2>&1; then
  [ -d "${BAG_B}" ] && "$(dirname "$0")/run_evo.sh" "${BAG_B}" /gt_odom /odom || true
  [ -d "${BAG_A}" ] && "$(dirname "$0")/run_evo.sh" "${BAG_A}" /gt_odom /odom || true
else
  echo "evo not installed; skipping trajectory analysis (pip install evo)."
fi

python3 "$(dirname "$0")/plot_cmd_vel.py" \
    --bag-a "${BAG_A}" --bag-b "${BAG_B}" \
    --out "${EVAL_DIR}/cmd_vel_compare.png" || true

echo "Done. Artifacts in ${EVAL_DIR}:"
echo "  clearance_A.csv / clearance_B.csv  (min-clearance logs)"
echo "  rosbag_mode_A / rosbag_mode_B      (SQLite3 bags)"
echo "  *_evo/ape_plot.pdf, rpe_plot.pdf   (trajectory error)"
echo "  cmd_vel_compare.png                (jerk comparison)"
