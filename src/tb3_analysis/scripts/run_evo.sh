#!/usr/bin/env bash
# =============================================================================
# Phase 5 - Trajectory analysis with evo.
#
# Compares the estimated trajectory (/odom = EKF output in Mode B, or GT TF in
# Mode A) against ground truth (/gt_odom) directly from a rosbag2 (SQLite3).
#
#   * evo_ape : Absolute Pose Error  -> long-term drift of the estimate.
#   * evo_rpe : Relative Pose Error with --delta -> short-term "spikes" caused
#               by the sensor noise (position jitter over small windows).
#
# Usage:
#   ./run_evo.sh <rosbag_dir> [ref_topic] [est_topic]
#
# Defaults: ref_topic=/gt_odom  est_topic=/odom
#
# Requires: pip install evo   (evo >= 1.11 supports `bag2`).
# =============================================================================
set -euo pipefail

BAG="${1:?Usage: run_evo.sh <rosbag_dir> [ref_topic] [est_topic]}"
REF_TOPIC="${2:-/gt_odom}"
EST_TOPIC="${3:-/odom}"
OUTDIR="${BAG%/}_evo"
mkdir -p "${OUTDIR}"

echo "=== evo on bag: ${BAG} ==="
echo "    reference : ${REF_TOPIC}"
echo "    estimate  : ${EST_TOPIC}"
echo "    output    : ${OUTDIR}"

if ! command -v evo_ape >/dev/null 2>&1; then
  echo "ERROR: evo not found. Install with: pip install evo" >&2
  exit 1
fi

# --- Absolute Pose Error (ATE): overall trajectory divergence ---
echo "--- evo_ape (ATE) ---"
evo_ape bag2 "${BAG}" "${REF_TOPIC}" "${EST_TOPIC}" \
  -va --align --correct_scale \
  --save_results "${OUTDIR}/ape.zip" \
  --plot_mode xy \
  --save_plot "${OUTDIR}/ape_plot.pdf"

# --- Relative Pose Error (RPE) with a small delta: short-term jitter/spikes ---
echo "--- evo_rpe (RPE, delta=1 frame) ---"
evo_rpe bag2 "${BAG}" "${REF_TOPIC}" "${EST_TOPIC}" \
  -va --delta 1 --delta_unit f \
  --save_results "${OUTDIR}/rpe.zip" \
  --plot_mode xy \
  --save_plot "${OUTDIR}/rpe_plot.pdf"

echo "=== done. Results in ${OUTDIR} ==="
echo "Tip: compare two runs with:"
echo "  evo_res ${OUTDIR}/ape.zip <other_mode>_evo/ape.zip -p"
