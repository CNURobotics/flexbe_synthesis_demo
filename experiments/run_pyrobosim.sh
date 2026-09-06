#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

WITH_FOLLOWUPS=0
MAIN_ARGS=()
while (($#)); do
  case "$1" in
    --with-followups)
      WITH_FOLLOWUPS=1
      ;;
    *)
      MAIN_ARGS+=("$1")
      ;;
  esac
  shift
done

export MAX_PARALLEL="${MAX_PARALLEL:-1}"

"$SCRIPT_DIR/run_experiment_plan.sh" \
  --tasks pyrobosim_stats,pyrobosim_p1 \
  --no-terminal-hold \
  "${MAIN_ARGS[@]}"

if (( WITH_FOLLOWUPS )); then
  "$SCRIPT_DIR/run_pyrobosim_p1_outliers.sh"
  "$SCRIPT_DIR/run_pyrobosim_p1_stats.sh"
fi
