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
  --tasks drone_demo_stats,drone_demo_fp \
  --no-terminal-hold \
  "${MAIN_ARGS[@]}"

if (( WITH_FOLLOWUPS )); then
  "$SCRIPT_DIR/run_drone_demo_outliers.sh"
  "$SCRIPT_DIR/run_drone_demo_stats_1sp_outliers.sh"
fi
