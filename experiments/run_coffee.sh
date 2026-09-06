#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export MAX_PARALLEL="${MAX_PARALLEL:-1}"

exec "$SCRIPT_DIR/run_experiment_plan.sh" \
  --tasks coffee_full_rerun \
  --no-terminal-hold \
  "$@"
