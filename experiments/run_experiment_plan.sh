#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE_ROOT="${WORKSPACE_ROOT:-$(cd "$REPO_ROOT/../.." && pwd)}"
SETUP_FILE="${SETUP_FILE:-$WORKSPACE_ROOT/install/setup.bash}"
SYNTHESIS_HOME="${FLEXBE_SYNTHESIS_HOME:-$HOME/.flexbe_synthesis}"
RUN_LOG="$SYNTHESIS_HOME/experiment_run.log"
STATE_DIR="$SYNTHESIS_HOME/run_state"
TASK_LOG_DIR="$SYNTHESIS_HOME/run_logs"
MAX_PARALLEL="${MAX_PARALLEL:-2}"
READY_TIMEOUT_S="${READY_TIMEOUT_S:-300}"
LAUNCH_TIMEOUT_S="${LAUNCH_TIMEOUT_S:-20}"
EXPERIMENT_TIMEOUT_S_OVERRIDE="${EXPERIMENT_TIMEOUT_S_OVERRIDE:-}"

TASKS=(
  coffee_full_rerun
  two_rivers_full_rerun
  pyrobosim_stats
  pyrobosim_p1
  drone_demo_stats
  drone_demo_fp
)

SLOW_FIRST_TASKS=(
  pyrobosim_p1
  drone_demo_fp
  pyrobosim_stats
  drone_demo_stats
  coffee_full_rerun
  two_rivers_full_rerun
)

usage() {
  cat <<USAGE
Usage: $(basename "$0") [--clear] [--slow-first] [--super-long] [--skip-long] [--skip-fp] [--no-terminal-hold]

Runs the experiments documented in experiments/experiment_run.md with at most
two active experiment runs at a time. The scheduler prints overall status in
this terminal and each experiment opens in its own terminal window when a
terminal emulator is available.

This is a from-scratch run: FLEXBE_SYNTHESIS_HOME must not exist or must be
empty, or the run aborts (move it aside or empty it yourself first). Once
confirmed empty (or created fresh), the first thing written into it is
computer.txt (date/uname -a/lscpu/free -h), before workspace initialization.

Options:
  --clear             delete FLEXBE_SYNTHESIS_HOME first for a fresh start
  --slow-first        launch the slower experiment runs before short reruns
  --super-long        use a 36000s synthesis timeout for every experiment row
  --skip-long         do not launch pyrobosim or drone tasks
  --skip-fp           do not launch the drone_demo_fp task
  --no-terminal-hold  do not keep per-task terminals open after completion

Environment overrides:
  WORKSPACE_ROOT      ROS workspace root (default: $WORKSPACE_ROOT)
  SETUP_FILE          setup file to source (default: $SETUP_FILE)
  FLEXBE_SYNTHESIS_HOME
                      synthesis output root (default: $SYNTHESIS_HOME)
  MAX_PARALLEL        concurrent experiments (default: 2)
  READY_TIMEOUT_S     workspace-init wait time (default: 300)
  LAUNCH_TIMEOUT_S    task launch wait time (default: 20)
  EXPERIMENT_TIMEOUT_S_OVERRIDE
                      override every per-row synthesis timeout

Logs:
  $RUN_LOG
  $TASK_LOG_DIR/<task>.log
USAGE
}

timestamp() {
  date '+%Y-%m-%d %H:%M:%S %z'
}

log() {
  local message="$*"
  mkdir -p "$(dirname "$RUN_LOG")"
  printf '[%s] %s\n' "$(timestamp)" "$message" | tee -a "$RUN_LOG"
}

die() {
  log "ERROR: $*"
  exit 1
}

validate_config() {
  [[ "$MAX_PARALLEL" =~ ^[1-9][0-9]*$ ]] || die "MAX_PARALLEL must be a positive integer, got: $MAX_PARALLEL"
  [[ "$READY_TIMEOUT_S" =~ ^[1-9][0-9]*$ ]] || die "READY_TIMEOUT_S must be a positive integer, got: $READY_TIMEOUT_S"
  [[ "$LAUNCH_TIMEOUT_S" =~ ^[1-9][0-9]*$ ]] || die "LAUNCH_TIMEOUT_S must be a positive integer, got: $LAUNCH_TIMEOUT_S"
  if [[ -n "$EXPERIMENT_TIMEOUT_S_OVERRIDE" ]]; then
    [[ "$EXPERIMENT_TIMEOUT_S_OVERRIDE" =~ ^[1-9][0-9]*$ ]] || die "EXPERIMENT_TIMEOUT_S_OVERRIDE must be a positive integer, got: $EXPERIMENT_TIMEOUT_S_OVERRIDE"
  fi
}

is_long_task() {
  local task="$1"
  [[ "$task" == pyrobosim_* || "$task" == drone_demo_* ]]
}

filter_long_tasks() {
  local filtered=()
  local task
  for task in "${TASKS[@]}"; do
    if ! is_long_task "$task"; then
      filtered+=("$task")
    fi
  done
  TASKS=("${filtered[@]}")
}

filter_fp_task() {
  local filtered=()
  local task
  for task in "${TASKS[@]}"; do
    if [[ "$task" != drone_demo_fp ]]; then
      filtered+=("$task")
    fi
  done
  TASKS=("${filtered[@]}")
}

ensure_setup() {
  if [[ ! -f "$SETUP_FILE" ]]; then
    printf 'ERROR: Setup file not found: %s\n' "$SETUP_FILE" >&2
    exit 1
  fi
  set +u
  # shellcheck source=/dev/null
  # shellcheck disable=SC1090
  source "$SETUP_FILE"
  set -u
  export FLEXBE_SYNTHESIS_HOME="$SYNTHESIS_HOME"
}

state_file() {
  printf '%s/%s.state' "$STATE_DIR" "$1"
}

write_state() {
  local task="$1"
  local state="$2"
  local rc="${3:-}"
  {
    printf 'task=%s\n' "$task"
    printf 'state=%s\n' "$state"
    printf 'time=%s\n' "$(timestamp)"
    if [[ -n "$rc" ]]; then
      printf 'rc=%s\n' "$rc"
    fi
  } > "$(state_file "$task")"
}

write_failed_state() {
  local task="$1"
  local rc="${2:-1}"
  if [[ -n "${STATE_DIR:-}" && -d "${STATE_DIR:-}" ]]; then
    write_state "$task" FAILED "$rc"
  fi
}

read_state() {
  local task="$1"
  local file
  file="$(state_file "$task")"
  [[ -f "$file" ]] || {
    printf 'PENDING'
    return
  }
  awk -F= '$1 == "state" { print $2; found = 1 } END { if (!found) print "UNKNOWN" }' "$file"
}

run_cmd() {
  printf '\n[%s] $' "$(timestamp)"
  printf ' %q' "$@"
  printf '\n'
  "$@"
}

synthesis_timeout_s() {
  local default_timeout_s="$1"
  printf '%s\n' "${EXPERIMENT_TIMEOUT_S_OVERRIDE:-$default_timeout_s}"
}

init_workspace() {
  if [[ -f "$SYNTHESIS_HOME/workspace_defn.yaml" ]]; then
    log "Workspace already initialized: $SYNTHESIS_HOME/workspace_defn.yaml"
    return
  fi

  log "Initializing FlexBE synthesis workspace from empty $SYNTHESIS_HOME"
  local init_log="$TASK_LOG_DIR/init_workspace.log"
  local ready_marker="$STATE_DIR/init_workspace.ready"
  rm -f "$ready_marker"

  setsid bash -lc "
    set -Eeuo pipefail
    set +u
    source '$SETUP_FILE'
    set -u
    export FLEXBE_SYNTHESIS_HOME='$SYNTHESIS_HOME'
    exec stdbuf -oL -eL ros2 launch drone_demo_flexbe_synthesis drones_gates.launch.py
  " > "$init_log" 2>&1 &
  local launch_pid=$!

  local waited=0
  while (( waited < READY_TIMEOUT_S )); do
    if grep -q 'Ready to begin FlexBE synthesis' "$init_log" 2>/dev/null; then
      touch "$ready_marker"
      break
    fi
    if ! kill -0 "$launch_pid" 2>/dev/null; then
      break
    fi
    sleep 1
    waited=$((waited + 1))
    if (( waited % 15 == 0 )); then
      log "Still waiting for workspace initialization (${waited}s elapsed)"
    fi
  done

  if [[ ! -f "$ready_marker" ]]; then
    log "Workspace initialization log follows:"
    tail -n 80 "$init_log" | tee -a "$RUN_LOG"
    kill -INT "-$launch_pid" 2>/dev/null || true
    local stop_wait=0
    while kill -0 "$launch_pid" 2>/dev/null && (( stop_wait < 30 )); do
      sleep 1
      stop_wait=$((stop_wait + 1))
    done
    if kill -0 "$launch_pid" 2>/dev/null; then
      kill -TERM "-$launch_pid" 2>/dev/null || true
      sleep 5
    fi
    if kill -0 "$launch_pid" 2>/dev/null; then
      kill -KILL "-$launch_pid" 2>/dev/null || true
    fi
    wait "$launch_pid" 2>/dev/null || true
    die "Did not see readiness message within ${READY_TIMEOUT_S}s"
  fi

  log "Workspace readiness detected; stopping initialization launch"
  kill -INT "-$launch_pid" 2>/dev/null || true
  local stop_wait=0
  while kill -0 "$launch_pid" 2>/dev/null && (( stop_wait < 30 )); do
    sleep 1
    stop_wait=$((stop_wait + 1))
  done
  if kill -0 "$launch_pid" 2>/dev/null; then
    log "Initialization launch did not stop after SIGINT; sending SIGTERM"
    kill -TERM "-$launch_pid" 2>/dev/null || true
    sleep 5
  fi
  if kill -0 "$launch_pid" 2>/dev/null; then
    log "Initialization launch did not stop after SIGTERM; sending SIGKILL"
    kill -KILL "-$launch_pid" 2>/dev/null || true
  fi
  wait "$launch_pid" 2>/dev/null || true
  [[ -f "$SYNTHESIS_HOME/workspace_defn.yaml" ]] || die "workspace_defn.yaml was not created"
  log "Workspace initialized"
}

coffee_full_rerun() {
  local coffee_exp="$SYNTHESIS_HOME/experiments/coffee"
  mkdir -p "$coffee_exp/matrices" "$coffee_exp/runs"
  cd "$WORKSPACE_ROOT/src/flexbe_synthesis"
  run_cmd python3 -m flexbe_synthesis_slugs.helpers.coffee_matrix \
    --full \
    --n-trials 5 \
    "$coffee_exp/matrices/coffee_full.yaml"
  run_cmd python3 -m flexbe_synthesis_slugs.helpers.batch_run_harness \
    --matrix "$coffee_exp/matrices/coffee_full.yaml" \
    --out-dir "$coffee_exp/runs/coffee_full_rerun" \
    --state-mappings flexbe_synthesis_generic/mappings/infinite_mappings.yaml \
    --slugs-bin /usr/local/bin/slugs \
    --synthesis-timeout-s "$(synthesis_timeout_s 60)"
  run_cmd python3 -m flexbe_synthesis_slugs.helpers.experiment_postprocess \
    --run-dir "$coffee_exp/runs/coffee_full_rerun" \
    --refresh-summary \
    --split-encoding-tables \
    --split-prefix coffee \
    --summary-plot
}

two_rivers_full_rerun() {
  local two_rivers_exp="$SYNTHESIS_HOME/experiments/two_rivers"
  mkdir -p "$two_rivers_exp/matrices" "$two_rivers_exp/runs"
  cd "$WORKSPACE_ROOT/src/flexbe_synthesis_demo/river_crossing/flexbe_river_crossing_synthesis"
  run_cmd python3 -m flexbe_river_crossing_synthesis.two_rivers_matrix \
    --full \
    --n-trials 5 \
    "$two_rivers_exp/matrices/two_rivers_full.yaml"
  run_cmd python3 -m flexbe_synthesis_slugs.helpers.batch_run_harness \
    --matrix "$two_rivers_exp/matrices/two_rivers_full.yaml" \
    --out-dir "$two_rivers_exp/runs/two_rivers_full_rerun" \
    --state-mappings "$WORKSPACE_ROOT/src/flexbe_synthesis/flexbe_synthesis_generic/mappings/global_mappings.yaml" \
    --slugs-bin /usr/local/bin/slugs \
    --synthesis-timeout-s "$(synthesis_timeout_s 60)"
  run_cmd python3 -m flexbe_synthesis_slugs.helpers.experiment_postprocess \
    --run-dir "$two_rivers_exp/runs/two_rivers_full_rerun" \
    --refresh-summary \
    --summary-plot
}

pyrobosim_stats() {
  local pyrobosim_exp="$SYNTHESIS_HOME/experiments/pyrobosim"
  mkdir -p "$pyrobosim_exp/matrices" "$pyrobosim_exp/runs"
  cd "$WORKSPACE_ROOT/src/flexbe_synthesis_demo/pyrobosim_demo/pyrobosim_flexbe_synthesis"
  run_cmd python3 -m pyrobosim_flexbe_synthesis.pyrobosim_matrix \
    p0 --stats \
    --random-seeds 20 \
    "$pyrobosim_exp/matrices/pyrobosim_stats.yaml"
  run_cmd python3 -m flexbe_synthesis_slugs.helpers.batch_run_harness \
    --matrix "$pyrobosim_exp/matrices/pyrobosim_stats.yaml" \
    --out-dir "$pyrobosim_exp/runs/pyrobosim_stats" \
    --state-mappings "$WORKSPACE_ROOT/src/flexbe_synthesis_demo/pyrobosim_demo/pyrobosim_flexbe_synthesis/mappings/batch_mappings.yaml" \
    --slugs-bin /usr/local/bin/slugs \
    --synthesis-timeout-s "$(synthesis_timeout_s 1800)"
  run_cmd python3 -m flexbe_synthesis_slugs.helpers.experiment_postprocess \
    --run-dir "$pyrobosim_exp/runs/pyrobosim_stats" \
    --refresh-summary \
    --summary-plot
}

# PyRoboSim P1 (door-modelling extension of P0): FullP1 hand-written baseline
# plus the enumerated capability sweep (4S/4F/4SP/4FP) plus the one-hot 1S
# baseline (`--full`), single fixed alphabetic ordering -- see
# pyrobosim_matrix.py's module docstring for why P1 never sweeps ordering.
#
# `--full` (the one-hot 1S row) used to be left off by default -- its own
# help text called it "not solvable within 10 hours" -- but a 2026-08-24
# backfill actually resolved it in ~23,307s (~6.5h) once given a real
# budget, so leaving it off just silently dropped a real, release data
# point (the P1 one-hot-vs-enumerated comparison) from every run since.
# It's genuinely one-hot-P1's slowest configuration, though, so it gets its
# own harness call at a 36000s (10h) budget -- matching the margin already
# used elsewhere in this domain -- rather than folding it into the 28800s
# "fast" call below and risking a false TIME OUT on a slower machine or an
# unlucky CUDD-reordering draw.
#
# FullP1 is split into its own harness call at a 20s cap, not the 28800s the
# enumerated rows get: its well-separation check has never resolved even at
# 43200s across four prior attempts, so spending main-track budget on it
# again would just be waiting to fail the same way again. 20s is
# enough to record a placeholder (TIME OUT) row so summary table generation
# has complete placeholder rows before the real attempt runs; it deliberately does
# NOT try for real resolution, including FullP1's realizability half (~3900s
# on its own), which otherwise would resolve fine at a normal timeout -- that
# real attempt, at a budget actually sized to have a chance, is
# run_pyrobosim_p1_outliers.sh's job, and is now a required follow-up for
# this row, not a conditional one.
pyrobosim_p1() {
  local pyrobosim_exp="$SYNTHESIS_HOME/experiments/pyrobosim"
  mkdir -p "$pyrobosim_exp/matrices" "$pyrobosim_exp/runs"
  cd "$WORKSPACE_ROOT/src/flexbe_synthesis_demo/pyrobosim_demo/pyrobosim_flexbe_synthesis"
  run_cmd python3 -m pyrobosim_flexbe_synthesis.pyrobosim_matrix \
    p1 --full \
    "$pyrobosim_exp/matrices/pyrobosim_p1_default.yaml"

  python3 - "$pyrobosim_exp/matrices/pyrobosim_p1_default.yaml" \
    "$pyrobosim_exp/matrices/pyrobosim_p1_fast.yaml" \
    "$pyrobosim_exp/matrices/pyrobosim_p1_onehot.yaml" \
    "$pyrobosim_exp/matrices/pyrobosim_p1_fullp1.yaml" <<'PYEOF'
import sys
import yaml

src, fast_out, onehot_out, fullp1_out = sys.argv[1:5]
with open(src, encoding='utf-8') as f:
    rows = yaml.safe_load(f)['rows']
fast = [r for r in rows if r.get('column_indicator') not in ('FullP1', '1S')]
onehot = [r for r in rows if r.get('column_indicator') == '1S']
fullp1 = [r for r in rows if r.get('column_indicator') == 'FullP1']
with open(fast_out, 'w', encoding='utf-8') as f:
    yaml.safe_dump({'rows': fast}, f, sort_keys=False)
with open(onehot_out, 'w', encoding='utf-8') as f:
    yaml.safe_dump({'rows': onehot}, f, sort_keys=False)
with open(fullp1_out, 'w', encoding='utf-8') as f:
    yaml.safe_dump({'rows': fullp1}, f, sort_keys=False)
print(
    f"Split pyrobosim_p1_default.yaml: {len(fast)} fast row(s), "
    f"{len(onehot)} one-hot row(s), {len(fullp1)} FullP1 row(s)"
)
PYEOF

  run_cmd python3 -m flexbe_synthesis_slugs.helpers.batch_run_harness \
    --matrix "$pyrobosim_exp/matrices/pyrobosim_p1_fast.yaml" \
    --out-dir "$pyrobosim_exp/runs/pyrobosim_p1_default" \
    --state-mappings "$WORKSPACE_ROOT/src/flexbe_synthesis_demo/pyrobosim_demo/pyrobosim_flexbe_synthesis/mappings/batch_mappings.yaml" \
    --slugs-bin /usr/local/bin/slugs \
    --synthesis-timeout-s "$(synthesis_timeout_s 28800)" \
    --well-separation-timeout-s "$(synthesis_timeout_s 28800)"

  run_cmd python3 -m flexbe_synthesis_slugs.helpers.batch_run_harness \
    --matrix "$pyrobosim_exp/matrices/pyrobosim_p1_onehot.yaml" \
    --out-dir "$pyrobosim_exp/runs/pyrobosim_p1_default" \
    --state-mappings "$WORKSPACE_ROOT/src/flexbe_synthesis_demo/pyrobosim_demo/pyrobosim_flexbe_synthesis/mappings/batch_mappings.yaml" \
    --slugs-bin /usr/local/bin/slugs \
    --synthesis-timeout-s "$(synthesis_timeout_s 36000)" \
    --well-separation-timeout-s "$(synthesis_timeout_s 36000)" \
    --append

  run_cmd python3 -m flexbe_synthesis_slugs.helpers.batch_run_harness \
    --matrix "$pyrobosim_exp/matrices/pyrobosim_p1_fullp1.yaml" \
    --out-dir "$pyrobosim_exp/runs/pyrobosim_p1_default" \
    --state-mappings "$WORKSPACE_ROOT/src/flexbe_synthesis_demo/pyrobosim_demo/pyrobosim_flexbe_synthesis/mappings/batch_mappings.yaml" \
    --slugs-bin /usr/local/bin/slugs \
    --synthesis-timeout-s "$(synthesis_timeout_s 20)" \
    --well-separation-timeout-s "$(synthesis_timeout_s 20)" \
    --append

  # P1 rows are always the single fixed alphabetic-ordering / unbounded-reorder
  # ("auto") treatment -- experiment_postprocess table writer's defaults (threshold_250/random)
  # match P0's swept matrix, not this one, and silently select zero rows if
  # left unset.
  run_cmd python3 -m flexbe_synthesis_slugs.helpers.experiment_postprocess \
    --run-dir "$pyrobosim_exp/runs/pyrobosim_p1_default" \
    --refresh-summary \
    --table-policy auto \
    --table-ordering alphabetic
}

# 6 of 1SP's (one-hot, S-liveness, pending) 22 orderings -- alphabetic,
# domain, and random seeds 6/10/12/16 -- have repeatedly hit the
# well-separation analyzer's cap (previously 1800s) and shown
# ANALYSIS_INCOMPLETE ("TIME OUT"), while the other 16 reliably resolve in
# well under 700s and 3600s realizability is never the bottleneck for any of
# them. Since drone_demo_matrix.py's seeds are `range(seed_count)`, not
# re-rolled per run, these are the *same* 6 every from-scratch run, not a
# guess -- so, same reasoning as FullP1/1FP/2FP, there's no point spending
# real budget (let alone the 21600s/6h actually confirmed to clear them, via
# the retired run_drone_demo_long_retry.sh) attempting them here. This task
# splits them into their own harness call at a 20s cap, same placeholder
# treatment as the other known-pathological cells; the other 126 rows
# (including 1SP's 16 good orderings) get the real 3600s budget.
# run_drone_demo_stats_1sp_outliers.sh is the real attempt, and re-discovers
# whatever's still incomplete from master.csv rather than trusting this
# hardcoded list, so it also self-corrects if an unlisted ordering turns out
# to be slow this time.
drone_demo_stats() {
  local drone_exp="$SYNTHESIS_HOME/experiments/drone_demo"
  mkdir -p "$drone_exp/matrices" "$drone_exp/runs"
  cd "$WORKSPACE_ROOT/src/flexbe_synthesis_demo/drone_demo/drone_demo_flexbe_synthesis"
  run_cmd python3 -m drone_demo_flexbe_synthesis.drone_demo_matrix \
    --stats \
    --random-seeds 20 \
    "$drone_exp/matrices/drone_demo_stats.yaml"

  python3 - "$drone_exp/matrices/drone_demo_stats.yaml" \
    "$drone_exp/matrices/drone_demo_stats_fast.yaml" \
    "$drone_exp/matrices/drone_demo_stats_1sp_slow.yaml" <<'PYEOF'
import sys
import yaml

src, fast_out, slow_out = sys.argv[1:4]
with open(src, encoding='utf-8') as f:
    rows = yaml.safe_load(f)['rows']

known_slow_seeds = {6, 10, 12, 16}


def is_known_slow(row):
    if not (row.get('encoding') == 'one-hot' and row.get('liveness') == 'S' and row.get('pending') is True):
        return False
    mode = row.get('ordering_mode')
    if mode in ('alphabetic', 'domain'):
        return True
    return mode == 'random' and row.get('ordering_seed') in known_slow_seeds


fast = [r for r in rows if not is_known_slow(r)]
slow = [r for r in rows if is_known_slow(r)]
with open(fast_out, 'w', encoding='utf-8') as f:
    yaml.safe_dump({'rows': fast}, f, sort_keys=False)
with open(slow_out, 'w', encoding='utf-8') as f:
    yaml.safe_dump({'rows': slow}, f, sort_keys=False)
print(f"Split drone_demo_stats.yaml: {len(fast)} fast row(s), {len(slow)} known-slow 1SP row(s)")
PYEOF

  run_cmd python3 -m flexbe_synthesis_slugs.helpers.batch_run_harness \
    --matrix "$drone_exp/matrices/drone_demo_stats_fast.yaml" \
    --out-dir "$drone_exp/runs/drone_demo_stats" \
    --state-mappings "$WORKSPACE_ROOT/src/flexbe_synthesis_demo/drone_demo/drone_demo_flexbe_synthesis/mappings/batch_mappings.yaml" \
    --slugs-bin /usr/local/bin/slugs \
    --synthesis-timeout-s "$(synthesis_timeout_s 3600)" \
    --well-separation-timeout-s "$(synthesis_timeout_s 3600)"

  run_cmd python3 -m flexbe_synthesis_slugs.helpers.batch_run_harness \
    --matrix "$drone_exp/matrices/drone_demo_stats_1sp_slow.yaml" \
    --out-dir "$drone_exp/runs/drone_demo_stats" \
    --state-mappings "$WORKSPACE_ROOT/src/flexbe_synthesis_demo/drone_demo/drone_demo_flexbe_synthesis/mappings/batch_mappings.yaml" \
    --slugs-bin /usr/local/bin/slugs \
    --synthesis-timeout-s "$(synthesis_timeout_s 20)" \
    --well-separation-timeout-s "$(synthesis_timeout_s 20)" \
    --append

  run_cmd python3 -m flexbe_synthesis_slugs.helpers.experiment_postprocess \
    --run-dir "$drone_exp/runs/drone_demo_stats" \
    --refresh-summary \
    --summary-plot
}

# Both of drone_demo_fp's rows (1FP, 2FP) are the known-pathological cells --
# unlike pyrobosim_p1, there's no "fast" row to protect here, so the whole
# task gets a 20s cap instead of the 28800s (8h)/3600s it used to run at.
# Escalating single-cell attempts during development went
# 900s -> 7200s -> 18000s -> 36000s -> 57600s -> 86400s before 1FP finally
# resolved at ~35,413s (~9.8h) realizability; 2FP has run past 24h without a
# verdict in at least one attempt. Neither was ever going to resolve inside a
# main-track-sane budget, so 20s just records a placeholder (TIME OUT) pair
# of rows for summary table generation before the real attempt runs --
# run_drone_demo_outliers.sh, at budgets actually sized to have a chance, is
# now a required follow-up for both rows, not a conditional one.
drone_demo_fp() {
  local drone_exp="$SYNTHESIS_HOME/experiments/drone_demo"
  mkdir -p "$drone_exp/matrices" "$drone_exp/runs"
  cd "$WORKSPACE_ROOT/src/flexbe_synthesis_demo/drone_demo/drone_demo_flexbe_synthesis"
  run_cmd python3 -m drone_demo_flexbe_synthesis.drone_demo_matrix \
    --fp \
    "$drone_exp/matrices/drone_demo_fp.yaml"
  run_cmd python3 -m flexbe_synthesis_slugs.helpers.batch_run_harness \
    --matrix "$drone_exp/matrices/drone_demo_fp.yaml" \
    --out-dir "$drone_exp/runs/drone_demo_fp" \
    --state-mappings "$WORKSPACE_ROOT/src/flexbe_synthesis_demo/drone_demo/drone_demo_flexbe_synthesis/mappings/batch_mappings.yaml" \
    --slugs-bin /usr/local/bin/slugs \
    --synthesis-timeout-s "$(synthesis_timeout_s 20)" \
    --well-separation-timeout-s "$(synthesis_timeout_s 20)"
  # drone_demo_fp's rows are alphabetic-ordering / threshold_250 (single fixed
  # ordering, not swept) -- experiment_postprocess table writer's default --table-ordering
  # ('random') would match zero rows here, same empty-table issue as P1.
  run_cmd python3 -m flexbe_synthesis_slugs.helpers.experiment_postprocess \
    --run-dir "$drone_exp/runs/drone_demo_fp" \
    --refresh-summary \
    --table-policy threshold_250 \
    --table-ordering alphabetic
}

run_task() {
  local task="$1"
  mkdir -p "$TASK_LOG_DIR" "$STATE_DIR"
  local task_log="$TASK_LOG_DIR/$task.log"
  exec > >(tee -a "$task_log") 2>&1

  write_state "$task" RUNNING
  trap 'rc=$?; if (( rc != 0 )); then write_failed_state "$task" "$rc"; log "FAILED $task rc=$rc"; fi' EXIT

  ensure_setup
  log "START $task"
  local rc=0
  "$task" || rc=$?
  if (( rc == 0 )); then
    write_state "$task" DONE "$rc"
    log "COMPLETE $task rc=0"
  else
    write_state "$task" FAILED "$rc"
    log "FAILED $task rc=$rc"
  fi
  trap - EXIT
  return "$rc"
}

terminal_command_for_task() {
  local task="$1"
  local hold="$2"
  local quoted_script quoted_task quoted_workspace quoted_setup quoted_home quoted_timeout_override
  printf -v quoted_script '%q' "$SCRIPT_PATH"
  printf -v quoted_task '%q' "$task"
  printf -v quoted_workspace '%q' "$WORKSPACE_ROOT"
  printf -v quoted_setup '%q' "$SETUP_FILE"
  printf -v quoted_home '%q' "$SYNTHESIS_HOME"
  printf -v quoted_timeout_override '%q' "$EXPERIMENT_TIMEOUT_S_OVERRIDE"
  if [[ "$hold" == "1" ]]; then
    # shellcheck disable=SC2016
    printf 'WORKSPACE_ROOT=%s SETUP_FILE=%s FLEXBE_SYNTHESIS_HOME=%s EXPERIMENT_TIMEOUT_S_OVERRIDE=%s %s --run-task %s; rc=$?; echo; echo "Task %s finished with rc=$rc."; echo "Log: %s/%s.log"; echo "This terminal is left open for inspection."; exec bash -i' \
      "$quoted_workspace" "$quoted_setup" "$quoted_home" "$quoted_timeout_override" "$quoted_script" "$quoted_task" "$task" "$TASK_LOG_DIR" "$task"
  else
    printf 'WORKSPACE_ROOT=%s SETUP_FILE=%s FLEXBE_SYNTHESIS_HOME=%s EXPERIMENT_TIMEOUT_S_OVERRIDE=%s %s --run-task %s' \
      "$quoted_workspace" "$quoted_setup" "$quoted_home" "$quoted_timeout_override" "$quoted_script" "$quoted_task"
  fi
}

open_task_terminal() {
  local task="$1"
  local hold="$2"
  local title="flexbe experiment: $task"
  local cmd
  cmd="$(terminal_command_for_task "$task" "$hold")"

  if command -v gnome-terminal >/dev/null 2>&1; then
    if gnome-terminal --title="$title" -- bash -lc "$cmd" >/dev/null 2>&1; then
      return 0
    fi
    log "gnome-terminal failed to open for $task; trying another launch method"
  fi
  if command -v xfce4-terminal >/dev/null 2>&1; then
    if xfce4-terminal --title="$title" --command "bash -lc '$cmd'" >/dev/null 2>&1; then
      return 0
    fi
    log "xfce4-terminal failed to open for $task; trying another launch method"
  fi
  if command -v konsole >/dev/null 2>&1; then
    if konsole --new-tab --title "$title" -e bash -lc "$cmd" >/dev/null 2>&1; then
      return 0
    fi
    log "konsole failed to open for $task; trying another launch method"
  fi
  if command -v xterm >/dev/null 2>&1; then
    xterm -T "$title" -e bash -lc "$cmd" >/dev/null 2>&1 &
    return 0
  fi
  if command -v tmux >/dev/null 2>&1 && [[ -n "${TMUX:-}" ]]; then
    tmux new-window -n "$task" "$cmd"
    return 0
  fi

  cmd="$(terminal_command_for_task "$task" "0")"
  log "No terminal emulator found; running $task in background with log $TASK_LOG_DIR/$task.log"
  bash -lc "$cmd" >/dev/null 2>&1 &
}

print_status() {
  local task
  printf '\n[%s] Overall experiment status\n' "$(timestamp)"
  for task in "${TASKS[@]}"; do
    printf '  %-24s %s\n' "$task" "$(read_state "$task")"
  done
}

schedule_tasks() {
  local hold_terminals="$1"
  local next=0
  local active=0
  local task state all_done

  while :; do
    active=0
    all_done=1
    for task in "${TASKS[@]}"; do
      state="$(read_state "$task")"
      case "$state" in
        RUNNING|LAUNCHING)
          active=$((active + 1))
          all_done=0
          ;;
        PENDING|UNKNOWN)
          all_done=0
          ;;
        DONE|FAILED)
          ;;
      esac
    done

    while (( active < MAX_PARALLEL && next < ${#TASKS[@]} )); do
      task="${TASKS[$next]}"
      next=$((next + 1))
      write_state "$task" LAUNCHING
      log "LAUNCH $task"
      open_task_terminal "$task" "$hold_terminals"
      active=$((active + 1))
    done

    for task in "${TASKS[@]}"; do
      state="$(read_state "$task")"
      if [[ "$state" == "LAUNCHING" ]]; then
        local state_time state_epoch now_epoch age
        state_time="$(awk -F= '$1 == "time" { print $2; exit }' "$(state_file "$task")")"
        state_epoch="$(date -d "$state_time" +%s 2>/dev/null || printf '0')"
        now_epoch="$(date +%s)"
        age=$((now_epoch - state_epoch))
        if (( state_epoch == 0 || age >= LAUNCH_TIMEOUT_S )); then
          log "Task $task stayed LAUNCHING for ${age}s; marking FAILED"
          write_state "$task" FAILED 124
        fi
      fi
    done

    print_status | tee -a "$RUN_LOG"

    if (( all_done == 1 && next >= ${#TASKS[@]} )); then
      break
    fi
    sleep 30
  done

  local failed=0
  for task in "${TASKS[@]}"; do
    if [[ "$(read_state "$task")" == "FAILED" ]]; then
      failed=1
    fi
  done

  if (( failed )); then
    log "One or more experiment runs failed; see $TASK_LOG_DIR"
    return 1
  fi
  log "All experiment runs completed"
}

clear_synthesis_home() {
  case "$SYNTHESIS_HOME" in
    ""|"/"|"$HOME")
      die "Refusing to clear unsafe FLEXBE_SYNTHESIS_HOME: ${SYNTHESIS_HOME:-<empty>}"
      ;;
  esac

  if [[ -e "$SYNTHESIS_HOME" ]]; then
    log "Clearing synthesis home: $SYNTHESIS_HOME"
    rm -rf -- "$SYNTHESIS_HOME"
  fi
}

require_empty_synthesis_home() {
  mkdir -p "$SYNTHESIS_HOME"
  if find "$SYNTHESIS_HOME" -mindepth 1 -maxdepth 1 | read -r _; then
    printf 'Refusing to run: %s already exists and is not empty.\n' "$SYNTHESIS_HOME" >&2
    printf 'This is a from-scratch run -- move it aside or empty it yourself, then rerun.\n' >&2
    exit 1
  fi
}

record_system_info() {
  local info_file="$SYNTHESIS_HOME/computer.txt"
  log "Recording system info to $info_file"
  {
    printf '=== date ===\n'
    date
    printf '\n=== uname -a ===\n'
    uname -a
    printf '\n=== lscpu ===\n'
    lscpu
    printf '\n=== free -h ===\n'
    free -h
  } > "$info_file"
}

main() {
  local clear_home=0
  local hold_terminals=1
  local slow_first=0
  local skip_long=0
  local skip_fp=0
  while (($#)); do
    case "$1" in
      --clear)
        clear_home=1
        ;;
      --slow-first|--slower-first)
        slow_first=1
        ;;
      --super-long)
        EXPERIMENT_TIMEOUT_S_OVERRIDE=36000
        ;;
      --skip-long)
        skip_long=1
        ;;
      --skip-fp)
        skip_fp=1
        ;;
      --no-terminal-hold)
        hold_terminals=0
        ;;
      --run-task)
        shift
        [[ $# -gt 0 ]] || die "--run-task requires a task name"
        run_task "$1"
        return
        ;;
      -h|--help)
        usage
        return
        ;;
      *)
        die "Unknown argument: $1"
        ;;
    esac
    shift
  done

  validate_config
  ensure_setup
  if (( slow_first )); then
    TASKS=("${SLOW_FIRST_TASKS[@]}")
  fi
  if (( skip_long )); then
    filter_long_tasks
  fi
  if (( skip_fp )); then
    filter_fp_task
  fi
  if (( clear_home )); then
    clear_synthesis_home
  fi
  require_empty_synthesis_home

  mkdir -p "$STATE_DIR" "$TASK_LOG_DIR"
  : > "$RUN_LOG"
  log "Experiment runner started"
  log "Workspace root: $WORKSPACE_ROOT"
  log "Synthesis home: $SYNTHESIS_HOME"
  log "Max parallel runs: $MAX_PARALLEL"
  if [[ -n "$EXPERIMENT_TIMEOUT_S_OVERRIDE" ]]; then
    log "Per-row synthesis timeout override: ${EXPERIMENT_TIMEOUT_S_OVERRIDE}s"
  fi
  if (( skip_long )); then
    log "--skip-long enabled: pyrobosim and drone tasks will not generate data"
  fi
  if (( skip_fp )); then
    log "--skip-fp enabled: drone_demo_fp will not generate data"
  fi
  log "Task order: ${TASKS[*]}"

  record_system_info
  init_workspace
  schedule_tasks "$hold_terminals"
}

main "$@"
