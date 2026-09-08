#!/usr/bin/env bash
set -Eeuo pipefail

# Runs the full release dataset sequence end to end, unattended, over a
# multi-day window: the main track, then all required outlier/follow-up
# scripts, strictly one stage at a time. This machine (4 cores, ~5.5 GB
# free) must never run more than one of these scripts, or more than
# MAX_PARALLEL=2 synthesis rows within the main track, concurrently.
#
# pyrobosim_p1_stats is the 4S/4F ordering-dispersion sweep added on top of
# the three long outlier scripts; see the script header.
#
# By default, any failed stage stops the sequence. Later stages mutate the same
# run directories and depend on the main track's placeholders, so continuing is
# only for deliberate salvage runs.
#
# This script does NOT run export_release_data.py. Run it only once every
# stage above has genuinely concluded and you have inspected the outcome.
#
# Because this spans days, run it somewhere that survives a dropped terminal
# or SSH session, e.g.:
#   tmux new -s weekend_run 'experiments/weekend_run.sh'
#   nohup experiments/weekend_run.sh > ~/weekend_run.out 2>&1 &

EXPERIMENTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$EXPERIMENTS_DIR/.." && pwd)"
WORKSPACE_ROOT="${WORKSPACE_ROOT:-$(cd "$REPO_ROOT/../.." && pwd)}"
SYNTHESIS_HOME="${FLEXBE_SYNTHESIS_HOME:-$HOME/.flexbe_synthesis}"
RUN_LOG="$SYNTHESIS_HOME/run_logs/weekend_run.log"

STAGE_KEYS=(main pyrobosim_p1_outliers pyrobosim_p1_stats drone_demo_outliers drone_demo_stats_1sp_outliers)

declare -A STAGE_SCRIPT=(
  [main]="$EXPERIMENTS_DIR/run_experiment_plan.sh"
  [pyrobosim_p1_outliers]="$EXPERIMENTS_DIR/run_pyrobosim_p1_outliers.sh"
  [pyrobosim_p1_stats]="$EXPERIMENTS_DIR/run_pyrobosim_p1_stats.sh"
  [drone_demo_outliers]="$EXPERIMENTS_DIR/run_drone_demo_outliers.sh"
  [drone_demo_stats_1sp_outliers]="$EXPERIMENTS_DIR/run_drone_demo_stats_1sp_outliers.sh"
)
declare -A STAGE_LABEL=(
  [main]="Main track (coffee, two_rivers, pyrobosim_stats, pyrobosim_p1, drone_demo_stats, drone_demo_fp)"
  [pyrobosim_p1_outliers]="PyRoboSim P1 outlier (FullP1 well-separation, real attempt)"
  [pyrobosim_p1_stats]="PyRoboSim P1 4S/4F ordering sweep (2 fixed + 20 random, threshold_250)"
  [drone_demo_outliers]="drone_demo outliers (1FP/2FP, real attempt)"
  [drone_demo_stats_1sp_outliers]="drone_demo_stats outliers (1SP's known-slow orderings, real attempt)"
)
# Estimated wall-clock hours, [low, high].
declare -A STAGE_LOW_H=( [main]=14 [pyrobosim_p1_outliers]=12 [pyrobosim_p1_stats]=1.5 [drone_demo_outliers]=24 [drone_demo_stats_1sp_outliers]=0.2 )
declare -A STAGE_HIGH_H=( [main]=16 [pyrobosim_p1_outliers]=13 [pyrobosim_p1_stats]=15 [drone_demo_outliers]=48 [drone_demo_stats_1sp_outliers]=36 )
declare -A STAGE_NOTE=(
  [main]="from-scratch run; requires \$FLEXBE_SYNTHESIS_HOME empty"
  [pyrobosim_p1_outliers]="may never reach a verdict -- a firm 'exceeds 12h' bound is an acceptable outcome"
  [pyrobosim_p1_stats]="4S/4F only -- 4SP/4FP/FullP1 stay single-trial, too expensive/unresolvable to sweep"
  [drone_demo_outliers]="2FP has exceeded 24h without a verdict in at least one prior attempt"
  [drone_demo_stats_1sp_outliers]="often finishes in minutes if nothing is left to retry; up to 36h worst case"
)

STAGE_RC=()
STAGE_ELAPSED_S=()

usage() {
  cat <<USAGE
Usage: $(basename "$0") [--clear] [--hold-terminals] [--continue-on-failure] [-- <extra args for main track>]

Runs the main-track experiment script, then all outlier/follow-up scripts
(including the PyRoboSim P1 4S/4F ordering sweep), strictly in sequence.
See run_pyrobosim_p1_stats.sh for what that stage covers and why.

Options:
  --clear            wipe \$FLEXBE_SYNTHESIS_HOME before this wrapper creates
                      its log; the main track otherwise refuses to start unless
                      that directory is already empty
  --hold-terminals    let run_experiment_plan.sh's per-task terminals stay
                      open after completion (default: --no-terminal-hold,
                      since nobody is expected to be watching over a weekend)
  --continue-on-failure
                      log a failed stage and continue to later stages anyway
                      (default: stop immediately, because follow-up stages
                      mutate the same run directories)
  -- ARGS             everything after -- is forwarded to run_experiment_plan.sh
                      (the main-track stage) only

This script does not run export_release_data.py. Run it by hand once you
have inspected the outcome.

Recommended for multi-day runs, choose one:
  tmux new -s weekend_run 'experiments/weekend_run.sh --clear'
  or
  nohup experiments/weekend_run.sh --clear > ~/weekend_run.out 2>&1 &

Log: $RUN_LOG
USAGE
}

timestamp() {
  date '+%Y-%m-%d %H:%M:%S %z'
}

log() {
  mkdir -p "$(dirname "$RUN_LOG")"
  printf '[%s] %s\n' "$(timestamp)" "$*" | tee -a "$RUN_LOG"
}

clear_synthesis_home() {
  case "$SYNTHESIS_HOME" in
    ""|"/"|"$HOME")
      printf 'ERROR: Refusing to clear unsafe FLEXBE_SYNTHESIS_HOME: %s\n' "${SYNTHESIS_HOME:-<empty>}" >&2
      exit 1
      ;;
  esac

  if [[ -e "$SYNTHESIS_HOME" ]]; then
    printf '[%s] Clearing synthesis home: %s\n' "$(timestamp)" "$SYNTHESIS_HOME"
    rm -rf -- "$SYNTHESIS_HOME"
  fi
}

hours_from_now() {
  local seconds
  seconds="$(echo "$1 * 3600 / 1" | bc)"
  date -d "+${seconds} seconds" '+%Y-%m-%d %H:%M %z'
}

remaining_low_high_h() {
  local from_index="$1"
  local low_sum=0
  local high_sum=0
  local i
  for (( i = from_index; i < ${#STAGE_KEYS[@]}; i++ )); do
    local key="${STAGE_KEYS[$i]}"
    low_sum="$(echo "$low_sum + ${STAGE_LOW_H[$key]}" | bc)"
    high_sum="$(echo "$high_sum + ${STAGE_HIGH_H[$key]}" | bc)"
  done
  printf '%s %s\n' "$low_sum" "$high_sum"
}

check_detached() {
  if [[ -n "${TMUX:-}" || -n "${STY:-}" ]]; then
    return
  fi
  log "WARNING: not running inside tmux/screen, and this sequence can take days."
  log "WARNING: if this terminal/SSH session drops, this script (and any stage"
  log "WARNING: it hasn't finished launching) will die. Consider re-running under"
  log "WARNING: tmux, screen, or nohup ... & instead. Continuing in 10s (Ctrl-C to stop)."
  sleep 10
}

run_stage() {
  local index="$1"
  local key="${STAGE_KEYS[$index]}"
  local total="${#STAGE_KEYS[@]}"
  local human_index="$((index + 1))"
  shift
  local extra_args=("$@")

  local remaining
  remaining="$(remaining_low_high_h "$index")"
  local rem_low rem_high
  read -r rem_low rem_high <<<"$remaining"

  log "=================================================================="
  log "Stage $human_index/$total: ${STAGE_LABEL[$key]}"
  log "  script: ${STAGE_SCRIPT[$key]}"
  log "  estimated duration: ${STAGE_LOW_H[$key]}-${STAGE_HIGH_H[$key]}h (${STAGE_NOTE[$key]})"
  log "  projected stage finish: $(hours_from_now "${STAGE_LOW_H[$key]}") to $(hours_from_now "${STAGE_HIGH_H[$key]}")"
  log "  projected finish of ALL remaining stages: $(hours_from_now "$rem_low") to $(hours_from_now "$rem_high")"
  log "=================================================================="

  local start_epoch end_epoch rc=0
  start_epoch="$(date +%s)"
  "${STAGE_SCRIPT[$key]}" "${extra_args[@]}" || rc=$?
  end_epoch="$(date +%s)"

  local elapsed=$((end_epoch - start_epoch))
  STAGE_RC[$index]=$rc
  STAGE_ELAPSED_S[$index]=$elapsed

  if (( rc == 0 )); then
    log "Stage $human_index/$total (${key}) COMPLETE after $((elapsed / 3600))h$(( (elapsed % 3600) / 60 ))m"
  else
    log "Stage $human_index/$total (${key}) FAILED rc=$rc after $((elapsed / 3600))h$(( (elapsed % 3600) / 60 ))m"
  fi
  return "$rc"
}

print_final_summary() {
  local total="${#STAGE_KEYS[@]}"
  local i key rc elapsed status total_elapsed=0
  log "=================================================================="
  log "weekend_run.sh summary"
  for (( i = 0; i < total; i++ )); do
    key="${STAGE_KEYS[$i]}"
    if [[ -z "${STAGE_RC[$i]:-}" ]]; then
      log "  $((i + 1))/$total ${key}: NOT RUN"
      continue
    fi
    rc="${STAGE_RC[$i]}"
    elapsed="${STAGE_ELAPSED_S[$i]}"
    total_elapsed=$((total_elapsed + elapsed))
    status="SUCCESS"
    (( rc != 0 )) && status="FAILED (rc=$rc)"
    log "  $((i + 1))/$total ${key}: $status, $((elapsed / 3600))h$(( (elapsed % 3600) / 60 ))m"
  done
  log "  total elapsed: $((total_elapsed / 3600))h$(( (total_elapsed % 3600) / 60 ))m"
  log "Next (not run by this script, once you have inspected the outcome):"
  log "  python3 experiments/export_release_data.py"
  log "=================================================================="
}

main() {
  local main_track_args=()
  local stop_on_failure=1
  local hold_terminals=0
  local clear_home=0

  while (($#)); do
    case "$1" in
      --clear)
        clear_home=1
        ;;
      --hold-terminals)
        hold_terminals=1
        ;;
      --stop-on-failure)
        stop_on_failure=1
        ;;
      --continue-on-failure)
        stop_on_failure=0
        ;;
      --)
        shift
        main_track_args+=("$@")
        break
        ;;
      -h|--help)
        usage
        return
        ;;
      *)
        printf 'ERROR: Unknown argument: %s\n' "$1" >&2
        usage >&2
        exit 1
        ;;
    esac
    shift
  done

  if (( ! hold_terminals )); then
    main_track_args+=(--no-terminal-hold)
  fi

  command -v bc >/dev/null 2>&1 || { printf 'ERROR: this script needs `bc` for duration arithmetic.\n' >&2; exit 1; }

  if (( clear_home )); then
    clear_synthesis_home
  fi

  mkdir -p "$(dirname "$RUN_LOG")"
  log "weekend_run.sh started"
  log "Workspace root: $WORKSPACE_ROOT"
  log "Synthesis home: $SYNTHESIS_HOME"
  local total_remaining
  total_remaining="$(remaining_low_high_h 0)"
  local total_low total_high
  read -r total_low total_high <<<"$total_remaining"
  log "Projected total duration: ${total_low}-${total_high}h -- finish by $(hours_from_now "$total_low") to $(hours_from_now "$total_high")"

  check_detached

  local overall_rc=0
  local index=0
  for key in "${STAGE_KEYS[@]}"; do
    local rc=0
    if [[ "$key" == "main" ]]; then
      run_stage "$index" "${main_track_args[@]}" || rc=$?
    else
      run_stage "$index" || rc=$?
    fi
    if (( rc != 0 )); then
      overall_rc=1
      if (( stop_on_failure )); then
        log "Stopping: stage '$key' failed (pass --continue-on-failure to keep going instead)."
        print_final_summary
        exit "$rc"
      fi
    fi
    index=$((index + 1))
  done

  print_final_summary
  exit "$overall_rc"
}

main "$@"
