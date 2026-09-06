#!/usr/bin/env python3
"""Repair trials whose 'Realize HFSM' stage failed, without re-running Slugs.

`batch_run_harness.py` (in `flexbe_synthesis_slugs`) runs every stage of a
trial in one shot, including the expensive `slugs` synthesis call. When
only a downstream, post-synthesis bug is fixed -- e.g. the mapped-autonomy
`TypeError` in `SMGenConfig.get_autonomy_list` that made every Coffee and
Crazyflie trial (and PyRoboSim P1's `4S`/`4SP`) report `flexbe_hfsm_realized:
No` despite realizability, well-separation, and the post-reduction audit all
having passed -- there's no need to re-pay that cost: `slugs`' own
explicit-strategy JSON output (`<spec_name>.json`) and the compiled
`.structuredslugs` file are already on disk in each trial directory and
fully determine the synthesized automaton
(`SlugsSynthesizerHelper.gen_slugs_automaton_from_json`), so it can be
reloaded in milliseconds instead of re-invoked.

This script finds every row in an existing `master.csv` that reached a
valid, realizable, audit-passed reduced strategy but never got
`flexbe_hfsm_realized: Yes` (see `needs_repair` below -- that combination
only happens when SM generation or GUI layout crashed, not a real
synthesis/audit failure such as GOAL_UNREACHABLE_TRAP, which halts the
pipeline *before* reduction and so never reaches this state), reloads its
cached automaton, and reruns only the cheap post-synthesis chain: recompile
spec (ms) -> reload automaton (ms, no `slugs` call) -> audit -> reduce ->
audit -> SM-generate -> SM-layout -> count-specs. Everything upstream
(realizability time, CUDD stats, well-separation) is left untouched and
copied forward from the existing row, since none of it is affected by a bug
downstream of reduction.

Prerequisite: run this after sourcing the workspace (same as every other
script in this directory), since it imports the real pipeline packages
rather than reimplementing them:

    cd "$WORKSPACE_ROOT" && source install/setup.bash

Usage:

    python3 experiments/resume_realize_hfsm.py \\
        --matrix "$FLEXBE_SYNTHESIS_HOME/experiments/coffee/matrices/coffee_full.yaml" \\
        --out-dir "$FLEXBE_SYNTHESIS_HOME/experiments/coffee/runs/coffee_full_rerun"

The matrix YAML must be the same one originally used to produce `--out-dir`
(needed to recover `spec_path`/`capabilities_path`/etc. for each row --
`master.csv` alone doesn't retain enough to rebuild the spec). Add
`--dry-run` first to see what would be repaired without touching anything.

Safe to re-run: rows that don't need repair are left byte-for-byte as they
were. Before overwriting `master.csv`/`summary.csv` in place, the previous
`master.csv` is copied to `master.csv.pre_resume_realize_hfsm_<UTC
timestamp>.bak` in the same directory, matching this repo's existing
in-place-retry convention (see e.g. `drone_demo_stats/master.csv.pre_1sp_
outliers_*.bak`).
"""

from __future__ import annotations

import argparse
import csv
import os
from pathlib import Path
import platform
import shutil
import sys
import time

try:
    from flexbe_synthesis_slugs.helpers import batch_run_harness as bh
    from flexbe_synthesis_slugs.helpers.slugs_synthesizer_helper import (
        SlugsSynthesizerHelper,
    )
except ImportError:
    print(
        'ERROR: could not import flexbe_synthesis_slugs.\n'
        'Source the workspace install/setup.bash first, same as the other '
        'manual commands described in experiments/README.md.',
        file=sys.stderr,
    )
    raise


def needs_repair(row: dict) -> bool:
    """Return whether a master.csv row shows the SM-gen/layout crash signature.

    Realizable, reduction completed (`reduced_sm_size` populated), and the
    post-reduction audit passed clean (`audit_valid == 'True'`, so this is
    not a GOAL_UNREACHABLE_TRAP or other legitimate halt-before-SM-gen
    case) -- yet `flexbe_hfsm_realized` is still 'No'. That combination
    only happens when SM generation or layout raised after reduction, which
    is exactly the mapped-autonomy bug's signature.
    """
    return (
        row.get('flexbe_hfsm_realized') == 'No'
        and row.get('audit_valid') == 'True'
        and row.get('reduced_sm_size') not in (None, '')
    )


def load_cached_automaton(trial_dir: Path, row: bh.MatrixRow, state_mappings: dict) -> dict:
    """Reconstruct the synthesized automaton dict from persisted artifacts.

    Reads the same `<spec_name>.json` (`slugs --jsonOutput`) and
    `<spec_name>.structuredslugs` files `SlugsSynthesizerHelper` would have
    just produced by calling `slugs` -- but since this trial already
    completed synthesis successfully the first time, those files already
    hold the answer. Never invokes the `slugs` binary.
    """
    helper = SlugsSynthesizerHelper(
        str(trial_dir),
        state_mappings['transition_outcomes'],
        state_mappings['sm_outcome_mappings'],
        spec_name=row.spec_name,
    )
    input_vars, output_vars = helper.read_structuredslugs()
    json_path = Path(helper.specs_output_dir_path) / f'{row.spec_name}.json'
    if not json_path.exists():
        raise FileNotFoundError(
            f"No persisted strategy JSON at '{json_path}' -- this trial's "
            "synthesis output wasn't cached; it needs a real re-run, not a resume."
        )
    automaton = helper.gen_slugs_automaton_from_json(str(json_path), input_vars, output_vars)
    return automaton.to_dict()


def resume_trial(
    row: bh.MatrixRow,
    run_index: int,
    out_dir: Path,
    state_mappings: dict,
    system_capabilities: dict,
    args,
    original_row: dict,
) -> dict:
    """Rerun only the post-synthesis stages for one previously-completed trial."""
    trial_dir = out_dir / 'runs' / row.row_id / f'trial_{run_index:02d}'
    stage_times = bh.empty_stage_times()

    # Deliberately do NOT call compiler_main here. It would rewrite
    # .structuredslugs/.slugsin from a freshly-rebuilt in-memory spec, and
    # spec generation is not guaranteed byte-identical across separate
    # process invocations (observed in testing: a freshly-rebuilt spec for a
    # non-full-spec row added an extra 'finished' output variable the
    # original run's compiled spec never had, which would silently
    # misinterpret the cached JSON's positional variable encoding). Building
    # `spec` in-memory for the auditor below is fine -- only *writing it
    # over the persisted compiled files* is the hazard, so that step is
    # skipped and every variable list is sourced from the untouched,
    # persisted .structuredslugs/.slugsin the original run actually
    # synthesized against.
    # A hand-written full spec already contains the complete request.  Calling
    # bh.build_spec() would run request_spec again and require the mapped SM
    # success outcome (normally ``finished``) to be a proposition in that
    # custom spec.  That is unnecessary for a resume and rejects valid cached
    # full-spec trials such as Coffee.  Load the source spec for the auditor,
    # while continuing to use the persisted compiled files for the automaton.
    if row.full_spec:
        spec = bh._time_process(
            'spec_loader_time_s',
            stage_times,
            lambda: bh.spec_loader_main([
                row.spec_name,
                row.system_name,
                str(row.spec_path),
            ]).process()[0],
        )
    else:
        spec = bh.build_spec(row, state_mappings, system_capabilities, stage_times)

    automaton = bh._time_process(
        'synthesizer_time_s',
        stage_times,
        lambda: load_cached_automaton(trial_dir, row, state_mappings),
    )

    original_realizable = original_row.get('realizable') == 'True'
    reloaded_realizable = isinstance(automaton, dict) and bool(automaton.get('automaton'))
    if reloaded_realizable != original_realizable:
        raise RuntimeError(
            f"Reloaded automaton realizable={reloaded_realizable} but the "
            f"existing row says realizable={original_realizable} for "
            f"'{trial_dir}' -- the cached strategy looks stale or corrupt; "
            'this row needs a real re-run, not a resume.'
        )
    original_ap_i = original_row.get('ap_i')
    original_ap_o = original_row.get('ap_o')
    reloaded_ap_i = len(automaton.get('input_variables', []))
    reloaded_ap_o = len(automaton.get('output_variables', []))
    if (
        original_ap_i not in (None, '') and str(reloaded_ap_i) != str(original_ap_i)
    ) or (
        original_ap_o not in (None, '') and str(reloaded_ap_o) != str(original_ap_o)
    ):
        raise RuntimeError(
            f"Reloaded automaton has |AP_I|={reloaded_ap_i}, |AP_O|={reloaded_ap_o} "
            f"but the existing row says |AP_I|={original_ap_i}, |AP_O|={original_ap_o} "
            f"for '{trial_dir}' -- the persisted .structuredslugs doesn't match what "
            'this row originally synthesized against; this row needs a real '
            're-run, not a resume.'
        )

    original_audit = bh._time_process(
        'original_auditor_time_s',
        stage_times,
        lambda: bh.run_audit(
            automaton, spec, row, trial_dir, state_mappings, system_capabilities,
            args.auditor_timeout_s,
        ),
    )
    bh._time_process(
        'mealy_graph_time_s',
        stage_times,
        lambda: bh.mealy_graph_main([
            automaton, str(trial_dir), row.spec_name, getattr(args, 'mealy_graph_config', {}),
        ]).process()[0],
    )

    reduced_automaton, reducer_ec = bh._time_process(
        'reduction_time_s',
        stage_times,
        lambda: bh.reducer_main([automaton]).process(),
    )
    if bh._fatal_code(reducer_ec) is not None:
        raise RuntimeError(f"Reduction failed for '{trial_dir}': ec={reducer_ec}")

    reduced_audit = bh._time_process(
        'auditor_time_s',
        stage_times,
        lambda: bh.run_audit(
            reduced_automaton, spec, row, trial_dir, state_mappings, system_capabilities,
            args.auditor_timeout_s,
        ),
    )
    bh._time_process(
        'reduced_mealy_graph_time_s',
        stage_times,
        lambda: bh.mealy_graph_main([
            reduced_automaton, str(trial_dir),
            getattr(args, 'reduced_automaton_name', 'reduced_automaton'),
            getattr(args, 'mealy_graph_config', {}),
        ]).process()[0],
    )

    state_defn, sm_generator_ec = bh._time_process(
        'sm_generation_time_s',
        stage_times,
        lambda: bh.sm_generator_main([reduced_automaton, row.system_name]).process(),
    )
    if bh._fatal_code(sm_generator_ec) is None:
        state_defn, sm_layout_ec = bh._time_process(
            'sm_layout_time_s',
            stage_times,
            lambda: bh.sm_layout_main([
                state_defn, str(trial_dir), getattr(args, 'use_fallback_layout', False),
            ]).process(),
        )
    else:
        sm_layout_ec = sm_generator_ec

    (count_specs,) = bh._time_process(
        'count_specs_time_s',
        stage_times,
        lambda: bh.count_specs_main([str(trial_dir), row.spec_name]).process(),
    )

    generate_specs_time_s = bh.stage_time_sum(stage_times, bh.GENERATE_SPECS_STAGE_TIME_FIELDS)
    realize_hfsm_time_s = bh.stage_time_sum(stage_times, bh.REALIZE_HFSM_STAGE_TIME_FIELDS)
    flexbe_hfsm_realized = 'Yes' if isinstance(state_defn, list) and state_defn else 'No'

    # Start from the existing row so every column this function doesn't
    # explicitly touch (realizability time, CUDD stats, well-separation,
    # ...) is preserved exactly as it was -- only the fields genuinely
    # downstream of reduction are overwritten.
    row_data = dict(original_row)
    row_data.update({
        'timestamp': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
        'hostname': platform.node(),
        # Use batch_run_harness.py's own location (inside the flexbe_synthesis
        # repo), not this script's -- this script lives in the sibling
        # flexbe_synthesis_demo repo, so Path(__file__) would resolve to the
        # wrong repository's HEAD.
        'git_sha_flexbe_synthesis': bh.git_sha(Path(bh.__file__).resolve().parents[3]),
        'compiler_time_s': stage_times.get('compiler_time_s'),
        'original_auditor_time_s': original_audit['auditor_time_s'],
        'mealy_graph_time_s': stage_times.get('mealy_graph_time_s'),
        'reduction_time_s': stage_times.get('reduction_time_s'),
        'auditor_time_s': reduced_audit['auditor_time_s'],
        'reduced_mealy_graph_time_s': stage_times.get('reduced_mealy_graph_time_s'),
        'sm_generation_time_s': stage_times.get('sm_generation_time_s'),
        'sm_layout_time_s': stage_times.get('sm_layout_time_s'),
        'count_specs_time_s': stage_times.get('count_specs_time_s'),
        'generate_specs_time_s': generate_specs_time_s,
        'realize_hfsm_time_s': realize_hfsm_time_s,
        'slugs_sm_size': bh._extracted_sm_size(automaton, reloaded_realizable),
        'reduced_sm_size': bh._extracted_sm_size(reduced_automaton, True),
        'state_defn_size': len(state_defn) if isinstance(state_defn, list) else None,
        'flexbe_hfsm_realized': flexbe_hfsm_realized,
        'failure': '' if bh._fatal_code(sm_layout_ec) is None else str(sm_layout_ec),
    })
    row_data.update(bh.prefix_audit(original_audit, 'original'))
    row_data.update(bh.prefix_audit(reduced_audit, 'audit'))

    # `elapsed` only covers this resume's cheap re-run (recompile -> reload
    # cached automaton -> audit -> reduce -> audit -> SM-gen -> SM-layout ->
    # count-specs) -- it excludes the original, already-paid realizability/
    # synthesis/well-separation cost that dominates most rows' true total.
    # Recompute the field the same way a from-scratch `run_trial` effectively
    # does (its wall-clock `elapsed` and the sum of `STAGE_TIME_FIELDS` track
    # each other closely there), from row_data's now-merged fields: preserved
    # originals for realizability/well-separation/synthesizer, freshly
    # measured values for everything downstream of reduction.
    overall_pipeline_time_s = 0.0
    for field in bh.STAGE_TIME_FIELDS:
        value = row_data.get(field)
        if value in (None, ''):
            continue
        try:
            overall_pipeline_time_s += float(value)
        except (TypeError, ValueError):
            continue
    row_data['overall_pipeline_time_s'] = overall_pipeline_time_s
    return row_data


def repair_run_dir(args):
    """Repair every needs-repair row in one harness run directory in place."""
    out_dir = Path(args.out_dir).expanduser()
    master_path = out_dir / 'master.csv'
    summary_path = out_dir / 'summary.csv'
    if not master_path.exists():
        raise FileNotFoundError(f"No master.csv at '{master_path}'")

    with master_path.open(newline='', encoding='utf-8') as master_file:
        reader = csv.DictReader(master_file)
        fieldnames = reader.fieldnames
        existing_rows = list(reader)
    by_key = {(row['row_id'], row['run_index']): row for row in existing_rows}

    matrix_rows = bh.load_matrix(Path(args.matrix).expanduser())
    state_mappings = bh.load_yaml(Path(args.state_mappings).expanduser())
    synthesis_home = args.synthesis_home or os.getenv('FLEXBE_SYNTHESIS_HOME')
    system_cache = {}

    repaired = 0
    still_broken = 0
    skipped_not_found = 0
    for row in matrix_rows:
        for run_index in range(row.n_trials):
            key = (row.row_id, str(run_index))
            original_row = by_key.get(key)
            if original_row is None:
                skipped_not_found += 1
                continue
            if not needs_repair(original_row):
                continue

            cache_key = (row.system_name, str(row.capabilities_path.resolve()))
            if cache_key not in system_cache:
                print(f'Preparing capabilities: {row.capabilities_path}', flush=True)
                system_cache[cache_key] = bh.prepare_system_capabilities(
                    row, state_mappings, synthesis_home=synthesis_home,
                )
            system_capabilities = system_cache[cache_key]

            print(f'Repairing {row.row_id} trial_{run_index:02d} ...', flush=True)
            if args.dry_run:
                repaired += 1
                continue
            try:
                new_row = resume_trial(
                    row, run_index, out_dir, state_mappings, system_capabilities, args,
                    original_row,
                )
            except Exception as exc:  # noqa: BLE001 - report and keep going
                print(f'  FAILED: {exc}', flush=True)
                still_broken += 1
                continue

            by_key[key] = new_row
            if new_row.get('flexbe_hfsm_realized') == 'Yes':
                repaired += 1
                print('  -> now realized.', flush=True)
            else:
                still_broken += 1
                print(
                    f"  -> still not realized (failure={new_row.get('failure')!r}).",
                    flush=True,
                )

    if skipped_not_found:
        print(
            f'{skipped_not_found} matrix row(s) had no matching master.csv row '
            '(not part of this run directory) -- left untouched.',
            flush=True,
        )

    if args.dry_run:
        print(f'[dry run] {repaired} row(s) would be attempted; nothing written.', flush=True)
        return

    if repaired == 0 and still_broken == 0:
        print('Nothing needed repair.', flush=True)
        return

    backup_path = master_path.with_name(
        f"master.csv.pre_resume_realize_hfsm_{time.strftime('%Y%m%d_%H%M%S', time.gmtime())}.bak"
    )
    shutil.copy2(master_path, backup_path)

    ordered_rows = [by_key[(row['row_id'], row['run_index'])] for row in existing_rows]
    with master_path.open('w', newline='', encoding='utf-8') as master_file:
        writer = csv.DictWriter(master_file, fieldnames=fieldnames, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(ordered_rows)
    bh.write_summary(master_path, summary_path)
    print(
        f'Repaired {repaired} row(s), {still_broken} still not realized. '
        f"Backed up prior master.csv to '{backup_path}'. "
        f"Rewrote '{master_path}' and '{summary_path}'.",
        flush=True,
    )


def main():
    """Parse arguments and repair the target run directory."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--matrix', required=True,
        help='Matrix YAML originally used to produce --out-dir.',
    )
    parser.add_argument(
        '--out-dir', required=True,
        help="Existing harness run directory (contains master.csv/summary.csv).",
    )
    parser.add_argument(
        '--state-mappings', default=str(bh.default_state_mappings_path()),
        help='Path to global_mappings.yaml (default: repo-local copy).',
    )
    parser.add_argument(
        '--synthesis-home', default=None,
        help='Override $FLEXBE_SYNTHESIS_HOME for capability preparation.',
    )
    parser.add_argument('--auditor-timeout-s', type=float, default=60.0)
    parser.add_argument(
        '--dry-run', action='store_true',
        help='List rows that would be repaired without running or writing anything.',
    )
    args = parser.parse_args()
    repair_run_dir(args)


if __name__ == '__main__':
    main()
