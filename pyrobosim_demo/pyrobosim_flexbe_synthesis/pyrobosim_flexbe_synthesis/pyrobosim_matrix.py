#!/usr/bin/env python3

# Copyright 2026 Christopher Newport University
# Capable Humanitarian Robotics and Intelligent Systems Lab (CHRISLAB)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Generate PyRoboSim experiment matrix YAML for the Slugs batch harness.

P0 and P1 are generated separately (`p0`/`p1` subcommands) because they need
different `--full` semantics. P0 is cheap enough (10 s - 700 s per trial) to
sweep variable ordering the same way the Two Rivers matrix does, so `p0 --full`
deepens that ordering/seed sweep. P1 is not: per the PyRoboSim synthesis
README, p1_full_spec and p1_capabilities_parsed take 5400-6100 s per trial,
and p1_capabilities (one-hot) resolved only once given a much longer budget
(~23,307 s / ~6.5 h at a 12 h cap; it did not complete within the originally
documented 900-7200 s range this file's other rows use). P1 rows are
therefore always single fixed-ordering baselines regardless of `--full` —
`p1 --full` instead means "also attempt the expensive p1_capabilities row";
pass it a generous `--synthesis-timeout-s`/`--well-separation-timeout-s`
(e.g. 43200, 12h) by hand -- it isn't part of the default `pyrobosim_p1`
main-track task or either outlier script, since it's neither cheap enough
for the main track nor pathological enough to need a dedicated outlier
script.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import yaml

_GOALS = ['bread0=3', 'butter0=3']
_SM_OUTCOMES = ['finished']
_AUDIT_GOALS = ['finished']


def package_root() -> Path:
    """Return the pyrobosim_flexbe_synthesis package source root."""
    return Path(__file__).resolve().parents[1]


def _reordering_treatments(include_off=True):
    """
    Return the CUDD reordering policies used for the current release sweeps.

    `include_off` gates the unsifted (`--no-reorder`) policy. Only `--full`
    passes `include_off=True` — the default and smoke matrices skip it
    entirely, since letting large PyRoboSim capability BDDs go completely
    unsifted was observed to blow past the documented per-trial timings by
    hours during an overnight run, well beyond what the `off` vs
    `threshold_250` comparison is meant to cost.
    """
    treatments = []
    if include_off:
        treatments.append({
            'reordering_enabled': False,
            'reordering_threshold': None,
        })
    treatments.append({
        'reordering_enabled': True,
        'reordering_threshold': 250,
    })
    return treatments


def ordering_rows(full=False, random_seeds=40, include_off=None, stats=False):
    """
    Return ordering and reordering treatments for the P0 sweep.

    `include_off` defaults to `full` — the unsifted policy is only exercised
    by the `--full` sweep, where comparing it against `threshold_250` is a
    deliberate research axis; the default (compact) sweep and `--stats`
    (`stats=True`) always reorder. `stats` uses `random_seeds` random seeds
    like `--full` does, but without pulling in `off` (unless `full` is also
    set) — see `_add_p0_subparser`'s `--stats` help text.
    """
    if include_off is None:
        include_off = full
    reorderings = _reordering_treatments(include_off=include_off)
    rows = [
        {
            'ordering_mode': ordering_mode,
            'ordering_seed': None,
            **reordering,
        }
        for ordering_mode in ('alphabetic', 'domain')
        for reordering in reorderings
    ]
    if full or stats:
        rows.extend(
            {
                'ordering_mode': 'random',
                'ordering_seed': seed,
                **reordering,
            }
            for reordering in reorderings
            for seed in range(random_seeds)
        )
    else:
        rows.extend(
            {
                'ordering_mode': 'random',
                'ordering_seed': 0,
                **reordering,
            }
            for reordering in reorderings
        )
    return rows


def _fixed_ordering():
    """Return the single ordering treatment used for expensive P1 baseline rows."""
    return {
        'ordering_mode': 'alphabetic',
        'ordering_seed': None,
        'reordering_enabled': True,
        'reordering_threshold': None,
    }


# Baseline `encoding` sentinel -> the source table's non-scenario column label.
_BASELINE_INDICATOR = {
    'full_spec': 'Full',
    'full_spec_move_only': 'MoveOnly',
    'full_spec_p1': 'FullP1',
}
# Capability `encoding` -> the source table's leading column digit (2 = one-hot, 3 = enumerated).
# P0 uses digits 2/3 (not 1/2) to match the source table's case-study column numbering; P1's
# enumerated sweep then occupies digit 4 -- see `_p1_baseline_rows`'s `digit='4'` override.
_ENCODING_DIGIT = {'one-hot': '2', 'enumerated': '3'}


def _full_spec_row(
    *,
    system_name,
    spec_name,
    spec_path,
    capabilities_path,
    encoding,
    ordering,
    n_trials,
):
    """Build one hand-written full-spec baseline row."""
    return {
        'example': 'pyrobosim',
        'system_name': system_name,
        'spec_name': spec_name,
        'spec_path': str(spec_path),
        'capabilities_path': str(capabilities_path),
        'encoding': encoding,
        'liveness': 'na',
        'pending': False,
        'column_indicator': _BASELINE_INDICATOR[encoding],
        'full_spec': True,
        'initial_conditions': [],
        'goals': [],
        'sm_outcomes': list(_SM_OUTCOMES),
        'n_trials': n_trials,
        **ordering,
    }


def _capability_row(
    *,
    system_name,
    spec_name,
    spec_path,
    capabilities_path,
    encoding,
    liveness,
    ordering,
    n_trials,
    pending=False,
    digit=None,
):
    """
    Build one capability-generated matrix row.

    `audit_goals` is set to the real `finished` SM outcome rather than
    `goals` (world-state predicates like 'bread0=3'), which the strategy
    auditor does not understand as an outcome name.

    `pending` toggles `slugs_pending_specification`, which derives one-hot
    `<cap>_p` pending propositions from each capability's own completed vs.
    non-completed outcome remapping (no explicit "pending" field needed in
    the capability YAML) — the same mechanism Two Rivers' `MoveState2R`
    pending sweep relies on. `move_robot`/`grab_item`/`place_item` each
    define a `transition_relation: failure: [log_*_failed]` entry, i.e. both
    a completed and a non-completed outcome, so this is meaningful here too;
    the `log_*` bookkeeping capabilities have only one outcome and are
    silently skipped by the pending process.

    `digit` overrides the leading column-indicator digit that would
    otherwise be derived from `encoding` via `_ENCODING_DIGIT`. P0 uses the
    default (`2`/`3` for one-hot/enumerated, matching the source table's column
    numbering); P1's capability sweep passes `digit='4'` so its columns read
    `4S`/`4F`/`4SP`/`4FP` -- the source table's scenario-4 label -- rather than
    colliding visually with P0's `3S`-family columns despite both using the
    enumerated encoding. P1's one-hot row separately pins `digit='1'` so it
    doesn't inherit P0's new one-hot default (`2`).
    """
    digit = digit if digit is not None else _ENCODING_DIGIT[encoding]
    return {
        'example': 'pyrobosim',
        'system_name': system_name,
        'spec_name': spec_name,
        'spec_path': str(spec_path),
        'capabilities_path': str(capabilities_path),
        'encoding': encoding,
        'liveness': liveness,
        'pending': pending,
        'column_indicator': f"{digit}{liveness}{'P' if pending else ''}",
        'initial_conditions': [],
        'goals': list(_GOALS),
        'audit_goals': list(_AUDIT_GOALS),
        'sm_outcomes': list(_SM_OUTCOMES),
        'n_trials': n_trials,
        **ordering,
    }


def _p0_paths(root):
    """Return (spec_path, capabilities_path) for each P0 spec variant."""
    return {
        'full_spec': (
            root / 'specs' / 'p0_full_spec.yaml',
            root / 'capabilities' / 'pyrobosim.yaml',
        ),
        'move_only_spec': (
            root / 'specs' / 'p0_move_only_spec.yaml',
            root / 'capabilities' / 'pyrobosim.yaml',
        ),
        'capabilities': (
            root / 'specs' / 'p0_capabilities.yaml',
            root / 'capabilities' / 'p0_capabilities.yaml',
        ),
    }


def _p1_paths(root):
    """Return (spec_path, capabilities_path) for each P1 spec variant."""
    return {
        'full_spec': (
            root / 'specs' / 'p1_full_spec.yaml',
            root / 'capabilities' / 'pyrobosim.yaml',
        ),
        'capabilities': (
            root / 'specs' / 'p1_capabilities.yaml',
            root / 'capabilities' / 'p1_capabilities.yaml',
        ),
    }


def _p1_baseline_rows(root, *, n_trials, include_unsolvable, stats=False, random_seeds=20):
    """
    Build the fixed-ordering P1 baseline rows (see module docstring).

    Only the enumerated encoding sweeps liveness (S/F) and pending, under the
    `4`-prefix (scenario 4) rather than the `3`-prefix P0 uses for
    the same encoding, so these columns read distinctly from P0's `3S`
    family despite sharing an encoding -- the one-hot encoding's own S-liveness
    baseline (see `include_unsolvable`) took ~23,307 s (~6.5 h) to resolve
    once actually given the room, so 4F/4SP/4FP-equivalent one-hot rows
    (typically costlier than plain S-liveness) would almost certainly need
    many more hours each for uncertain payoff, not something to fold into
    the routine sweep.
    4F/4SP/4FP close the release-critical Fair-Outcome-with-pending gap (the
    goal-unreachable trap tested for P0 in Fig. 15) at bounded added cost.

    `stats` swaps 4S/4F (pending=False) from a single fixed ordering to the
    same `threshold_250`-only, 2-fixed-plus-`random_seeds`-random sweep P0's
    `--stats` uses (see `ordering_rows`), for real dispersion data instead of
    an n=1 directional read. 4SP/4FP (pending=True) and FullP1 are
    deliberately left as single fixed-ordering rows regardless of `stats`:
    4SP/4FP's own single-ordering baseline (~3875 s / ~10,866 s) already
    flags high blowup risk under an unfavorable ordering, at a scale where
    22x that risk per cell isn't worth it for this sweep; FullP1 never
    resolves its well-separation check at all (see
    run_pyrobosim_p1_outliers.sh), so sweeping it would just multiply a
    guaranteed timeout.
    """
    p1 = _p1_paths(root)
    fixed = _fixed_ordering()
    swept = ordering_rows(stats=True, random_seeds=random_seeds) if stats else None
    rows = [
        _full_spec_row(
            system_name='p1_full_spec_demo',
            spec_name='p1_full_spec',
            spec_path=p1['full_spec'][0],
            capabilities_path=p1['full_spec'][1],
            encoding='full_spec_p1',
            ordering=fixed,
            n_trials=n_trials,
        ),
    ]
    for liveness in ('S', 'F'):
        for pending in (False, True):
            orderings = swept if (stats and not pending) else [fixed]
            rows.extend(
                _capability_row(
                    system_name='p1_capabilities_parsed_demo',
                    spec_name='p1_parsed',
                    spec_path=p1['capabilities'][0],
                    capabilities_path=p1['capabilities'][1],
                    encoding='enumerated',
                    liveness=liveness,
                    pending=pending,
                    digit='4',
                    ordering=ordering,
                    n_trials=n_trials,
                )
                for ordering in orderings
            )
    if include_unsolvable:
        rows.append(
            _capability_row(
                system_name='p1_capabilities_demo',
                spec_name='p1_capabilities',
                spec_path=p1['capabilities'][0],
                capabilities_path=p1['capabilities'][1],
                encoding='one-hot',
                liveness='S',
                # Pinned to '1', not the shared one-hot default (now '2', to match P0's
                # release column numbering) -- '1' is unused by P0 now, keeps this column
                # visually distinct from P0's one-hot digit, and matches the
                # column_indicator ('1S') already baked into the pyrobosim_p1_default
                # run's existing master.csv from the completed one-hot P1 test.
                digit='1',
                ordering=fixed,
                n_trials=n_trials,
            )
        )
    return rows


def build_p0_matrix(
    full=False, random_seeds=40, smoke=False, stats=False, n_trials=1, pending_only=False,
):
    """
    Build the P0-only matrix: capability sweep plus hand-written baselines.

    `pending_only` restricts the capability sweep to `pending=True` and drops
    the full-spec/move-only baselines entirely (they have no pending axis).
    It exists to backfill the pending axis into a matrix already run before
    pending existed: the emitted rows are exactly the delta versus a prior
    `build_p0_matrix(full=..., random_seeds=..., pending_only=False)` call
    with the same arguments, so running them with
    `batch_run_harness --append` against that run's `--out-dir` adds the new
    `pending=True` rows without duplicating anything already collected.
    Ignored when `smoke=True` (smoke never varies pending).

    `stats` is like `full` for ordering-sweep depth (uses `random_seeds`
    random seeds instead of 1) but never includes the `off` reordering
    policy, regardless of `full` -- a statistics-focused sweep at
    `threshold_250` only, for when `off`'s risk of a multi-hour outlier isn't
    worth taking. Takes precedence over `full` (like `smoke` does over both).
    """
    root = package_root()
    p0 = _p0_paths(root)
    rows = []

    if smoke:
        # One row per pipeline variant (capability, full-spec, move-only),
        # single fixed ordering — exercise all three build paths cheaply.
        ordering = ordering_rows(full=False)[0]
        rows.append(
            _capability_row(
                system_name='p0_capabilities_demo',
                spec_name='p0_capabilities',
                spec_path=p0['capabilities'][0],
                capabilities_path=p0['capabilities'][1],
                encoding='one-hot',
                liveness='S',
                ordering=ordering,
                n_trials=n_trials,
            )
        )
        rows.append(
            _full_spec_row(
                system_name='p0_full_spec_demo',
                spec_name='p0_full_spec',
                spec_path=p0['full_spec'][0],
                capabilities_path=p0['full_spec'][1],
                encoding='full_spec',
                ordering=ordering,
                n_trials=n_trials,
            )
        )
        rows.append(
            _full_spec_row(
                system_name='p0_move_only_spec_demo',
                spec_name='p0_move_only_spec',
                spec_path=p0['move_only_spec'][0],
                capabilities_path=p0['move_only_spec'][1],
                encoding='full_spec_move_only',
                ordering=ordering,
                n_trials=n_trials,
            )
        )
        return {'rows': rows}

    if stats:
        full = False  # --stats takes precedence over --full: never include `off`.
    orderings = ordering_rows(full=full, random_seeds=random_seeds, stats=stats)

    pending_values = (True,) if pending_only else (False, True)
    for encoding, system_name, spec_name in (
        ('one-hot', 'p0_capabilities_demo', 'p0_capabilities'),
        ('enumerated', 'p0_capabilities_parsed_demo', 'p0_parsed'),
    ):
        for liveness in ('S', 'F'):
            for pending in pending_values:
                for ordering in orderings:
                    rows.append(
                        _capability_row(
                            system_name=system_name,
                            spec_name=spec_name,
                            spec_path=p0['capabilities'][0],
                            capabilities_path=p0['capabilities'][1],
                            encoding=encoding,
                            liveness=liveness,
                            pending=pending,
                            ordering=ordering,
                            n_trials=n_trials,
                        )
                    )

    if not pending_only:
        for ordering in orderings:
            rows.append(
                _full_spec_row(
                    system_name='p0_full_spec_demo',
                    spec_name='p0_full_spec',
                    spec_path=p0['full_spec'][0],
                    capabilities_path=p0['full_spec'][1],
                    encoding='full_spec',
                    ordering=ordering,
                    n_trials=n_trials,
                )
            )
            rows.append(
                _full_spec_row(
                    system_name='p0_move_only_spec_demo',
                    spec_name='p0_move_only_spec',
                    spec_path=p0['move_only_spec'][0],
                    capabilities_path=p0['move_only_spec'][1],
                    encoding='full_spec_move_only',
                    ordering=ordering,
                    n_trials=n_trials,
                )
            )

    return {'rows': rows}


def build_p1_matrix(full=False, n_trials=1, stats=False, random_seeds=20):
    """
    Build the P1-only matrix: single fixed-ordering baseline rows.

    P1 never sweeps ordering by default, regardless of `full` — see module
    docstring. `full` gates the expensive `p1_capabilities` (one-hot) row,
    which resolved in ~23,307 s (~6.5 h) once actually given the room -- set
    `--synthesis-timeout-s` deliberately (e.g. `43200`, 12h, for real margin)
    when using it; it is not part of the routine `pyrobosim_p1` main-track
    task.

    `stats` is the one exception: it sweeps ordering for 4S/4F only (2 fixed
    + `random_seeds` random, `threshold_250` only, same shape as P0's
    `--stats`) -- see `_p1_baseline_rows` for why 4SP/4FP/FullP1 stay fixed
    regardless.
    """
    root = package_root()
    return {
        'rows': _p1_baseline_rows(
            root,
            n_trials=n_trials,
            include_unsolvable=full,
            stats=stats,
            random_seeds=random_seeds,
        ),
    }


def _add_p0_subparser(subparsers):
    """Add the `p0` subcommand: capability sweep plus hand-written baselines."""
    p0_parser = subparsers.add_parser(
        'p0',
        help='Generate P0-only matrix rows (capability sweep + baselines).',
    )
    p0_parser.add_argument('output', help='Matrix YAML output path.')
    p0_parser.add_argument(
        '--full',
        action='store_true',
        help='Deepen the ordering sweep to 4 fixed rows + --random-seeds '
             'random seeds x 2 reordering policies per cell (off included). '
             'Without --full: 2 fixed rows + 1 random seed, threshold_250 only.',
    )
    p0_parser.add_argument(
        '--smoke',
        action='store_true',
        help='Generate one row per pipeline variant (capability, full-spec, '
             'move-only), single fixed ordering (threshold_250). Overrides '
             '--stats and --full.',
    )
    p0_parser.add_argument(
        '--stats',
        action='store_true',
        help='Deepen the ordering sweep to 2 fixed rows + --random-seeds '
             'random seeds, threshold_250 only -- never includes off, unlike '
             '--full. For generating a statistically meaningful random-seed '
             "distribution (e.g. --random-seeds 20) without off's risk of a "
             'multi-hour outlier. Overrides --full.',
    )
    p0_parser.add_argument(
        '--random-seeds',
        type=int,
        default=40,
        help='Number of random ordering seeds for --full or --stats.',
    )
    p0_parser.add_argument(
        '--n-trials',
        type=int,
        default=1,
        help='Exact repeated trials per matrix row.',
    )
    p0_parser.add_argument(
        '--pending-only',
        action='store_true',
        help='Emit only the pending=True capability rows, no pending=False '
             'rows and no full-spec/move-only baselines. Use this to backfill '
             'the pending axis into a matrix already run before pending '
             'existed: pass the same --full/--random-seeds/--n-trials used '
             'for that prior run, then feed the result to batch_run_harness '
             "with --append against that prior run's --out-dir. Ignored with --smoke.",
    )


def _add_p1_subparser(subparsers):
    """Add the `p1` subcommand: single fixed-ordering baseline rows."""
    p1_parser = subparsers.add_parser(
        'p1',
        help='Generate P1-only matrix rows (single fixed-ordering baselines).',
    )
    p1_parser.add_argument('output', help='Matrix YAML output path.')
    p1_parser.add_argument(
        '--full',
        action='store_true',
        help='Also emit the p1_capabilities (one-hot, non-parsed) baseline '
             'row. Documented in the PyRoboSim synthesis README as not '
             'solvable within 10 hours — off by default so a normal run '
             'cannot silently stall on it. Unlike p0 --full, this does not '
             'change ordering: P1 rows are always a single fixed ordering.',
    )
    p1_parser.add_argument(
        '--n-trials',
        type=int,
        default=1,
        help='Exact repeated trials per matrix row. Keep this at 1 unless '
             'you have the compute budget to multiply an already multi-hour row.',
    )
    p1_parser.add_argument(
        '--stats',
        action='store_true',
        help='Sweep ordering for 4S/4F only (2 fixed + --random-seeds random, '
             'threshold_250 only, same shape as p0 --stats) instead of a '
             'single fixed ordering. FullP1/4SP/4FP are unaffected -- always '
             'single fixed ordering regardless of --stats.',
    )
    p1_parser.add_argument(
        '--random-seeds',
        type=int,
        default=20,
        help='Number of random ordering seeds for --stats.',
    )


def main():
    """Run the PyRoboSim matrix generator CLI."""
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest='system', required=True)
    _add_p0_subparser(subparsers)
    _add_p1_subparser(subparsers)
    args = parser.parse_args()

    if args.system == 'p0':
        matrix = build_p0_matrix(
            full=args.full,
            random_seeds=args.random_seeds,
            smoke=args.smoke,
            stats=args.stats,
            n_trials=args.n_trials,
            pending_only=args.pending_only,
        )
    else:
        matrix = build_p1_matrix(
            full=args.full,
            n_trials=args.n_trials,
            stats=args.stats,
            random_seeds=args.random_seeds,
        )

    output = Path(args.output).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(yaml.safe_dump(matrix, sort_keys=False), encoding='utf-8')
    print(f"Wrote {len(matrix['rows'])} PyRoboSim {args.system} matrix row(s) to '{output}'")


if __name__ == '__main__':
    main()
