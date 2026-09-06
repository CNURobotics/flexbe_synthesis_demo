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
Generate drone_demo (drones_gates) experiment matrix YAML for the Slugs batch harness.

There is only one capability model here (`capabilities/drones_gates.yaml`), unlike
PyRoboSim's cheap-P0/expensive-P1 split, so this generator is single-tier like the Two
Rivers one. But per-trial synthesis is noticeably slower than Two Rivers (tens of seconds,
not a few) now that the model carries the gate-obstacle-avoidance capabilities, so the
default (non-`--full`) sweep follows PyRoboSim's caution rather than Two Rivers'
permissiveness: the unsifted (`off`) CUDD reordering policy is only exercised under
`--full`, not by default — see `_reordering_treatments`.

There is no hand-written full-spec baseline for this demo (unlike Two Rivers'
`two_rivers.yaml`/`two_rivers_parsed.yaml` or PyRoboSim's `p0_full_spec.yaml`): only the
capability-generated rows are produced. `specs/drones_gates.yaml` is a small environment
overlay for those rows (route bootstrap facts), not a standalone spec.

`liveness='F', pending=True` (both encodings) is excluded from the default and `--full`
sweeps entirely, mirroring PyRoboSim's exclusion of its expensive one-hot
`p1_capabilities` row (which needed hours, not the 900-7200s range, once actually run) from
its own routine sweep. It didn't just run long: it failed to complete the realizability
check itself within a 900s budget, and separately within a 7200s (2 hour) budget, in
testing. Since even 7200s isn't confirmed sufficient, sweeping it (`--full` would otherwise
multiply its cost across many orderings) isn't practical. `--fp` generates just those 2 rows
(alphabetic ordering only, threshold_250) for anyone who wants to let it run much longer
(e.g. `--synthesis-timeout-s 18000`, 5 hours) and see.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import yaml

_ROUTE_INITIAL_CONDITIONS = ['begin_game_a', 'gate1=0', 'gate2=11', 'gate3=6', 'gate4=13']
_GOALS = ['log_finished']
# Unlike Two Rivers/PyRoboSim's audit_goals=['finished'], this model has a
# real, designed terminal-failure path (execution failure -> recovery landing
# -> disarm -> 'failed', see log_recovery_finished), reachable from a genuine
# in-flight execution failure. audit_goals=['finished'] alone configures a
# *task-completion* audit ("always eventually reach success"), under which
# that terminal-failure branch is correctly flagged GOAL_UNREACHABLE_TRAP
# (see flexbe_synthesis_slugs/docs/auditor.md, "Goal Outcomes") -- reaching
# 'failed' does not vacuously satisfy a success-only goal, and there is no
# path back to 'finished' from there by design. Including both here instead
# configures a *termination* audit ("always eventually reach some terminal
# state"), which is the correct check for this model: it does have a
# non-vacuous liveness goal (log_finished/route completion) driving
# synthesis, but the auditor should accept either legitimate terminal SM
# outcome, not treat the recovery-and-report-failure path as a defect.
_AUDIT_GOALS = ['finished', 'failed']
_SM_OUTCOMES = ['finished']
# Random-seed count for the default (non-`--full`) sweep. `--full` uses --random-seeds
# (default 40) instead; this is only for the compact default matrix.
_DEFAULT_RANDOM_SEEDS = 10
# (liveness, pending) combinations excluded from the standard cell sweep -- see module
# docstring. Only reachable via `--fp`.
_EXCLUDED_CELLS = {('F', True)}


def package_root() -> Path:
    """Return the drone_demo_flexbe_synthesis package source root."""
    return Path(__file__).resolve().parents[1]


def _reordering_treatments(include_off=True):
    """
    Return the CUDD reordering policies used for the sweep.

    `include_off` gates the unsifted (`--no-reorder`) policy. Only `--full` passes
    `include_off=True` — the default matrix skips it entirely, following the PyRoboSim
    precedent: letting a capability model with this many sys_props (gate route selection,
    per-gate boundary flags, the two shared obstacle-avoidance flags, flight-phase flags)
    go completely unsifted risked the same kind of BDD blowup PyRoboSim's overnight `off`
    run hit, well past what the `off` vs `threshold_250` comparison is meant to cost. This
    hasn't been separately re-verified for drones_gates specifically; treat it as a
    precaution carried over from that finding, not an independent measurement.
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
    Return ordering and reordering treatments for the sweep.

    `include_off` defaults to `full` — the unsifted policy is only exercised by the
    `--full` sweep; the default (compact) sweep and `--stats` always reorder. The default
    sweep uses `_DEFAULT_RANDOM_SEEDS` random seeds; `--full` and `--stats` both use
    `random_seeds` instead, but only `--full` includes `off`.
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
    seed_count = random_seeds if (full or stats) else _DEFAULT_RANDOM_SEEDS
    rows.extend(
        {
            'ordering_mode': 'random',
            'ordering_seed': seed,
            **reordering,
        }
        for reordering in reorderings
        for seed in range(seed_count)
    )
    return rows


_ENCODING_DIGIT = {'one-hot': '1', 'enumerated': '2'}


def _capability_row(root, *, system_name, encoding, liveness, pending, ordering, n_trials):
    """Build one capability-generated drones_gates matrix row."""
    digit = _ENCODING_DIGIT[encoding]
    return {
        'example': 'drone_demo',
        'system_name': system_name,
        'spec_name': 'drones_gates',
        'spec_path': str(root / 'specs' / 'drones_gates.yaml'),
        'capabilities_path': str(root / 'capabilities' / 'drones_gates.yaml'),
        'encoding': encoding,
        'liveness': liveness,
        'pending': pending,
        'column_indicator': f"{digit}{liveness}{'P' if pending else ''}",
        'initial_conditions': list(_ROUTE_INITIAL_CONDITIONS),
        'goals': list(_GOALS),
        'audit_goals': list(_AUDIT_GOALS),
        'sm_outcomes': list(_SM_OUTCOMES),
        'n_trials': n_trials,
        **ordering,
    }


def build_matrix(full=False, random_seeds=40, smoke=False, fp=False, stats=False, n_trials=1):
    """
    Build the drones_gates matrix: capability sweep only (no full-spec baseline).

    `smoke` emits one row per encoding (2 rows total), single fixed ordering
    (`alphabetic`, `threshold_250`), `pending=False`, `liveness='S'` — enough to exercise
    both encodings' build path cheaply before committing to a full sweep. Takes precedence
    over `fp`, `stats`, and `full`.

    `fp` emits only the `liveness='F', pending=True` rows (both encodings, alphabetic
    ordering only, threshold_250) excluded from the standard sweep below -- see module
    docstring. Takes precedence over `stats` and `full`.

    `stats` is like `full` for ordering-sweep depth (uses `random_seeds` random seeds
    instead of the default sweep's `_DEFAULT_RANDOM_SEEDS`) but never includes `off`,
    regardless of `full` -- a statistics-focused sweep at `threshold_250` only, for when
    `off`'s risk of a multi-hour outlier isn't worth taking. Takes precedence over `full`.

    Otherwise, the standard sweep covers every (encoding, liveness, pending) cell except
    `('F', True)`.
    """
    root = package_root()
    rows = []

    if smoke:
        ordering = ordering_rows(full=False)[0]
        for encoding, system_name in (
            ('one-hot', 'drone_gates_capabilities_demo'),
            ('enumerated', 'drone_gates_capabilities_parsed_demo'),
        ):
            rows.append(
                _capability_row(
                    root,
                    system_name=system_name,
                    encoding=encoding,
                    liveness='S',
                    pending=False,
                    ordering=ordering,
                    n_trials=n_trials,
                )
            )
        return {'rows': rows}

    if fp:
        ordering = ordering_rows(full=False)[0]
        for encoding, system_name in (
            ('one-hot', 'drone_gates_fp_demo'),
            ('enumerated', 'drone_gates_fp_parsed_demo'),
        ):
            rows.append(
                _capability_row(
                    root,
                    system_name=system_name,
                    encoding=encoding,
                    liveness='F',
                    pending=True,
                    ordering=ordering,
                    n_trials=n_trials,
                )
            )
        return {'rows': rows}

    if stats:
        full = False  # --stats takes precedence over --full: never include `off`.
    orderings = ordering_rows(full=full, random_seeds=random_seeds, stats=stats)

    for encoding, system_name in (
        ('one-hot', 'drone_gates_capabilities_demo'),
        ('enumerated', 'drone_gates_capabilities_parsed_demo'),
    ):
        for liveness in ('S', 'F'):
            for pending in (False, True):
                if (liveness, pending) in _EXCLUDED_CELLS:
                    continue
                for ordering in orderings:
                    rows.append(
                        _capability_row(
                            root,
                            system_name=system_name,
                            encoding=encoding,
                            liveness=liveness,
                            pending=pending,
                            ordering=ordering,
                            n_trials=n_trials,
                        )
                    )

    return {'rows': rows}


def main():
    """Run the drone_demo matrix generator CLI."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', help='Matrix YAML output path.')
    parser.add_argument(
        '--full',
        action='store_true',
        help='Deepen the ordering sweep to 4 fixed rows + --random-seeds random seeds '
             'x 2 reordering policies per cell (off included). Without --full: 2 fixed '
             'rows + 10 random seeds, threshold_250 only. Excludes liveness=F,pending=True '
             'either way -- see --fp.',
    )
    parser.add_argument(
        '--smoke',
        action='store_true',
        help='Generate one row per encoding (one-hot, enumerated), single fixed '
             'ordering (threshold_250). Overrides --fp, --stats, and --full.',
    )
    parser.add_argument(
        '--fp',
        action='store_true',
        help='Generate only the liveness=F, pending=True rows (both encodings, '
             'alphabetic ordering, threshold_250) excluded from the default and --full '
             'sweeps -- these did not complete the realizability check within a 900s '
             'or even a 7200s budget in testing. Use a much longer '
             '--synthesis-timeout-s (e.g. 18000, 5 hours) when running this. '
             'Overrides --stats and --full.',
    )
    parser.add_argument(
        '--stats',
        action='store_true',
        help='Deepen the ordering sweep to 2 fixed rows + --random-seeds random '
             'seeds, threshold_250 only -- never includes off, unlike --full. For '
             'generating a statistically meaningful random-seed distribution '
             "(e.g. --random-seeds 20) without off's risk of a multi-hour outlier. "
             'Overrides --full.',
    )
    parser.add_argument(
        '--random-seeds',
        type=int,
        default=40,
        help='Number of random ordering seeds for --full or --stats.',
    )
    parser.add_argument(
        '--n-trials',
        type=int,
        default=1,
        help='Exact repeated trials per matrix row.',
    )
    args = parser.parse_args()

    matrix = build_matrix(
        full=args.full,
        random_seeds=args.random_seeds,
        smoke=args.smoke,
        fp=args.fp,
        stats=args.stats,
        n_trials=args.n_trials,
    )
    output = Path(args.output).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(yaml.safe_dump(matrix, sort_keys=False), encoding='utf-8')
    print(f"Wrote {len(matrix['rows'])} drone_demo matrix row(s) to '{output}'")


if __name__ == '__main__':
    main()
