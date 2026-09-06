# Experiment Utilities

This directory contains scripts for reproducing and packaging the release
experiment datasets for the FlexBE synthesis demonstrations. The scripts assume
they are run from a sourced ROS workspace that contains both this repository and
`flexbe_synthesis`.

Most scripts write under `FLEXBE_SYNTHESIS_HOME`, which defaults to
`~/.flexbe_synthesis`. For isolated runs, set it explicitly:

```bash
export FLEXBE_SYNTHESIS_HOME=/tmp/flexbe_synthesis_release_run
```

## Main Workflow

`run_experiment_plan.sh` runs the main experiment set:

- Coffee baseline from `flexbe_synthesis`
- Two Rivers release sweep
- PyRoboSim P0 statistics sweep
- PyRoboSim P1 baseline rows
- Drone demo statistics sweep
- Drone demo Fair-Outcome-with-pending rows

Start from an empty synthesis home:

```bash
./experiments/run_experiment_plan.sh --clear --no-terminal-hold
```

The script records machine information, starts each task, writes task state files
under `$FLEXBE_SYNTHESIS_HOME/run_state`, and stores logs under
`$FLEXBE_SYNTHESIS_HOME/run_logs`.

`weekend_run.sh` runs the main workflow and the follow-up outlier scripts in
sequence. Use it when you want the complete long-running dataset pass:

```bash
./experiments/weekend_run.sh --clear
```

## Follow-Up Scripts

`run_pyrobosim_p1_outliers.sh` retries the long PyRoboSim P1 FullP1
well-separation row with a larger budget.

`run_pyrobosim_p1_stats.sh` adds the PyRoboSim P1 `4S` and `4F` ordering sweep
to the same run directory used by the main workflow.

`run_drone_demo_outliers.sh` retries the drone demo `1FP` and `2FP` rows with a
larger synthesis budget.

`run_drone_demo_stats_1sp_outliers.sh` retries only the drone demo `1SP`
ordering rows whose well-separation checks did not finish in the main run.

`run_drone_demo_long_retry.sh` is an older combined retry helper for `1FP`,
`2FP`, and incomplete `1SP` rows. Prefer the more focused scripts above for new
runs.

`retry_drone_demo_timeouts.py` builds a retry matrix from an existing
`drone_demo_stats/master.csv` by selecting rows with non-zero return codes.

## Repair And Export

`resume_realize_hfsm.py` repairs runs where synthesis finished but HFSM
realization failed or was interrupted. It reuses cached synthesis byproducts
instead of rerunning Slugs.

`run_well_separation_checks.py` scans existing Slugs byproducts and writes
well-separation CSV/JSONL summaries for later aggregation.

`export_release_data.py` copies the canonical completed run directories into a
clean `release_data/` bundle. It exports `master.csv`, `summary.csv`, `tables/`,
and `plots/`, while scrubbing local home-directory paths and hostnames from CSV
cells.

Run a dry pass first:

```bash
python3 experiments/export_release_data.py --dry-run
```

Then export:

```bash
python3 experiments/export_release_data.py
```

Before publishing the bundle, inspect the exported CSV files for local paths or
hostnames and confirm that the canonical run names in `export_release_data.py`
match the dataset you intend to ship.
