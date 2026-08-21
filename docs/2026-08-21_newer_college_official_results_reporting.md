# Official Newer College CBS Results Reporting Contract

**Date frozen:** 2026-08-21
**Dataset:** `2021-ouster-os0-128-alphasense`
**Results root:**
`/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/newer_college_official_benchmark`

This is the mandatory reporting structure for the paper's Newer College CBS
experiments. It covers all nine logical sequences and the two paper conditions,
`cbs_off` and `cbs_on`.

## Directory contract

```text
newer_college_official_benchmark/
├── benchmark_manifest.json
├── sequences/
│   ├── quad-easy/
│   │   ├── cbs_off/
│   │   └── cbs_on/
│   ├── quad-medium/{cbs_off,cbs_on}/
│   ├── quad-hard/{cbs_off,cbs_on}/
│   ├── stairs/{cbs_off,cbs_on}/
│   ├── cloister/{cbs_off,cbs_on}/
│   ├── park/{cbs_off,cbs_on}/
│   ├── maths-easy/{cbs_off,cbs_on}/
│   ├── maths-medium/{cbs_off,cbs_on}/
│   └── maths-hard/{cbs_off,cbs_on}/
├── pilots/
└── aggregate/
```

Every official result slot contains:

```text
<sequence>/<mode>/
├── STATUS.md
├── REPORT.md
├── result.json
├── SHA256SUMS
├── metrics/
├── trajectories/
│   ├── raw/
│   └── aligned/
├── figures/
│   ├── trajectories_xy.{png,pdf}
│   ├── trajectories_3d.{png,pdf}
│   └── translation_ape.{png,pdf}
├── rerun/
├── configs/
├── logs/
└── provenance/
```

Large `.rrd` recordings are not duplicated. Their absolute source path, size,
SHA256, and `rerun rrd verify` result are recorded in `result.json` and
`provenance/source_artifacts.tsv`. The small trajectories, metrics,
configuration snapshots, figures, and provenance files are copied into the
result package and covered by `SHA256SUMS`.

## Required reported quantities

For both GLIM and Kimera, every result records:

- translation APE RMSE in metres;
- pose count;
- evaluated estimator duration;
- estimator path length;
- matched Oxford reference path length;
- mean, median, standard deviation, minimum, and maximum translation APE;
- rigid alignment rotation and translation;
- timestamp-association tolerance.

At sequence level it records:

- reported Oxford ground-truth start and end timestamps;
- elapsed Oxford ground-truth time;
- Oxford ground-truth distance travelled;
- full-sequence versus partial coverage;
- CBS mode, direction, activity count, recording ID, and requested duration;
- Rerun recording/blueprint paths and hashes;
- exact estimator configurations and run provenance.

The paper metric is translation APE RMSE after one Umeyama/Kabsch SE(3)
alignment per estimator, with scale fixed at one, in Oxford Base. No Sim(3),
time-offset fitting, trajectory deformation, or estimator feedback is allowed.

## Status and admission rules

- `PENDING`: no evaluated run exists.
- `PILOT`: a partial window or debugging run. It is preserved under `pilots/`
  and never enters the paper table.
- `INCOMPLETE`: an attempted official run with missing coverage or artifacts.
- `COMPLETE`: a verified full logical sequence eligible for aggregation.

The finalizer refuses `COMPLETE` unless it receives:

- the explicit `--full-sequence` assertion;
- standardized GLIM and Kimera metrics and TUM trajectories;
- a canonical `.rrd` and `.rbl`, both passing `rerun rrd verify`;
- at least one provenance file;
- a CBS activity count: exactly zero for `cbs_off`, positive for `cbs_on`.

The collector admits a sequence to the paper table only when both its off and
on packages are `COMPLETE`, both claim full-sequence coverage, and their Oxford
GT start/end/distance coverage matches. This prevents comparing two modes on
different physical intervals. It also requires all admitted CBS-on packages to
use one consistent exchange direction; `g2k`, `k2g`, and `bidirectional` runs
cannot be silently mixed under a single `CBS on` paper column.

## Tool

Use:

`src/cbsms/tools/newer_college_results.py`

It runs in the existing Rerun SDK environment:

```bash
PY=/home/yeranis/.local/share/pipx/venvs/rerun-sdk/bin/python
ROOT=/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/newer_college_official_benchmark
```

The result tree is already initialized. Reinitialization is idempotent:

```bash
$PY src/cbsms/tools/newer_college_results.py init --root "$ROOT"
```

Finalize an official CBS-off run after its standardized evaluation directory
contains `metrics.json`, `kimera_metrics_base.json`, `ground_truth_base.tum`,
`kimera_ground_truth_base.tum`, `glim_odom_base.tum`, and `kimera_base.tum`:

```bash
$PY src/cbsms/tools/newer_college_results.py finalize \
  --result-dir "$ROOT/sequences/maths-easy/cbs_off" \
  --sequence maths-easy \
  --mode cbs_off \
  --status COMPLETE \
  --full-sequence \
  --source-run <run-dir> \
  --evaluation-dir <run-dir>/evaluation \
  --recording-id <recording-id> \
  --cbs-direction none \
  --cbs-activity-count 0 \
  --rrd <canonical.rrd> \
  --rbl <canonical.rbl> \
  --source-report <run-report.md> \
  --config-dir <glim-config> \
  --config-dir <kimera-config> \
  --provenance-file <run-manifest>
```

For CBS on, use `--mode cbs_on`, the actual direction (`g2k`, `k2g`, or
`bidirectional`), and the verified positive CBS insertion/activity count.

After every finalized run, regenerate all aggregate outputs:

```bash
$PY src/cbsms/tools/newer_college_results.py collect --root "$ROOT"
```

## Aggregate outputs

- `aggregate/official_results_long.csv`: machine-readable row per
  sequence/mode/estimator.
- `aggregate/official_results.json`: complete machine-readable registry and
  admission state.
- `aggregate/paper_table.md`: working human-readable paper table.
- `aggregate/paper_table.tex`: LaTeX table source.
- `aggregate/completion_matrix.md`: off/on completion and admission status.
- `aggregate/trajectory_figures.md`: index of all admitted trajectory plots.

The average row is emitted only when the complete nine-sequence off/on matrix
is available. Missing results remain visibly blank; they are never silently
averaged away.

## Current validated seed

The existing Maths-Easy 80-second CBS-off result has been packaged under:

`pilots/maths-easy/cbs_off/80s_20260821`

It demonstrates and validates the workflow, figures, hashes, and Rerun
provenance. It remains `PILOT`, because it is not the complete logical
Maths-Easy sequence, and therefore does not populate the official paper table.

## Status

**FROZEN AS THE OFFICIAL NEWER COLLEGE RESULT-PACKAGING AND PAPER-TABLE CONTRACT.**
