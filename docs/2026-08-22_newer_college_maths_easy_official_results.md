# Newer College Maths-Easy official CBS-off/on results

**Frozen:** 2026-08-22

**Release:** `2021-ouster-os0-128-alphasense`

**Sequence:** Collection 3, Maths-Easy

**Workspace:** `/home/yeranis/repos/V4RL/cbs_gtsam4.3`

This document preserves the first complete official Newer College result pair
and the exact experiment contract to reuse for the remaining eight logical
sequences. Generated runs, ROS bags, and Rerun recordings remain outside Git;
their paths and hashes are recorded here and in the local official packages.

Read this together with:

- [the official result-packaging contract](2026-08-21_newer_college_official_results_reporting.md);
- [the Newer College Rerun preset](2026-08-21_newer_college_cbs_off_rerun_preset.md);
- [the original GLIM baseline handoff](2026-08-20_newer_college_original_glim_baseline_handoff.md).

## Dataset interval

The complete logical sequence consists of both original bag pieces, in this
order:

```text
2021-04-07-13-49-03_0-math-easy.bag
2021-04-07-13-52-31_1-math-easy.bag
```

The pieces span approximately `215.999079 s`; the official runner requests
`216.5 s` to ensure playback reaches the physical end. Replay rate is `0.75x`.
Ground truth is Collection 3 `ground_truth/gt_state_easy.csv`.

The result packager reports the complete supplied Oxford ground-truth interval:

```text
start:    1617799772.937803 s
end:      1617799988.824909 s
duration: 215.887105942 s
distance: 263.621222553 m
```

## Frozen estimator setup

GLIM:

- edited `cbs_gtsam4.3` implementation;
- optimized GPU Ouster OS0-128 profile;
- `/os_cloud_node/points` and `/os_cloud_node/imu`;
- Oxford Ouster calibration and metadata;
- complete factor-graph history enabled for Rerun.

Kimera:

- edited `cbs_gtsam4.3` implementation;
- physical Alphasense `cam4` monocular stream;
- `/alphasense_driver_ros/imu`;
- Collection-3 cam4/IMU calibration;
- `nr_states: 50`;
- `numOptimize: 1`;
- two initial camera frames skipped by the compressed-image bridge.

The GLIM and Kimera body frames are respectively `os_imu` and
`imu_sensor_frame`. The static Kimera-base-to-GLIM-external transform is:

```text
translation:       [-0.051, -0.021, -0.041] m
quaternion (xyzw): [1.0, 0.0, 0.0, 0.0]
```

Do not reuse Collection-3 calibration blindly for another collection. The
estimator structure and reporting contract are reusable; bags, calibration,
ground truth, timestamps, and resulting alignments remain collection- and
sequence-specific.

## CBS-off result

CBS bridge/backend and all health-aware paths were disabled. The estimator log
was checked for zero accepted CBS activity.

| Estimator | translation APE RMSE [m] | duration [s] | estimated path [m] | poses |
|---|---:|---:|---:|---:|
| GLIM | 0.080682818 | 215.086019 | 250.915610 | 1,838 |
| Kimera | 2.832943058 | 215.595340 | 234.140069 | 925 |

Kimera aligned height RMSE was `0.470024114 m`.

## CBS-on setup and result

The official condition uses the original, unweighted, persistent,
bidirectional CBS path:

- G→K and K→G enabled;
- `20.0 s` receive-start delay;
- `0.20 s` relative-belief horizon;
- `schur_relative_between` covariance;
- duration-aware receiver edge matching;
- covariance scale `1.0` in both directions;
- no health-aware covariance inflation or active DCReg policy;
- persistent `BetweenFactor<Pose3>` insertion.

| Estimator | translation APE RMSE [m] | duration [s] | estimated path [m] | poses |
|---|---:|---:|---:|---:|
| GLIM | 0.081006742 | 215.086019 | 250.678915 | 1,446 |
| Kimera | 2.702541650 | 215.595340 | 226.491891 | 925 |

Accepted CBS activity:

```text
G→K: 1,481 factors
K→G:   220 factors
total: 1,701 factors
```

Relative to CBS off, Kimera RMSE improved by `0.130401408 m` (`4.60%`).
GLIM changed by `+0.000323924 m` (`+0.40%`), effectively neutral at this
scale. These are one-sequence observations, not yet a cross-dataset claim.

## Full 6x6 covariance audit

Every accepted CBS factor has a preserved sender and receiver-inserted 6x6
covariance in GTSAM tangent order:

```text
[rot_x, rot_y, rot_z, trans_x, trans_y, trans_z]
```

The recorded ROS arrays contained `50,538` nested belief entries because
rolling windows repeat and update earlier edges. The audit matches sender edge,
receiver acceptance/insertion, and receiver trace/diagonal, so it reports the
`1,701` factors actually inserted rather than counting ROS entries.

Validation:

- all `1,701 / 1,701` inserted covariances are SPD;
- G→K receiver-log agreement is exact at logged precision;
- K→G maximum relative receiver-log error is `3.838316e-06`;
- G→K median inserted trace is `1.78884263e-06`;
- K→G median inserted trace is `3.93872363e-04`.

The reusable generator is:

```text
tools/cbs_inserted_factor_covariance_report.py
```

It produces a CSV and compressed NPZ containing every full matrix, full-matrix
samples, component-wise p05/median/p95 matrices, and a machine-readable audit
summary. A complete CBS-on result is now rejected by
`tools/newer_college_results.py` unless this audit is complete and its factor
count equals the declared CBS activity count.

## Evaluation and visualization contract

- Translation APE is evaluated in Oxford Base.
- Each estimator receives one independent rigid SE(3) Umeyama/Kabsch
  alignment with scale fixed at one.
- No Sim(3), fitted clock offset, trajectory deformation, or estimator
  feedback is allowed.
- The Rerun recording contains time-growing trajectories, synchronized
  PointCloud2 and camera playback, Oxford GT, and both actual fixed-lag factor
  graphs.
- GLIM factor-graph context retains its full trajectory history.
- Synchronized video may be reconstructed from the original immutable bags to
  replace memory-evicted image entities; this must not alter estimator data.

## Local official result packages

```text
runs/newer_college_official_benchmark/sequences/maths-easy/cbs_off/
runs/newer_college_official_benchmark/sequences/maths-easy/cbs_on/
```

Both are `COMPLETE` and admitted to the aggregate paper table. Their package
`SHA256SUMS` files validate.

Canonical Rerun artifacts:

| mode | artifact | SHA256 |
|---|---|---|
| off | `cbs_off_full_official.rrd` | `baef1d4b43d3adc3543867378e939e04f94f752f8f2c19c2c4a02f16c70871ad` |
| off | `cbs_off_full.rbl` | `343860a1b330760b34f98ba8d284862c9c81f3681a5efd6ee843e62c8c15f60b` |
| on | `cbs_on_full_official.rrd` | `b2ad90e22e248be0e98acdebcb5e15c0df8c3b9eb5edf2d36eaf02d6fac9d1c9` |
| on | `cbs_on_full.rbl` | `e1586b80ceef1039734ad465b445d50f33c79dc95360d96fd830393568ddc1f0` |

The large Rerun and dataset files are deliberately not committed to GitHub.

## Status

**MATHS-EASY CBS-OFF/ON PAIR COMPLETE AND FROZEN AS THE FIRST OFFICIAL NEWER
COLLEGE RESULT.**
