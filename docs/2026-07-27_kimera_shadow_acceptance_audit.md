# Kimera Shadow Acceptance Audit

Date: 2026-07-27

## Purpose

This audit tests a narrow hypothesis from the Outdoor01 backward-window
investigation:

> Does Kimera's accepted backward update occur because the incremental iSAM2
> update did not relinearize and re-eliminate enough of the active graph?

The experiment keeps the production update unchanged. A copied smoother
receives the same new factors and values, forces relinearization and a full
linear solve, records the resulting candidate, and is then discarded.

The audit does not change:

- accepted estimator values;
- fixed-lag marginalization;
- SmartStereo track lifecycle;
- CBS factor insertion or removal;
- Gauss-Newton solver selection; or
- GTSAM 4.3.

The launch flag is
`kimera_shadow_acceptance_audit_enable:=true`. Its default is `false`.

## Implementation

The active production path is Kimera's
`cbs::IncrementalFixedLagBpsamSmoother`, rather than the separate persistent
local-covariance sidecar.

The implementation:

1. copies the pre-update smoother;
2. isolates the copied nonlinear factor graph so mutable SmartStereo caches are
   not shared with production;
3. performs the unchanged production update and accepts its result;
4. performs a shadow update on the copy with `force_relinearize=true` and
   `forceFullSolve=true`;
5. evaluates both candidates on the same initialized factor graph;
6. logs per-category factor errors and pose movement; and
7. discards the shadow smoother.

Relevant code:

- Shadow fixed-lag update and measurements:
  `src/cbs/include/cbs/bpsam/incremental_fixed_lag_bpsam_smoother.h`
- Production/shadow orchestration and logging:
  `src/Kimera-VIO/src/backend/VioBackend.cpp`
- Audit-only nonlinear graph isolation:
  `src/cbs/include/cbs/bpsam/bpsam.h`
- SmartStereo clone support:
  `src/gtsam/gtsam_unstable/slam/SmartStereoProjectionPoseFactor.h`
- Report parser:
  `src/cbsms/tools/analyze_kimera_shadow_acceptance.py`

## Experiment

Dataset and configuration:

- M3DGR Outdoor01;
- 30 seconds of bag time at bag rate 0.5;
- Kimera only, CBS disabled;
- 50-state fixed-lag window;
- `maxFeatureAge: 16`;
- GTSAM 4.3;
- Gauss-Newton.

Audit run:

```text
runs/20260727-234736_m3dgr_outdoor01_native_shadow_audit_30s_rate05
```

Diagnostic-off parity control:

```text
runs/20260727-225954_m3dgr_outdoor01_production_bpsam_gtsam43_gn_n50_age16_cbs_off_diagoff_30s_rate05
```

Generated report:

```text
runs/20260727-234736_m3dgr_outdoor01_native_shadow_audit_30s_rate05/parsed/shadow_acceptance/report.md
```

## Output Neutrality

The audit and control each produced 149
`KIMERA_BACKEND_POSE_PATH_ROW` records. All 149 common rows are identical.

This establishes that enabling the shadow audit did not alter the production
pose path in this experiment.

## Aggregate Results

Across 149 valid normal/shadow comparisons:

| metric | accepted normal | forced-full shadow |
|---|---:|---:|
| updates increasing audited nonlinear objective | 110 | 38 |
| objective ratio p50 | 1.00674 | 0.928501 |
| objective ratio p95 | 2.26288 | 1.23866 |
| objective ratio maximum | 53890.3 | 1.70508 |
| variables relinearized p50 | 100 | 149 |
| variables relinearized maximum | 156 | 156 |

The shadow candidate ended below the accepted normal candidate in 118 of 149
updates. Forced relinearization therefore fixes or reduces many incremental
update failures, especially early in the run.

The audited objective sums directly evaluable nonlinear factors.
`LinearContainerFactor` marginal priors require evaluation in iSAM2 delta
coordinates and appear as zero in this audit's category table. The earlier
frame-113 objective audit evaluated that prior separately.

## Critical Frame 113

Frame 113 is the metre-scale backward-window event.

| measurement | accepted normal | forced-full shadow |
|---|---:|---:|
| objective before | 237.296338 | 237.296338 |
| objective after | 366.847995 | 366.847301 |
| after / before | 1.54594883 | 1.54594590 |
| variables relinearized | 155 | 156 |
| variables re-eliminated | 156 | 156 |
| mean pose movement | 1.037321 m | effectively identical |
| maximum pose movement | 2.369342 m | effectively identical |

The normal and shadow accepted-state candidates differ by only:

- 0.000001 m mean translation;
- 0.000002 m maximum translation.

Factor-category changes:

| category | before | normal after | shadow after |
|---|---:|---:|---:|
| bias between | 10.127114 | 10.419148 | 10.419281 |
| IMU | 12.342670 | 162.112026 | 162.111309 |
| SmartStereo | 214.826554 | 194.316821 | 194.316711 |

Both candidates trade a SmartStereo improvement of about 20.51 for an IMU
increase of about 149.77. The earlier delta-coordinate audit additionally
measured a marginal-prior increase of about 1.07.

## Interpretation

The frame-113 event is not caused by too little incremental relinearization or
partial re-elimination:

- the production update already relinearized 155 of 156 variables;
- both paths re-eliminated all 156 variables;
- forcing the final variable to relinearize and forcing a full solve produced
  the same metre-scale candidate; and
- both candidates substantially increased the represented nonlinear
  objective.

`forceFullSolve` still computes one Gauss-Newton step from one linearization.
It is not a multi-iteration nonlinear solve and does not provide line search,
damping, or objective acceptance.

The evidence therefore supports a more precise diagnosis:

1. incremental caching/relinearization is harmful on many updates and a full
   shadow solve often improves them;
2. it is not the cause of the critical frame-113 backward revision;
3. frame 113 is an objective-increasing one-step Gauss-Newton candidate;
4. this audit gives no evidence of a broken fixed-lag Schur marginalization;
5. no production estimator policy has been changed.

## Next Safe Diagnostic

Before adding any production rollback policy, use another discarded shadow
candidate at the flagged frames:

1. begin from the same pre-update smoother;
2. run multiple nonlinear Gauss-Newton iterations, or backtrack the accepted
   step length;
3. evaluate every candidate on the same nonlinear graph;
4. include direct delta-coordinate evaluation of the linear marginal prior;
5. compare pose movement and factor-category errors with the accepted update;
6. discard all candidates.

This would distinguish an oversized one-step Gauss-Newton correction from a
deeper graph-model inconsistency while preserving the current marginalization
and CBS implementation.
