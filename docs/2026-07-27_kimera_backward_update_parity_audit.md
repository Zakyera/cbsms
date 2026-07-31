# Kimera Backward-Update Parity Audit

Date: 2026-07-27

## Scope

This audit investigates the large backward revisions of Kimera's active
fixed-lag pose window on M3DGR Outdoor01. It deliberately does not change:

- Kimera marginalization;
- SmartStereo track lifecycle;
- the marginal-prior construction;
- CBS factor insertion or removal;
- the Gauss-Newton solver selection; or
- the current GTSAM 4.3 dependency.

The production controls use a 50-state window, `maxFeatureAge: 16`,
`numOptimize: 1`, Gauss-Newton, bag rate 0.5, and no external CBS factors.

## Implementations Compared

### Clean upstream Kimera

- Kimera commit: `ce8c59b7`
- Smoother: native `gtsam::IncrementalFixedLagSmoother`
- CBS: absent
- GTSAM: 4.3 for the principal parity control

### Production Kimera

- Kimera commit: `86d677e2` plus the existing uncommitted CBS and diagnostic
  work
- Smoother: `cbs::IncrementalFixedLagBpsamSmoother`
- Internal solver: `cbs::BPSAM`, which directly derives from `gtsam::ISAM2`
- CBS bridge/backend: disabled in the controls
- GTSAM commit: `d9cde78de` (`4.3.0`)

## Fixed-Lag Code Parity

The production wrapper preserves the native GTSAM fixed-lag sequence:

1. update the key/timestamp maps;
2. find keys older than the lag;
3. create ordering constraints;
4. gather additional keys to re-eliminate;
5. call the iSAM2 update;
6. remove timestamps for unused keys;
7. call `marginalizeLeaves()`; and
8. erase marginalized timestamps.

Relevant code:

- Native GTSAM update:
  `src/gtsam/gtsam/nonlinear/IncrementalFixedLagSmoother.cpp`
- Production wrapper:
  `src/cbs/include/cbs/bpsam/incremental_fixed_lag_bpsam_smoother.h`

The complete `gtsam::ISAM2Params` object is copied into
`BPSAM::Params::sam_params_`. `BPSAM` constructs its `gtsam::ISAM2` base from
that object. No solver parameter was found to be dropped or replaced.

With CBS disabled, the BPSAM update adds only Kimera's local `newFactors`,
filters `newTheta` to genuinely new keys, and calls `gtsam::ISAM2::update()`.
The CBS pending and active-factor collections are empty.

## Principal Results

### Clean native GTSAM 4.3 Gauss-Newton

Run:

```text
clean_kimera_ws/runs/20260727_m3dgr_outdoor01_stock_ifls_gtsam43_gn_n50_age16_full_objective_30s_rate05
```

Across 149 updates:

- 89 accepted updates increased the complete measured objective;
- no update had mean active-window movement above 0.5 m;
- the largest mean movement was 0.460408 m at frame 137; and
- frame 148 moved all 50 poses backward by 0.353008 m on average while the
  objective increased from 588.603 to 1173.356, a ratio of 1.993.

Therefore native GTSAM 4.3 Gauss-Newton can accept the same class of
objective-worsening, whole-window backward revision without CBS or BPSAM.

### Production BPSAM with CBS disabled

Run:

```text
runs/20260727_m3dgr_outdoor01_production_bpsam_gtsam43_gn_n50_age16_cbs_off_objective_30s_rate05
```

Across 149 updates:

- 109 accepted updates increased the complete measured objective;
- frame 113 moved the active window by 1.029933 m on average and 2.316544 m
  at the maximum;
- the objective increased from 268.494419 to 399.112759, a ratio of 1.486;
- IMU error increased by 149.766349;
- SmartStereo error decreased by only 20.510217; and
- the evaluated marginal-prior error increased by 1.070193.

The frame-113 accepted state is therefore worse than the actual pre-update
state. The move is not justified by the complete local objective and is not a
physical backward-motion estimate.

### Diagnostic observer control

Diagnostic-enabled run:

```text
runs/20260727_m3dgr_outdoor01_production_bpsam_gtsam43_gn_n50_age16_cbs_off_movement_only_30s_rate05
```

Diagnostic-disabled run:

```text
runs/20260727-225954_m3dgr_outdoor01_production_bpsam_gtsam43_gn_n50_age16_cbs_off_diagoff_30s_rate05
```

The diagnostic-disabled profile adds only:

```text
--cbs_active_factor_diagnostic_enable=false
```

Both runs produced 149 `KIMERA_BACKEND_POSE_PATH_ROW` records. After removing
the normal logging prefixes, all 149 records are byte-for-byte identical.
Frame 113 has the same accepted pose in both runs:

```text
optimized state position:      [0.420849, 6.648000, 0.153187] m
increment-only prediction:     [0.457051, 9.794890, 0.133531] m
state-to-prediction distance:  3.147160 m
```

The pre/post `calculateEstimate()` diagnostic observer does not cause or
perturb the failure.

## What Is Ruled Out

The current evidence rejects these explanations:

- Rerun camera movement or visualization recentering;
- an external GLIM factor pulling Kimera backward;
- CBS factor lifecycle or covariance scaling;
- the fixed-lag diagnostic observer;
- Dogleg as the cause, because the class also occurs with Gauss-Newton;
- GTSAM 4.2 as the sole cause, because it occurs with GTSAM 4.3;
- an unbounded SmartStereo track, because `maxFeatureAge` is below the
  50-state lag in these controls; and
- a direct Schur-complement/covariance-copy failure during
  `marginalizeLeaves()`, because the retained marginal and joint covariance
  audits are preserved at logged precision.

## Current Conclusion

The backward revision is a real accepted-update pathology. It is not evidence,
by itself, that the custom CBS marginalization implementation is corrupt.

The narrowest supported diagnosis is:

1. The incremental fixed-lag state has a stale nonlinear linearization
   structure, including linear marginal information.
2. A one-pass undamped Gauss-Newton iSAM2 update can produce a candidate that
   is worse than the actual estimate immediately before the update.
3. iSAM2 has no general nonlinear objective acceptance guard for this
   Gauss-Newton step.
4. Native GTSAM 4.3 reproduces the class of failure.
5. The production BPSAM trajectory reaches a larger failure earlier, but this
   audit has not identified a concrete unconditional BPSAM code defect that
   explains that amplification.

Calling this a proven "marginalization bug" would overstate the evidence.
Calling the accepted frame-113 update valid optimizer behavior would also be
wrong: it substantially increases the complete represented objective.

## Safest Next Experiment

Use the smoother backup already created by
`VioBackend::updateSmoother()` to add a default-off shadow acceptance audit:

1. Evaluate the complete post-update graph on the actual pre-update estimate,
   including new factors and initialized new values.
2. Evaluate the same graph at the accepted post-update estimate.
3. When the accepted objective is worse, restore a copy of the pre-update
   smoother and retry with `force_relinearize=true` and
   `forceFullSolve=true`, while retaining Gauss-Newton.
4. Log the normal candidate and retry candidate without changing the estimator
   output.
5. Promote the retry to a configurable rollback guard only if the shadow
   experiment consistently reduces the complete objective and removes the
   large backward revisions.

The objective evaluator must handle `LinearContainerFactor` in delta
coordinates. Its ordinary `error(Values)` call returns zero and is not a valid
measurement of the fixed-lag marginal prior.

This experiment leaves marginalization and SmartStereo behavior unchanged and
isolates update acceptance before any production policy is modified.
