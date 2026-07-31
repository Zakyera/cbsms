# Kimera Fixed-Lag Marginalization Audit

Date: 2026-07-20

## Question

The Rerun factor-graph inspector showed active Kimera poses revising backward
around `x132` through `x140`, even though the chained output trajectory continued
forward. The audit asks whether CBS changes, stale visual factors, or an invalid
fixed-lag marginal prior caused that behavior.

## Runs

- Covariance and one-state-retained reference audit:
  `runs/20260720-190828_m3dgr_outdoor01_kimera_fixed_lag_shadow_mapped_audit_30s_rate05`
- Per-landmark SmartStereo separator audit:
  `runs/20260720-193035_m3dgr_outdoor01_kimera_smart_separator_audit_30s_rate05`

Both runs use M3DGR Outdoor01, pure Kimera, CBS bridge disabled, a 30 s bag
window, and bag rate 0.5.

## Results

### 1. Marginalization preserves the retained Gaussian covariance

Across 124 marginalizations, the relative matrix difference before versus after
`marginalizeLeaves()` was zero at logged precision for:

- the oldest retained pose marginal covariance;
- the newest retained pose marginal covariance; and
- the 12x12 joint covariance of those two retained poses.

The retained state estimate also moved only at numerical roundoff during
`marginalizeLeaves()` itself. This rejects a direct Schur-complement or prior
insertion algebra failure in the tested path.

### 2. The long SmartStereo factor is legitimate and is removed

At frame 131, landmark 8846 has one retained SmartStereo factor with slot 15404
and keys:

```text
x105|x106|x107|x108|x109|x110|x111|x112|x113|x114|x115|x116|x117|
x118|x119|x120|x121|x122|x123|x124|x125|x126|x127|x128|x129|x130
```

The factor touches the state being marginalized, `x105`, and every other pose
in the active window. It is absent from subsequent separator rows after the
frame-131 marginalization. This rejects the hypothesis that this factor remains
in the graph with a stale out-of-window key.

### 3. The factor creates a full-window separator

The marginal prior key sets change as follows:

```text
frame 130: b105|v105|x105...x123       (21 keys)
frame 131: b106|v106|x106...x130       (27 keys)
frame 132: b107|v107|x107...x130       (26 keys)
frame 133: b108|v108|x108...x130       (25 keys)
frame 134: b109|v109|x109...x133       (27 keys)
```

After frame 131, the fixed-variable set contains 25 pose keys, `x106...x130`.
This is the expected separator produced when `x105` is eliminated from a factor
that also contains `x106...x130`.

### 4. The dense-key prior is low rank but fixes every key

The frame-131 prior has 27 keys and 159 scalar dimensions, but its audit at the
next update reports rank 33 and nullity 126. Despite that low rank, GTSAM adds
every key appearing in the marginal factor to `fixedVariables_`.

This follows native GTSAM code in `ISAM2::marginalizeLeaves()`:

```cpp
nonlinearFactorsToAdd.emplace_shared<LinearContainerFactor>(factor);
for (Key factorKey : *factor) {
  fixedVariables_.insert(factorKey);
}
```

Upstream `IncrementalFixedLagSmoother` calls this same implementation. BPSAM
does not override `marginalizeLeaves()`; it inherits `gtsam::ISAM2`.

### 5. The next nonlinear update diverges from the retained-state reference

At frame 132:

```text
normal fixed-lag pose movement mean:       0.02356 m
normal newest retained x131 movement:      0.10252 m
one-state-retained reference mean:         0.00342 m
reference x131 movement:                   0.00451 m
normal-vs-reference mean difference:       0.02690 m
normal-vs-reference maximum at x132:       0.12030 m
```

At frame 135, the normal-versus-reference maximum reaches 0.16431 m.

The reference is an audit comparator, not a batch ground truth: it clones the
pre-marginalization native `gtsam::ISAM2`, retains one state that the normal
fixed-lag smoother removes, and receives the same next-cycle factors and factor
removals. It demonstrates sensitivity to the marginalization boundary and
linearization policy.

### 6. Lag-only ablation materially changes the estimator result

Run:
`runs/20260720-195051_m3dgr_outdoor01_kimera_fixed_lag100_ablation_30s_rate05`

The `M3DGRMonoOriginalLag100` run-time parameter folder is byte-for-byte the
same as `M3DGRMonoOriginal`, except for `nr_states: 100` instead of 25.

```text
                                      lag 25       lag 100
Kimera APE RMSE                       2.7669 m      0.4640 m
Kimera RPE RMSE                       0.1597 m      0.2040 m
estimated path length                 7.4972 m     17.0335 m
ground-truth path length             16.6660 m     16.6660 m
```

For lag 100 around frames 129 through 136, normal fixed-lag versus the
one-state-retained reference agrees to approximately `1e-8` m or better. The
large 12-16 cm boundary-induced disagreements seen with lag 25 disappear.

Increasing the lag does not eliminate all nonlinear active-state revisions. For
example, the lag-100 graph still performs large corrections at some updates.
Those corrections are not, by themselves, evidence of a marginalization bug.
The controlled result is that the severe trajectory under-travel and the
normal-versus-reference boundary discrepancy are strongly dependent on the
25-state marginalization structure.

## Conclusion

No tested evidence supports a malformed Schur prior, covariance corruption, or
stale SmartStereo factor. The measured mechanism is:

1. A SmartStereo track spans the full fixed-lag pose window.
2. Marginalizing its oldest pose creates a marginal factor whose separator
   contains almost every active pose.
3. Native GTSAM fixes every separator key because the marginal factor is
   linear and cannot be relinearized.
4. The next nonlinear update can only absorb much of its correction near the
   newest, non-fixed states, producing the observed local backward revision.

This is a real fixed-lag nonlinear approximation problem for this graph
structure. It is not currently shown to be a CBS-specific or custom BPSAM bug.

The lag-only result also shows that this approximation is not merely cosmetic:
on the tested 30 s Outdoor01 segment it changes APE by more than 2 m and reduces
the lag-25 estimated path to less than half the ground-truth path length.

## Next Decisive Test

Add a default-off diagnostic ablation that prevents SmartStereo factors touching
the marginalization boundary from spanning the full active window. Compare the
same frames with:

- default factor lifecycle;
- boundary-crossing long SmartStereo factors omitted for one controlled run;
- a principled bounded-track replacement if the omission removes the jump.

The first ablation isolates causality. It is not proposed as the final estimator
policy because dropping a whole visual factor also discards valid information.

## 2026-07-27 N=50 Solver and Gauge Audit

The stronger back-and-forth motion observed in the saved Rerun recording was
retested on the first 60 s of Outdoor01 with a 50-state Kimera window. All
controls below processed the same 294 keyframe timestamps.

### Runs

- One-pass Gauss-Newton with detailed marginalization diagnostics:
  `runs/20260727-142505_m3dgr_outdoor01_kimera_n50_gauge_curvature_audit_60s_rate05_valid`
- Five-pass Gauss-Newton with lightweight transition diagnostics:
  `runs/20260727-145022_m3dgr_outdoor01_kimera_n50_numopt5_basic_audit_60s_rate025_exact`
- One-pass Dogleg with lightweight transition diagnostics:
  `runs/20260727-145911_m3dgr_outdoor01_kimera_n50_dogleg1_basic_audit_60s_rate025_exact`

The two diagnostic runtime configurations are:

- `runs/runtime_configs/M3DGRMonoOriginalN50NumOptimize5GaugeAudit`
- `runs/runtime_configs/M3DGRMonoOriginalN50DoglegAudit`

They are controls and are not the current production defaults.

### State 198 isolates a marginalization-boundary failure

At state 198, the one-pass Gauss-Newton update moved the 50 retained poses by:

```text
mean translation:                 3.241991 m
maximum translation:              5.077365 m
common world translation:         0.980332 m
mean residual after rigid motion:  2.261575 m
maximum residual:                  4.097530 m
mean rotation:                     0.503689 deg
maximum rotation:                  0.560117 deg
```

The corresponding unmarginalized shadow update moved by only 0.1353 m on
average. The fixed-lag result differed from that shadow by 3.4122 m on average.
The immediately preceding marginalization therefore changes the next solve
materially.

The marginal prior entering state 198 has:

```text
keys:                 23
pose keys:            21, x147 through x167
scalar dimension:     135
rank:                 25
nullity:              110
positive condition:   3.91e8
Gaussian error:       170.2316 before, 171.9492 after
```

The nonlinear factor errors for the same solve are:

```text
existing factor category        before       after
IMU                             503.0718    13597.7124
SmartStereo                     181.6370      230.0715
bias random walk                  5.3094       11.2436
```

The update is not descending the true nonlinear objective. In particular, the
IMU error increases by approximately 27 times in one solve.

### Marginalization construction still passes the direct checks

At states 197 and 198:

- `marginalizeLeaves()` changes retained state estimates only at numerical
  roundoff;
- the first retained pose marginal covariance is unchanged;
- the latest retained pose marginal covariance is unchanged; and
- their full 12x12 joint covariance is unchanged.

The logged relative matrix differences are exactly zero at output precision.
This rejects a key-ordering, Schur-complement, or covariance-copy error in the
tested marginalization operation.

The `LinearContainerFactor` created by native
`gtsam::ISAM2::marginalizeLeaves()` has no stored nonlinear linearization
point. Consequently its standard `NonlinearFactor::error(values)` is always
zero. The meaningful prior error in this audit is computed directly from its
stored Gaussian factor and the current iSAM2 delta.

### Global gauge weakness is present but is not the full jump

At state 198, all common translation and yaw curvature comes from the marginal
prior. IMU and SmartStereo factors are relative and contribute approximately
zero curvature to a common world transform. The weakest translation direction
has:

```text
curvature:              4.6217
equivalent 1D sigma:    0.4652 m
```

The gradient and curvature predict only about 0.0327 m of common translation
for that solve. The measured update contains 0.9803 m of common translation
plus 2.2616 m mean non-rigid deformation. Gauge weakness therefore contributes,
but it cannot explain the full 3.24 m movement.

### Solver controls

```text
solver                 APE RMSE   RPE RMSE   max mean window shift   updates >1 m
Gauss-Newton, 1 pass    3.1972 m   0.2694 m          3.2420 m             7
Gauss-Newton, 5 passes  2.2756 m   0.2640 m          4.3457 m*           18*
Dogleg, 1 pass          1.5053 m   0.2549 m          2.7117 m             6
```

`*` For the five-pass run, this is the largest individual pass and the count is
over all 1470 individual solves. Its first pass alone has a 2.6616 m maximum
mean shift and five updates above 1 m.

At state 198 specifically:

```text
Gauss-Newton, 1 pass:   3.2420 m
Gauss-Newton, 5 passes: 0.0984, 0.0190, 0.0199, 0.0826, 0.0017 m
Dogleg, 1 pass:         0.0125 m
```

Both repeated optimization and Dogleg suppress the original state-198 event,
and Dogleg materially improves APE. Neither removes all large active-window
revisions. This supports an unstable/stale nonlinear fixed-lag linearization
diagnosis, while rejecting the simpler claim that one malformed prior causes
every visible jump.

### SmartStereo span correlation

SmartStereo tracks do create wide separators. At state 197, the new prior
contains 21 consecutive pose keys, which cannot be produced by the adjacent IMU
chain alone. However, instantaneous shift magnitude is not positively
correlated with visual track span in this run:

```text
Pearson correlation with mean pose shift
live SmartStereo count:            +0.104
weighted mean pose-key span:       -0.194
tracks spanning at least 20 poses: -0.220
maximum pose-key span:             -0.458
```

Track span explains why many active poses become fixed linearization variables.
It does not by itself predict which update will jump. The immediate trigger is
the nonlinear balance among the stale marginal linearization, IMU chain, and
current SmartStereo factors.

### Updated conclusion

The observed backward motion is present in Kimera's optimized active state; it
is not generated by Rerun recentering. The available evidence supports this
mechanism:

1. Long SmartStereo tracks create a marginal separator over many active poses.
2. Native iSAM2 marks every separator key fixed, so those pose linearization
   points cannot move while the linear marginal prior remains active.
3. New IMU and visual factors continue to change the nonlinear optimum.
4. An undamped incremental step can then move the free tail by metres and even
   increase the nonlinear objective.
5. Dogleg limits some failures and improves accuracy, but other large
   corrections remain.

The next implementation experiment should target the wide, fixed SmartStereo
separator policy. Solver damping is useful as a guard, but it does not remove
the structural source of stale fixed variables.

## 2026-07-27 Upstream Stock Kimera Control

The active-window motion was reproduced in the separate clean Kimera workspace:

```text
clean_kimera_ws/runs/20260727-174002_m3dgr_outdoor01_stock_ifls_n50_factor_graph_60s_rate05
```

This control used:

- upstream Kimera-VIO commit `ce8c59b7`;
- Kimera's native `gtsam::IncrementalFixedLagSmoother`;
- CBS disabled;
- Outdoor01 for 60 dataset seconds at bag rate 0.5;
- `nr_states: 50`; and
- a read-only ROS-side backend-output observer.

The clean Kimera source changes required to build against the installed GTSAM
are type/include compatibility changes. The observer is in Kimera-VIO-ROS and
does not replace or modify the fixed-lag smoother. Its reported APE and RPE are
1.8974 m and 0.2552 m, versus 1.9137 m and 0.2576 m in the preceding unobserved
stock control.

The observer compares each active pose key with the optimized value of the same
key in the preceding backend output. Across 294 updates:

```text
mean movement per common active pose
  p50: 0.0395 m
  p95: 0.7751 m
  max: 2.1702 m

maximum individual active-pose movement
  p50: 0.0904 m
  p95: 1.4031 m
  max: 2.5071 m

updates with maximum movement > 0.5 m: 48
updates with maximum movement > 1.0 m: 20
updates with mean window movement > 0.5 m: 22
clearly backward updates: 95
clearly forward updates: 93
non-neutral direction sign flips: 95
```

Adjacent updates contain unambiguous whole-window reversals:

```text
frame 138: all 50 common poses backward, mean 1.1044 m
frame 139: all 50 common poses forward,  mean 1.1467 m

frame 182: all 50 common poses backward, mean 1.2213 m
frame 183: all 50 common poses forward,  mean 1.1029 m

frame 272: all 50 common poses backward, mean 1.9510 m
frame 273: all 50 common poses forward,  mean 2.1702 m
```

The mature stock graph contains approximately 51 pose keys, 50 IMU factors, 50
bias random-walk factors, one marginal factor, and a median of 2190 live
SmartStereo factors.

The complete generated assessment and per-update CSV are:

```text
clean_kimera_ws/runs/20260727-174002_m3dgr_outdoor01_stock_ifls_n50_factor_graph_60s_rate05/stock_ifls_factor_graph_assessment.md
clean_kimera_ws/runs/20260727-174002_m3dgr_outdoor01_stock_ifls_n50_factor_graph_60s_rate05/parsed/stock_ifls_active_window_revisions.csv
```

### Control conclusion

The back-and-forth active-window behavior is present in upstream stock Kimera
with CBS disabled. It is therefore not, by itself, evidence of a CBS/BPSAM
marginalization bug. The custom smoother can still change or amplify the
behavior, and the 3.242 m state-198 custom event remains larger than most stock
updates. The baseline mechanism must now be investigated as native
Kimera/GTSAM fixed-lag nonlinear behavior under the Outdoor01 SmartStereo, IMU,
and marginal-prior graph.

The published latest-pose trajectory can remain smooth while this happens. It
records each newest pose once, whereas the active-window audit repeatedly
observes older pose keys while they are being reoptimized.

## 2026-07-27 Stock N=50 Feature-Age Sweep

The supervisor's feature-lifetime hypothesis was tested directly on stock
Kimera with CBS disabled. Four matched Outdoor01 runs used `nr_states: 50` and
`maxFeatureAge` values 8, 12, 16, and 25.

Kimera drops a track only when `landmark_age > maxFeatureAge`. The observed
maximum SmartStereo pose-key counts were therefore exactly one greater than the
configured ages:

```text
maxFeatureAge                    8     12     16     25
maximum SmartStereo pose keys    9     13     17     26
maximum marginal separator       8     12     16     25
```

This confirms that feature age directly controls SmartStereo and marginal
separator width. It also confirms that the original N25/age25 profile permits
a visual factor to cover effectively the complete active window.

Shorter tracks did not eliminate active-window reversals:

```text
maxFeatureAge                    8       12       16       25
maximum mean window shift m      2.949   2.079    2.208    4.699
updates with mean shift >0.5 m  15      19       14       20
APE RMSE m                       3.272   1.672    2.931    2.205
```

Frame 246 exceeded 0.5 m mean movement in every run. At age 8, all 50 common
poses moved backward by 1.180 m on average while the current marginal factor
referenced only three pose keys. A full-width separator is therefore not
necessary for the failure.

The corrected diagnosis is:

1. Feature lifetime is bounded; tracks are not unbounded.
2. Equal feature age and window length is a real N25 configuration problem.
3. Wide visual separators can amplify stale-linearization behavior.
4. Separator width alone does not cause every reversal.
5. The common frame-246 event must be audited at the optimizer/objective level.

The full comparison is:

```text
clean_kimera_ws/runs/20260727_m3dgr_outdoor01_stock_ifls_n50_max_feature_age_sweep/report.md
```
