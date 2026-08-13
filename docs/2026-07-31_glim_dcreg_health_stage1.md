# GLIM DCReg observability-health sidecar: Stage 1

Date: 2026-07-31

## Scope

Stage 1 is a passive GLIM LiDAR-registration sidecar. It implements only:

- `off`: no monitor object, Hessian capture, Schur analysis, reference
  management, eigendecomposition, or structured health logging.
- `log_only`: calculate and log diagnostics without returning any
  estimator-control output.

It does not modify the scan optimizer, VGICP residuals, the smoother, poses,
CBS messages, covariance, Kimera, or a solver. It does not implement
`metadata`, `covariance_inflation`, or `solver_mitigation`.

The measured quantity is **relative directional LiDAR observability health**.
It is dimensionless. It is not expected metric error, probability of
correctness, calibrated covariance, or a ratio-to-centimetre model.

## Hessian capture and conventions

`OdometryEstimationCPU::create_factors()` constructs the scan-only
`matching_cost_factors`. For the MID360 VGICP configuration these are two
unary `IntegratedVGICPFactor` instances on `X(current)`, corresponding to the
two target voxel-map resolutions.

The sidecar captures each factor only after
`LevenbergMarquardtOptimizerExt::optimize()` returns and after the final
matching error is evaluated with the returned `values`. Each factor is
linearized separately at that final optimized scan pose. The extraction
accepts only a unary factor whose only key is `X(current)` and whose linearized
Hessian is exactly 6x6. Every factor Hessian is symmetrized before use.

The principal scan object is

```text
H_lidar = H_VGICP_level_0 + H_VGICP_level_1
```

Both terms use GTSAM `Pose3` local coordinates ordered
`[rx, ry, rz, tx, ty, tz]` and the same local right-perturbation tangent frame.
Only `matching_cost_factors` contribute. IMU, priors, marginal factors, CBS
factors, velocity, and bias are excluded.

Per-resolution information and aggregate information are both logged.
Source points are the one current preprocessed scan. Inlier counts from the
two resolutions are not added as though they were independent points. The
aggregate uses the minimum per-factor inlier count and fraction only as
conservative reference gates. Aggregate `trace_per_source_point` divides by
the unique source-scan point count; aggregate `trace_per_inlier` is unavailable
because no mathematically valid independent aggregate inlier count exists.

## Schur observability analysis

For a finite 6x6 LiDAR Hessian, first symmetrize it and partition it in
rotation-then-translation order:

```text
    [ H_RR  H_Rt ]
H = [             ]
    [ H_tR  H_tt ]
```

The complementary Schur matrices are

```text
S_R = H_RR - H_Rt H_tt^+ H_tR
S_t = H_tt - H_tR H_RR^+ H_Rt
```

The pseudoinverses use self-adjoint eigendecomposition and discard directions
below the configured relative threshold. Both Schur matrices are symmetrized
again. Small negative eigenvalues within the configured numerical tolerance
are treated as roundoff; genuinely indefinite Schur matrices are invalid and
remain passive.

For each rotational or translational eigensystem:

```text
S = V diag(lambda) V^T

r_i = lambda_max /
      max(lambda_i, epsilon_absolute + epsilon_relative * lambda_max)
```

`r_i > degeneracy_condition_threshold` sets the independent absolute DCReg
mask. This absolute mask is distinct from relative health.

Raw eigenvalues and eigenvectors are retained. Aligned modes are the same
orthonormal eigenvectors, only permuted by maximum absolute inner product and
sign-corrected against the previous stable basis (or physical XYZ basis for
the first sample). Squared components produce physical-axis contribution
ratios. The log records the original raw index, alignment confidence, and
spectral-cluster flags. A clustered or weakly aligned mode remains a mixed
direction; it is not labelled as a pure roll, pitch, yaw, X, Y, or Z mode.

## Healthy reference

Reference sources are `session`, `offline`, and `hybrid`. In `hybrid`, an
accepted offline profile is used; otherwise the monitor falls back to the
session bootstrap.

A session sample must pass all enabled gates:

- monitor, Hessian, and spectra valid;
- GLIM initialized;
- registration convergence observed through the existing pose-increment
  termination test;
- final linear solve successful;
- minimum source-point count;
- minimum inlier count and inlier fraction for every resolution;
- finite initial and final matching costs;
- optional absolute cost and relative cost-reduction gates;
- every rotational condition ratio at or below the rotational bootstrap
  safety limit;
- every translational condition ratio at or below the translational bootstrap
  safety limit;
- no mode selected by the absolute DCReg degeneracy mask;
- adequate mode-to-basis alignment, if required;
- no clustered rotational or translational mode, if configured;
- a stable rolling window of gate-passing candidates whose per-mode log-ratio
  range is within the configured limit. Rejected scans are never inserted,
  but an intermittent rejected scan does not erase prior accepted evidence.

The first scan is never accepted on its own. After the required accepted
samples, each reference component is the median of the accepted log condition
ratios. Until then `reference_ready=false`, `health_available=false`, and the
numeric relative-health fields are NaN. Rejected samples are never used in a
fallback median and cannot initialize or adapt the reference.

Bootstrap rejection reasons distinguish absolute rotational degeneracy,
absolute translational degeneracy, incompatibility with an offline nominal
envelope, insufficient support, unstable condition history, unconverged
registration, and numerical failure.

Adaptation uses a slow exponential update in log-ratio space only after the
same gates and stable-window test pass. It is frozen for a relative degraded
state, for health below `1 / maximum_reference_change_ratio`, and for abrupt
ratio changes. The adapted reference is bounded around its initial
session/offline anchor. It is not the best-ever observation and prolonged
degradation cannot redefine itself as normal.

An offline profile is JSON:

```json
{
  "schema_version": 2,
  "metadata": {
    "sensor_identifier": "livox_mid360",
    "registration_type": "VGICP",
    "voxel_resolutions": [0.5, 1.0],
    "pose_ordering": "rotation_translation",
    "tangent_convention": "local_right",
    "configuration_fingerprint": "required exact fingerprint"
  },
  "rotation_log_condition_ratios": [0.0, 0.0, 0.0],
  "translation_log_condition_ratios": [0.0, 0.0, 0.0],
  "rotation_log_condition_mad": [0.0, 0.0, 0.0],
  "translation_log_condition_mad": [0.0, 0.0, 0.0]
}
```

Generate a profile only from accepted healthy sessions with matching sensor
and GLIM registration settings: select aggregate CSV rows that passed the
reference gates, align their modes consistently, and store the per-mode median
of `log(condition_ratio)` and its per-mode median absolute deviation (MAD).
The profile is optional at runtime. Incompatible metadata is rejected;
`hybrid` falls back to session and `offline` remains unready. Hybrid
adaptation accepts only absolutely non-degenerate candidates inside the
configured MAD envelope and remains bounded to that envelope.

## Relative health and temporal state

Once the reference exists:

```text
d_i(t) = max(0, log(r_i(t)) - log(r_i,reference))
h_i,raw(t) = exp(-d_i(t))
           = clip(r_i,reference / r_i(t), 0, 1)
```

Thus health 1 means the current directional condition is no worse than the
accepted reference, 0.5 means the condition ratio is twice as poor, and 0.1
means ten times as poor. No metric pose-error claim follows from these values.

The Outdoor01 health minimum of 0.321 reported during the initial Stage 1
experiment is not scientifically interpretable. That run used a permissive
bootstrap limit and learned an already-degenerate opening segment (median
maximum rotation condition 28.6 and translation condition 83.1) as its
reference. Stage 1.6 rejects those samples at the default absolute threshold
10 and leaves relative health unavailable unless genuinely healthy evidence
or a compatible offline profile exists.

Temporal smoothing operates on log deterioration:

```text
d_bar_i(t) = (1 - alpha) d_bar_i(t-1) + alpha d_i(t)
h_i,smoothed(t) = exp(-d_bar_i(t))
```

A mode enters the relative degraded state only after `bad_frames_required`
accepted frames below `health_enter_threshold`. It leaves only after
`good_frames_required` accepted frames above the larger
`health_exit_threshold`. Raw health, smoothed health, absolute masks, relative
masks, and counters remain separate.

The default invalid-frame policy is `hold`. Other supported policies are
`decay_toward_unknown` and `mark_unavailable`. Invalid frames never update the
reference.

## Configuration

The optional section is under `odometry_estimation`:

```json
"dcreg_health": {
  "mode": "off",
  "detection": {
    "degeneracy_condition_threshold": 10.0,
    "epsilon_absolute": 1e-12,
    "epsilon_relative": 1e-9,
    "pseudoinverse_relative_threshold": 1e-8,
    "negative_eigenvalue_tolerance": 1e-9,
    "spectral_cluster_relative_gap": 0.05,
    "minimum_axis_alignment_confidence": 0.70
  },
  "reference": {
    "source": "hybrid",
    "sensor_identifier": "",
    "offline_profile_path": "",
    "require_profile_metadata_match": true,
    "bootstrap_minimum_samples": 30,
    "bootstrap_window_size": 10,
    "bootstrap_max_rotation_condition_ratio": 10.0,
    "bootstrap_max_translation_condition_ratio": 10.0,
    "temporal_stability_max_log_ratio_range": 0.50,
    "adaptation_enabled": true,
    "adaptation_rate": 0.01,
    "freeze_during_degradation": true,
    "maximum_reference_change_ratio": 2.0,
    "offline_nominal_mad_multiplier": 3.0,
    "offline_nominal_min_log_half_width": 0.05,
    "require_glim_initialized": true,
    "require_registration_converged": true,
    "require_linear_solve_success": true,
    "minimum_source_point_count": 100,
    "minimum_inlier_count": 100,
    "minimum_inlier_fraction": 0.05,
    "maximum_initial_cost": -1.0,
    "maximum_final_cost": -1.0,
    "minimum_relative_cost_reduction": -1.0,
    "require_axis_alignment_confidence": true,
    "reject_clustered_modes": true
  },
  "temporal": {
    "enabled": true,
    "smoothing_alpha": 0.10,
    "health_enter_threshold": 0.25,
    "health_exit_threshold": 0.50,
    "bad_frames_required": 3,
    "good_frames_required": 5,
    "invalid_frame_policy": "hold"
  },
  "support": {
    "combine_with_shape_health": false
  },
  "logging": {
    "enabled": false,
    "csv_path": "",
    "log_every_n_frames": 1,
    "asynchronous": true,
    "queue_capacity": 256,
    "flush_every_n_rows": 10
  }
}
```

If the section or its `mode` is absent, the mode is `off`. Invalid values log
a startup error, disable only the sidecar, and leave odometry running.
`combine_with_shape_health=true` and synchronous Stage 1 logging are rejected.
Negative cost thresholds disable those gates.

## Migration from the prototype

The experimental
`runs/runtime_configs/config_m3dgr_mid360_native_imu_clean` profile explicitly
sets `dcreg_health.mode=log_only` and removes the old
`scan_dcreg_diagnostics_enable` prototype flag. The legacy flag is ignored
with a compatibility warning if another private profile still contains it.
No old and new detector can run simultaneously. Profiles without the new
section remain `off`.

## Structured logging and reporting

The asynchronous bounded logger writes one aggregate row and one row per
matching factor to `glim_dcreg_health.csv`. Passive Hessian capture and health
analysis run in the odometry path; publication of the completed diagnostic
uses a non-blocking bounded enqueue. CSV formatting and file I/O happen on the
logger worker. A full queue or contended queue lock drops the diagnostic
sample rather than blocking odometry. With logging disabled, no row is
formatted or written. This is the frozen Stage 1 architecture.

Schema version 1 contains:

- configuration fingerprint, registration type, ordering, and tangent
  convention;
- timestamp, frame, aggregate/factor marker, factor index, and resolution;
- validity, factorization status, convergence, and linear-solve status;
- per-factor support and cost values;
- aggregate and per-factor Hessian rank, condition, scale, and normalized
  traces;
- raw/aligned Schur spectra and bases;
- condition and reference ratios;
- raw/smoothed health;
- absolute and relative masks plus hysteresis counters;
- original aligned-mode indices, contribution ratios, alignment confidence,
  and cluster flags;
- reference update decision/reason, invalid reason, and monitor runtime.

`cbsms_experiment.py report` consumes this dedicated file directly and never
reconstructs health from console text. It summarizes counts, lower-tail health,
runtime, reference decisions, and per-resolution support.

Useful plots are:

- current and reference condition ratios on a log scale;
- raw and smoothed rotational/translational health;
- absolute DCReg masks and relative degraded masks;
- inlier count/fraction and cost by VGICP resolution;
- aggregate Hessian trace per source point;
- reference update decisions across healthy geometry, corridor entry, and
  recovery.

The GLIM Rerun factor-graph inspector exposes these same live diagnostics
under `glim/dcreg_health`.

## Why excluded objects remain excluded

The DCReg preconditioner is an optimization aid, not a covariance or
statistical uncertainty model, so it is not used here. The complete GLIM
smoother Hessian mixes LiDAR geometry with IMU, priors, marginalization, and
other factors that could conceal LiDAR-only degeneracy, so the sidecar uses
only the final scan-registration factors.

No Stage 1 path writes health into CBS/ROS interfaces and no covariance policy
is compiled into this monitor.
