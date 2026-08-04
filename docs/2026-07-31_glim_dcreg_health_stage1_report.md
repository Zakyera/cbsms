# GLIM DCReg observability-health sidecar: Stage 1 report

Date: 2026-07-31

Status: **implementation and unit validation complete; baseline regression
acceptance blocked by the explicit Stage 1 stop condition.** No Stage 2 work
was performed.

The focused Stage 1.6 follow-up supersedes the regression and reference
conclusions in Sections 9--12:

```text
stage16_diagnosis/STAGE16_REGRESSION_AND_REFERENCE_REPORT.md
```

The design and configuration reference is:

```text
src/cbsms/docs/2026-07-31_glim_dcreg_health_stage1.md
```

## 1. Git state

Before edits:

| repository | branch | HEAD | pre-existing status |
|---|---|---|---|
| cbsms | `cbsms/gtsam-4.3-develop` | `075e2aa3dafdc3b6c73e5fadb774dc21b87db358` | `?? tools/__pycache__/` |
| glim | `cbs-gtsam43-noetic` | `b1821311e3db65c5f68f3867f1019ff648f1c05b` | clean |
| glim_ros1 | `cbs-gtsam43-noetic` | `5a2b38e92e1a54e86a3d0e2e2c1d0ec74ef69e42` | clean |
| cbs | `cbsms/gtsam-4.3-develop` | `994b1d6a5c05fb38dd1b0731c6430ec11d2f1ff0` | clean |
| Kimera-VIO | `cbsms/gtsam-4.3-develop` | `d0b2a31adf17ced8995994373f6b286679e26799` | clean |
| Kimera-VIO-ROS | `cbsms/gtsam-4.3-develop` | `09d7e17b5d27f97f622f285f0f23a61e6748278d` | `?? scripts/__pycache__/` |
| gtsam_points | `cbs-gtsam43-noetic` | `068980ce5a3afb1fa12300d2fbc13d26172f093c` | clean |

After edits, branches and HEADs are unchanged. The two pre-existing
`__pycache__` directories remain untouched.

Modified tracked files:

```text
src/glim/include/glim/odometry/callbacks.hpp
src/glim/include/glim/odometry/odometry_estimation_cpu.hpp
src/glim/include/glim/odometry/scan_dcreg_diagnostics.hpp
src/glim/src/glim/odometry/odometry_estimation_cpu.cpp
src/glim/src/glim/odometry/scan_dcreg_diagnostics.cpp
src/glim/src/test/scan_dcreg_diagnostics_test.cpp
src/glim_ros1/src/glim_ros/glim_factor_graph_inspector.cpp
src/cbsms/tools/cbsms_experiment.py
src/cbsms/tools/send_raw_cbs_dashboard_blueprint.py
src/cbsms/tools/test_cbsms_experiment_runner.py
```

New tracked documents:

```text
src/cbsms/docs/2026-07-31_glim_dcreg_health_stage1.md
src/cbsms/docs/2026-07-31_glim_dcreg_health_stage1_report.md
```

The experiment-generated runtime profile under
`runs/runtime_configs/config_m3dgr_mid360_native_imu_clean` is not part of a
nested Git repository. It was migrated from the old prototype flags to
`dcreg_health.mode=log_only`.

No CBS, Kimera, public ROS message, VGICP residual, optimizer, smoother,
covariance, or solver file was changed.

## 2. Exact Hessian capture

Capture is in `OdometryEstimationCPU::create_factors()` after
`LevenbergMarquardtOptimizerExt::optimize()` returns. The returned final
`values` are used for:

1. final aggregate and per-factor matching cost;
2. separate linearization of each member of `matching_cost_factors`;
3. strict validation that each factor is unary and keyed only on
   `X(current)`;
4. extraction of the exact 6x6 Hessian block;
5. symmetrization.

For the tested two-level MID360 VGICP profile:

```text
H_lidar = sym(H_level_0) + sym(H_level_1)
```

The source is already 6x6; it is not a diagonal block selected from a
two-pose factor. Both unary factors use GTSAM `Pose3` local ordering
`[rx, ry, rz, tx, ty, tz]` in the same target-pose local,
right-perturbation tangent frame. The aggregate therefore requires no adjoint
transport.

Only `matching_cost_factors` are visited. The capture excludes IMU, priors,
marginal factors, CBS factors, velocity, and bias.

The same current scan supplies both resolutions. Per-factor support is
preserved, but inlier counts are not added. The aggregate trace is normalized
only by the unique source-scan point count. Aggregate trace per inlier is
unavailable.

## 3. Prototype migration and operating modes

The old `scan_dcreg_diagnostics_enable` path no longer executes. If the legacy
flag appears in an unmodified private profile, GLIM warns that it is ignored.
It cannot run concurrently with the new sidecar.

The new optional section defaults to `off` when absent.

- `off`: no monitor construction or new Hessian capture, eigendecomposition,
  reference management, or health logging.
- `log_only`: passive analysis, internal callback, Rerun diagnostics, and
  dedicated asynchronous CSV only.

The pre-existing generic `scan_health` computation is preserved independently
of the new mode.

No `metadata`, `covariance_inflation`, or `solver_mitigation` mode exists in
Stage 1. There is no estimator-control output in the monitor API.

## 4. Equations and interpretation

With `H` ordered rotation then translation:

```text
S_R = H_RR - H_Rt H_tt^+ H_tR
S_t = H_tt - H_tR H_RR^+ H_Rt
```

The pseudoinverse is thresholded by the configured relative threshold. Schur
matrices are symmetrized before self-adjoint eigendecomposition. Small
negative roundoff and true indefinite spectra have separate states.

For each eigenmode:

```text
r_i = lambda_max /
      max(lambda_i, epsilon_absolute + epsilon_relative * lambda_max)
```

The absolute mask is `r_i > degeneracy_condition_threshold`.

After mode permutation/sign alignment:

```text
d_i = max(0, log(r_i) - log(r_i_reference))
h_i_raw = exp(-d_i) = clip(r_i_reference / r_i, 0, 1)

d_bar_i(t) = (1-alpha) d_bar_i(t-1) + alpha d_i(t)
h_i_smoothed = exp(-d_bar_i)
```

Health is relative directional observability. It is not metric error,
probability, covariance, or a calibration to ground truth.

Raw and aligned spectra/bases, original indices, squared physical-axis
contributions, alignment confidence, and clustered-eigenspace flags remain
separate. A low-confidence or clustered mode is not labelled as a pure
physical axis.

## 5. Reference rules

Supported sources are `session`, `offline`, and `hybrid`.

Every session candidate must pass the enabled initialization, convergence,
linear-solve, finite-analysis, source-point, per-factor inlier, matching-cost,
cost-reduction, bootstrap-condition, alignment, cluster, and temporal
stability gates. Only gate-passing candidates enter the stability window.
Intermittently rejected scans are not learned and do not erase prior
gate-passing evidence.

The session reference is the component-wise median of log condition ratios
from the required accepted samples. The tested profile requires 30 samples.
Before that, health is unavailable.

Adaptation occurs slowly in log space only in accepted stable windows. It
freezes during relative degradation and low current health, rejects abrupt
changes, and is bounded around the initial anchor. It never uses best-ever
values.

Offline profiles are schema-versioned JSON and can require exact registration
type, voxel-resolution, pose-ordering, tangent-convention, and configuration
fingerprint compatibility. Hybrid mode falls back to session on a missing or
incompatible profile.

## 6. Configuration defaults

```text
mode                                      off

detection.degeneracy_condition_threshold  10
detection.epsilon_absolute                 1e-12
detection.epsilon_relative                 1e-9
detection.pseudoinverse_relative_threshold 1e-8
detection.negative_eigenvalue_tolerance    1e-9
detection.spectral_cluster_relative_gap    0.05
detection.minimum_axis_alignment_confidence 0.70

reference.source                           hybrid
reference.sensor_identifier                ""
reference.offline_profile_path             ""
reference.require_profile_metadata_match   true
reference.bootstrap_minimum_samples        30
reference.bootstrap_window_size            10
reference.bootstrap_max_rotation_condition_ratio 10
reference.bootstrap_max_translation_condition_ratio 10
reference.temporal_stability_max_log_ratio_range 0.50
reference.adaptation_enabled               true
reference.adaptation_rate                  0.01
reference.freeze_during_degradation        true
reference.maximum_reference_change_ratio   2
reference.offline_nominal_mad_multiplier   3
reference.offline_nominal_min_log_half_width 0.05
reference.require_glim_initialized          true
reference.require_registration_converged   true
reference.require_linear_solve_success     true
reference.minimum_source_point_count       100
reference.minimum_inlier_count             100
reference.minimum_inlier_fraction          0.05
reference.maximum_initial_cost             -1 (disabled)
reference.maximum_final_cost               -1 (disabled)
reference.minimum_relative_cost_reduction  -1 (disabled)
reference.require_axis_alignment_confidence true
reference.reject_clustered_modes           true

temporal.enabled                            true
temporal.smoothing_alpha                    0.10
temporal.health_enter_threshold             0.25
temporal.health_exit_threshold              0.50
temporal.bad_frames_required                3
temporal.good_frames_required               5
temporal.invalid_frame_policy               hold

support.combine_with_shape_health           false

logging.enabled                             false
logging.csv_path                            ""
logging.log_every_n_frames                  1
logging.asynchronous                        true
logging.queue_capacity                      256
logging.flush_every_n_rows                  10
```

The migrated experimental profile overrides `mode=log_only`,
`logging.enabled=true`, and writes:

```text
${CBSMS_RUN_DIR}/parsed/glim_dcreg_health.csv
```

Invalid configuration disables only the sidecar. Synchronous logging and
support/shape combination are rejected in Stage 1.

## 7. Structured log

CSV schema version 2 has 179 columns. It stores configuration identity,
sensor identity, active reference source, offline-profile status,
timestamp/frame/factor/resolution identity, validity, convergence, per-factor
support and cost, aggregate/per-factor Hessian scale/rank/condition,
raw/aligned spectra and bases, condition/reference ratios, raw/smoothed
health, absolute/relative masks, counters, contributions, confidence,
clusters, reference decision/reason, invalid reason, and evaluation time.

There is one aggregate row plus one row for each matching factor. Formatting
and file writes occur on a bounded asynchronous worker. The odometry thread
uses a non-blocking enqueue and drops a diagnostic sample rather than block.
When logging is disabled, no row formatting occurs.

`cbsms_experiment.py report` loads this file directly. It does not infer health
from combined console output.

The Rerun inspector uses `glim/dcreg_health/**`; the updated blueprint has
condition, health, status/runtime, Schur-spectrum, and mixed weak-mode views.

## 8. Build and tests

Commands:

```bash
docker exec cbsms_ws bash -lc \
  'source /opt/ros/noetic/setup.bash; \
   source /workspace/cbs_gtsam4.3/devel/setup.bash; \
   cd /workspace/cbs_gtsam4.3; \
   catkin build glim glim_ros --no-status --summarize -j8 -p1'

docker exec cbsms_ws bash -lc \
  'source /opt/ros/noetic/setup.bash; \
   source /workspace/cbs_gtsam4.3/devel/setup.bash; \
   cd /workspace/cbs_gtsam4.3; \
   catkin run_tests glim --no-status --summarize; \
   catkin_test_results build/glim'

cd src/cbsms/tools
python3 -m unittest test_cbsms_experiment_runner.py
python3 -m py_compile \
  cbsms_experiment.py \
  test_cbsms_experiment_runner.py \
  send_raw_cbs_dashboard_blueprint.py
```

Results:

- GLIM/GLIM ROS build: passed. The existing GLIM ROS OpenCV 3.4/4.2 linker
  conflict warnings remain.
- GLIM: 46 tests, 0 errors, 0 failures, 0 skipped. This includes 23 explicit
  DCReg diagnostics/health tests.
- cbsms runner: 4 tests passed.
- Python syntax checks: passed.
- `git diff --check`: passed in all changed repositories.

The GLIM tests cover the requested well-conditioned, weak translation, weak
rotation, mixed direction, sign, ordering, clustering, uniform scale,
near-singular, numerical-negative, indefinite, bootstrap, degenerate start,
degrade/recover, outlier, anti-ratchet, `off`, `log_only`, missing-reference,
and logging-disabled cases. An additional intermittent-gate bootstrap test was
added after the first live run exposed that edge case.

## 9. Outdoor01 runs with live Rerun

All three runs used the same nominal 60 sensor seconds, 0.25x bag rate,
MID360 clean configuration, N100 Kimera profile, GLIM `observe_only`, shadow
belief capture, and live Rerun.

```text
Stage 0 baseline:
runs/20260731-125120_m3dgr_outdoor01_dcreg_health_stage0_baseline_60s_rate025

Stage 1 off:
runs/20260731-133607_m3dgr_outdoor01_dcreg_health_stage1_off_60s_rate025

Stage 1 log_only final:
runs/20260731-134642_m3dgr_outdoor01_dcreg_health_stage1_log_only_final_60s_rate025
```

All returned roslaunch code 0. Wall durations from the one-second-resolution
manifest timestamps were 258 s, 259 s, and 258 s respectively.

`off` produced no health CSV and no new sidecar/logger startup message.

Final `log_only` health results:

```text
aggregate rows                    569
per-factor rows                  1138
valid aggregate analyses          569
factorization-ok                  569
linear-solve successful           569
registration-converged candidates 260
reference/health available rows   261
accepted bootstrap samples         30
0.5 m factor rows                 569
1.0 m factor rows                 569
rotation health p05 / min       0.753 / 0.698
translation health p05 / min    0.350 / 0.321
relative degraded rows              0
```

These relative-health values, including the minimum 0.321, are retained only
as a historical record and are not scientifically interpretable. Stage 1.5
showed that the permissive bootstrap had accepted already-degenerate geometry:
the 30-sample reference had median maximum rotation condition 28.6 and
translation condition 83.1, versus roughly 4--5 in known healthy runs.
Stage 1.6 replaces that bootstrap rule with separate limits defaulting to the
absolute DCReg threshold 10; an absolute weak mask now always rejects a
candidate.

Monitor runtime:

```text
off monitor calls                  0
off monitor computation            0 ms
log_only median                     0.0319 ms/scan
log_only p95                        0.0611 ms/scan
log_only maximum                    0.0959 ms/scan
```

## 10. Example aggregate rows

The following are abridged from the final dedicated CSV.

Reference initialization / accepted normal:

```text
frame=309 timestamp=1735888041.9204972 valid=1 reference_ready=1
reason=session_reference_initialized
R ratio=[1,1.7188,34.3746] R ref=[1,1.9793,28.5916]
T ratio=[146.5576,138.6046,1] T ref=[56.9135,83.1471,1]
R health smooth=[1,1,0.8318]
T health smooth=[0.3883,0.5999,1]
relative mask=[0,0,0,0,0,0]
```

Most deteriorated observed Outdoor01 row:

```text
frame=432 timestamp=1735888054.2198887 valid=1 reference_ready=1
reason=registration_not_converged
R health raw=[1,1,0.8872] smooth=[1,0.9730,0.7893]
T health raw=[0.2980,0.5971,1] smooth=[0.3209,0.6296,1]
relative mask=[0,0,0,0,0,0]
```

This is an example of degraded observability relative to the reference, but
not a declared degraded state: 0.3209 is above the 0.25 enter threshold. The
synthetic healthy/degraded/healthy test verifies the actual hysteresis state
transition because this Outdoor01 interval did not contain one.

Later recovery:

```text
frame=558 timestamp=1735888066.8205159 valid=1 reference_ready=1
R health raw=[1,0.9368,1] smooth=[1,0.8845,0.9921]
T health raw=[0.7805,0.8092,1] smooth=[0.6472,0.8272,1]
relative mask=[0,0,0,0,0,0]
```

## 11. Baseline regression and stop condition

The static no-op properties are verified:

- the monitor is not constructed in `off`;
- all new capture/analysis code is guarded by the monitor pointer;
- the `off` unit test records zero evaluations and zero formatted rows;
- the monitor API has no estimator-control output;
- `log_only` does not mutate its estimator input;
- no covariance or CBS/Kimera interface was changed.

However, the required recorded-run equivalence was not demonstrated.

| measurement | Stage 0 | `off` | `log_only` |
|---|---:|---:|---:|
| GLIM odometry samples | 570 | 570 | 563 |
| Kimera odometry samples | 294 | 294 | 294 |
| unique G2K shadow edges | 568 | 568 | 561 |
| G2K shadow rows | 12787 | 12802 | 12614 |
| G2K queued factor rows | 2385 | 2375 | 2374 |
| K2G inserted into GLIM (`observe_only`) | 0 | 0 | 0 |
| GLIM GT RMSE (m) | 0.9678 | 0.8240 | 0.9607 |
| Kimera GT RMSE (m) | 1.0723 | 0.7517 | 1.1037 |

For the 570 matching GLIM timestamps in Stage 0 versus `off`, the maximum
absolute pose-component difference was approximately `0.687`. Latest-edge
G2K shadow comparisons over all 568 common edges had maximum relative-mean
component difference `0.0721` and maximum covariance-entry difference
`7.07e-6`.

For Stage 0 versus `log_only`, 563 GLIM timestamps and 561 shadow edges were
common. Maximum pose-component difference was approximately `0.676`, maximum
relative-mean component difference `0.0678`, and maximum covariance-entry
difference `4.90e-5`.

These are not within a defensible floating-point no-op tolerance. This report
does not attribute the differences to the sidecar or to runtime
nondeterminism because no repeated pre-change control distribution was
established. Per the user's stop condition:

```text
mode=off changes baseline output -> stop and report rather than guess
```

Therefore Stage 1 must not be accepted as regression-proven or merged until
the project owner decides how to establish a deterministic baseline, such as
a reproducible single-thread/downsampling configuration or repeated
same-binary controls with an explicitly approved statistical tolerance.

## 12. Manual review and unresolved issues

Manual review is required for:

1. the failed recorded-run no-op equivalence above;
2. the exact meaning and intermittency of GLIM's existing pose-increment
   convergence signal;
3. the policy that a stability window is formed from gate-passing candidates
   rather than requiring consecutive global scans;
4. whether clustered modes should be rejected from reference bootstrap or
   accepted as mixed subspaces;
5. offline-profile production and cross-session basis alignment;
6. whether the configuration fingerprint should become a cryptographic hash
   covering more GLIM robust-weighting settings.

No issue was found with the audited 6x6 Hessian source or tangent convention.
There is no CBS timestamp association or covariance-basis claim in Stage 1,
because no metadata or covariance path was implemented.

No work should proceed to metadata or covariance inflation until this report
is reviewed and the baseline regression blocker is resolved.

## 13. Stage 1.5–1.8 regression-resolution addendum

The regression blocker described in Section 11 was subsequently diagnosed and
must not be read as the final Stage 1 result.

Repeated Stage 0 controls established substantial production nondeterminism
from the multithreaded sensor path. A deterministic preconverted PointCloud2
bag and ordered replay harness then removed the Livox-conversion/IMU callback
ordering ambiguity. With that input, three repetitions of Stage 0, candidate
with the section absent, candidate with explicit `off`, and candidate with
`log_only` were exactly equal for GLIM poses, consecutive increments, G-to-K
means, and G-to-K covariances. The maximum measured difference was `0.0` at a
`1e-12` tolerance.

The session-reference bootstrap was also corrected. Outdoor01 now accepts
0/569 session-reference candidates and leaves the reference unavailable;
Dynamic01 initializes a healthy session reference with median maximum Schur
condition ratios near 4–5; a compatible offline/hybrid profile remains usable
during the degenerate Outdoor01 startup without adapting from those samples.
The earlier Outdoor01 minimum health of 0.321 used an already-degenerate
reference and is invalid for scientific interpretation.

Stage 1.8 separated health computation, queue/worker scheduling, and full CSV
logging in test-only variants. Fifteen repetitions per variant did not
localize a stable scheduling effect. Thirty `off` runs showed that the earlier
LM-iteration and log-covariance-trace standardized effects lay inside the
empirical off-only null distribution.

In the final 20 randomized production pairs, `log_only - off` was:

- `+0.042707` LM iterations per scan, 95% CI
  `[-0.016347, 0.102814]`;
- `-0.002053%` mean G-to-K covariance trace, 95% CI
  `[-0.026307%, 0.021761%]`;
- zero event/output-count differences, queue drops, or producer waits.

The deterministic mathematical no-op proof is therefore complete, and the
production LM/covariance criteria pass. Formal production approval remains
withheld only under the locked Stage 1.8 preregistration because the absolute
consecutive-increment dispersion limits (`0.04 m`, `0.007 rad`) were exceeded
by `0.043866 m` and `0.010162 rad`. Both results were approximately `1.03x`
the natural `off`/`off` envelope and passed the separately preregistered
`1.25x` envelope criterion. This is a protocol/margin-review blocker, not
evidence that the passive monitor changes GLIM.

No metadata, covariance inflation, Kimera integration, or solver mitigation
was added. Changing the DCReg mathematics, healthy-reference logic, VGICP,
GLIM, CBS, or Kimera is not justified by these regression results.
