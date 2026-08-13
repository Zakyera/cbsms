# GLIM DCReg observability-health Stage 1 closeout

Date: 2026-08-04

Status: accepted for engineering integration and passive experimentation.

This document freezes and closes Stage 1. It supersedes the regression status
in the historical 2026-07-31 implementation report while retaining that
report as the detailed implementation record.

## 1. Final decision

Stage 1 provides a passive GLIM LiDAR-observability sidecar. The deterministic
no-op regression passed exactly, healthy-reference behavior passed, and the
structured logger remained non-blocking with no observed loss. The normal
production LM-iteration and G-to-K covariance margins also passed.

Two preregistered absolute consecutive-increment dispersion margins narrowly
failed. Both measurements were approximately `1.03x` the natural `off`/`off`
dispersion and passed the separately predefined `1.25x` natural-envelope
criterion. This is retained as a formal preregistration limitation. It is not
evidence that Stage 1 changes the estimator.

Engineering integration and passive experiments may proceed with Stage 1.
Covariance inflation is not approved or implemented.

## 2. Frozen scope

The only public modes are:

- `off`
- `log_only`

The default is `off`. If the `dcreg_health` section or its mode is absent, the
sidecar is bypassed. The off path creates no monitor, reference manager,
health logger, queue, worker, or health file and performs no health Hessian
capture, Schur analysis, or eigendecomposition.

`compute_only` and `enqueue_only` remain hidden Stage 1.8 diagnostic variants.
They are not accepted configuration modes and can only refine `log_only`
during controlled tests through `GLIM_DCREG_TEST_VARIANT`.

The following are frozen until a separately reviewed specification explicitly
changes them:

- Schur-complement and condition-ratio mathematics;
- eigenmode characterization and ambiguity handling;
- relative-health definition;
- healthy-reference bootstrap, offline/hybrid compatibility, and adaptation;
- smoothing and hysteresis thresholds;
- logger queue, overflow, batching, and worker architecture;
- public operating modes and default mode.

No Stage 1 code or configuration performs metadata integration, covariance
inflation, solver mitigation, or estimator control.

## 3. What Stage 1 measures

The sidecar analyzes the LiDAR-only final scan-registration information:

```text
H_lidar = H_VGICP_level_0 + H_VGICP_level_1
```

Both inputs are unary factors on the current pose, use rotation-then-
translation ordering, share the local right-perturbation tangent frame, and
are evaluated at the final optimized scan state. IMU, priors, marginal
factors, CBS factors, velocity, and bias are excluded.

The output is relative directional LiDAR observability health. It is not:

- expected metric pose error;
- a probability that the pose is correct;
- calibrated covariance;
- a replacement for matching support or registration quality;
- a DCReg condition-ratio-to-centimetre model.

Support, matching quality, absolute DCReg masks, and relative-to-reference
health remain separate diagnostics.

## 4. Configuration example

The safest production configuration is to omit the section or explicitly use:

```json
"odometry_estimation": {
  "dcreg_health": {
    "mode": "off"
  }
}
```

The accepted passive experiment configuration is:

```json
"odometry_estimation": {
  "dcreg_health": {
    "mode": "log_only",
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
      "sensor_identifier": "livox_mid360",
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
      "enabled": true,
      "csv_path": "${CBSMS_RUN_DIR}/parsed/glim_dcreg_health.csv",
      "log_every_n_frames": 1,
      "asynchronous": true,
      "queue_capacity": 256,
      "flush_every_n_rows": 10
    }
  }
}
```

The experiment runner resolves `CBSMS_RUN_DIR` for each run. Direct GLIM users
must provide a concrete writable path when logging is enabled. Invalid
configuration disables only the sidecar and leaves the estimator on its
existing path. Synchronous Stage 1 logging and support/health score
combination are rejected.

## 5. Reference validation

The frozen healthy-reference gates produced the intended behavior:

| Case | Outcome |
|---|---|
| Outdoor01, session only | 0/569 candidates accepted; reference unavailable; health unavailable |
| Dynamic01, session only | reference ready at frame 50; 429/557 candidates accepted |
| Dynamic01 nominal shape | median maximum rotation ratio 4.43418; translation ratio 4.16873 |
| Outdoor01, compatible hybrid profile | offline reference available for all 557 samples; 0 degraded session adaptations accepted |

The earlier Outdoor01 minimum health of 0.321 was computed against an
already-degraded reference and is invalid for scientific interpretation.
Persistent degenerate startup must continue to report no session reference,
not apparent health equal to one.

## 6. Deterministic regression result

The Stage 1.7 deterministic replay policy uses measurement timestamp as the
primary key and original bag-record order as the equal-timestamp tie-breaker.
The preconverted PointCloud2 content preserves ordered point records,
intensity, ring, and point offset time. A 600-cloud comparison against the
live conversion path found zero mismatches; offset time spanned
`0`–`101345293 ns`.

Three repetitions of each case were run:

```text
A: Stage 0 control
B: Stage 1, dcreg_health section absent
C: Stage 1, explicit mode=off
D: Stage 1, mode=log_only
```

Every run consumed 12,599 ordered events, processed 569 scans, emitted 570
GLIM poses and 3,393 G-to-K rows, and used event-sequence SHA-256:

```text
9112b78205cde928c56f9c64dafa74eb77c37dc4e5447cbc408ff7c3d15c2d50
```

All within-case and A/C, B/C, and C/D comparisons had maximum difference
`0.0` at `1e-12` for:

- scan registration outputs;
- GLIM poses;
- consecutive relative increments;
- G-to-K relative means;
- G-to-K covariance.

Section absent and explicit off used the same complete bypass.

## 7. Production natural-envelope result

The final production test used 20 randomized/interleaved `off`/`log_only`
pairs under the normal multithreaded configuration.

| Measurement | Log-only minus off | 95% CI | Decision |
|---|---:|---:|---|
| LM iterations per scan | +0.042707 | [-0.016347, 0.102814] | pass +/-0.20 margin |
| Mean G-to-K covariance trace | -0.002053% | [-0.026307%, 0.021761%] | pass +/-2% margin |
| Mean G-to-K log trace | +0.00000595 | [-0.000220, 0.000230] | no persistent shift |

The matched-pose translation and rotation criteria passed. Consecutive-
increment dispersion was:

| Measurement | Log-only/off | Natural off/off | Ratio | Decision |
|---|---:|---:|---:|---|
| Translation RMSE median | 0.043866 m | 0.042674 m | 1.0279 | failed 0.04 m absolute; passed 1.25x envelope |
| Rotation RMSE median | 0.010162 rad | 0.009956 rad | 1.0207 | failed 0.007 rad absolute; passed 1.25x envelope |

The absolute limits were tighter than the subsequently measured natural
off/off medians. They were not redefined after observing the results. The
formal failure is preserved, while the engineering interpretation relies on
the predefined natural-envelope comparison and the exact deterministic no-op
proof.

Thirty off runs were also resampled into 10,000 pseudo-group splits. The
previous Stage 1.7 effects (`g=-0.659` for LM and `g=-0.510` for covariance
log trace) fell inside the off-only 95% null ranges. No stable scheduling
effect was localized to health computation, queueing, formatting, or file
I/O.

## 8. Logger and runtime result

The frozen logger uses bounded non-blocking publication, capacity 256, drop-
newest overflow, and ten-row write batches. CSV formatting and file I/O run on
the logger worker. Health analysis itself remains passive odometry-path work.

In the final 20-pair experiment:

- maximum queue depth: 1;
- dropped snapshots: 0;
- dropped output rows: 0;
- producer waits: 0;
- mean queue publication: 0.002094 ms/scan;
- p95 per-run maximum publication: 0.009763 ms;
- median health-monitor work: 7.940 ms/run, or 0.01395 ms/scan;
- odometry-thread file I/O: none.

Diagnostics are expendable. Future saturation must drop diagnostics and
increment the exposed counters; it must never delay GLIM to preserve a log.

## 9. Build and test summary

The accepted implementation was validated with:

```text
catkin build glim glim_ros --no-deps
cmake --build build/glim --target scan_dcreg_diagnostics_test
ctest --test-dir build/glim --output-on-failure
python3 -m unittest discover -s src/cbsms/tools -p 'test_*.py' -v
```

Results:

- GLIM and GLIM ROS build: pass, with pre-existing warnings only;
- GLIM DCReg diagnostic tests: 39/39 pass;
- CBSMS reporting tests: 11/11 pass;
- deterministic A/B/C/D integration regression: exact pass at `1e-12`.

The tests cover Schur mathematics, mixed/clustered modes, bootstrap rejection,
offline/hybrid profiles, smoothing/hysteresis, no-op behavior, hidden
diagnostic variants, queue overflow/drop behavior, immutable diagnostic
records, logger failure isolation, and safe shutdown.

## 10. Artifact index

Artifacts remain local because the bags and full run trees are large. These
small reports are authoritative entry points relative to the
`cbs_gtsam4.3` workspace root:

| Artifact | SHA-256 |
|---|---|
| `stage17_diagnosis/STAGE17_DETERMINISTIC_INPUT_ORDER_REPORT.md` | `e6b641d139b21afe1fce295d267bb6d69bf6c7d8ff5aa70125c5bdc32236c24f` |
| `stage17_diagnosis/reference_audit/reference_validation_summary.json` | `66848e06c509ed0f986b306e53696bbcc784db3d8314b701dad7a34ff56326a8` |
| `stage18_diagnosis/deterministic_analysis.json` | `eb82c378f6b3cfa336416ee8fb22665c42caa72caefd4d32105f1200b00856c8` |
| `stage18_diagnosis/localization_analysis.json` | `134981c563a75786ba1b185e2b1de112b0bc11a57a86c1512d5445bdd0af5ccc` |
| `stage18_diagnosis/off_only_null_distribution.json` | `36766f63264f2600b011ff31ef9f4342c1a04a473fafa5a132d14c41d222f5ba` |
| `stage18_diagnosis/final_production_analysis.json` | `450b1d35aff58e2c16e66bb088d2af15a1d4b3a0b80582c691baa53e1af89e99` |
| `stage18_diagnosis/reference_validation.json` | `95f556b264f7b785125b5493594c378d550375b39df3818d6c08c903b54b8535` |
| `stage18_diagnosis/STAGE18_PRODUCTION_SCHEDULING_ISOLATION_REPORT.md` | `37a29c27a1d8eb779c238e72085f1670a55a33b9e4d6bec950cb717b257ece6d` |

The two independent converted-bag message manifests have the identical
content hash:

```text
e7c27d8135e631814d0022a88c718ae7948e8b548bcce5d03f9a31f10e32ed1a
```

The source Outdoor01 bag SHA-256 recorded in the conversion manifest is
`62f0d0b3f5098b5920d3c4125313daa031b176813b472b1e685b2f98c311de77`.

## 11. Known limitations

1. Relative health requires a compatible offline profile or enough accepted
   healthy session samples. A persistently degenerate startup intentionally
   leaves health unavailable.
2. Health is relative observability shape, not metric-error calibration or a
   statistically calibrated covariance.
3. Clustered eigenspaces and weak axis alignment remain mixed-direction
   diagnostics and must not be presented as confident pure physical axes.
4. Uniform loss of information is represented by separate support/scale
   diagnostics, not by condition ratios alone.
5. Production multithreaded GLIM remains naturally nondeterministic. The
   deterministic harness is the primary no-op proof.
6. The bounded logger queue is non-blocking but is not a strict fully
   preallocated real-time SPSC ring.
7. Stage 1 has no timestamp association between health samples and CBS edge
   intervals, no tangent-frame transport into CBS covariance coordinates, and
   no multi-scan eigenbasis aggregation. Those belong to a future Stage 2
   specification.

## 12. Explicit non-modification statement

Stage 1 does not modify, replace, scale, or control:

- any CBS message field or public ROS message;
- outgoing G-to-K relative means or covariance;
- CBS factor insertion or weighting;
- Kimera inputs, optimization, or outputs;
- VGICP residuals, correspondences, updates, or solver;
- GLIM scan optimization, smoother, trajectory, priors, velocity, or bias;
- the DCReg targeted PCG solver, which remains unused.

When Stage 1 is absent, off, invalid, or lacks a healthy reference, the
existing GLIM–CBS–Kimera path remains authoritative and unchanged.

## 13. Closeout boundary

Stage 1 stops here. Do not begin covariance inflation.

Any Stage 2 work must arrive as a separate reviewed specification covering at
least timestamp association, tangent-frame transport, multi-scan basis
aggregation, and optional metadata-only CBS integration. Until then, Stage 1
remains frozen in `off`/`log_only` form with default `off`.
