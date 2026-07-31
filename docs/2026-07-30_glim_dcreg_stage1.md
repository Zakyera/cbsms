# GLIM DCReg Stage 1: Read-Only Degeneracy Diagnostics

Date: 2026-07-30

## Scope

Stage 1 adds observation only. It characterizes the LiDAR scan-registration
geometry after GICP/VGICP optimization and publishes the result to logs,
experiment reports, and Rerun.

It does **not** change:

- the LM scan-registration update;
- GLIM iSAM2 values or factors;
- scan surrogate precision;
- poses, velocities, or IMU biases;
- CBS messages, covariances, or factor weights;
- Kimera.

## Data Flow

1. GLIM finishes the normal frame-to-model registration.
2. The existing LiDAR-only matching graph is linearized at the optimized scan
   pose.
3. The 6-by-6 Hessian block for `X(current)` is read in GTSAM `Pose3` local
   tangent order:

   ```text
   [rx, ry, rz, tx, ty, tz]
   ```

4. Rotation and translation are separated with complementary Schur
   reductions:

   ```text
   S_R = H_RR - H_Rt H_tt^+ H_tR
   S_t = H_tt - H_tR H_RR^+ H_Rt
   ```

   `+` denotes a symmetric pseudo-inverse. This keeps the diagnostic defined
   when an eliminated 3-by-3 block is rank deficient.

5. Each Schur matrix is eigendecomposed. Eigenvalues are sorted from weakest
   to strongest.
6. A mode is marked weak when its strongest-to-current eigenvalue ratio is
   greater than the configured threshold, or its eigenvalue is numerically
   non-positive.
7. Physical-axis weakness is the squared projection of the weak eigenspace
   onto each local tangent axis. The value is in `[0, 1]`.
8. A callback sends the same result to the existing GLIM Rerun factor-graph
   inspector.

## Interpretation

- A large rotation condition ratio means the scan constrains some local
  rotation directions much less than the strongest rotation direction.
- A large translation condition ratio means the same for local translation.
- `local_rx_weakness` through `local_tz_weakness` show which body-local axes
  contribute to the weak eigenspace.
- Rerun arrows start at the current GLIM pose. Local eigenvectors are rotated
  into the world frame only for display.
- Eigenvector sign is arbitrary. An arrow can flip 180 degrees between
  neighboring frames while representing the same weak direction.

The Stage 1 axis projection is deliberately simpler than DCReg's greedy
one-to-one physical-axis alignment. It does not claim a unique axis label when
weak modes are mixed.

## Controls

GLIM odometry JSON:

```json
"scan_dcreg_diagnostics_enable": false,
"scan_dcreg_condition_threshold": 10.0,
"scan_dcreg_spectral_ratio_cap": 1e12
```

The feature is default-off in code. It is enabled for the canonical MID360
M3DGR runtime configuration:

```text
runs/runtime_configs/config_m3dgr_mid360_native_imu_clean/config_odometry_cpu.json
```

ROS/Rerun:

```text
glim_dcreg_visualization_stride:=5
```

The diagnostic CSV row remains scan-rate. Only Rerun publishing is decimated.
The default display stride is five scans.

## Logs and Reports

Raw log marker:

```text
GLIM_SCAN_DCREG_ROW
```

Generated artifacts:

```text
parsed/glim_dcreg.csv
parsed/glim_dcreg_summary.csv
summary.json -> glim_dcreg_summary
summary.md -> GLIM DCReg Stage 1
```

The raw row contains validity/status, eliminated-block ranks, negative
eigenvalue counts, Schur condition ratios, six eigenvalues, six spectral
ratios, six local-axis weakness values, and six weak-mode flags.

## Rerun

The canonical blueprint now has a top-level tab:

```text
GLIM DCReg Stage 1 Inspector
```

It is also embedded in the CBS-On dual factor-graph inspector.

Main entity paths:

```text
glim/dcreg/condition/*
glim/dcreg/eigenvalue/rotation/*
glim/dcreg/eigenvalue/translation/*
glim/dcreg/spectral_ratio/*
glim/dcreg/axis_weakness/*
glim/dcreg/status/*
glim/dcreg/rank/*
glim/dcreg/spatial/*
```

Activate the dedicated tab:

```bash
/home/yeranis/.local/share/pipx/venvs/rerun-sdk/bin/python \
  /home/yeranis/repos/V4RL/cbs_gtsam4.3/src/cbsms/tools/send_raw_cbs_dashboard_blueprint.py \
  --app-id cbsms \
  --url rerun+http://127.0.0.1:9876/proxy \
  --active-tab dcreg
```

## Validation

Build:

```bash
docker exec cbsms_ws bash -lc \
  'source /opt/ros/noetic/setup.bash; cd /workspace/cbs_gtsam4.3; \
   catkin build glim glim_ros --no-status --summarize -j8 -p1'
```

Focused tests:

```bash
docker exec cbsms_ws bash -lc \
  'source /opt/ros/noetic/setup.bash; cd /workspace/cbs_gtsam4.3; \
   catkin run_tests glim --no-status --summarize'
```

The three Stage 1 tests cover:

- an uncoupled diagonal Hessian;
- complementary rotation/translation Schur reductions with coupling;
- rank-deficient eliminated blocks.

All pass.

The first 60 sensor-second Outdoor01 live run, with scan-rate Rerun publishing,
completed with return code zero:

```text
runs/20260730-124838_m3dgr_outdoor01_dcreg_stage1_60s_rate025_20260730_1248
```

It produced 519 diagnostic rows:

- valid: 519;
- invalid: 0;
- minimum rotation/translation eliminated-block ranks: 3 / 3;
- indefinite rotation/translation Schur scans: 0 / 0;
- rotation condition p50 / p95 / max:
  `31.7766 / 44.6342 / 52.5241`;
- translation condition p50 / p95 / max:
  `140.4882 / 190.0246 / 238.3350`.

The lower-overhead display stride of five was introduced after that run.

The final 60 sensor-second Outdoor01 validation used the default display stride
of five and completed with return code zero:

```text
runs/20260730-132131_m3dgr_outdoor01_dcreg_stage1_stride5_60s_rate025_20260730_1304
```

It produced 547 diagnostic rows:

- valid: 547;
- invalid: 0;
- minimum rotation/translation eliminated-block ranks: 3 / 3;
- indefinite rotation/translation Schur scans: 0 / 0;
- rotation condition p50 / p95 / max:
  `30.9288 / 45.5542 / 55.3099`;
- translation condition p50 / p95 / max:
  `140.6632 / 193.3311 / 223.0897`;
- GLIM DCReg visualization warnings: 0.

Rerun was open on the host, the `GLIM DCReg Stage 1 Inspector` blueprint tab
was active, and the recording used:

```text
m3dgr_outdoor01_dcreg_stage1_stride5_60s_rate025_20260730_1304
```

## Source Map

Core diagnostic:

```text
src/glim/include/glim/odometry/scan_dcreg_diagnostics.hpp
src/glim/src/glim/odometry/scan_dcreg_diagnostics.cpp
```

Hessian extraction and scan-rate logging:

```text
src/glim/include/glim/odometry/odometry_estimation_cpu.hpp
src/glim/src/glim/odometry/odometry_estimation_cpu.cpp
```

Callback:

```text
src/glim/include/glim/odometry/callbacks.hpp
src/glim/src/glim/odometry/callbacks.cpp
```

Rerun:

```text
src/glim_ros1/src/glim_ros/glim_factor_graph_inspector.cpp
src/cbsms/tools/send_raw_cbs_dashboard_blueprint.py
```

Experiment parsing/reporting:

```text
src/cbsms/tools/cbsms_experiment.py
```

Tests:

```text
src/glim/src/test/scan_dcreg_diagnostics_test.cpp
```

## Stage Boundary

Do not use these diagnostics to precondition an update, suppress a direction,
alter scan precision, or reweight a factor without a separate controlled
Stage 2 experiment. Threshold 10 is the DCReg-style starting value, not yet a
calibrated decision boundary for GLIM/MID360.
