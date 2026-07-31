# GLIM DCReg Stage 3: Multi-Dataset Calibration Protocol

Date: 2026-07-30

## Decision

The first calibration target is deliberately narrow:

```text
estimator: GLIM
LiDAR: Livox MID-360
runtime profile: m3dgr_mid360_native_imu_clean
DCReg input: LiDAR registration Hessian before CBS factor injection
output target: directional expected GLIM relative-pose error covariance
```

No other LIO is in scope. S3E and Boreas are inventoried so they cannot be
mixed silently into this calibration, but their Velodyne data is excluded from
the MID360 v1 fit.

This stage does not fit a calibration function and does not change any
covariance, pose, factor, CBS message, GLIM optimization, or Kimera
optimization.

## Added Artifacts

Versioned dataset and split manifest:

```text
src/cbsms/config/glim_dcreg_calibration_manifest_v1.json
```

Manifest validator and inventory report:

```text
src/cbsms/tools/glim_dcreg_calibration_protocol.py
```

Focused tests:

```text
src/cbsms/tools/test_glim_dcreg_calibration_protocol.py
```

The experiment runner now accepts:

```text
--dcreg-calibration-manifest
--dcreg-sequence-id
```

The selected sequence record is copied into each run's `manifest.json`.
If its ground-truth body transform is unverified, the runner still records the
DCReg spectra, eigenbasis, and exact outgoing belief covariance, but the
report deliberately skips directional error projection.

The runner now also expands its wall-time deadline by `1 / bag_rate`. A
60-second sensor interval at rate 0.25 receives at least 240 seconds plus the
requested timeout padding, preventing a nominal 60-second collection from
being truncated merely because replay is intentionally slow.

## Why The Body-Frame Gate Is Required

Dynamic01 and Wheel-float01 contain high-rate changing 6-DoF poses on:

```text
/vrpn_client_node/UGV/pose
```

The message parent frame is `world`; the tracked rigid body is named `UGV`.
The local M3DGR calibration contains the camera, camera IMU, Avia, MID360, and
sensor-IMU transforms, but it does not identify the transform from the
OptiTrack `UGV` rigid-body origin to the MID360 pose published by GLIM.

A fixed body-frame mismatch conjugates relative poses:

```text
T_mid360_i_j = X^-1 T_tracked_i_j X
```

It therefore affects translation and rotation, not merely the global
trajectory alignment. A free SE2 or SE3 world alignment cannot repair the
missing rigid-body transform. The manifest consequently marks both sequences:

```text
status: collect_pending_extrinsic
directional audit: skip_unverified_pose_extrinsic
```

Official sources checked:

```text
https://github.com/sjtuyinjie/M3DGR
https://github.com/sjtuyinjie/M3DGR/blob/main/calibration.md
```

The official repository describes the OptiTrack system and the calibration
page defines source-to-target sensor transforms, but neither source currently
documents the tracked `UGV` rigid-body-to-MID360 transform.

## Split Protocol

The calibration split is by complete source sequence:

```text
random edge split: forbidden
overlapping windows across partitions: forbidden
split unit: source_sequence / split_group
minimum calibration sequences: 3
minimum validation sequences: 1
minimum test sequences: 1
freeze split before fitting: required
```

This prevents temporally adjacent edges from the same trajectory appearing in
both training and evaluation. Boreas windows from the same original recording
share one `split_group` to make the same rule explicit for future cross-sensor
work.

The split is not frozen and model fitting is currently prohibited.

## Ground-Truth Inventory

The validator reads every local TUM-format truth file and checks:

- file availability;
- parse validity and quaternion normalization;
- timestamp ordering;
- position span;
- observed orientation span;
- declared position-only versus full-pose content;
- bag/GT time overlap when bag timing is recorded;
- fit-partition separation by `split_group`;
- verified GT-body-to-GLIM pose transform for every fit partition;
- completeness and return code of referenced shadow runs.

Current primary MID360 sequences:

| sequence | GT content | source | current use |
|---|---|---|---|
| Dynamic01 | full pose, 52,134 rows, 179.996 deg orientation span | OptiTrack `UGV` | healthy shadow collection; fit blocked on extrinsic |
| Outdoor01 | position only, 6,174 rows | RTK | translation-direction evidence only |
| Wheel-float01 | full pose, 36,059 rows, 179.991 deg orientation span | OptiTrack `UGV` | healthy shadow collection; fit blocked on extrinsic |

Outdoor01 contains 135 very small backward timestamp steps, with maximum
`1.91e-5 s`; the validator reports this as a warning rather than a fatal
ordering failure.

## Valid 60-Sensor-Second Shadow Collections

All three comparable runs used:

```text
bag duration: 60 sensor seconds
bag rate: 0.25
glim_cbs_mode: observe_only
cbs_health_aware_enable: false
glim_dcreg_belief_shadow_enable: true
glim_dcreg_visualization_stride: 5
covariance mutation: none
Rerun: live DCReg inspector
```

| sequence | valid scans | rotation condition p50 / p95 / max | translation condition p50 / p95 / max | weak scans R / T |
|---|---:|---:|---:|---:|
| Dynamic01 | 567 | 4.388 / 5.581 / 6.017 | 4.187 / 6.622 / 8.940 | 0 / 0 |
| Outdoor01 | 569 | 29.881 / 44.529 / 60.528 | 131.667 / 184.591 / 230.259 | 568 / 569 |
| Wheel-float01 | 569 | 4.649 / 5.700 / 6.239 | 4.160 / 6.361 / 8.221 | 0 / 0 |

Run directories:

```text
runs/20260731-105940_m3dgr_dynamic01_dcreg_multidataset_collection_valid_60sensorsec_rate025
runs/20260730-144847_m3dgr_outdoor01_dcreg_directional_shadow_isolated_valid_60s_rate025
runs/20260731-111028_m3dgr_wheel_float01_dcreg_multidataset_collection_valid_60sensorsec_rate025
```

Rerun recordings:

```text
m3dgr_dynamic01_dcreg_multidataset_valid_20260731_105927
m3dgr_outdoor01_dcreg_directional_shadow_isolated_valid_20260730_144833
m3dgr_wheel_float01_dcreg_multidataset_valid_20260731_111011
```

Dynamic01 recovered 565 unique shadow edges and Wheel-float01 recovered 567.
Their reports show:

```text
Directional error projection was deliberately skipped.
Policy: skip_unverified_pose_extrinsic
```

Two earlier short pilots used only 120 seconds of wall allowance at 0.25 bag
rate and covered about 27 sensor seconds. They remain reproducibility
artifacts, but must not be used as the canonical 60-second collections:

```text
runs/20260730-163845_m3dgr_dynamic01_dcreg_multidataset_collection_shadow_60s_rate025
runs/20260731-105607_m3dgr_wheel_float01_dcreg_multidataset_collection_shadow_60s_rate025
```

## Finding

The current local sequences clearly span two geometric regimes:

```text
Dynamic01 and Wheel-float01: healthy at ratio threshold 10
Outdoor01: strongly degenerate, especially in translation
```

That is useful for designing the protocol, but not enough to fit and validate
a dataset-independent calibration curve. There is only one currently usable
degenerate MID360 source sequence, and its truth is position-only RTK.
Splitting Outdoor01 edges randomly would create leakage and would only learn an
Outdoor01 correction.

## Calibration Model Reserved For The Next Stage

After eligibility is satisfied, the first model should remain simple and
directional:

```text
r_i = lambda_max / lambda_i
sigma_cal_i^2 = sigma_nominal_i^2(delta_t, motion) * g(log(r_i))
h_i = min(1, 1 / g(log(r_i)))
```

`g` should be monotonic and fitted separately for translation and rotation.
The target is held-out expected squared error after subtracting or otherwise
accounting for reference uncertainty. Threshold 10 remains a diagnostic label,
not the fitted confidence value.

Candidate comparisons must include:

```text
GLIM raw covariance only
DCReg geometry only
GLIM raw covariance plus DCReg
```

The reconstructed covariance must be positive semidefinite and expressed in
the documented GTSAM local tangent order:

```text
[rx, ry, rz, tx, ty, tz]
```

## Required Next Inputs

Before fitting:

1. obtain or independently verify `T_tracked_UGV_mid360` for OptiTrack
   sequences;
2. add more independent MID360 source sequences containing degeneracy;
3. assign at least 3/1/1 distinct source sequences to
   calibration/validation/test;
4. freeze that split;
5. rerun or report every experiment with the live DCReg Rerun inspector;
6. fit only in shadow mode and validate directional coverage on unseen
   sequences.

## Validation

```bash
cd /home/yeranis/repos/V4RL/cbs_gtsam4.3/src/cbsms/tools
python3 -m unittest -v \
  test_cbsms_experiment_runner.py \
  test_glim_dcreg_directional_audit.py \
  test_glim_dcreg_calibration_protocol.py
python3 -m py_compile \
  cbsms_experiment.py \
  glim_dcreg_calibration_protocol.py
python3 glim_dcreg_calibration_protocol.py
git diff --check
```

Results:

```text
9 focused Python tests passed
manifest inventory valid
split frozen: false
model fitting allowed: false
git diff --check: clean
```
