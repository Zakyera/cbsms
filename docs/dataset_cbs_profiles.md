# Dataset CBS Profiles

CBS algorithmic settings should stay dataset agnostic. Dataset-specific launch
profiles are responsible for declaring the sensor topics, estimator body frames,
calibration/extrinsics, evaluation ground truth, and visualization mode.

## Required Frame Contract

For every CBS ON dataset profile, record:

- dataset name and sequence
- GLIM body frame
- Kimera body frame
- whether the exchanged relative odometry is already in the receiver body frame
- if not, the fixed body-frame transform used for relative odometry conversion
- calibration source

CBS exchanges relative odometry. If a sender measurement is expressed in sender
body frame `S`, but the receiver graph uses body frame `R`, the receiver must
insert:

```text
Z_R = T_R_S * Z_S * T_R_S^-1
cov_R = Ad(T_R_S) * cov_S * Ad(T_R_S)^T
```

where `T_R_S` maps sender-frame coordinates into receiver-frame coordinates.

## S3E Alpha

- GLIM body frame: same frame convention as the Kimera/CBS setup used in the
  successful Square 1 reference experiment.
- Kimera body frame: S3E Alpha Kimera body frame.
- CBS relative odometry conversion: identity / disabled.
- Keep the stable Square 1 settings:
  - raw/original CBS relative odometry
  - `cbs_health_aware_enable:=false`
  - temporary-linear odometry factors
  - already-applied gate enabled
  - `cbs_odom_duration_gate_enable:=true`
  - `cbs_odom_horizon_sec:=0.20`
  - `cbs_odom_horizon_tolerance_sec:=0.06`
  - covariance scale `1.0`
  - no live Rerun for metric runs
  - artifact replay after runs

## M3DGR

- GLIM body frame: `camera_imu_link`
- Kimera body frame: `camera_imu_link`
- Calibration source: M3DGR `calibration.md` plus the GLIM
  `T_lidar_imu` convention in the M3DGR GLIM runtime config.
- Kimera CBS bridge external pose frame: `camera_imu_link`
- GLIM runtime config:
  `/workspace/cbs_gtsam4.3/runs/runtime_configs/config_m3dgr_camera_imu_experimental`
- GLIM uses the M3DGR camera IMU:
  `/camera/imu`
- GLIM camera-IMU white-noise/preintegration confidence is matched to Kimera
  `M3DGRMonoOriginal`:
  `imu_gyro_noise=1.6968e-04`, `imu_acc_noise=2.0000e-03`,
  `imu_int_noise=1.0e-8`.
- The legacy mixed-body static TF remains available as an override in
  `m3dgr_glim_kimera_experiment.launch`, but is disabled by default:

```text
parent: camera_imu_link
child:  livox_avia_imu

T_camera_imu_link_livox_avia_imu =
[  0.0502661  -0.9982900  -0.0298331   0.039151162 ]
[ -0.0116117   0.0292847  -0.9995040   0.111984435 ]
[  0.9986680   0.0505876  -0.0101199   0.236306953 ]
[  0.0000000   0.0000000   0.0000000   1.000000000 ]

TUM form:
0.039151162 0.111984435 0.236306953
0.507715982 -0.497277044 0.477055852 0.517066473
```

With this setup, Kimera converts:

- GLIM -> Kimera incoming beliefs already in `camera_imu_link`
- Kimera -> GLIM outgoing beliefs already in `camera_imu_link`

GLIM can consume Kimera beliefs in the same body frame without a separate
receiver-side conversion.

### M3DGR MID360

- GLIM body frame: `livox_frame`
- Kimera body frame: `camera_imu_link`
- Calibration source: M3DGR `calibration.md`, `mid360 -> camera_imu`.
- GLIM runtime config:
  `/workspace/cbs_gtsam4.3/runs/runtime_configs/config_m3dgr_mid360_native_imu_clean`
- GLIM uses the MID360 LiDAR and MID360 IMU:
  - `/livox/mid360/lidar`
  - `/livox/mid360/imu`
- GLIM publishes both MID360 LiDAR and MID360 IMU as `livox_frame`, uses
  identity `T_lidar_imu`, and scales acceleration with `acc_scale=9.80665`.
- Kimera CBS bridge external pose frame: `livox_frame`

```text
parent: camera_imu_link
child:  livox_frame

T_camera_imu_link_livox_frame =
[  0.0981574  -0.9951060   0.0113748  -0.003497660 ]
[  0.5142990   0.0409386  -0.8566330  -0.417688000 ]
[  0.8519750   0.0899349   0.5158010   0.198242000 ]
[  0.0000000   0.0000000   0.0000000   1.000000000 ]

TUM form:
-0.003497660 -0.417688000 0.198242000
0.367905538 -0.326718737 0.586665212 0.643214048
```

The M3DGR calibration page also lists a `mid360 -> mid360_imu` translation. Do
not compose that offset into the CBS body-frame TF unless the GLIM runtime
profile is changed to publish a distinct MID360 IMU body frame. With the current
profile, `livox_frame` is the pose frame GLIM publishes and the receiver-side
CBS conversion must target that frame.
