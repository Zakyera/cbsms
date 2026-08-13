# Stage 2C-A: GLIM DCReg Ground-Truth Frame-Chain Audit and Edge-Accuracy Validation Design

Date: 2026-08-07

Scope: audit and mathematical design only

Decision: **NO-GO for the requested healthy-versus-degraded full-6-DoF study with the currently verified local data**

## 1. Executive conclusion

The target scientific question is well posed:

> When Stage 1/2A reports lower GLIM LiDAR observability health, does the
> corresponding GLIM relative belief have greater ground-truth relative-motion
> error?

The edge-level quantities must be

```text
Z_G,ij  = inverse(T_G,i) * T_G,j
Z_GT,ij = inverse(T_GT,i) * T_GT,j
e_G,ij  = Logmap(inverse(Z_GT,ij) * Z_G,ij)
```

with separate rotational and translational magnitudes:

```text
rotation_error_rad = norm(e_G[0:3])
translation_error_m = norm(e_G[3:6])
```

No radians-to-metres combination, health-to-centimetres mapping, covariance
multiplier, probability, or active estimator decision is part of Stage 2C-A.

The audit found:

1. M3DGR Dynamic01 and Wheel-float01 contain real, changing OptiTrack 6-DoF
   poses, but the fixed transform from the tracked `UGV` rigid-body origin to
   GLIM's MID360/body frame is not present in the official calibration or the
   local project. They are therefore `EXTRINSIC_UNVERIFIED`.
2. M3DGR Outdoor01 is the known strongly degenerate sequence, but its supplied
   trajectory contains RTK position and an identity quaternion placeholder.
   It is `TRANSLATION_ONLY`, not full 6-DoF truth. Its local timestamps also
   contain duplicate and slightly backward steps that require source repair or
   an authoritative replacement before strict interpolation.
3. Four local S3E trajectories are position-only. Campus_Road_2 has changing
   orientation, but the exact dual-antenna RTK/heading reference point and its
   lever arm to Alpha's GLIM IMU frame are not documented in the local
   calibration. Campus_Road_2 is `EXTRINSIC_UNVERIFIED`.
4. Boreas supplies a verified full 6-DoF Applanix trajectory, official
   lidar-to-Applanix calibration, and hardware UTC synchronization. The local
   converter deliberately expresses the IMU and ground truth in the same
   Applanix body frame used by GLIM. The audited Boreas windows are therefore
   individually `FULL_6DOF_VERIFIED` for relative-edge evaluation.
5. The available Boreas windows all come from one original recording,
   `boreas-2020-11-26-13-58`. Stage 1 health has not been collected or
   reference-validated for the Boreas/Velodyne registration profile, and none
   of these windows has been demonstrated to be a LiDAR-degenerate sequence.
   The label "stationary start" is a motion description, not proof of
   geometric degeneracy.
6. No locally available pair currently satisfies the requested final gate:
   at least one healthy and one degraded dataset with verified full 6-DoF
   truth, verified body transforms, and compatible frozen Stage 1 health.

Consequently, this memo specifies the future evaluator but does not authorize
its scientific use on the current M3DGR healthy/degraded set.

## 2. Preservation and source state

No estimator source, message, configuration, build file, Stage 1 mathematics,
Stage 2A mathematics, or Stage 2B passive infrastructure was modified during
this audit. This memo is the only new audit artifact.

Safety patches were saved before the audit under:

```text
/home/yeranis/repos/V4RL/cbs_gtsam4.3/
  stage2c_audit/safety_before_audit_20260807/
```

The active repositories and audited HEADs were:

| Repository | Branch | HEAD |
|---|---|---|
| `glim` | `cbs-gtsam43-noetic` | `6c4189e117c61015a79001388debb27413c78fa6` |
| `glim_ros1` | `cbs-gtsam43-noetic` | `2dea515e9e38743b6fc73e7dfdf7e45d3ae5bf6f` |
| `liorf` | `cbsms/gtsam-4.3-develop` | `4286f982694dda550d1343ff21679f8ec2e708ba` |
| `cbs` | `cbsms/gtsam-4.3-develop` | `994b1d6a5c05fb38dd1b0731c6430ec11d2f1ff0` |
| `Kimera-VIO` | `cbsms/gtsam-4.3-develop` | `d0b2a31adf17ced8995994373f6b286679e26799` |
| `Kimera-VIO-ROS` | `cbsms/gtsam-4.3-develop` | `09d7e17b5d27f97f622f285f0f23a61e6748278d` |
| `cbsms` | `cbsms/gtsam-4.3-develop` | `873f60fa748df97bf9f5f7f9dcdb825b4f5ed30c` |

The pre-existing uncommitted Stage 2A/2B work was preserved. `git diff --check`
was clean in all seven repositories before this memo. No reset, checkout,
stash, commit, or push was performed.

Frozen documents read for this audit:

- `docs/2026-08-04_glim_dcreg_health_stage1_closeout.md`
- `docs/2026-08-04_glim_dcreg_stage2a_audit_design.md`
- `docs/2026-08-04_glim_dcreg_edge_metadata_stage2a_report.md`
- `docs/2026-08-05_dcreg_belief_shadow_stage2b.md`
- `docs/2026-08-05_dcreg_belief_shadow_stage2b_report.md`

## 3. Audited estimator and belief paths

### 3.1 Frozen scan health

The final scan registration is performed in
`glim/src/glim/odometry/odometry_estimation_cpu.cpp`:

- matching factors are created at lines 732-750;
- scan-only LM optimization runs at lines 824-838;
- Stage 1's immutable diagnostic input is populated at lines 854-948;
- the health timestamp is `frames[current]->stamp` and the frame key is
  `current` at lines 858-861;
- factor Hessians are extracted at the final optimized `values` at lines
  887-904;
- the resulting scan pose is written and the diagnostic callback is emitted at
  lines 1075-1081.

For the frozen two-level MID360 VGICP profile, Stage 1 analyzes

```text
H_lidar = H_VGICP_level_0 + H_VGICP_level_1
```

at the final optimized scan pose. It is a scan-to-rolling-map observability
diagnostic in the current pose's local right tangent, ordered rotation then
translation. It is not the Hessian of the outgoing CBS interval and is not an
independent information contribution that may be summed across scans.

### 3.2 Local GLIM smoother factors

The primary experiment must preserve the local estimator while prohibiting
incoming Kimera correction. The local GLIM path contains:

- scan-to-map matching during the local scan optimization;
- a scan-matched `BetweenFactor<Pose3>` from the preceding pose to the current
  pose and a `PriorFactor<Pose3>` on the current pose, constructed at
  `odometry_estimation_cpu.cpp:1091-1095`;
- an IMU preintegration factor between scan states when enough IMU samples are
  present, at `odometry_estimation_imu.cpp:345-350`;
- a bias random-walk `BetweenFactor`, at
  `odometry_estimation_imu.cpp:337-343`;
- a velocity between-factor fallback if insufficient IMU is available, at
  `odometry_estimation_imu.cpp:350-354`;
- initial pose damping, velocity prior, and bias damping, at
  `odometry_estimation_imu.cpp:280-284`;
- fixed-lag marginal information generated by the 5-second smoother.

The old pre-Stage-1 scan-health control is disabled in the canonical MID360
configuration:

```text
scan_health_enable: false
scan_health_apply_to_local_scan_precision: false
```

Stage 1 remains `log_only` and does not change the fixed scan factor precision.

### 3.3 Outgoing GLIM belief

`glim_ros1/src/glim_ros/cbs_bridge.cpp`:

- selects available pose keys and the configured 0.20-second time-horizon
  pairs at lines 2487-2549;
- reads the two smoother pose estimates at lines 2815-2820;
- constructs `measured_from_to = from_pose.between(to_pose)` at line 2821;
- obtains the exact joint marginal information and derives the right-local
  relative covariance at lines 2828-2849;
- stores the relative mean and covariance at lines 2860-2868.

Thus:

```text
Z_G,ij = T_G,i.between(T_G,j) = inverse(T_G,i) * T_G,j.
```

GTSAM `Pose3::Logmap` uses rotation-then-translation coordinates. The
ground-truth edge residual must use the same ordering and the same body frame.

## 4. Required primary GLIM isolation configuration

The primary experiment shall use **G-to-K publication enabled and K-to-G
reception disabled in effect**. This preserves the exact outgoing Stage 2A/2B
belief path while ensuring Kimera cannot correct GLIM.

The existing launch pattern already provides the required configuration:

```text
enable_cbs_bridge = true
glim_cbs_mode = observe_only
glim_cbs_odom_belief_in_topic = a unique unused topic
cbs_belief_receive_start_delay_sec = 9999.0
glim_cbs_odom_belief_out_topic = the Stage 2A/receiver topic
cbs_health_aware_enable = false
```

Evidence:

- `GlimCbsBridge::configure` maps unrecognized/non-injection modes to
  `ObserveOnly` at `cbs_bridge.cpp:697-711`.
- the dedicated GLIM-only M3DGR and Boreas launch files use `observe_only`, an
  unreachable input topic, and a 9999-second receive gate;
- the callback still exists, so every primary run must assert from structured
  counters that incoming received, matched, and injected factor counts are all
  zero.

This is preferable to disabling the bridge entirely because Stage 2A requires
the actual outgoing belief publication. The audit does not add a new runtime
switch.

Primary factor inventory:

```text
included:  local LiDAR scan matching and its local smoother factors
included:  IMU preintegration
included:  bias evolution / configured local priors
included:  fixed-lag marginal information
excluded:  all incoming K-to-G CBS factors
excluded:  all health-aware weighting or control
```

A secondary bidirectional-CBS experiment may be run later, but it must have a
separate result table and must not be pooled with the primary local-only GLIM
analysis.

## 5. Frame mathematics

### 5.1 Pose notation

Use active homogeneous transforms:

```text
T_A_B maps coordinates expressed in frame B into frame A.
```

If ground truth tracks body `G` and GLIM's comparison body is `B`, the required
static calibration is `T_G_B`. Then

```text
T_W_B(t) = T_W_G(t) * T_G_B.
```

The ground-truth edge in GLIM/Kimera body coordinates is

```text
Z_GT,B,ij
  = inverse(T_W_G(t_i) * T_G_B)
      * (T_W_G(t_j) * T_G_B)
  = inverse(T_G_B)
      * inverse(T_W_G(t_i)) * T_W_G(t_j)
      * T_G_B.
```

This conjugation is why a body extrinsic and its direction cannot be ignored.
A translated lever arm changes the translational part of a relative edge when
the platform rotates.

### 5.2 Unknown global world alignment cancels

For an unknown constant world transform `A`:

```text
inverse(A * T_i) * (A * T_j)
  = inverse(T_i) * inverse(A) * A * T_j
  = inverse(T_i) * T_j.
```

Therefore no APE-style global trajectory fit is needed or permitted for the
relative-edge target. This cancellation does **not** remove a right-side body
transform:

```text
inverse(T_i * T_G_B) * (T_j * T_G_B)
  = inverse(T_G_B) * inverse(T_i) * T_j * T_G_B.
```

No estimated-trajectory-to-GT alignment may be used to infer `T_G_B`.

### 5.3 Edge error

After both edges are in body `B`:

```text
e_G = Pose3::Logmap(inverse(Z_GT,B) * Z_G,B)
```

with

```text
e_G = [phi_x, phi_y, phi_z, rho_x, rho_y, rho_z].
```

Report:

```text
rotation_error_rad = sqrt(phi_x^2 + phi_y^2 + phi_z^2)
translation_error_m = sqrt(rho_x^2 + rho_y^2 + rho_z^2).
```

For these approximately 0.20-second edges, the full SE(3) Logmap translation
component is authoritative; it must not be replaced by a raw Cartesian
translation subtraction.

## 6. Local dataset inventory and timing audit

The following statistics were computed directly from the local pose files.
`max step` is the maximum positive step in source order, not an approved
interpolation threshold.

| Sequence | Rows | Duration | Approx. rate | Orientation | Source-order issues | Max positive step |
|---|---:|---:|---:|---|---|---:|
| M3DGR Dynamic01 | 52,134 | 175.150 s | 297.65 Hz effective; 360 Hz nominal | changing, 179.996 deg span | none | 0.089929 s |
| M3DGR Outdoor01 | 6,174 | 411.410 s | 15.0 Hz effective | all identity | 3,313 duplicate and 135 backward steps | 0.204335 s |
| M3DGR Wheel-float01 | 36,059 | 123.270 s | 292.54 Hz effective; 360 Hz nominal | changing, 179.991 deg span | no backward step; one long gap | 3.079954 s |
| S3E Campus_Road_2 Alpha | 111,551 | 1,580.889 s | 70.56 Hz effective | changing, 179.999 deg span | none | 0.129977 s |
| S3E Playground_2 Alpha | 220 | 219.001 s | about 1 Hz | all identity | none | 1.008481 s |
| S3E Square_1 Alpha | 384 | 453.001 s | sparse, about 1 Hz | all identity | long holes | 38.001400 s |
| S3E Square_2 Alpha | 197 | 238.001 s | sparse, about 1 Hz | all identity | long holes | 16.002228 s |
| S3E Teaching_Building_1 Alpha | 139 | 765.006 s | sparse, about 1 Hz | all identity | very long holes | 628.007036 s |
| Boreas moving 60 s window | 12,309 | 61.540 s | 200 Hz | changing, 95.699 deg span | none | 0.00500083 s |
| Boreas stationary-start 60 s window | 12,309 | 61.540 s | 200 Hz | changing, 1.317 deg span | none | 0.00500083 s |
| Boreas stationary-start 300 s window | 60,304 | 301.515 s | 200 Hz | changing, 101.933 deg span | none | 0.00500083 s |

All quaternion columns are TUM order `qx qy qz qw`. The audited quaternion
norm ranges are within approximately `1e-6` of unity for M3DGR and within
approximately `1e-9` for Boreas. S3E's local files are exactly normalized.

## 7. Dataset eligibility table

| Dataset/sequence | GT source and tracked body | Transform to GLIM body | Time status | Eligibility | Reason |
|---|---|---|---|---|---|
| M3DGR Dynamic01 | `/vrpn_client_node/UGV/pose`; OptiTrack `world -> UGV` | **missing** `T_UGV_mid360` or equivalent | ROS epoch overlap; exact sensor/OptiTrack offset not independently documented | `EXTRINSIC_UNVERIFIED` | real orientation, but tracked-body origin is unknown relative to MID360 |
| M3DGR Wheel-float01 | same OptiTrack `UGV` body | **missing** | same, plus a 3.08 s GT hole | `EXTRINSIC_UNVERIFIED` | body transform is a hard blocker; gap would invalidate affected edges |
| M3DGR Outdoor01 | `/mavros/global_position/raw/fix`; RTK antenna position | antenna lever arm missing | malformed duplicate/backward local timestamps | `TRANSLATION_ONLY` | all quaternions are identity placeholders; cannot validate rotational edges |
| M3DGR Corridor01/02, Elevator01 | files not present locally; official table says ArUco | unaudited | unaudited | `UNUSABLE_FOR_EDGE_VALIDATION` | bag, GT, and calibration chain must first be acquired and audited |
| S3E Campus_Road_2 Alpha | dual-antenna RTK position/heading; exact tracked reference point not identified locally | RTK/heading body to Alpha IMU lever arm missing | official unified time base and local epoch overlap; exact GT generation provenance still required | `EXTRINSIC_UNVERIFIED` | full-looking pose is insufficient without the tracked-body definition |
| S3E Playground_2, Square_1/2, Teaching_Building_1 Alpha | RTK position files | antenna lever arm missing | approximately 1 Hz with large holes in several sequences | `TRANSLATION_ONLY` | identity quaternions; several intervals too sparse for 0.20 s edge validation |
| Boreas moving 60 s | `gps_post_process.csv`; `T_ENU_applanix` | identity to converted GLIM `imu_link`/Applanix body; lidar calibration verified | hardware UTC; 200 Hz; zero added offset | `FULL_6DOF_VERIFIED` | valid frame and clock chain |
| Boreas stationary-start 60/300 s | same source sequence and calibration | same | same | `FULL_6DOF_VERIFIED` | valid for edge truth, but not an independent dataset or proven degeneracy case |

`FULL_6DOF_VERIFIED` here means the pose/frame/time source can support a
defensible relative edge. It does not mean Stage 1 health has been validated
for that sensor profile or that the sequence contains degeneracy.

## 8. Detailed ground-truth frame-chain audit

### 8.1 M3DGR

Local sources:

```text
Dynamic01/Dynamic01.bag
Dynamic01/Dynamic01.txt
Outdoor01/Outdoor01.bag
Outdoor01/Outdoor01.txt
Wheel-float01/Wheel-float01.bag
Wheel-float01/Wheel-float01.txt
```

The official M3DGR repository states:

- OptiTrack nominal localization accuracy is 1 mm at 360 Hz;
- RTK nominal accuracy is 0.8 cm horizontal and 1.5 cm vertical at 15 Hz;
- Dynamic/Wheel-type indoor sequences use Mocap and Outdoor uses RTK;
- pose files are evaluated in TUM format.

Sources:

- <https://github.com/sjtuyinjie/M3DGR>
- <https://github.com/sjtuyinjie/M3DGR/blob/main/calibration.md>

The calibration page defines sensor-to-sensor transforms, including the
MID360/camera-IMU chain used by this project. It does not define the position
or orientation of the OptiTrack rigid body named `UGV` relative to the
MID360, MID360 IMU, or `camera_imu_link`.

Consequences:

```text
T_world_UGV is available.
T_UGV_mid360 is not verified.
T_world_mid360 cannot be formed without guessing.
```

An SE(3) alignment between GLIM and OptiTrack would estimate exactly the
missing evaluation transform from the estimator being judged and is therefore
prohibited.

Outdoor01's local TUM file contains useful RTK positions but exactly identity
orientation in all 6,174 rows. The RTK antenna lever arm is also not the same
as the known sensor-to-sensor MID360 calibration. It cannot be used for the
primary 6-DoF edge target.

### 8.2 S3E

Local sources include ROS2 bags, ROS1 Alpha conversions, `alpha_gt.txt`, and
`src/datasets/S3E/alpha.yaml`.

The official S3E material describes synchronized/spatially calibrated sensor
streams and dual-antenna RTK ground truth:

- <https://dapengfeng.github.io/S3E/>
- <https://arxiv.org/abs/2210.13723>

The local `alpha.yaml` supplies camera-to-IMU and camera-to-lidar transforms.
It does not identify the exact phase center/reference point used in
`alpha_gt.txt` or supply the fixed transform from that RTK/heading reference
body to the IMU origin. The web-level statement "spatially calibrated" is not
enough to prove the missing transform direction and lever arm.

Campus_Road_2 contains changing quaternion orientation and is therefore not
translation-only, but it remains extrinsic-unverified. The remaining local
Alpha truth files contain identity orientation. The 1 Hz position rate and
large holes make several of them unsuitable even for short 0.20-second
translation-edge interpolation.

### 8.3 Boreas

The official Boreas data reference establishes:

- `gps_post_process.csv` contains the post-processed ground-truth pose in the
  Applanix frame at 200 Hz;
- the pose is `T_ENU_sensor`, with ENU x east, y north, z up;
- lidar timestamps are synchronized to UTC by hardwired Applanix NMEA/PPS;
- the recording computer is synchronized to UTC in the same fashion;
- the lidar-to-Applanix calibration is in `T_applanix_lidar.txt`;
- the pose uses GNSS, IMU, wheel encoder, RTX corrections, and POSPac RTS
  smoothing; typical position RMS is documented as approximately 2-4 cm.

Sources:

- <https://github.com/utiasASRL/pyboreas>
- <https://github.com/utiasASRL/pyboreas/blob/master/DATA_REFERENCE.md>

The local official calibration is:

```text
T_applanix_lidar =
  [ 0.7360555651  -0.6769211217   0   0    ]
  [ 0.6769211217   0.7360555651   0   0    ]
  [ 0              0              1   0.13 ]
  [ 0              0              0   1    ]
```

It maps lidar coordinates to Applanix coordinates. GLIM explicitly defines
`T_lidar_imu` to map IMU points into the lidar frame:

```text
p_lidar     = T_lidar_imu * p_imu
T_world_imu = T_world_lidar * T_lidar_imu.
```

The local Boreas converter therefore computes:

```text
T_lidar_imu = inverse(T_applanix_lidar)
```

and writes the exact inverse to
`runs/runtime_configs/config_boreas_applanix_cpu_odom_only/config_sensors.json`.
The converter also rotates raw IMU vectors from Boreas's raw body axes into
the Applanix axes and tags them `imu_link`. It writes TUM ground truth directly
from `gps_post_process.csv` using `T_ENU_applanix`.

Therefore, for this converted profile:

```text
GT tracked body = Applanix
GLIM IMU/body   = Applanix
T_GTbody_GLIMbody = identity
```

The Boreas Kimera launch also uses `base_link_frame_id=imu_link` and
`cbs_external_pose_frame_id=imu_link`, so the Stage 2B sender frame conversion
is identity for this profile. This does not authorize use of another Boreas
configuration without repeating the check.

Important independence limitation: the 60-second moving window, 60-second
stationary-start window, and 300-second window all come from
`boreas-2020-11-26-13-58`. They overlap and share the same truth solution,
map environment, calibration, and source recording. They form one split group.

## 9. Time synchronization and interpolation specification

### 9.1 Dataset manifest requirements

Every future eligible sequence must have an immutable evaluation manifest with:

- source bag and GT cryptographic hashes;
- GT pose convention and quaternion order;
- body-frame name and transform direction;
- source and checksum of every static transform;
- sensor and GT clock definitions;
- independently calibrated fixed time offset, or exactly zero if the source
  provides a shared clock;
- nominal pose rate;
- maximum accepted interpolation bracket;
- source-order validation results;
- permitted experiment split and source-sequence group.

No time offset may be estimated by minimizing GLIM, Kimera, or trajectory error
on evaluation data.

### 9.2 Loader validation

The future Stage 2C evaluator must not silently sort malformed truth. It shall:

1. parse source order;
2. reject non-finite positions or quaternions;
3. require quaternion norm within a declared tolerance and normalize only
   within that tolerance;
4. collapse only exact duplicate timestamp/payload records while counting
   them;
5. reject conflicting duplicates;
6. reject backward timestamps unless an independently documented source
   repair is applied before evaluation;
7. report coverage and every rejected endpoint.

The current Stage 2B offline helper sorts TUM poses and has no configurable
maximum interpolation gap. It is adequate for its prior guarded demonstration
but must not be used unchanged for the Stage 2C primary analysis.

### 9.3 Exact interpolation

For query time `t`:

- if an exact finite pose exists within `1e-9` seconds, use it exactly;
- otherwise require a unique ordered bracket `(t_0, t_1)`;
- require `t_0 < t < t_1` and `t_1 - t_0 <= max_gt_bracket_sec`;
- linearly interpolate translation;
- use normalized shortest-arc quaternion SLERP, flipping the second quaternion
  sign when the dot product is negative;
- reject extrapolation and out-of-coverage endpoints;
- log left gap, right gap, total bracket, interpolation fraction, and validity
  for both edge endpoints.

For the audited Boreas files, preregister:

```text
gt_time_offset_sec = 0.0
exact_match_tolerance_sec = 1e-9
max_gt_bracket_sec = 0.010001
```

This allows at most approximately two nominal 200 Hz periods and is larger
than the observed maximum 0.00500083-second truth step. It must be fixed before
examining health/error results.

No interpolation threshold is approved for M3DGR or S3E until their frame and
source-timestamp blockers are resolved.

## 10. Exact ground-truth edge construction

For each exact Stage 2B belief instance:

1. Read sender endpoints `t_i^G` and `t_j^G` from the exact legacy belief
   payload identified by its Stage 2A digest.
2. Interpolate `T_W_G(t_i)` and `T_W_G(t_j)` only if both endpoint checks pass.
3. Apply the verified body transform independently at both endpoints:

   ```text
   T_W_B(t) = T_W_G(t) * T_G_B.
   ```

4. Construct:

   ```text
   Z_GT,B = inverse(T_W_B(t_i)) * T_W_B(t_j).
   ```

5. Obtain the exact transformed sender edge `Z_G,B` recorded by Stage 2B, or
   reconstruct it from the exact payload through the already-tested shared
   frame-conversion helper.
6. Compute `e_G = Logmap(inverse(Z_GT,B) * Z_G,B)`.
7. Log the full six-vector plus separate rotational and translational norms.
8. Never require Kimera agreement for `gt_valid`.

For secondary receiver evaluation, use Stage 2B's exact pre-injection receiver
poses and their actual matched timestamps `t_a^K`, `t_b^K`:

```text
Z_GT,K = inverse(T_W_B(t_a^K)) * T_W_B(t_b^K)
e_K    = Logmap(inverse(Z_GT,K) * Z_K).
```

Do not reuse sender endpoint truth when the receiver endpoint times differ.
Report both endpoint timestamp errors and the interval-duration difference.

## 11. Repeated beliefs and the statistical unit

Rolling arrays republish stable sender edges. Publication rows are not
independent samples.

### 11.1 Primary unit

When the passive Stage 2B receiver is present, the primary unit is:

```text
one actual accepted Kimera factor instance
```

identified by sender session, publication sequence, belief ordinal, exact
payload digest, and Stage 2B's active decision observation. GLIM remains local
only because K-to-G reception is disabled. Kimera acceptance merely selects
the exact G-to-K payload instance; it does not define its correctness.

If multiple log rows describe the same active factor instance, retain one and
count duplicates. Do not count rejected/superseded rolling publications in the
primary correlation.

### 11.2 Stable-edge and interval sensitivity units

Two preregistered sensitivity analyses shall be reported separately:

- **stable sender edge:** group by sender session and stable edge digest; use
  the exact publication that produced the accepted factor;
- **unique endpoint interval:** group by exact endpoint timestamp bits; retain
  one accepted payload per interval and report payload conflicts.

If Kimera is not run, the fallback selection must be fixed before evaluation.
The recommended fallback is the last valid publication of each stable edge
before it leaves the rolling outgoing window, because it is the most mature
fixed-lag estimate. This fallback must not be mixed with accepted-factor rows.

### 11.3 Dependence-aware uncertainty

Use a hierarchy:

1. stable-edge clustering;
2. moving-block or stationary block bootstrap with a minimum 10-second block
   length, covering both the 5-second smoother lag and approximately 100
   10-Hz rolling-map scans;
3. sequence-level resampling when at least several independent source
   sequences are eligible.

Report sequence-stratified estimates regardless of pooling. With only one
source sequence, label confidence intervals as within-sequence uncertainty and
make no dataset-generalization claim. Naive row bootstrap is prohibited.

## 12. Join with frozen Stage 2A health

Attach only digest-matched Stage 2A fields:

- relative-health availability;
- rotational minimum and median health;
- translational minimum and median health;
- rotational/translational absolute-degenerate numerator, denominator, and
  fraction;
- rotational/translational relative-degraded numerator, denominator, and
  fraction;
- longest degraded run;
- reference source and lineage;
- support, source-point, inlier, matching-cost, convergence, and solve fields;
- ambiguity fractions.

Do not replace unavailable health with 1.0. Do not mix incompatible reference
lineages. Do not combine health, absolute degeneracy, support, cost, and
ambiguity into one confidence score.

Primary predictors:

```text
rot_health_min
trans_health_min
rot_absolute_degenerate_fraction/state
trans_absolute_degenerate_fraction/state
rot_degraded_fraction and longest run
trans_degraded_fraction and longest run
```

Support and matching fields are secondary explanatory variables, not hidden
health multipliers.

## 13. Statistical analysis plan

### 13.1 Primary hypotheses

Analyze separately:

1. rotational health versus GLIM rotational edge error;
2. translational health versus GLIM translational edge error;
3. absolute rotational degeneracy versus rotational edge-error distribution;
4. absolute translational degeneracy versus translational edge-error
   distribution;
5. degraded occupancy/persistence versus corresponding edge error.

Expected association signs, stated without making them acceptance criteria:

```text
health versus error:                 negative rank association
degenerate occupancy versus error:   positive rank association
degraded persistence versus error:   positive rank association
```

Required outputs:

- Spearman rank coefficient;
- dependence-aware 95% bootstrap interval;
- raw sample/stable-edge/sequence counts;
- monotonic health-bin plots;
- median, p75, p90, and p95 error per bin where sample support permits;
- dataset-stratified and explicitly labeled pooled results;
- missing-reference and invalid-GT counts.

Health bins must be fixed on development data or use prespecified bins, for
example `[0, .1), [.1, .25), [.25, .5), [.5, .75), [.75, 1]`. Quantile bins
computed independently on held-out data are descriptive only and must not be
used as a classifier.

No linear model is assumed. No `condition ratio -> centimetres` or
`health -> covariance` model may be fit.

### 13.2 Secondary estimator comparison

When receiver truth is valid, report rotation and translation separately for:

- GLIM error;
- Kimera error;
- which estimator is closer;
- GLIM-Kimera disagreement.

Classify descriptive failure cases:

- GLIM health low but GLIM edge accurate;
- GLIM health apparently healthy but GLIM edge inaccurate;
- both estimators accurate;
- both estimators inaccurate but agreeing;
- GLIM unhealthy and Kimera better;
- Kimera worse while GLIM is degraded but still more accurate;
- disagreement caused primarily by Kimera;
- missing health, reference, timing, or truth.

Kimera disagreement is never the primary target and never substitutes for
ground truth.

## 14. Calibration, development, and held-out split

The split unit is the original source sequence, not an extracted window.

Required future partition:

```text
reference/profile construction:
  independent healthy source sequences only

analysis development:
  independent sequences used to finalize plots, block length, and any bins

held-out evaluation:
  untouched healthy and degraded sequences with verified 6-DoF truth
```

Current constraints:

- Dynamic01 has already informed healthy Stage 1 reference expectations and
  cannot simultaneously be claimed as a fully untouched held-out sequence;
- Outdoor01 cannot be the full-6-DoF degraded held-out sequence;
- all current Boreas windows share one split group and cannot populate
  reference, development, and held-out partitions independently;
- individual edges do not replace independent dataset replication.

The final split must be written into a hashed manifest before computing any
thresholded or held-out result.

## 15. Scan-level versus edge-level interpretation

Stage 1 health describes the observability shape of an instantaneous
scan-to-rolling-map registration at the final scan pose. The evaluated
`Z_G,ij` is a short-horizon relative belief extracted from GLIM's fixed-lag
smoother.

The edge also reflects:

- IMU preintegration;
- scan-matched local between/prior factors;
- fixed-lag and marginal information;
- several scans over the interval;
- correlated rolling-map content.

Therefore Stage 1 health may be an informative indicator of belief error, but
it cannot equal the belief error or its covariance. A weak correlation would
not invalidate the scan-local observability detector; it would show that its
relationship to the smoother edge is limited or mediated by other factors.

## 16. Synthetic validation plan for a future evaluator

The following tests must pass before real-data analysis:

1. **World alignment cancellation:** apply a nontrivial left transform `A` to
   both endpoint poses; `Z_GT` must agree to `1e-12`.
2. **Identity body transform:** with `T_G_B=I`, edge construction equals direct
   `between` to `1e-12`.
3. **Nontrivial body conjugation:** compare explicit endpoint transformation
   against `inverse(T_G_B) * Z_Gbody * T_G_B` to `1e-12`.
4. **Lever-arm rotation coupling:** use a 1 m lever arm and a 90-degree body
   rotation; the transformed relative translation must show the geometrically
   predicted arc/chord displacement and must not equal the untransformed edge.
5. **Transform-direction rejection:** swapping `T_G_B` with its inverse must
   fail the known-pose expected result.
6. **Exact endpoint:** an exact timestamp returns the exact stored pose.
7. **SLERP midpoint:** identity to 180-degree yaw at the midpoint gives a
   90-degree yaw with unchanged interpolated translation expectation.
8. **Quaternion sign:** `q` and `-q` endpoints yield identical interpolation.
9. **Out of coverage:** either endpoint outside GT coverage invalidates the
   edge.
10. **Excessive gap:** a bracket just above `max_gt_bracket_sec` invalidates the
    endpoint; equality at the configured boundary is accepted.
11. **Duplicate consistency:** exact duplicate timestamp/payload is collapsed
    and counted; a conflicting duplicate invalidates the source.
12. **Backward time:** a negative source-order step is rejected, not sorted
    away.
13. **Zero error:** `Z_G=Z_GT` gives a six-vector and both norms below `1e-12`.
14. **Pure rotation error:** inject `Expmap([0,0,0.1,0,0,0])`; rotational norm
    is 0.1 rad and translational norm is zero within numerical tolerance.
15. **Pure translation error:** inject `Expmap([0,0,0,0.2,0,0])`;
    translational norm is 0.2 m and rotational norm is zero.
16. **Mixed error:** compare the exact six-vector against GTSAM `Pose3`
    operations, not a component subtraction.
17. **Repeated publications:** ten rows of one stable edge contribute one
    primary stable-edge unit.
18. **Accepted payload selection:** only the payload digest observed in the
    active accepted factor instance enters the primary table.
19. **Receiver timing:** sender and receiver endpoints construct different GT
    edges when their matched timestamps differ.
20. **No Kimera dependency:** GLIM error remains computable when receiver
    matching and disagreement are unavailable.
21. **Missing health:** valid GT edge plus unavailable reference retains GT
    error and explicit unavailable health.
22. **Block bootstrap:** resampling keeps all rows of a stable edge together
    and preserves configured temporal blocks.
23. **Frame implementation cross-check:** Python/native evaluator results
    match a GTSAM `Pose3::between`/`Logmap` test fixture to `1e-12` for moderate
    synthetic poses.

## 17. Offline evaluator implementation plan

No implementation was made in this task. A future approved implementation
should be a strictly offline extension, preferably a new tool rather than a
mutation of the online Stage 2B worker.

Proposed tool boundary:

```text
inputs:
  Stage 2B structured shadow CSV
  Stage 2A/legacy identity fields already embedded in that CSV
  verified dataset/evaluation manifest
  source ground-truth pose file
  verified body-extrinsic file

outputs:
  ground_truth_audit.json
  edge_accuracy_all_instances.csv
  edge_accuracy_accepted_factors.csv
  edge_accuracy_stable_edges.csv
  correlations_by_sequence.json
  block_bootstrap_intervals.json
  health_error_plots.svg/png
  optional saved Rerun .rrd
```

Required implementation changes relative to the current Stage 2B offline
helper:

- validate timestamps in source order; do not silently sort;
- enforce dataset-specific interpolation gaps;
- record exact endpoint interpolation diagnostics;
- require a hashed, explicitly verified frame-chain manifest;
- deduplicate by accepted factor and stable edge;
- use block/stable-edge/sequence-aware bootstrap rather than row bootstrap;
- retain radians and metres separately;
- produce no ROS control output and no estimator-consumed file.

The evaluator should use double precision throughout and verify its pose math
against GTSAM. Ground truth is read only offline and never feeds Stage 1
reference formation, Stage 2A metadata, Stage 2B online records, or either
estimator.

## 18. Real-data validation plan after the blockers are resolved

For every eligible sequence:

1. run GLIM with outgoing G-to-K publication, Stage 1 `log_only`, Stage 2A
   metadata, and K-to-G effectively disabled;
2. record all zero-injection counters and the resolved configuration hash;
3. retain the exact Stage 2A/2B identity and accepted factor streams;
4. evaluate only body- and time-verified GT edges;
5. produce timelines of health, absolute degeneracy, support, and GLIM GT edge
   error;
6. produce health-versus-error scatter and monotonic-bin plots;
7. write an Rerun `.rrd` containing the trajectory, GT, accepted edge spans,
   health, and separate rotational/translational edge errors;
8. report missing/invalid denominators and interpolation gaps;
9. report each dataset separately before any pooled result.

Minimum required evidence set for a GO:

- one independent healthy full-6-DoF sequence with verified body transform;
- one independent geometrically degraded full-6-DoF sequence with verified
  body transform;
- preferably additional source sequences so reference, development, and
  held-out sets are genuinely separate.

The Boreas profile may be used for a frame/evaluator smoke test now, but not as
the claimed healthy/degraded validation until frozen Stage 1 health is enabled
without mathematical modification, its reference behavior is validated for
that sensor/config fingerprint, and a degenerate interval is demonstrated
independently of GT error.

## 19. Exact missing evidence and recommended acquisition

Any one of the following routes can resolve the primary blocker.

### Route A: complete M3DGR calibration

Obtain from the dataset authors or survey independently:

```text
T_tracked_UGV_mid360
```

or an equivalent fully chained transform such as
`T_tracked_UGV_camera_imu * T_camera_imu_mid360`, including:

- transform direction;
- axes and handedness;
- OptiTrack rigid-body marker definition/origin;
- translation in metres;
- quaternion/matrix convention;
- calibration uncertainty and date;
- confirmation that the same mounting was used for Dynamic01 and
  Wheel-float01.

For a degraded M3DGR full-6-DoF sequence, acquire Corridor/planar GT plus the
ArUco-to-MID360/body transform, or a full INS orientation trajectory for
Outdoor01 together with the RTK antenna-to-MID360 lever arm and verified clock
offset. Do not synthesize Outdoor01 orientation from GLIM.

### Route B: expand Boreas

Select at least two additional independent Boreas source sequences with:

- official full Applanix truth and `T_applanix_lidar`;
- one independently identified healthy geometry sequence;
- one independently identified corridor/planar/sparse or otherwise
  LiDAR-degenerate sequence;
- frozen, compatible GLIM/DCReg configuration fingerprints;
- source-sequence-level split separation.

Run Stage 1 in passive `log_only` mode and validate reference behavior before
using the edge errors. Do not call low motion or a stationary vehicle
"degenerate" without the DCReg geometry evidence.

### Route C: complete S3E frame provenance

Obtain the exact Alpha ground-truth device frame definition and
`T_GTbody_imu_link`, including dual-antenna phase-center/heading baseline and
lever arm. Use only sequences whose pose files contain real orientation at a
rate and continuity sufficient for 0.20-second edges.

### Route D: collect a new sequence

Record the frozen GLIM input sensors together with an independently calibrated
6-DoF reference system. Survey the reference-body-to-MID360/IMU transform
before the evaluation run. Include healthy geometry, a designed planar or
corridor segment, recovery, changing orientation during degeneration, and
separate source sequences for reference/development/held-out use.

## 20. Remaining scientific risks

Even after the frame blockers are resolved:

1. Stage 1 scan health and smoother-edge error are different objects; IMU and
   marginal information may hide scan-level weakness over 0.20 seconds.
2. Consecutive edges and rolling-map registrations are correlated.
3. Ground-truth systems have nonzero orientation, position, and timestamp
   uncertainty; this must be described but not converted into a health model.
4. Boreas postprocessed truth uses IMU and wheel data related to sensors used
   by GLIM, so estimator and truth errors are not perfectly independent. It is
   still valid reference data, but the shared-sensor provenance must be stated.
5. A healthy reference profile is sensor/configuration specific. A MID360
   profile cannot be silently reused for Boreas Velodyne.
6. Global alignment cancellation assumes one constant left world transform;
   time-varying reference-frame errors do not cancel.
7. A fitted body transform or time offset from the evaluation trajectories
   would leak estimator error into the ground truth and is prohibited.

## 21. Explicit non-goals and Stage boundary

Stage 2C-A does not:

- alter Stage 1, Stage 2A, or Stage 2B mathematics;
- change GLIM, VGICP, the smoother, CBS, or Kimera;
- modify any belief mean, covariance, factor, weight, gate, or optimizer;
- transport or aggregate DCReg directions or Hessians;
- treat Kimera as truth;
- map health or condition ratio to metric error;
- fit covariance inflation;
- implement active coordination.

## 22. Final decision

The frame and time chain is sufficiently established to use Boreas for a
future evaluator smoke test. It is not sufficient to answer the requested
healthy-versus-degraded GLIM DCReg scientific question because the current
verified full-6-DoF data does not contain an independently established healthy
and degraded evaluation pair with compatible Stage 1 health.

**NO-GO:**

The currently available datasets do not support defensible full 6-DoF
relative-edge accuracy validation across both healthy and degraded geometry.
Obtain the M3DGR tracked-UGV-to-MID360 transform plus full-6-DoF degraded truth,
or add independent Boreas/new sequences with verified frame chains and proven
healthy/degraded LiDAR geometry before implementing the primary Stage 2C
evaluation.
