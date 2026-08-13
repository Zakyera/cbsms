# Stage 2C-A GEODE Ground-Truth Frame-Chain Addendum

Date: 2026-08-07

Scope: read-only dataset, calibration, frame, and timing audit. No estimator,
DCReg, CBS, Kimera, message, covariance, or weighting implementation was
changed.

Decision: **GEODE is suitable for a passive degraded-sequence pilot, but it
does not by itself close the full Stage 2C healthy-versus-degraded validation
gate.**

## 1. Executive answer

GEODE materially improves the situation relative to M3DGR.

The key ambiguity is now resolved by an official maintainer response: the
input to GEODE's `gamma2GT_gnss.py` evaluation script must be a trajectory in
the **LiDAR coordinate frame**, not the IMU frame. The released script then
maps the Gamma/Livox trajectory into the Beta/ground-truth coordinate system.

This makes `Inland_Waterways_Short_Gamma` a defensible full-6-DoF degraded
pilot candidate, provided that the evaluator:

1. converts the frozen GLIM IMU-body belief into the Gamma LiDAR body using
   the exact extrinsic configured in that GLIM run;
2. applies the official Gamma-to-GT conjugation exactly;
3. deterministically sorts the supplied GT timestamps;
4. excludes the first approximately 11 seconds without GT coverage;
5. invalidates endpoints that cross excessive GT interpolation gaps; and
6. records calibration sensitivity rather than fitting any transform to the
   evaluation trajectory.

However, GEODE is deliberately a degeneracy benchmark. The paper classifies
Flat Ground as two translational plus one rotational degeneracy direction and
Inland Waterways as one translational degeneracy direction. It does not give us
an independently verified healthy Gamma/Livox reference sequence. Therefore:

- **GO for a passive GEODE ingestion and edge-evaluator pilot.**
- **NO-GO for the final claim that relative health predicts true edge error
  across an independently healthy-versus-degraded held-out dataset set.**

No ratio-to-centimetre model is authorized. Kimera disagreement remains a
secondary estimator comparison, not ground truth.

## 2. Preserved source state

The active source repositories were not reset, stashed, committed, pushed, or
modified during the audit. Pre-existing Stage 1/2A/2B work remains intact.

Audited repository state:

| Repository | Branch | HEAD |
|---|---|---|
| `glim` | `cbs-gtsam43-noetic` | `6c4189e117c61015a79001388debb27413c78fa6` |
| `glim_ros1` | `cbs-gtsam43-noetic` | `2dea515e9e38743b6fc73e7dfdf7e45d3ae5bf6f` |
| `liorf` | `cbsms/gtsam-4.3-develop` | `4286f982694dda550d1343ff21679f8ec2e708ba` |
| `cbs` | `cbsms/gtsam-4.3-develop` | `994b1d6a5c05fb38dd1b0731c6430ec11d2f1ff0` |
| `Kimera-VIO` | `cbsms/gtsam-4.3-develop` | `d0b2a31adf17ced8995994373f6b286679e26799` |
| `Kimera-VIO-ROS` | `cbsms/gtsam-4.3-develop` | `09d7e17b5d27f97f622f285f0f23a61e6748278d` |
| `cbsms` | `cbsms/gtsam-4.3-develop` | `873f60fa748df97bf9f5f7f9dcdb825b4f5ed30c` |

The official GEODE dataset repository was inspected at:

```text
PengYu-Team/GEODE_dataset
commit c6e930623d4fed450d7fc50e16e3ffe0288b692b
```

The audit began with `git diff --check` clean. This addendum and the correction
to the local download manifest are the only audit writes.

## 3. Local GEODE artifacts

Root:

```text
/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/datasets/GEODE
```

### 3.1 Primary sequences

| Sequence | Sensor data | Ground truth | GT type |
|---|---:|---:|---|
| Flat Surfaces Smooth Gamma | 807,978,452 B | 23,315 TUM poses | Vicon 6-DoF |
| Inland Waterways Short Gamma | 3,239,293,141 B | 29,922 TUM poses | CG610 RTK/INS 6-DoF, aligned to Beta |

Checksums:

```text
flat_surfaces_smooth.bag
  ea34c071e67aad9c5935176de392016029fea15f553520c83e220ab445b04889
flat_surfaces_smooth_gt.txt
  1c05a7a0f0b0b0d9d3fd5132c48821413bd8e3ea5c55e0d3e9beee04a9745d86
Inland_Waterways_Short_Gamma.bag
  bd58ef1435cbfb41c2996f0d5121d96378d5aca8a6109b3e5b707a312ab91c00
Inland_Waterways_Short_gt.txt
  e46b0be316e350624775ebefb06b6c17ddae1dcbdd8c5e11c0450030962f7cc5
gamma_config.yaml
  f67eaba499306f08d81a2f8dfe8459bb0246d4ee7294b03478f28affb56cc47a
```

The complete download provenance is in
`src/datasets/GEODE/DOWNLOAD_MANIFEST.md`.

### 3.2 Additional evidence

- `Offroad7_Gamma` was downloaded, but its GT folder was not publicly
  readable and its bag contains no GT topic.
- `Tunneling_tunnel1_gt.txt` contains position only; all four quaternion
  fields are zero. It cannot support 6-DoF edge validation.
- `bridge01_gt.txt` is a full-pose GT candidate, but the corresponding 4.1 GB
  Alpha bag was not downloaded.

## 4. Authoritative GEODE evidence

Primary sources:

1. GEODE paper: <https://arxiv.org/html/2409.04961v2>
2. Official dataset repository:
   <https://github.com/PengYu-Team/GEODE_dataset>
3. Official GT-frame issue and maintainer answer:
   <https://github.com/PengYu-Team/GEODE_dataset/issues/16>
4. Official evaluation-script explanation:
   <https://github.com/PengYu-Team/GEODE_dataset/issues/13>
5. Current unresolved Gamma-extrinsic report:
   <https://github.com/PengYu-Team/GEODE_dataset/issues/19>

The sources establish the following.

### 4.1 Ground-truth sources

- Outdoor bridge, urban tunnel, inland-waterway, and off-road sequences use a
  CHCNAV CG610 RTK/INS to provide full 6-DoF poses.
- Inland and off-road runs mount Alpha, Beta, Gamma, and GT equipment on one
  aluminum rack.
- The CG610 trajectory is aligned to the Beta device by hand-eye calibration.
- Alpha and Gamma are related to that GT coordinate by multi-LiDAR
  calibration.
- Flat-surface sequences use a Vicon system for full 6-DoF poses.
- Metro-tunnel Leica truth is position-only.

### 4.2 Evaluation body frame

In issue 16, a user asked exactly whether an evaluation-script input should
represent the LiDAR or IMU frame. The GEODE maintainer answered that it must
be in the **LiDAR coordinate frame**.

The README and issue 13 then specify:

- Beta LiDAR trajectories can be compared directly with the released
  inland/off-road GT.
- Gamma LiDAR trajectories must first pass through
  `gamma2GT_gnss.py`.

This is the authoritative body-frame contract that was missing from the first
Stage 2C-A audit.

### 4.3 Known calibration caveats

The published Gamma YAML contains quality problems that must not be hidden:

- `gyro_std` appears twice, where the second field is evidently intended to
  be a random-walk value;
- `T_LiDAR_CamL` appears twice, with different values;
- a current open issue (#19, created 2026-03-26) claims the Gamma extrinsic is
  wrong but contains no supporting numbers or maintainer resolution;
- the paper acknowledges that calibration was performed once per device and
  may lose precision over the week-long collection period;
- the paper also acknowledges remaining synchronization latency/uncertainty
  for host-timestamped devices.

The same Gamma IMU-LiDAR matrix is used by the official GEODE FAST-LIO and
FAST-LIVO configurations, so the release is internally consistent. That is
not the same as an independent physical revalidation.

The Leica/prism warning in issue 16 does not invalidate the CG610 full-6-DoF
Inland GT. The maintainer explains that the Leica conversion approximates a
missing prism offset. Inland uses the separate GNSS/INS path and
`gamma2GT_gnss.py`.

## 5. Exact GLIM-to-GEODE frame chain

Use the convention:

```text
T_A_B maps coordinates in B into A.
```

Let:

```text
I = Gamma Xsens IMU body used by GLIM
L = Gamma Livox LiDAR body
B = Beta/GT evaluation body
W = arbitrary GLIM world frame
```

### 5.1 Gamma IMU to Gamma LiDAR

The official Gamma file calls the following matrix `T_IMU_LiDAR`:

```text
E = T_I_L =
  0.999620   0.027463   0.002445   0.049258
 -0.027517   0.999299   0.025398  -0.012500
 -0.001746  -0.025456   0.999674   0.026946
  0          0          0          1
```

The official FAST-LIO configuration uses the same rotation and translation as
its LiDAR-w.r.t.-IMU extrinsic.

GLIM's configuration variable has the opposite name/direction:

```text
T_lidar_imu = T_L_I = inverse(E)
p_lidar = T_L_I * p_imu
```

GLIM's optimized pose key is `T_W_I`, and its code computes:

```text
T_W_L = T_W_I * E.
```

For one GLIM IMU-body relative belief:

```text
Z_I,ij = inverse(T_W_I,i) * T_W_I,j
```

the same physical motion expressed as a Gamma LiDAR-body relative transform
is:

```text
Z_L,ij = inverse(E) * Z_I,ij * E.
```

This is a body-frame conjugation. It must not be replaced by rotating or
translating six pose components separately.

### 5.2 Gamma LiDAR to released GT body

The official `gamma2GT_gnss.py` defines:

```text
q_S (w,x,y,z) =
  (0.9998828, -0.0057758, 0.0022253, 0.0140019)
t_S = (0.0305, -0.5959, 0.0902) metres
```

and forms the homogeneous matrix `S`. For every input LiDAR pose it executes:

```text
T_W_B = T_W_L * inverse(S).
```

Therefore the corresponding relative edge is:

```text
Z_B,ij = S * Z_L,ij * inverse(S).
```

The ground-truth edge is:

```text
Z_GT,ij = inverse(T_GT,B(t_i)) * T_GT,B(t_j).
```

The GLIM edge error to evaluate is:

```text
e_G,ij = Pose3::Logmap(inverse(Z_GT,ij) * Z_B,ij)

rotation_error_rad = norm(e_G,ij[0:3])
translation_error_m = norm(e_G,ij[3:6])
```

Radians and metres remain separate.

### 5.3 Required implementation cross-check

Before using real data, a future evaluator must confirm numerically that the
two paths agree at `1e-12`:

```text
path A: convert both endpoint poses, then call between()
path B: conjugate the relative belief using the equations above
```

It must also verify that any existing GLIM-to-Kimera external/base-frame
conjugation is undone or shared exactly. A raw Kimera-base payload must not be
mistaken for an IMU- or LiDAR-body edge.

## 6. Timestamp and interpolation audit

### 6.1 Raw integrity

| Quantity | Flat Smooth | Inland Short Gamma |
|---|---:|---:|
| Bag duration | 82.147 s | 475.417 s |
| LiDAR scans | 821 | 4,753 |
| `/imu/data` messages | 8,209 | 47,540 |
| GT poses | 23,315 | 29,922 |
| GT duration | 82.080 s | 471.864 s |
| Non-unit quaternion maximum norm error | `2.22e-16` | `3.33e-16` |
| Negative adjacent GT timestamp steps | 0 | 1,987 |
| Exact duplicate GT timestamps | 0 | 0 |
| Maximum sorted GT gap | 0.0999 s | 0.8599 s |

Both bag LiDAR and IMU header streams are monotonic.

Flat GT covers all 821 LiDAR scan timestamps. Inland GT begins approximately
11.06 seconds after the bag; 4,643 of 4,753 LiDAR stamps lie inside GT
coverage.

The Inland file's rows are not in time order. Sorting is mandatory and must be
treated as source normalization, not time-offset estimation. The official
issue tracker had already reported pose timestamp problems and a maintainer
said corrected truth would be supplied. The file downloaded on 2026-08-07
still exhibits non-monotonic row order.

### 6.2 Fixed interpolation policy

For a pilot, use:

- stable sort by exact timestamp;
- reject exact duplicates with conflicting pose values;
- exact-match endpoints without interpolation;
- linear translation interpolation;
- shortest-arc normalized quaternion SLERP;
- no extrapolation;
- both edge endpoints must be valid;
- maximum allowed left-to-right GT bracket: **0.05 seconds**;
- log left gap, right gap, total bracket, and interpolation fraction.

This limit is conservative relative to the nominal 100 Hz outdoor GT but
does not pretend missing samples are present.

At a 0.05 s maximum bracket:

| Sequence | Valid LiDAR scan stamps | Fraction of all scans |
|---|---:|---:|
| Flat Smooth | 814 / 821 | 99.15% |
| Inland Short Gamma | 3,185 / 4,753 | 67.01% |

For Inland, 3,185 of the 4,643 timestamps inside GT coverage (68.60%) pass
this bracket rule. Relaxing the threshold after observing edge errors is
prohibited. Sensitivity tables may separately report 0.02, 0.10, and 0.20 s
without changing the primary result.

No time offset may be fitted by minimizing GLIM error. A nonzero offset needs
independent synchronization evidence.

## 7. Dataset eligibility

| Dataset | GT | Body/frame evidence | Timing | Status for Stage 2C |
|---|---|---|---|---|
| GEODE Inland Short Gamma | CG610 full 6-DoF, aligned to Beta | official LiDAR-input contract plus `gamma2GT_gnss.py` | usable after sorting, startup exclusion, and 0.05 s gap gate | `FULL_6DOF_VERIFIED` for a released-calibration pilot; degraded case |
| GEODE Flat Smooth Gamma | Vicon full 6-DoF, single Gamma device | official workflow implies direct LiDAR evaluation, but no explicit Vicon-target-to-LiDAR transform is published | excellent overlap; 99.15% pass 0.05 s gate | `EXTRINSIC_UNVERIFIED` for strict short-edge claims; useful sensitivity case |
| GEODE Offroad7 Gamma | no locally accessible GT | n/a | sensor bag valid | unusable for GT edge validation |
| GEODE Tunneling Tunnel1 Gamma | Leica position only | position-only | not audited further | `TRANSLATION_ONLY` |
| GEODE Bridge01 | CG610 full pose | Alpha single-device path, not yet locally audited | no local bag | GT-only candidate |
| Boreas audited windows | Applanix full 6-DoF | official lidar/Applanix chain already verified | verified | `FULL_6DOF_VERIFIED`; potential healthy case, but Stage 1 health not yet characterized |

`FULL_6DOF_VERIFIED` here means the released coordinate-chain contract is
sufficient to implement a transparent pilot. It does not mean the calibration
has zero uncertainty. The open Gamma-extrinsic report must be carried as a
known limitation and tested through non-fitted sensitivity analysis.

## 8. Why GEODE does not yet supply the healthy control

GEODE's purpose is to benchmark degeneracy. The paper's own scenario table
states:

```text
Flat Ground       2 translational + 1 rotational degeneracy
Inland Waterways  1 translational degeneracy
Offroad           2 translational + 1 rotational degeneracy
Metro Tunnels     1 translational + 1 rotational degeneracy
Urban Tunnel      1 translational degeneracy
Bridges           1 translational degeneracy
```

The website label `Easy` for Inland Short describes benchmark difficulty. It
does not mean its LiDAR geometry is healthy for DCReg.

Also, the frozen Stage 1 healthy profile is sensor/configuration specific.
The MID360 Dynamic01 profile must not be silently reused for Gamma/Livox Avia.
For relative GEODE health we need one of:

1. a genuinely healthy window in a Gamma sequence that passes every frozen
   bootstrap gate;
2. an independent healthy Gamma/Livox run from the same calibrated rig and
   GLIM registration configuration; or
3. a compatible offline Gamma profile created from such runs.

If no sample passes, `reference_ready=false` and relative health remains
unavailable. That is correct behavior. Absolute DCReg masks, condition ratios,
support, and matching quality remain available and may be evaluated without
inventing a healthy reference.

## 9. Exact next experiment

No active estimator change is required.

The next authorized passive pilot should:

1. create a GEODE Gamma GLIM runtime profile using the published
   `T_IMU_LiDAR`, inverted into GLIM's `T_lidar_imu` convention;
2. preconvert Livox scans deterministically while preserving ordered points,
   six line IDs, and nanosecond offsets;
3. run GLIM with Stage 1 `log_only`, G-to-K publication enabled, and K-to-G
   correction disabled;
4. show the Rerun visualization during the requested experiment;
5. run Flat Smooth and Inland Short Gamma separately;
6. record whether any session reference becomes ready without weakening the
   frozen gates;
7. evaluate Inland edges only where both GT endpoints pass the 0.05 s bracket
   policy;
8. compare endpoint-conversion and relative-conjugation paths at `1e-12`;
9. report absolute degeneracy separately from relative health;
10. run calibration sensitivity using declared perturbations, never a fitted
    transform;
11. preserve repeated CBS edge deduplication and block-bootstrap rules from
    the main Stage 2C-A memo.

The primary scientific unit remains the unique accepted belief/factor
instance, not every rolling-array republication.

## 10. Acceptance tests for the future evaluator

Minimum synthetic tests:

1. identity IMU-LiDAR and Gamma-GT transforms;
2. nontrivial rotations and lever arms for both transforms;
3. endpoint conversion equals edge conjugation at `1e-12`;
4. unknown left world alignment cancels;
5. omitting either body extrinsic changes the expected edge;
6. exact GT timestamp;
7. normalized SLERP inside a valid bracket;
8. quaternion sign flip gives identical interpolation;
9. raw out-of-order GT rows give the same result after stable sort;
10. conflicting duplicate timestamp is invalid;
11. one endpoint outside GT coverage invalidates the edge;
12. a 0.050000 s bracket passes and a larger bracket fails according to a
    precisely defined floating-point comparison;
13. raw G-to-K payload and GLIM endpoint-derived edge agree after the audited
    bridge frame conversion;
14. rotation and translation errors are reported separately;
15. GT is not used by Stage 1 reference construction.

## 11. Scientific boundaries

This audit does not authorize:

- health-to-metre or condition-ratio-to-centimetre regression;
- covariance inflation;
- active CBS weighting or gating;
- Kimera or GLIM factor modification;
- threshold tuning on the evaluation sequence;
- fitted time offsets or fitted extrinsics;
- interpreting repeated rolling publications as independent samples;
- calling GLIM-Kimera agreement ground truth.

Stage 1/2A health remains a scan-to-rolling-map observability indicator. The
evaluated CBS edge also contains IMU, fixed-lag smoothing, marginal, and local
factor effects. A relationship may be informative without being exact.

## 12. Decision and remaining blockers

The M3DGR-style unknown-body-frame blocker is substantially resolved for
GEODE Inland Short Gamma: the official maintainers explicitly define a
LiDAR-frame input and publish the Gamma-to-GT conversion.

Remaining blockers to a final Stage 2C scientific approval are:

1. no independently verified healthy Gamma/Livox reference run is available;
2. Stage 1 has not yet been run on the downloaded GEODE bags;
3. Inland GT contains non-monotonic row order and large gaps, leaving only
   67.01% of scans under the fixed 0.05 s primary gate;
4. the Gamma calibration has an unresolved public issue and release-file
   inconsistencies;
5. Flat Vicon truth lacks an explicitly published Vicon-target-to-LiDAR
   transform even though the official workflow implies direct comparison;
6. Boreas is frame-verified but not yet health-characterized as the healthy
   control, and it uses a different LiDAR/configuration.

**NO-GO:**

The currently downloaded GEODE data are sufficient for a transparent passive
degraded-sequence pilot and evaluator verification, but not yet for the final
healthy-versus-degraded, held-out full-6-DoF scientific claim. Proceed next
with the passive GEODE GLIM/DCReg runs and Rerun visualization; do not proceed
to active weighting.
