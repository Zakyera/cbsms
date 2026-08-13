# Stage 2C-B: passive GEODE Gamma pilot and 6-DoF edge evaluator

Date: 2026-08-08  
Status: **PILOT PASS — degraded-sequence evaluator verified; final healthy-versus-degraded validation pending**

## 1. Scientific scope and result

This stage added only GEODE adapters, manifests, runtime profiles, offline
evaluation tools, tests, experiment wrappers, and passive Rerun output. It did
not change Stage 1, Stage 2A, Stage 2B, GLIM, VGICP, CBS, Kimera, any
covariance, any factor, or any optimizer behavior.

The pilot answers the requested questions as follows:

1. Frozen Stage 1 diagnostics run correctly on GEODE Gamma data.
2. Neither downloaded Gamma sequence naturally passed the frozen healthy
   session-reference gates. `reference_ready=false` and relative health stayed
   unavailable; no value of 1 was substituted.
3. The released Inland Waterways frame chain and ground truth produce 2,164
   valid, deduplicated, full-6-DoF short edges at the fixed 0.05 s GT rule.
4. Absolute DCReg degeneration and true GLIM edge errors can be described
   together, but this particular run does not provide a balanced healthy versus
   degraded comparison: degeneration occupies nearly all valid intervals.
5. No compatible Gamma healthy profile was created or imported.

This is not a ratio-to-centimetre calibration, a covariance model, a
probability model, or an active trust policy.

## 2. Repository preservation and source state

Safety patches captured before edits are under:

```text
stage2c_b/safety_before_20260807/
```

No repository was reset, stashed, committed, or pushed. All pre-existing user
changes were preserved. Branches and HEADs before and after Stage 2C-B are
unchanged:

| Repository | Branch | HEAD |
|---|---|---|
| `glim` | `cbs-gtsam43-noetic` | `6c4189e117c61015a79001388debb27413c78fa6` |
| `glim_ros1` | `cbs-gtsam43-noetic` | `2dea515e9e38743b6fc73e7dfdf7e45d3ae5bf6f` |
| `liorf` | `cbsms/gtsam-4.3-develop` | `4286f982694dda550d1343ff21679f8ec2e708ba` |
| `cbs` | `cbsms/gtsam-4.3-develop` | `994b1d6a5c05fb38dd1b0731c6430ec11d2f1ff0` |
| `Kimera-VIO` | `cbsms/gtsam-4.3-develop` | `d0b2a31adf17ced8995994373f6b286679e26799` |
| `Kimera-VIO-ROS` | `cbsms/gtsam-4.3-develop` | `09d7e17b5d27f97f622f285f0f23a61e6748278d` |
| `cbsms` | `cbsms/gtsam-4.3-develop` | `873f60fa748df97bf9f5f7f9dcdb825b4f5ed30c` |

The existing dirty states in `glim_ros1`, `liorf`, Kimera, and `cbsms` are from
the preserved Stage 1/2A/2B work. Stage 2C-B itself changes only the following
new `cbsms` files:

```text
config/geode_gamma_stage2c_b_profile.json
config/geode_stage2c_b_provenance.json
config/geode_stage2c_b_sensitivity_plan.json
tools/audit_geode_gamma_conversion.py
tools/geode_export_pointcloud_samples.py
tools/geode_extract_stage2a_metadata.py
tools/geode_gamma_offline_adapter.py
tools/geode_stage2c_b_evaluator.py
tools/geode_stage2c_b_flat_rerun.py
tools/geode_stage2c_b_gtsam_check.cpp
tools/prepare_geode_gamma_glim_config.py
tools/run_geode_stage2c_b.py
tools/test_geode_gamma_offline_adapter.py
tools/test_geode_stage2c_b_evaluator.py
docs/2026-08-08_geode_stage2c_b_pilot_report.md
```

Large bags, manifests, run products, and `.rrd` recordings remain untracked
under `stage2c_b/` and are not intended for Git.

## 3. Official provenance

The machine-readable manifest is
`config/geode_stage2c_b_provenance.json`. Stable local copies of the immutable
official files are under `stage2c_b/provenance/`.

| Source | Commit/file SHA-256 |
|---|---|
| GEODE dataset repository | `c6e930623d4fed450d7fc50e16e3ffe0288b692b` |
| GEODE evaluation repository | `1f008a7249e36393a1752622de50660b77b5b7f4` |
| `gamma_config.yaml` | `19cda4d48f5b185d93293393846a387a2b093c67dc9ec3a646ed8f826b587dbe` |
| `gamma2GT_gnss.py` | `258452c5439128d2a375dd7bf8b2b648b7f4550adbfb39a32b981e3604bca01e` |
| FAST-LIO Gamma/Carol YAML | `5ce02d44635225ccf1fd4ef6780cfe7b4331784bc40fea018f9704b833383cb3` |
| FAST-LIVO Gamma/Carol YAML | `6ac2d8aa3369ebadaf50982615591a8db8bca6f7eca69d7b8e272aac3846a682` |
| GEODE issue 16 comments, including maintainer LiDAR-frame statement | `07f1e3400f4f6ec279a1ae0cfea64d1f8c92bd3a5d96950922899e6c495f49ee` |
| Flat bag | `ea34c071e67aad9c5935176de392016029fea15f553520c83e220ab445b04889` |
| Flat GT | `1c05a7a0f0b0b0d9d3fd5132c48821413bd8e3ea5c55e0d3e9beee04a9745d86` |
| Inland bag | `bd58ef1435cbfb41c2996f0d5121d96378d5aca8a6109b3e5b707a312ab91c00` |
| Inland GT | `e46b0be316e350624775ebefb06b6c17ddae1dcbdd8c5e11c0450030962f7cc5` |

No unofficial calibration, trajectory alignment, estimated time offset, or
locally fitted transform was used.

## 4. Deterministic Gamma adapter

`geode_gamma_offline_adapter.py` uses a test-only, single-thread dispatch
schedule matching `glim_rosbag`'s deterministic deskew barrier:

1. preserve every message header and serialized sensor payload;
2. emit one processable cloud;
3. emit IMUs through the first sample at or after that cloud's maximum point
   timestamp;
4. only then emit the next cloud;
5. assign monotonically increasing rosbag record times to encode this dispatch
   schedule, while leaving measurement timestamps untouched.

This distinction matters because GEODE's next scan header may precede the first
IMU sample that covers the previous scan's final point. Simple header-time
sorting fails the deterministic GLIM deskew barrier.

The final Flat source cloud has no IMU coverage through its final point. It is
preserved by the source bag/hash but explicitly excluded from the processable
test bag. No IMU was fabricated and no partially deskewed scan was sent to
GLIM.

### Repeated-conversion hashes

Independent A/B conversions were byte-identical for both sequences.

| Sequence | Events | Event/content SHA-256 | Output bag SHA-256 | Excluded uncovered scans |
|---|---:|---|---|---:|
| Flat | 9,029 | `5b0107a13ec3d3a0849e7795cd43cc7a718c14380bf1b439849e81a03b34ecf0` | `116cda4f3f5b6b639f5c532f2e0178d0cbece5418674420bf005c1df7cbe2068` | 1 terminal scan |
| Inland | 52,293 | `6bce724689f5da9902c60a85d6de4f27427f23385de7f101bd78eafc70ad8e8b` | `7cc8d95d5dc3179368b048332b374bc92aadbf8a5027d52ec9dd06913f220bcd` | 0 |

### Sensor preservation

For every processable scan, ordered packed records were identical for
`x,y,z,t,intensity,ring`; `t` remains per-point nanoseconds and all six line
IDs are preserved. IMU messages are serialized byte-for-byte unchanged.

| Quantity | Flat | Inland |
|---|---:|---:|
| Converted clouds | 820 | 4,753 |
| IMUs | 8,209 | 47,540 |
| Converted points | 19,680,000 | 114,072,000 |
| Offset-time range | 0–99,867,785 ns | 0–99,863,865 ns |
| Points per line ID | 3,280,000 | 19,012,000 |
| IMU acceleration-norm range | 6.230–16.645 m/s² | 8.675–10.890 m/s² |
| IMU angular-rate-norm range | 0.000849–1.590 rad/s | 0.000318–0.119 rad/s |
| Semantic payload mismatch count | 0 | 0 |

The Livox `tag` field remains omitted because the frozen GLIM converter does
not consume it. Rosbag record-time changes are recorded separately as dispatch
scheduling changes and are not measurement-semantic changes.

## 5. Gamma GLIM runtime profile

The profile is `config/geode_gamma_stage2c_b_profile.json` (SHA-256
`a129572cb13361994e7d7b4dc779aaed76135ffe3814b0152046b17ff4d57f2a`).

The same registration configuration was used for Flat and Inland. Only input
paths changed. Runtime properties were:

- Gamma Livox points: `/geode/gamma/points`;
- Xsens IMU: `/imu/data`, acceleration already in m/s², `acc_scale=1`;
- preprocessing and odometry threads: 1;
- local LiDAR matching, IMU, and fixed-lag marginal prior remain;
- incoming K→G topic is an unused `/geode/stage2c_b/no_k_to_g` topic;
- CBS bridge mode is `observe_only`;
- G→K direct short-horizon belief publication remains enabled;
- Stage 1 is `log_only`; Stage 2A is passive metadata-only;
- Stage 2B, health-aware covariance, active weighting, PCG mitigation, and
  Rerun estimator extensions are disabled.

Generated runtime config manifests:

```text
log_only  547cde14218092f985dad03160b9bc3339c616675458b894685d8cad438cb5dc
off       f0bc380c789732227549650572b8957e810b907e02b19c3e3692ae06c4639ea2
```

## 6. Exact frame chain

The convention is `T_A_B`: coordinates in B mapped into A.

The released Gamma transform is `E = T_I_L`:

```text
 0.999620  0.027463  0.002445  0.049258
-0.027517  0.999299  0.025398 -0.012500
-0.001746 -0.025456  0.999674  0.026946
 0         0         0         1
```

The released six-decimal rotation is projected once to the nearest proper SO(3)
matrix. Raw orthogonality error is `1.6553e-6`; the projection change is
`8.2765e-7` in Frobenius norm. GLIM receives `T_L_I = inverse(E)` as:

```text
translation: [-0.0495361864003073, 0.0118243760537580, -0.0267401916196076]
quaternion xyzw:
[0.0127157365308847, -0.00104793797416711,
 0.0137474138704330, 0.999824094769852]
```

The official Carol/Gamma-to-Beta/GT transform `S` uses:

```text
q wxyz = [0.9998828, -0.0057758, 0.0022253, 0.0140019]
t m    = [0.0305, -0.5959, 0.0902]
```

For a GLIM IMU-body edge:

```text
Z_I = inverse(T_W_I_i) * T_W_I_j
Z_L = inverse(E) * Z_I * E
Z_B = S * Z_L * inverse(S)
Z_GT = inverse(T_GT_B(t_i)) * T_GT_B(t_j)
e_G = Pose3::Logmap(inverse(Z_GT) * Z_B)
```

Rotation error is `norm(e_G[0:3])` in radians and translation error is
`norm(e_G[3:6])` in metres. They are never combined.

The standalone GTSAM fixture reports:

```text
endpoint versus edge path       8.185836401376032e-16
G→K Logmap payload round-trip    1.712375058312158e-16
left world alignment cancellation 1.755471590204187e-15
quaternion sign invariance       1.694065894508601e-19
omit-E discrepancy               7.598466780201231e-02
omit-S discrepancy               4.195492685515602e-01
```

Sampled real endpoint conversions agree with relative conjugation at maximum
`6.929e-13`, below the accepted `1e-12` limit. The G→K payload is generated by
`Pose3::Logmap(from_pose.between(to_pose))` in the GLIM IMU-body frame, so the
evaluator does not apply the Kimera bridge conjugation or double-transform it.

## 7. Flat Surfaces Smooth results

The complete processable sequence ran to GLIM return code 0.

| Quantity | Result |
|---|---:|
| Aggregate Stage 1 samples | 788 |
| Accepted session-reference candidates | 0 |
| Reference-ready samples | 0 |
| Absolute rotational-degenerate samples | 200 |
| Absolute translational-degenerate samples | 779 |
| Median maximum rotation condition ratio | 8.695 |
| Median maximum translation condition ratio | 12.688 |
| p95 maximum rotation condition ratio | 12.337 |
| p95 maximum translation condition ratio | 20.602 |

Reference rejection reasons were:

```text
absolute_translation_degeneracy                         580
absolute_rotation_degeneracy+absolute_translation_degeneracy 199
registration_not_converged                                8
absolute_rotation_degeneracy                               1
```

This sequence did not become a healthy Gamma reference and was not forced to
do so. Its Vicon trajectory is included only as a separately named,
frame-uncertain visual trace because the Vicon-target-to-Gamma-LiDAR extrinsic
remains unverified. No Flat 6-DoF edge-error claim is made.

## 8. Inland Waterways health/reference results

The complete sequence ran to GLIM return code 0.

| Quantity | Result |
|---|---:|
| Aggregate Stage 1 samples | 4,616 |
| Valid LiDAR-health samples | 4,297 |
| No-LiDAR-support samples | 319 |
| Accepted session-reference candidates | 0 |
| Reference-ready samples | 0 |
| Absolute rotational-degenerate samples | 3,664 |
| Absolute translational-degenerate samples | 4,138 |
| Median maximum rotation condition ratio | 47.998 |
| Median maximum translation condition ratio | 86.050 |
| p95 maximum rotation condition ratio | 200.598 |
| p95 maximum translation condition ratio | 324.428 |

The frozen reference rejection reasons were 3,606 joint absolute rotation and
translation degeneracy, 531 translation degeneracy, 58 rotation degeneracy,
319 numerical/no-support cases, 58 non-converged registrations, 40 unstable
history cases, and four ambiguity/alignment cases. Relative health was
unavailable for every evaluated edge, exactly as required.

## 9. Inland GT parsing and coverage

The deterministic parser uses Decimal timestamps, stable sorting with source
row provenance, normalized quaternions, exact timestamp use, linear
translation interpolation, shortest-arc normalized SLERP, no extrapolation,
no time fitting, and no trajectory alignment.

| Quantity | Result |
|---|---:|
| Input/normalized GT rows | 29,922 / 29,922 |
| Source backward timestamp steps | 1,987 |
| Conflicting/harmless duplicate timestamps | 0 / 0 |
| Non-finite rows | 0 |
| Coverage | 1706079753.4648209–1706080225.3289666 |
| Median GT gap | 0.0100303 s |
| p95 GT gap | 0.0564216 s |
| Maximum GT gap | 0.8598757 s |
| GT gaps over 0.05 s | 1,691 |

Equality at an exact 0.05 s total bracket passes. Both endpoints must pass.

## 10. Edge selection and primary 0.05 s evaluation

Rolling republications are not independent samples. In this local-only pilot,
where no Kimera factor acceptance exists, the frozen fallback policy selects
the last valid publication for each stable sender edge.

| Quantity | Result |
|---|---:|
| Belief publication rows | 27,636 |
| Republication rows removed | 23,032 |
| Unique stable sender edges | 4,604 |
| GT-valid unique edges at 0.05 s | 2,164 |
| GT-invalid unique edges | 2,440 |
| Outside GT coverage/startup | 79 |
| GT bracket above 0.05 s | 2,361 |
| Relative-health-available edges | 0 |

Asynchronous rosbag callback recording placed some metadata before its legacy
belief in file order. The offline extractor therefore joins by Stage 2A
publication sequence and independently recomputes each exact 128-bit payload
digest. All 4,612 arrays and 27,636 entries matched; digest mismatches,
unsupported sequences, duplicates, and entry-count mismatches were all zero.
No belief or metadata bytes were changed.

## 11. Primary edge-error results

Across the 2,164 GT-valid unique edges:

| Error | Minimum | Median | Mean | p95 | Maximum |
|---|---:|---:|---:|---:|---:|
| Rotation [rad] | 0.000186 | 0.002902 | 0.055893 | 0.034918 | 3.053800 |
| Translation [m] | 0.025800 | 0.454910 | 4.273905 | 0.530225 | 1263.908770 |

GLIM undergoes a large late-run divergence: the first edge above 10 m occurs
at sender time `1706080183.0055969`, and 82 GT-valid edges exceed 10 m. These
outliers are retained; they are not used to fit calibration, a time offset, or
a health threshold.

### Absolute-degeneracy stratification

| Group | Edges | Rotation median / p95 [rad] | Translation median / p95 [m] |
|---|---:|---:|---:|
| Rotationally absolute-degenerate | 1,925 | 0.002787 / 0.010363 | 0.453865 / 0.506885 |
| Rotationally non-degenerate, valid denominator | 169 | 0.003301 / 0.041136 | 0.453435 / 0.474083 |
| Rotational absolute state unavailable | 70 | 1.266210 / 2.485994 | 40.366867 / 471.838768 |
| Translationally absolute-degenerate | 2,062 | 0.002838 / 0.012928 | 0.453870 / 0.506408 |
| Translationally non-degenerate, valid denominator | 32 | 0.003383 / 0.020794 | 0.438341 / 0.478432 |
| Translational absolute state unavailable | 70 | 1.266210 / 2.485994 | 40.366867 / 471.838768 |

The worst errors concentrate in numerically unavailable/no-support intervals,
not in a balanced absolute-degenerate versus healthy comparison. Since 98.47%
of eligible translation-mask edges are already degenerate, these tables do not
establish a useful binary separation threshold.

Exploratory Spearman associations on deduplicated unique edges, without an
independence or causality claim, were:

```text
worst rotational condition ratio vs rotational error     rho = 0.103 (n=2094)
worst translational condition ratio vs translation error  rho = 0.376 (n=2094)
source-point count vs translation error                   rho = -0.227 (n=2164)
final matching cost vs translation error                  rho = -0.061 (n=2164)
```

These are descriptive rank associations only. No regression or conversion to
metres was fitted.

## 12. GT interpolation sensitivity

The 0.05 s rule was fixed before primary evaluation. Secondary gating tables
change coverage as expected:

| Maximum bracket | Valid edges | Rotation median / p95 [rad] | Translation median / p95 [m] |
|---:|---:|---:|---:|
| 0.02 s | 1,688 | 0.002892 / 0.029923 | 0.453524 / 0.524387 |
| **0.05 s primary** | **2,164** | **0.002902 / 0.034918** | **0.454910 / 0.530225** |
| 0.10 s | 3,296 | 0.002947 / 0.042098 | 0.454395 / 0.577276 |
| 0.20 s | 4,196 | 0.002955 / 0.611796 | 0.455197 / 30.409580 |

The calibration-sensitivity plan is frozen in
`config/geode_stage2c_b_sensitivity_plan.json`. It intentionally contains no
perturbation magnitudes because the released sources and public issue provide
no defensible quantitative uncertainty bounds for E or S. Only exact
released-calibration results are reported; calibration sensitivity remains
pending rather than using arbitrary or fitted perturbations.

## 13. Deterministic no-op validation

Full Flat runs compared health `off` with frozen Stage 1 `log_only` using the
same deterministic bag and one-thread profile.

The following were byte-identical, which is stronger than the `1e-12`
requirement:

```text
consumed input event CSV
GLIM pose CSV
G→K belief CSV, including means and covariances
GLIM_POSE_STAGE_ROW stream
GLIM_TARGET_UPDATE_ROW stream
CBS_ODOM_OUTGOING_ROW stream
CBS_ODOM_RELATIVE_COVARIANCE_ROW stream
```

Counts were also identical: 9,029 input events, 789 GLIM trajectory rows, 787
belief arrays, 4,707 rolling belief rows, and 4,707 outgoing relative
covariance rows. No K→G message or factor was observed. The differing
`GLIM_SCAN_HEALTH_ROW` stream is the expected passive diagnostic addition.

## 14. Runtime, queue, and storage

| Quantity | Flat log-only | Inland log-only |
|---|---:|---:|
| Estimator wrapper wall time | 40.133 s | 39.352 s |
| Health samples | 788 | 4,616 |
| Health worker total | 136.757 ms | 625.451 ms |
| Health snapshot/log drops | 0 / 0 | 0 / 0 |
| Maximum health/logger queue depth | 1 | 1 |
| Stage 2A metadata arrays | 787 | 4,612 |
| Stage 2A descriptor copy mean | 1.073 µs | 1.120 µs |
| Stage 2A worker mean/array | 99.455 µs | 66.283 µs |
| Stage 2A producer waits | 0 | 0 |
| Stage 2A drops/orphans/cache anomalies | 0 | 0 |
| Stage 2A maximum health/descriptor/pending queues | 1/1/1 | 1/1/1 |
| Stage 2A maximum cached records | 8 | 12 |
| Belief+metadata bag | 9.1 MiB | 54 MiB |
| Metadata bandwidth | 96.4 kB/s | 94.3 kB/s |
| Complete run directory | 33 MiB | 171 MiB |

The Flat health-off control took 38.530 s; log-only added 1.602 s wall time
(4.16%) in this short single-thread harness while estimator outputs remained
byte-identical. There was no estimator-thread file I/O, no producer wait, and
no dropped diagnostic.

## 15. Rerun recordings

Both recordings pass `rerun rrd verify`.

| Recording | Size | SHA-256 |
|---|---:|---|
| `stage2c_b/runs/flat_smooth_gamma_log_only_v1/flat_smooth_gamma_stage2c_b.rrd` | 3.0 MiB | `c42e3da3c23795bdb7bc2f2332689b09286625a3e574ef44a9b74784816c65b6` |
| `stage2c_b/runs/inland_waterways_short_gamma_log_only_v1/inland_waterways_short_gamma_stage2c_b.rrd` | 1.4 MiB | `03b53105244b9fe0c0293c37d6409da8c714e71b28d352c1ac600776b2c83f38` |

Inland contains bounded LiDAR samples, GLIM and GT trajectories, GT-valid edge
errors, interpolation gaps, DCReg absolute masks/condition summaries,
reference readiness, support, and matching costs. Its one-time world alignment
is visualization-only and never enters edge evaluation. Flat contains bounded
LiDAR samples, GLIM trajectory, separately named unverified Vicon trajectory,
condition ratios, masks, reference state, support/costs, and unique CBS edge
durations. It deliberately contains no Flat edge-error trace.

## 16. Build and tests

Build:

```text
catkin build liorf glim glim_ros cbs cbsms kimera_vio kimera_vio_ros \
  --no-deps --no-status --summarize -j8 -p2
```

Result: all 11 scheduled packages and dependencies succeeded, with no build
warnings.

Tests:

```text
devel/scan_dcreg_diagnostics_test
devel/lib/glim_ros/dcreg_edge_metadata_test
python3 src/m3dgr_tools/test/test_offline_livox_conversion.py
python3 src/cbsms/tools/test_geode_gamma_offline_adapter.py
python3 src/cbsms/tools/test_cbsms_experiment_runner.py
python3 src/cbsms/tools/test_glim_dcreg_calibration_protocol.py
python3 src/cbsms/tools/test_glim_dcreg_directional_audit.py
python3 src/cbsms/tools/test_geode_stage2c_b_evaluator.py
```

Results:

- frozen Stage 1: 39/39 passed;
- frozen Stage 2A: 31/31 passed;
- original Livox converter: 2/2 passed;
- GEODE deskew adapter: 3/3 passed;
- existing cbsms reporting: 4/4 + 3/3 + 4/4 passed;
- GEODE frame/GT/evaluator: 21/21 passed;
- standalone GTSAM frame/payload fixture: passed;
- Flat deterministic no-op integration: byte-identical pass;
- both Rerun recordings: verified.

## 17. Acceptance and remaining blocker

### PILOT PASS

The passive GEODE degraded-sequence adapter and released-calibration evaluator
are verified. Point timing and line IDs are preserved, both Gamma sequences run
without estimator source changes, the E/S chain is validated below `1e-12`,
Inland supplies a nontrivial set of strict GT edges, errors remain split into
radians and metres, GT stays offline, and all active estimator/covariance paths
remain untouched.

### Exact blocker to the final claim

No downloaded Gamma sequence provides a frozen-gate-compatible healthy
reference. Flat is mostly translationally degenerate and has an unverified
Vicon target extrinsic; Inland is heavily degenerate and later diverges. Thus
this pilot cannot yet make the final held-out statement that health separates
healthy from degraded GLIM edge accuracy across independent Gamma sequences.

The next scientifically defensible requirement is an independently verified
healthy Gamma/Livox sequence using the same E, sensor, timing, and unchanged
GLIM configuration that naturally passes every frozen Stage 1 reference gate.
Calibration uncertainty bounds for E and S would also be needed for a
non-arbitrary calibration-sensitivity study.

No active weighting, covariance inflation, factor gating, directional
transport, or health-to-error model should begin from this pilot alone.
