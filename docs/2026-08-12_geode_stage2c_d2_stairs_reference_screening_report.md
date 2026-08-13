# Stage 2C-D2: final GEODE Gamma healthy-reference screening

Date: 2026-08-12  
Final outcome: **C. NO HEALTHY REFERENCE**

## 1. Executive decision

`Stairs_Gamma` did not initialize a frozen Stage 1 session reference and did
not satisfy the independent offline-profile contract:

```text
online_reference_ready = false
offline_profile_qualified = false
accepted bootstrap candidates = 1 / 3865
required accepted candidates = 30
accepted interval duration = 0.0 s
```

No Gamma profile was exported. Short and Medium were not rerun in hybrid or
offline mode, and no relative-health/error analysis was performed. This is the
required conservative behavior: one individually acceptable scan is not a
normal-observability reference interval.

The planned public GEODE Gamma healthy-reference search is now exhausted:

- Offroad7: 0 / 1,809 accepted;
- Tunnel1: 26 / 2,056 accepted, below 30 and not continuous;
- Tunnel4: 30 individually accepted, but maximum gap 2.1002 s exceeds 0.5 s;
- Stairs: 1 / 3,865 accepted and no online reference.

Samples were not combined across recordings. Frozen thresholds, gates,
continuity rules, Stage 1 mathematics, Stage 2A/2B semantics, GLIM, VGICP,
CBS, Kimera, covariances, and optimization were not changed.

## 2. Preserved source state

Before work, `git status --short`, `git diff --stat`, and `git diff --check`
were recorded for every nested repository. All diff checks passed. Binary
patches and untracked-file archives were saved without altering the active
worktrees under:

```text
/home/yeranis/stage2c_d2_safety_20260812/
```

No reset, discard, stash, commit, or push occurred. Branches and HEADs remained
unchanged:

| Repository | Branch | HEAD |
|---|---|---|
| `glim` | `cbs-gtsam43-noetic` | `6c4189e117c61015a79001388debb27413c78fa6` |
| `glim_ros1` | `cbs-gtsam43-noetic` | `2dea515e9e38743b6fc73e7dfdf7e45d3ae5bf6f` |
| `cbsms` | `cbsms/gtsam-4.3-develop` | `873f60fa748df97bf9f5f7f9dcdb825b4f5ed30c` |
| `liorf` | `cbsms/gtsam-4.3-develop` | `4286f982694dda550d1343ff21679f8ec2e708ba` |
| `cbs` | `cbsms/gtsam-4.3-develop` | `994b1d6a5c05fb38dd1b0731c6430ec11d2f1ff0` |
| `Kimera-VIO` | `cbsms/gtsam-4.3-develop` | `d0b2a31adf17ced8995994373f6b286679e26799` |
| `Kimera-VIO-ROS` | `cbsms/gtsam-4.3-develop` | `09d7e17b5d27f97f622f285f0f23a61e6748278d` |

Stage 2C-D2 added only:

```text
src/cbsms/config/geode_stage2c_d2_stairs_provenance.json
src/cbsms/tools/geode_stage2c_d2_reference_audit.py
src/cbsms/tools/test_geode_stage2c_d2_reference_audit.py
src/cbsms/docs/2026-08-12_geode_stage2c_d2_stairs_reference_screening_report.md
```

The tool is offline reporting code. It cannot configure or invoke an
estimator, export a profile, or change a reference decision.

## 3. Official Stairs audit and provenance

Authoritative sources:

- GEODE site: <https://thisparticle.github.io/geode/>
- official repository: <https://github.com/PengYu-Team/GEODE_dataset>
- official Drive root:
  <https://drive.google.com/drive/folders/1hEn3sBAvQhSdUFnGMZCCv-W0Ynj2rWBs>
- Gamma inventory:
  <https://drive.google.com/drive/folders/1w3mi2iUHVHQq3kytuJV3p4BPFYN2V2gL>
- Stairs folder:
  <https://drive.google.com/drive/folders/11HUQeR8h0LKJhp3XOBeGM20FdwygoYMk>
- Gamma subfolder:
  <https://drive.google.com/drive/folders/162gErfmiDXYAnDQOpNXDFVpONfHvpc5k>
- exact bag:
  <https://drive.google.com/file/d/1Hnyv8XRM4zIcaB5UpDvcqeMe46otJ7jr/view>

Official inventory and downloaded file:

| Field | Value |
|---|---|
| Sequence | `Stairs_Gamma` / `stairs_gamma.bag` |
| Platform | Gamma handheld: Livox Avia + Xsens MTi-30 |
| Published size | 2.9 GB |
| Exact Drive size | 3,071,682,740 bytes |
| Published duration/distance | 390 s / 300 m |
| Bag duration | 390.752805 s |
| Bag SHA-256 | `932a5490497634d8e0497c0013c973d804dc892d37d1c0cab661e20141be514d` |
| GEODE repository commit | `c6e930623d4fed450d7fc50e16e3ffe0288b692b` |
| Gamma calibration SHA-256 | `19cda4d48f5b185d93293393846a387a2b093c67dc9ec3a646ed8f826b587dbe` |

The bag contains:

| Topic | Type | Count |
|---|---|---:|
| `/livox/lidar` | `livox_ros_driver/CustomMsg` | 3,907 |
| `/imu/data` | `sensor_msgs/Imu` | 39,071 |
| `/livox/imu` | `sensor_msgs/Imu` | 79,200 |
| left compressed camera | `sensor_msgs/CompressedImage` | 3,907 |
| right compressed camera | `sensor_msgs/CompressedImage` | 3,907 |

The frozen adapter uses `/livox/lidar` and Xsens `/imu/data`; cameras and the
Livox-internal IMU are not estimator inputs in this experiment.

The official repository documents Leica RTC360 ground-truth maps for stairway
sequences. It also states that PALoc alignment did not yield a Gamma Stairs
ground-truth trajectory because Gamma odometry drifted. Therefore Stairs has
no verified timestamped full-6DoF trajectory and was used only for reference
screening, never for an accuracy claim.

The machine-readable provenance file is:

```text
src/cbsms/config/geode_stage2c_d2_stairs_provenance.json
```

### Download integrity note

An initial `gdown --continue` attempt appended incompatible partial content.
That artifact was quarantined and never used. A clean non-resumed download
matched the exact Drive size and produced the source hash above. Derived files
from the failed attempt remain clearly named under
`stage2c_d2/stairs/*corrupt_resume*` and `converted/stairs_a.*`; none is cited
as a valid artifact.

## 4. Deterministic adapter and semantic audit

Adapter version: `geode_gamma_deskew_barrier_v2`  
Adapter source SHA-256:
`d16d2659a8b12e45702e436d81aa86e011f30ae07a31fe6067a2296ba4da0766`

Two clean independent conversions, `stairs_b` and `stairs_c`, were exactly
byte-identical:

| Artifact/property | Value |
|---|---|
| Converted bag SHA-256 | `566a597dab959d188875f85a4f1c962468c40eaef05d933b38eac8fba52b23b4` |
| Converted bag size | 1,703,556,230 bytes |
| Event/message manifest SHA-256 | `85a3682ecf3b612e451754021711fb92b15d715db37b5a25cebc50de35a5b9dc` |
| Event count | 42,977 |
| Processable PointCloud2 scans | 3,906 |
| Xsens IMU messages | 39,071 |
| Excluded terminal scan | 1 |
| Reason | no later IMU sample covers its maximum point time |

The authoritative serialized content, timestamps, event order, and semantic
hashes matched across conversions. Bag byte equality also passed.

Semantic comparison against the official CustomMsg source found zero payload
mismatches:

- ordered point records checked: 93,744,000;
- fields: `x,y,z,t,intensity,ring`, 18-byte point step;
- per-point offset range: 0 to 99,864,865 ns;
- line IDs 0 through 5: exactly 15,624,000 points each;
- ordered point-content SHA-256:
  `a43cd8f6c85b06b3ac64e796f2aa6a0073d5db89748beafbd615c7b8050197a0`;
- IMU serialized-content SHA-256:
  `679906ca963c5b4a823b0b05d63cd4594ec94e26e67b50834f6a857489cec829`;
- IMU acceleration norm range: 0.845560 to 34.832101;
- IMU angular-rate norm range: 0.000874 to 2.514874;
- full point bounding box min: `[0.0, -218.557999, -49.456001]` m;
- full point bounding box max: `[427.074005, 181.367996, 82.180000]` m;
- full point centroid: `[3.891718, 0.025290, 0.119544]` m.

The Livox `tag` byte remains deliberately absent because it is not part of the
frozen GLIM PointCloud2 schema and GLIM does not consume it. No point timing,
reflectivity, or line ID needed by GLIM was lost.

## 5. Frozen GLIM runtime

The run used the unchanged Gamma `log_only` configuration:

```text
config generation manifest SHA-256:
20ee041e9420015a23c82954e320f0fb7900e6d88ff300011b0d9d1347cdccc8

Stage 1 configuration fingerprint:
VGICP|1|9.9999999999999998e-13|1.0000000000000001e-09|1e-08|0.5|1
```

Properties:

- local GLIM LiDAR, IMU, bias-evolution, and fixed-lag marginal information;
- two VGICP resolutions, 0.5 m and 1.0 m;
- Stage 1 `dcreg_health.mode=log_only`;
- outgoing G-to-K observe-only publication;
- incoming topic `/geode/stage2c_b/no_k_to_g`;
- receive gate delayed by 999,999 s;
- no covariance inflation, active health policy, DCReg PCG, solver change, or
  K-to-G correction.

Run result:

| Metric | Result |
|---|---:|
| GLIM return code | 0 |
| Timed out | false |
| Wall time | 173.625634 s |
| Poses | 3,866 |
| Processed scan updates | 3,865 |
| G-to-K belief arrays | 3,864 |
| Incoming K-to-G messages | 0 |
| Incoming matches/injections | 0 / 0 |

Every processed update reported exactly four newly inserted local factors:

- one IMU-bias between factor;
- one IMU factor (9–30 IMU samples integrated per interval; no fallback);
- two LiDAR matching-cost factors, one at each VGICP resolution.

Thus the structured update log accounts for 15,460 post-initialization local
factor insertions: 3,865 bias, 3,865 IMU, and 7,730 LiDAR factors. Fixed-lag
marginalization occurred on 3,055 updates and marginalized 3,817 frames; it is
not counted as an extra `new_factors` entry. No K-to-G factor entered GLIM.

## 6. Condition and support results

Aggregate records: 3,865  
Valid aggregate Hessians: 3,864  
Linear-solve success: 3,865  
Registration-converged: 1,715

### Maximum Schur condition ratios per scan

| Statistic | Rotation | Translation |
|---|---:|---:|
| minimum | 1.000 | 1.000 |
| p10 | 2.163 | 3.037 |
| median | 10.569 | 6.388 |
| p90 | 72.023 | 10.751 |
| p95 | 74.639 | 14.160 |
| maximum | 1,202.316 | 163.413 |

Absolute-degenerate occupancy, with 3,865 aggregate rows as denominator:

| Group | Count | Fraction |
|---|---:|---:|
| rotational | 1,986 | 51.384% |
| translational | 465 | 12.031% |
| either / combined | 2,276 | 58.887% |

### Per-resolution support and matching

| Metric | 0.5 m | 1.0 m |
|---|---:|---:|
| median source points | 7,865 | 7,865 |
| median inliers | 7,832 | 7,856 |
| median inlier fraction | 0.994098 | 0.998348 |
| median initial cost | 764.687 | 1,601.464 |
| median final cost | 379.863 | 1,064.196 |
| converged rows | 1,715 | 1,715 |
| valid rows | 3,864 | 3,864 |

Support and matching cost are kept separate from directional conditioning and
are not combined into a score.

## 7. Reference decisions

### A. Frozen online Stage 1 decision

```text
online_reference_ready = false
first_reference_ready_frame = unavailable
first_reference_ready_timestamp = unavailable
bootstrap accepted = 1
adaptation accepted = 0
```

The sole accepted sample was:

| Field | Value |
|---|---:|
| frame | 762 |
| timestamp | 1705545294.0694501 |
| maximum rotation ratio | 3.446836 |
| maximum translation ratio | 5.988980 |
| source points | 11,079 |
| minimum factor inliers | 11,075 |
| minimum factor inlier fraction | 0.999639 |
| 0.5 m cost initial/final | 614.352 / 332.677 |
| 1.0 m cost initial/final | 1,435.532 / 1,272.426 |

It was recorded as `bootstrap_sample_accepted`, but one sample cannot
initialize the frozen session reference.

Rejection reason counts are mutually exclusive Stage 1 decisions for the
remaining 3,864 aggregate rows:

| Reason | Count |
|---|---:|
| `absolute_rotation_degeneracy` | 1,811 |
| rotation + translation absolute degeneration | 175 |
| `absolute_translation_degeneracy` | 290 |
| `registration_not_converged` | 1,140 |
| `unstable_condition_history` | 411 |
| `axis_alignment_confidence_low` | 16 |
| `translation_spectral_cluster` | 12 |
| `rotation_spectral_cluster` | 8 |
| `numerical_failure` | 1 |

### B. Independent offline-profile decision

```text
offline_profile_qualified = false
minimum accepted sample check = false
minimum accepted span check = false
maximum accepted gap check = false (no multi-sample interval exists)
Stage 1 reference-ready check = false
configuration fingerprint check = true
```

No profile was exported. Stairs was not combined with Tunnel4 or any other
recording. Relative health remains explicitly unavailable.

## 8. Conditional Short/Medium branch

The frozen task authorizes a Short/Medium hybrid rerun only after every Stairs
offline criterion passes. Since Stairs failed, this branch was not run.

Consequently:

- no Short or Medium estimator artifact changed;
- no reference lineage was created;
- no degraded Inland sample could adapt a profile;
- no relative-health/error plot was produced;
- existing verified absolute-conditioning/error results remain unchanged.

## 9. Passive runtime and queue behavior

Raw frozen Stage 1 statistics:

```text
GLIM_DCREG_MONITOR_STATS_ROW,full_log_only,3865,53.321515000,0.030915000
GLIM_DCREG_PRODUCER_STATS_ROW,3865,4101.632623000,2.754759000,8215.442133000,6.105063000
GLIM_DCREG_LOGGER_STATS_ROW,3865,3865,0,0,11595,1,9.381324000,0.133576000,1,0
GLIM_DCREG_WORKER_STATS_ROW,3865,627.668466000,538.812245000,87.353293000,0
```

The worker processed all 3,865 snapshots. The logger dropped zero snapshots
and zero rows, with maximum queue depth one. Stage 2A published 3,864 metadata
arrays with zero drops, zero producer waits, maximum health/descriptor queue
depth one, and no worker failure. All work remained passive.

## 10. Rerun visualization

The recording contains bounded point-cloud samples, the GLIM trajectory,
condition ratios, absolute-degeneracy flags, candidate acceptance, reference
readiness, per-resolution support/inliers/costs, and convergence. With only one
accepted candidate, the absence of a continuous accepted interval is visible
directly.

```text
stage2c_d2/stairs/rerun/stairs_gamma_stage2c_d2.rrd
size: 21,937,520 bytes
SHA-256: 2cbc2ebb38c0b0bb1eed2f2d45f4e39744108ded8e92ad61caed6eac444c358e
```

`rerun rrd verify` result: `1 file verified without error`.

## 11. Tests and build status

No compiled source changed, so no rebuild was necessary. Frozen binaries were
used. Commands included:

```text
docker exec cbsms_ws ... devel/scan_dcreg_diagnostics_test
docker exec cbsms_ws ... devel/lib/glim_ros/dcreg_edge_metadata_test
docker exec cbsms_ws ... devel/lib/kimera_vio_ros/dcreg_shadow_analysis_test
docker exec cbsms_ws ... python3 test_geode_gamma_offline_adapter.py
docker exec cbsms_ws ... python3 test_geode_stage2c_b_evaluator.py
docker exec cbsms_ws ... python3 test_geode_stage2c_c_analysis.py
docker exec cbsms_ws ... python3 test_geode_stage2c_d1_sanity_audit.py
docker exec cbsms_ws ... python3 test_geode_stage2c_d2_reference_audit.py
docker exec cbsms_ws ... python3 test_dcreg_shadow_stage2b_analysis.py
docker exec cbsms_ws ... python3 test_cbsms_experiment_runner.py
python3 test_glim_dcreg_calibration_protocol.py
python3 test_glim_dcreg_directional_audit.py
```

| Suite | Result |
|---|---:|
| Stage 1 DCReg | 39 / 39 pass |
| Stage 2A metadata | 31 / 31 pass |
| Stage 2B shadow | 35 / 35 pass |
| GEODE adapter | 3 / 3 pass |
| GEODE frame/GT evaluator | 21 / 21 pass |
| Stage 2C-C analysis | 20 / 20 pass |
| Stage 2C-D1 sanity | 9 / 9 pass |
| Stage 2C-D2 qualification | 6 / 6 pass |
| Stage 2B offline analysis | 7 / 7 pass |
| Experiment runner | 4 / 4 pass |
| Calibration protocol | 3 / 3 pass |
| Directional audit | 4 / 4 pass |
| **Total** | **182 / 182 pass** |

New D2 tests cover exactly 29 failing, exactly 30 continuous passing, a gap
above 0.5 s failing, online readiness remaining distinct from offline
qualification, prohibition on cross-run sample combination, and failure being
unable to authorize profile export, hybrid reruns, relative health, or GT use.

## 12. Artifacts

Primary valid artifacts:

```text
stage2c_d2/stairs/converted/stairs_b_manifest.json
stage2c_d2/stairs/converted/stairs_b_messages.jsonl
stage2c_d2/stairs/converted/stairs_b_semantic_audit.json
stage2c_d2/stairs/converted/stairs_c_manifest.json
stage2c_d2/stairs/converted/stairs_c_messages.jsonl
stage2c_d2/stairs/reference_screening/stairs_gamma_screen.json
stage2c_d2/stairs/reference_screening/stairs_gamma_detailed_audit.json
stage2c_d2/stairs/reference_screening/stairs_gamma_accepted_samples.csv
stage2c_d2/stairs/runs/stairs_gamma_log_only_v1/run_manifest.json
stage2c_d2/stairs/runs/stairs_gamma_log_only_v1/events/
stage2c_d2/stairs/runs/stairs_gamma_log_only_v1/parsed/glim_dcreg_health.csv
stage2c_d2/stairs/rerun/stairs_gamma_stage2c_d2.rrd
stage2c_d2/stairs/rerun/stairs_gamma_stage2c_d2_summary.json
```

Large bags and generated artifacts are outside source control and must not be
committed.

## 13. Replacement healthy-Gamma recording specification

The next step is external data acquisition, not another GEODE search and not a
gate change. Request or record one sequence with:

- the Livox Avia Gamma sensor and Xsens MTi-30;
- the same calibration, or a newly measured and independently verified full
  IMU-to-LiDAR calibration;
- raw Livox fields preserving `x,y,z`, reflectivity/intensity, all six line
  IDs, and per-point nanosecond offsets;
- original Xsens timestamps, acceleration units, angular-rate units, and
  documented hardware/clock synchronization;
- 60–180 seconds of ordinary, non-aggressive motion;
- a geometrically rich environment with intersecting walls, corners, multiple
  surface orientations, near and far depth, vertical structure, and irregular
  objects or vegetation;
- preferably timestamped full-6DoF CG610 or Vicon ground truth;
- an official, verified body-frame/extrinsic chain from the tracked GT body to
  Gamma IMU and LiDAR frames;
- documented timestamp convention, fixed offsets, GT rate, and coverage.

The recording is needed to construct a normal-observability profile under the
existing gates. It must not be used to fit a condition-to-error law, covariance
multiplier, or metric-error calibration.

## 14. Scientific limitations and boundary

- Stairs is a degenerate stairwell sequence, not a demonstrated healthy
  control.
- Its Leica map is not a verified timestamped Gamma 6-DoF trajectory.
- DCReg condition ratios measure local registration observability; support and
  matching cost describe different properties.
- None proves global correspondence or a correct optimizer basin.
- No same-Gamma healthy reference now exists, so Gamma relative health remains
  unavailable.
- Existing Short/Medium absolute-condition versus true-edge-error results are
  preliminary within-sequence associations, not a healthy-versus-degraded
  validation.
- Medium remains evidence that plausible displacement magnitude can coexist
  with a largely wrong local motion direction.
- No result in this task authorizes active CBS weighting, covariance inflation,
  factor gating, directional transport, or coordinator control.

## 15. Final outcome

**C. NO HEALTHY REFERENCE**

Stairs did not initialize a valid frozen Stage 1 reference. The planned GEODE
healthy Gamma reference search is formally exhausted. The exact next blocker
is the absence of an independently recorded, rich-geometry, same-Gamma
sequence that naturally supplies at least 30 temporally continuous accepted
samples under the frozen gates.
