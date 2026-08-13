# Stage 2B passive receiver-side shadow analysis: implementation and validation report

Date: 2026-08-05

Status: **implementation complete, scientifically preliminary, strict regression approval blocked**.

Stage 1 and Stage 2A remained frozen. No commit or push was performed.

## 1. Executive result

Stage 2B now passively joins each exact GLIM-to-Kimera legacy CBS belief
instance with its Stage 2A edge-health metadata and the exact Kimera state pair
selected by the active receiver. It publishes/logs the frame-consistent raw
relative-motion disagreement

```text
r = Pose3::Logmap(Z_sender_base.inverse() * Z_receiver)
```

in `[rotation, translation]` order, with rotation and translation norms kept
separate.

The implementation does not change any GLIM state, VGICP operation, Stage 1
health value, Stage 2A metadata value, CBS mean, CBS covariance, robust kernel,
factor decision, Kimera factor, Kimera optimization, or estimator output by
design. It emits no trust weight, covariance multiplier, threshold decision,
or feedback command.

The online infrastructure worked on four real sequences with exact digest
association and no producer waits or diagnostic drops. The final isolated
20-second receiver run processed 168 belief arrays, published 168 shadow
arrays/993 entries, and used at most queue depths 2/2/24 out of 256.

The scientific answer is not yet “health predicts accuracy.” Dynamic01,
Outdoor01, and Wheel-float01 do not currently have a verified full 6-DoF
ground-truth-body to Kimera-base transform. Health-versus-disagreement
associations were dataset-dependent and sometimes had the opposite sign from
the simple expected relationship. Stage 2A metadata is useful for exact
stratification and failure analysis, but current evidence is insufficient for
active weighting.

The strict `1e-12` no-op gate did not pass. The corrected receiver replay
showed the same active factor stream with Stage 2B disabled and enabled, but
Kimera outputs differed by up to `1.70e-9 m` and `3.85e-10 rad`. The same
numerical envelope occurred between two disabled/off runs of the same binary.
This is evidence of receiver replay/numerical repeatability, not evidence of a
Stage 2B control effect, but the task explicitly requires exact `1e-12`.
Therefore Stage 2B is **not formally regression-approved** under the stated
criterion.

## 2. Preserved Git/source state

Safety patches were written before editing under:

```text
stage2b_implementation/safety_before_edits_20260805/
```

No reset, checkout, stash, commit, or push was performed. Existing Stage 1,
Stage 2A, and user changes were preserved.

| Repository | Branch | HEAD before and after |
|---|---|---|
| `glim` | `cbs-gtsam43-noetic` | `6c4189e117c61015a79001388debb27413c78fa6` |
| `glim_ros1` | `cbs-gtsam43-noetic` | `2dea515e9e38743b6fc73e7dfdf7e45d3ae5bf6f` |
| `liorf` | `cbsms/gtsam-4.3-develop` | `4286f982694dda550d1343ff21679f8ec2e708ba` |
| `cbs` | `cbsms/gtsam-4.3-develop` | `994b1d6a5c05fb38dd1b0731c6430ec11d2f1ff0` |
| `Kimera-VIO` | `cbsms/gtsam-4.3-develop` | `d0b2a31adf17ced8995994373f6b286679e26799` |
| `Kimera-VIO-ROS` | `cbsms/gtsam-4.3-develop` | `09d7e17b5d27f97f622f285f0f23a61e6748278d` |
| `cbsms` | `cbsms/gtsam-4.3-develop` | `873f60fa748df97bf9f5f7f9dcdb825b4f5ed30c` |

Final `git diff --check` passed in every involved repository. The worktrees
remain intentionally dirty because Stage 1, Stage 2A, and Stage 2B are not
committed. Pre-existing Python cache directories were preserved.

## 3. Files added or changed for Stage 2B

### `liorf`

- `msg/DcregBeliefShadowAnalysis.msg`: one exact belief-instance report.
- `msg/DcregBeliefShadowAnalysisArray.msg`: versioned array output.
- `CMakeLists.txt`: registers only the new optional messages; legacy messages
  are unchanged.

### `Kimera-VIO`

- `include/kimera-vio/backend/VioBackend-definitions.h`: additive immutable
  identity and exact active-match diagnostic structures.
- `include/kimera-vio/backend/VioBackend.h`
- `include/kimera-vio/backend/VioBackendModule.h`
- `include/kimera-vio/pipeline/Pipeline.h`
- `src/backend/VioBackend.cpp`
- `src/backend/VioBackendModule.cpp`

These files add a diagnostics-only observer for the exact active receiver
state-selection result. The existing matching and factor code remains
authoritative.

### `Kimera-VIO-ROS`

- `include/kimera_vio_ros/CbsRelativeFrameConversion.h`
- `src/CbsRelativeFrameConversion.cpp`
- `include/kimera_vio_ros/DcregSha256.h`
- `src/DcregSha256.cpp`
- `include/kimera_vio_ros/NonBlockingSpscQueue.h`
- `include/kimera_vio_ros/DcregShadowAnalysis.h`
- `src/DcregShadowAnalysis.cpp`
- `include/kimera_vio_ros/DcregShadowAnalyzer.h`
- `src/DcregShadowAnalyzer.cpp`
- `test/DcregShadowAnalysisTest.cpp`
- `include/kimera_vio_ros/KimeraVioRos.h`
- `src/KimeraVioRos.cpp`
- `launch/kimera_vio_ros.launch`
- `launch/kimera_vio_ros_m3dgr_mono.launch`
- `CMakeLists.txt`

### `glim_ros1`

Only optional launch argument pass-through was added for the Stage 2B Kimera
configuration. Frozen Stage 2A publisher code was not mathematically changed.

### `cbsms`

- `config/kimera_dcreg_shadow_stage2b.example.yaml`
- `tools/dcreg_shadow_stage2b_analysis.py`
- `tools/test_dcreg_shadow_stage2b_analysis.py`
- `docs/2026-08-05_dcreg_belief_shadow_stage2b.md`
- this report.
- `tools/cbsms_experiment.py`: creates the run's `parsed/` directory before
  launch so the passive worker can open its dedicated CSV; estimator behavior
  is unchanged.

### Test-only artifacts outside source repositories

- deterministic merged bags and manifests;
- single-thread Kimera parameter overlay;
- absent/disabled/enabled replay runners and comparisons;
- real-run analysis summaries, SVGs, and `.rrd` recordings.

## 4. Audited active receiver path

The legacy subscriber remains in
`Kimera-VIO-ROS/src/KimeraVioRos.cpp`:

1. `initializeHeadlessCbsBeliefBridge` creates the active legacy subscriber.
2. `poseOdomBeliefInCallback` receives the unchanged
   `liorf/pose_odom_belief_array`.
3. It applies the existing receive-delay gate and ignores self beliefs.
4. It reconstructs the relative motion with `Pose3::Expmap` and the right-local
   6x6 covariance in rotation-then-translation order.
5. It obtains the configured external/body transform from TF.
6. It calls the shared pure frame-conversion helper.
7. It buffers the converted `ExternalOdometryBelief` through the existing
   active path.
8. Only after the active buffer call returns does it enqueue an immutable copy
   for Stage 2B.

Kimera's active historical state resolver is
`VioBackend::resolveExternalBeliefStamp` in
`Kimera-VIO/src/backend/VioBackend.cpp`. It searches the retained keyframe
timestamp map only through the current frame, chooses minimum absolute
timestamp error, retains the earlier/lower key on an exact tie, then applies
the existing timestamp and interval-duration gates. Existing ranking resolves
multiple candidates targeting the same receiver edge. Existing BPSAM/factor
logic decides insertion.

The observer copies the exact chosen Kimera frame IDs, timestamps, pre-injection
poses, match type, ambiguity count, terminal state, and factor-accepted state.
Stage 2B never performs a second state-pair selection.

Missing, rejected, superseded, retry-pending, already-applied, receive-gated,
and frame-conversion-failed cases remain distinct diagnostics.

## 5. Shadow architecture and failure isolation

Default and absent-section behavior is off. When off, no analyzer, UUID,
subscriber, cache, SPSC queue, worker, publisher, callback, or output file is
created.

Enabled data flow is:

```text
existing active legacy callback --after active buffering--> belief SPSC queue
Stage 2A metadata --dedicated ROS callback queue-----------> metadata SPSC queue
exact active receiver observer ----------------------------> receiver SPSC queue
                                                               |
                                                               v
                                                        shadow worker
                                                               |
                                               association/residual/CSV/topic
```

There are four bounded preallocated-capacity, drop-newest SPSC queues: belief,
metadata, receiver observation, and output. Pushes never wait. All association,
SHA-256 work beyond copied payload receipt, Pose3 residual calculation,
formatting, file I/O, and shadow publication occur on the worker.

The metadata subscriber has its own one-thread ROS callback queue. It does not
share Kimera's estimator/sensor callback queue. The worker and metadata queue
are optional and exist only when enabled.

Worker errors, association expiry, and queue overflow can only drop or mark a
shadow record unavailable. They cannot drop, delay, or modify a belief or
factor.

## 6. Exact belief–metadata association

Stage 2B receives the already-consumed legacy array from the active callback;
it deliberately does not add a second legacy subscriber. It joins that copy to
Stage 2A metadata by:

- sender session UUID;
- Stage 2A publication sequence;
- belief ordinal;
- array header stamp and frame ID;
- sender endpoint indices and exact endpoint timestamp bits;
- stable-edge digest;
- exact belief-payload digest.

The payload digest is independently recomputed from the received legacy
belief using Stage 2A's versioned canonical big-endian encoding and the first
128 bits of SHA-256. It covers the array identity, source agent, indices,
binary64 timestamps, six mean values, 36 covariance values, relax factor,
ordinal, and publication sequence.

Digest/schema/ordinal/endpoint conflicts make the metadata unavailable but do
not affect the belief. Exact duplicate metadata is suppressed and counted;
conflicting duplicate metadata invalidates the descriptive association.
Bounded caches permit either arrival order and expire without holding the
active receiver.

## 7. Receiver matching and frame conversion

For the exact pair selected by the active receiver:

```text
Z_receiver = T_receiver_from.inverse() * T_receiver_to
```

The sender belief uses the same pure helper as the active receiver:

```text
Z_sender_base     = C * Z_sender_external * C.inverse()
Sigma_sender_base = Ad_C * Sigma_sender_external * Ad_C.transpose()
```

where `C = T_base_external`. This is the right-local Pose3 covariance rule.
The full SE(3) adjoint is used; translated extrinsics can couple rotation and
translation. Tests cover identity, nontrivial rotation, nontrivial translation,
production-formula equality, and finite-difference perturbations.

The online disagreement is:

```text
r = Logmap(Z_sender_base.inverse() * Z_receiver)
rotational_disagreement = norm(r[0:3])       # rad
translational_disagreement = norm(r[3:6])    # m
```

Radians and metres are never combined into one score. Online values are
called disagreement, not error.

## 8. Receiver covariance and normalized disagreement

An exact receiver relative covariance needs the joint marginal of the matched
Kimera state pair and the correct `between` Jacobian. The passive output path
does not expose that joint object. Independent endpoint marginals are not
added.

Accordingly:

- `receiver_relative_covariance_available=false` at runtime;
- `compute_receiver_relative_covariance=true` is rejected;
- `compute_normalized_disagreement=true` is rejected;
- no chi-square threshold or decision exists.

A controlled pure pseudoinverse helper is unit tested for future validated
inputs, but it is unreachable through the approved runtime configuration.

## 9. Shadow message schemas

New schemas are version 1 / semantics version 1.

`DcregBeliefShadowAnalysis` includes:

- exact sender/session/publication/ordinal/digest identity;
- receiver session and local receipt sequence;
- belief–metadata and receiver-state validity;
- sender/receiver endpoint indices, timestamps, errors, interval durations,
  exact/nearest match type, and ambiguity counts;
- transformed sender motion, exact receiver motion, six residual components,
  and separate norms;
- sender covariance and explicit receiver/normalized unavailability;
- the frozen Stage 2A scalar entry without reinterpretation;
- descriptive health state and numeric invalid-reason mask;
- creation, latency, drop, and orphan counters.

`DcregBeliefShadowAnalysisArray` includes array-level schema/semantics,
receiver/sender sessions, Stage 2A publication identity/version, cumulative
operational counters, and entries.

Unknown schemas are separate-topic data and safely ignorable.

Final ROS MD5 values:

| Message | MD5 |
|---|---|
| legacy `liorf/pose_odom_belief` | `ad44b731245f49faea905204e6eeb4d5` |
| legacy `liorf/pose_odom_belief_array` | `170588018ad937e1dc0349d1bcb97cf8` |
| frozen Stage 2A metadata array | `90af5e860ae54a9939167ba56195ef6f` |
| Stage 2B entry | `b31995d171a183814df243912f3df3a9` |
| Stage 2B array | `3c504acd5de6ac0e350914a9d0dcca2e` |

The two legacy MD5 values are unchanged.

## 10. Public configuration

```yaml
dcreg_shadow_analysis:
  enabled: false

  belief_topic: /kimera/cbs/odom_belief_in
  metadata_topic: /glim/cbs/dcreg_edge_health_metadata
  output_topic: /kimera/cbs/dcreg_shadow_analysis
  structured_csv_path: ""

  maximum_pending_beliefs: 512
  maximum_pending_metadata: 512
  maximum_receiver_states: 4096
  association_timeout_sec: 2.0

  compute_raw_disagreement: true
  compute_receiver_relative_covariance: false
  compute_normalized_disagreement: false
  beta_receiver_covariance: 1.0
  model_floor_rotation: 0.0
  model_floor_translation: 0.0
  pseudoinverse_relative_threshold: 1.0e-10
  negative_eigenvalue_tolerance: 1.0e-10

  publication_queue_capacity: 256
  overflow_policy: drop_newest
```

Unknown fields, invalid numbers, a belief-topic mismatch, a disabled CBS
bridge, unsupported covariance diagnostics, or any overflow policy except
`drop_newest` disable only Stage 2B with a clear startup message.

## 11. Unit and integration tests

Focused suites:

| Suite | Result |
|---|---:|
| frozen Stage 1 GLIM diagnostics/reference/logger | 39/39 pass |
| frozen Stage 2A association/identity/summary | 31/31 pass |
| Stage 2B C++ receiver/analyzer | 35/35 pass |
| `cbsms` Python reporting/offline analysis | 18/18 pass |
| Total | **123/123 pass** |

Stage 2B tests cover digest vectors and independent Stage 2A digest
reproduction, association conflicts and missing data, frame conversion and
finite differences, exact/nearest receiver states, pure/mixed residuals,
covariance validity, missing/reference health labels, active rejection states,
legacy-byte immutability, queue overflow/ordering, ground-truth edge formulas,
and passive defaults.

The package-wide historical `testKimeraVioRos.cpp` target still contains its
pre-existing unconditional false assertion; it is unrelated to Stage 2B. The
focused Stage 2B target is clean.

Build commands:

```bash
docker exec cbsms_ws bash -lc '
  source /opt/ros/noetic/setup.bash
  cd /workspace/cbs_gtsam4.3
  source devel/setup.bash
  catkin build liorf kimera_vio kimera_vio_ros glim_ros --no-status
'

docker exec cbsms_ws bash -lc '
  source /opt/ros/noetic/setup.bash
  cd /workspace/cbs_gtsam4.3
  source devel/setup.bash
  catkin build kimera_vio_ros --no-status
'
```

Both builds passed; the final build completed all 16 dependency packages with
no warnings or failures.

Test commands:

```bash
/workspace/cbs_gtsam4.3/devel/scan_dcreg_diagnostics_test
/workspace/cbs_gtsam4.3/devel/lib/glim_ros/dcreg_edge_metadata_test
/workspace/cbs_gtsam4.3/devel/lib/kimera_vio_ros/dcreg_shadow_analysis_test
python3 -m unittest discover -s tools -p 'test_*.py'
```

## 12. Deterministic input and no-op regression

### 12.1 Input determinism

The final 20-second raw-camera merged bag was generated twice. Both outputs
contained the same 4,936 serialized events and the same authoritative content
hash:

```text
a249b0549500e3603bd42d8d101f72f9d03b5403f831f7810504994073639a6e
```

Per-topic inputs were:

| Topic | Count | Serialized content SHA-256 |
|---|---:|---|
| `/camera/color/image_raw` | 600 | `55e7123faedaca27f41e569654430ee187e63374a9aeef0e340f8f0272721a43` |
| `/camera/imu` | 4,000 | `7352f4e6ff02fe52adeba37d96d5cc2606d821c08353eab125409c6b47945141` |
| Stage 2A metadata | 168 | `dcd847d6474de9c05c08deef3425c68c3b805e656f9c7a025a453adbb13230a1` |
| legacy G-to-K belief input | 168 | `c58abd07ad96f25e7de3e1db7b65b1ffacaaf59543b4667cc6c1f24ec6ec4123` |

Ordering is bag-record timestamp, with original source record order retained
for equal timestamps.

### 12.2 Harness correction

The first replay attempt initialized Kimera on varying first camera frames.
The root cause was that `RosOnlineDataProvider` waited for simulated time, so
bag playback started before all Kimera subscribers finished construction.

The corrected test-only harness:

- primes `/clock` before launching Kimera;
- waits for all estimator input subscriptions;
- applies a three-second ready barrier;
- decodes camera images into the bag, removing the live conversion node;
- uses `parallel_run: 0`;
- sets common numerical-library thread counts to one;
- drains for 20 seconds after replay;
- records only structured ROS outputs.

### 12.3 Results and blocker

An absent-section versus explicit-disabled pair with the same initialization
and active factor stream showed the receiver's natural numerical envelope:

| Quantity | Maximum | Median | p95 | RMSE |
|---|---:|---:|---:|---:|
| pose rotation | `3.855e-10 rad` | `1.304e-14` | `3.556e-10` | `1.665e-10` |
| pose translation | `1.697e-9 m` | `1.294e-13` | `1.406e-9` | `5.848e-10` |
| increment rotation | `1.273e-11 rad` | `2.565e-14` | `1.069e-11` | `5.797e-12` |
| increment translation | `1.986e-10 m` | `2.585e-13` | `1.579e-11` | `2.413e-11` |
| belief rotation | `1.766e-11 rad` | `5.848e-12` | `1.081e-11` | `6.627e-12` |
| belief translation | `2.265e-11 m` | `1.596e-12` | `1.159e-11` | `5.364e-12` |
| belief covariance element | `1.333e-10` | `7.140e-14` | `7.892e-11` | `2.991e-11` |

After the metadata callback was isolated from Kimera's estimator queue, the
explicit-disabled and enabled runs:

- initialized at the identical Kimera timestamp `1735888008.101804495`;
- emitted 100 Kimera poses and 99 outgoing Kimera beliefs each;
- emitted the identical 324-row active CBS factor stream with SHA-256
  `dd5f7d00b0f0752644e7f782fd11b117119de47a44e9ea80c2def891b737594f`;
- had the same numerical envelope shown in the table above;
- differed in serialized legacy output bytes and exceeded `1e-12`.

The enabled run's only new topic was the 168-message shadow array topic. Input
legacy belief and Stage 2A metadata bytes were fixed by the input manifest and
were not mutated.

Because the task requires exact `1e-12`, the deterministic approval criterion
fails. The fact that enabled/disabled equals the separately measured off/off
envelope prevents attributing the difference to Stage 2B, but it does not
satisfy exact equality.

Artifacts:

```text
stage2b_implementation/deterministic_receiver/raw_input_a_manifest.json
stage2b_implementation/deterministic_receiver/raw_input_b_manifest.json
stage2b_implementation/deterministic_receiver/comparison_isolated.json
stage2b_implementation/deterministic_receiver/numeric_comparison_disabled_vs_enabled_isolated.json
stage2b_implementation/deterministic_receiver/runs/D5_enabled_finalmetrics/
```

## 13. Active factor-stream comparison

The strongest receiver-side control comparison available in the corrected
harness is the active factor detail stream. Explicit-disabled and enabled
Stage 2B runs produced the exact same 324 rows and exact same stream hash.

Stage 2B observes terminal states after the existing active decision. It
cannot set acceptance, alter the converted belief, modify covariance, add a
factor, or call the optimizer. Unit tests independently verify that constructing
a shadow entry leaves serialized legacy belief bytes unchanged.

## 14. Real-data validation

All real runs were 25-second headless M3DGR windows. Rerun output was saved to
offline `.rrd` files; no live visualization was placed in the estimator path.

### 14.1 Dynamic01 session reference

- 4,884 exact metadata matches.
- 828 exact receiver disagreements.
- 221 beliefs accepted by the active factor stream.
- Stage 2A states: 3,708 healthy, 1,176 reference unavailable during
  bootstrap; all 221 accepted-factor records with health were healthy.
- Accepted-factor median disagreement: `0.001377 rad`, `0.001667 m`.
- Accepted-factor p95: `0.006114 rad`, `0.008176 m`.
- Spearman rotational health versus rotation disagreement:
  `rho=+0.157`, bootstrap 95% `[+0.037,+0.280]`.
- Spearman translational health versus translation disagreement:
  `rho=+0.369`, bootstrap 95% `[+0.234,+0.496]`.

The positive sign is opposite the simple “lower health, larger disagreement”
expectation. Dynamic01 mostly spans a narrow healthy regime; it does not
validate a monotonic health-to-disagreement law.

Rerun:

```text
stage2b_implementation/runs/20260805-134509_dynamic_stage2b_25s_headless/stage2b_analysis/Dynamic01_stage2b_shadow.rrd
```

### 14.2 Outdoor01 session-only

- 4,956 exact metadata matches.
- 852 exact receiver disagreements.
- 234 active accepted factors.
- `reference_ready=false`; no relative-health values were fabricated.
- Every interval was labeled absolutely degenerate from the available
  absolute mask.
- Accepted-factor median disagreement: `0.000517 rad`, `0.002982 m`.
- Accepted-factor p95: `0.007400 rad`, `0.015720 m`.

The shadow analyzer remains useful without a healthy reference: absolute
degeneracy, support, matching, active receiver state, and raw disagreement are
still explicit.

Rerun:

```text
stage2b_implementation/runs/20260805-142609_outdoor01_stage2b_session_25s/stage2b_analysis/Outdoor01_session_stage2b_shadow.rrd
```

### 14.3 Outdoor01 compatible offline/hybrid

- 4,932 exact metadata matches.
- 876 exact receiver disagreements.
- 234 active accepted factors.
- Relative health was available immediately from the compatible offline
  lineage; degraded samples did not adapt the reference.
- All intervals remained absolutely degenerate.
- Accepted-factor median disagreement: `0.000118 rad`, `0.004636 m`.
- Spearman rotational health versus rotation disagreement:
  `rho=+0.094`, 95% `[-0.044,+0.228]`.
- Spearman translational health versus translation disagreement:
  `rho=-0.185`, 95% `[-0.310,-0.064]`.

The translation result is directionally consistent with lower health and
larger disagreement, but the effect is modest and sequence-specific.

Rerun:

```text
stage2b_implementation/runs/20260805-143240_outdoor01_stage2b_hybrid_25s/stage2b_analysis/Outdoor01_hybrid_stage2b_shadow.rrd
```

### 14.4 Wheel-float01 additional geometry sequence

- 4,908 exact metadata matches.
- 714 exact receiver disagreements.
- 169 active accepted factors.
- 3,804 healthy records after 1,104 bootstrap-unavailable records.
- Accepted-factor median disagreement: `0.000063 rad`, `0.002594 m`.
- Spearman rotational health versus rotation disagreement:
  `rho=+0.173`, 95% `[-0.014,+0.351]`.
- Spearman translational health versus translation disagreement:
  `rho=+0.221`, 95% `[+0.070,+0.373]`.
- 233 receiver observations became descriptive orphans at expiry/shutdown;
  active beliefs/factors were not dropped.

Rerun:

```text
stage2b_implementation/runs/20260805-144036_wheel_float01_stage2b_session_25s_retry/stage2b_analysis/Wheel_float01_stage2b_shadow.rrd
```

The earlier `20260805-143749...` wheel-float run had a run-directory collision
and is excluded from scientific results.

## 15. Ground-truth evaluation

The offline tool implements:

```text
T_world_base(t) = T_world_gt_body(t) * T_gt_body_base
Z_gt = T_world_base(t_i).inverse() * T_world_base(t_j)
e_sender = Logmap(Z_gt.inverse() * Z_sender_base)
e_receiver = Logmap(Z_gt.inverse() * Z_receiver)
```

with exact timestamp interpolation (linear translation and quaternion SLERP).
It refuses to run exact edge-error analysis without an explicit JSON transform
marked verified.

Current limitation:

- Dynamic01 and Wheel-float01 track a UGV/body frame whose fixed transform to
  `camera_imu_link` is not verified.
- Outdoor01 exposes RTK position with placeholder/insufficient orientation and
  no verified antenna lever arm to Kimera base.

Therefore sender and receiver ground-truth edge errors, “which estimator is
closer,” and health-versus-accuracy correlations are unavailable. No transform
was fitted from the evaluation data and no approximate endpoint-marginal or
position-only 6-DoF claim was substituted.

## 16. Scientific interpretation and failure cases

What Stage 2A metadata currently does well:

- exactly identifies health records belonging to a received belief instance;
- distinguishes reference-unavailable from healthy;
- stratifies absolute degeneracy, relative health, support, matching quality,
  ambiguity, and receiver disagreement without combining them;
- shows when a belief accepted by Kimera came from geometry Stage 1 considered
  degenerate;
- preserves repeated rolling-array publications as distinct instances.

What current evidence does not establish:

- a monotonic relationship between health and disagreement across datasets;
- a relationship between health and GLIM ground-truth edge accuracy;
- whether Kimera or GLIM is closer to truth in any reported interval;
- whether both estimators agree while both are wrong;
- any calibrated probability, metric error, covariance scale, or control
  threshold.

Observed failure/ambiguity examples:

- Outdoor01 hybrid can have severe/absolute LiDAR degeneracy and still show
  small GLIM–Kimera disagreement. Agreement is not correctness.
- Dynamic01's “healthy” values correlate positively with disagreement over a
  narrow healthy range. A health signal is not guaranteed to explain every
  receiver difference.
- Absolute-degenerate occupancy was constant in Outdoor01, so its rank
  correlation is mathematically undefined.
- Rolling belief entries and neighboring scan intervals are temporally
  correlated; row-bootstrap confidence intervals are descriptive, not
  independent-sample guarantees.
- Missing reference remains a first-class state, not health 1.0.

No causal claim or classification threshold is made.

## 17. Runtime, memory, queue, and drop measurements

Final isolated enabled run (`D5_enabled_finalmetrics`, 19.897 seconds):

| Metric | Value |
|---|---:|
| belief/metadata arrays received | 168 / 168 |
| receiver observations | 1,165 |
| shadow arrays/entries published | 168 / 993 |
| dropped belief/metadata/receiver/output records | 0 / 0 / 0 / 0 |
| producer waits | 0 |
| worker failures | 0 |
| orphan belief/metadata/receiver | 0 / 0 / 29 |
| max belief/metadata/receiver queue depth | 2 / 2 / 24 |
| max pending belief/metadata/receiver | 2 / 2 / 49 |
| configured queue capacity | 256 each |
| max active belief enqueue | `2.744 us` |
| max active receiver enqueue | `2.894 us` |
| worker CPU time | `50.669 ms` total |
| association time | `8.814 ms` total |
| residual time | `2.724 ms` total |
| published bytes | `2,666,106` |
| average shadow array size | about `15.9 kB` |
| shadow bandwidth | about `134 kB/s` |

The 29 orphan receiver observations are expired/nonterminal/superseded
diagnostic outcomes; they did not remove beliefs or factors. All drops and
orphans are explicit.

Primary 25-second full-system runs likewise recorded zero waits, zero drops,
zero worker failures, and maximum producer enqueue times from approximately
`4.9 us` to `11.4 us`. Wheel-float recorded the 233 descriptive receiver
orphans noted above.

No shadow file I/O runs on Kimera's optimization or active belief callback
thread. Shutdown drains/finalizes bounded diagnostics only.

## 18. Backward compatibility and passivity result

- Existing belief message definitions and MD5 values are unchanged.
- Existing consumers can ignore the separate topic.
- Default/absent mode constructs no Stage 2B resources.
- The active legacy receiver still buffers the converted belief before any
  shadow enqueue.
- Exact active receiver state-pair selection is observed, not duplicated.
- Explicit-disabled and enabled isolated runs emitted identical active factor
  streams.
- No receiver relative covariance approximation is introduced.
- No existing topic consumes Stage 2B output.
- No metadata, covariance, weighting, rejection, or optimization path reads
  Stage 2B output.

## 19. Approval decision

### Passive infrastructure

The code is suitable for continued **passive experimentation**: it is bounded,
optional, non-blocking, exact in identity and active-state observation, and it
produces useful descriptive artifacts.

### Formal Stage 2B approval

**NO-GO under the task's strict acceptance criteria.** The deterministic
enabled/disabled output comparison exceeds `1e-12`, even though it matches the
independently measured disabled/disabled receiver envelope and the active
factor stream is byte-identical.

The remaining blocker is a receiver-side direct deterministic event harness
or an accepted near-machine tolerance derived from Kimera's measured
repeatability. The criterion must not be weakened silently.

### Scientific usefulness

Stage 2A metadata appears useful for exact descriptive stratification and for
identifying when accepted beliefs arise under absolute degeneracy. It is not
yet proven useful for predicting GLIM accuracy relative to ground truth, and
its relationship with sender–receiver disagreement is not stable across the
current sequences.

### Active weighting readiness

Active weighting is **not ready** because:

1. no verified 6-DoF ground-truth frame transform supports edge-accuracy
   evaluation;
2. health/disagreement correlations are inconsistent across datasets;
3. agreement cannot distinguish both-estimators-wrong cases;
4. exact receiver relative covariance is unavailable;
5. the strict deterministic regression gate remains open;
6. no held-out calibration/evaluation threshold study exists.

No covariance inflation, active coordinator, gating, factor weighting,
directional transport, interval Hessian, or solver mitigation was implemented.
