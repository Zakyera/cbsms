# Stage 2B: passive receiver-side DCReg belief shadow analysis

Date: 2026-08-05

Status: passive implementation complete; strict regression approval is blocked
by the receiver replay's measured numerical nondeterminism. Stage 1 and Stage
2A are frozen. This document describes only the passive Stage 2B boundary.

## 1. Meaning

Stage 2B observes one exact GLIM-to-Kimera CBS relative-belief instance and
asks two descriptive questions:

1. What relative motion did GLIM send, and what relative motion existed
   between the exact Kimera states selected by the active receiver?
2. Do Stage 2A's conservative LiDAR-health summaries covary with that
   sender–receiver disagreement and, where a fully verified ground-truth body
   transform exists, with edge accuracy?

The primary online quantity is

```text
r = Logmap(inverse(Z_sender_base) * Z_receiver)
```

with GTSAM ordering

```text
[rotation_x, rotation_y, rotation_z, translation_x, translation_y,
 translation_z].
```

Rotation and translation norms are always reported separately. Online `r` is
called disagreement, not error.

Stage 2B never emits a trust weight, covariance multiplier, accept/reject
decision, factor command, or optimizer input. Stage 2A health is not treated
as covariance, probability, confidence, or metric error.

## 2. Audited active receiver path

The legacy belief subscriber is created in
`Kimera-VIO-ROS/src/KimeraVioRos.cpp` by
`KimeraVioRos::initializeHeadlessCbsBeliefBridge`. The callback
`KimeraVioRos::poseOdomBeliefInCallback`:

1. preserves the legacy array and belief payload;
2. skips self beliefs and the existing receive-delay gate;
3. reconstructs the relative pose with `gtsam::Pose3::Expmap`;
4. reconstructs the right-local 6x6 covariance;
5. obtains the configured external-to-base transform from TF;
6. converts the pose and covariance into `camera_imu_link`;
7. buffers the unchanged active `ExternalOdometryBelief` path.

The exact shared frame-conversion helper is
`Kimera-VIO-ROS/src/CbsRelativeFrameConversion.cpp`:

```text
Z_base     = C * Z_external * inverse(C)
Sigma_base = Ad_C * Sigma_external * transpose(Ad_C)
```

where `C = T_base_external`. The covariance is a right-local Pose3 covariance
ordered rotation then translation. A finite-difference test covers the full
adjoint, including the rotation/translation coupling created by a translated
extrinsic.

The active historical state resolver is
`Kimera-VIO/src/backend/VioBackend.cpp` in
`VioBackend::resolveExternalBeliefStamp`. It searches the stored Kimera
keyframe timestamp registry up through the current key, chooses minimum
absolute timestamp distance, retains the first/lower key on an exact tie, and
applies the existing timestamp and interval-duration gates. Competing sender
beliefs resolving to one receiver edge are ranked by the existing active score.
The BPSAM insertion decision remains authoritative.

Stage 2B registers an additive observer. At the active decision point it copies
the selected receiver keys, timestamps, pre-injection poses, sender pose and
sender covariance into an immutable diagnostic. It does not select a second
pair and does not query the optimizer.

## 3. Exact belief–metadata identity

The existing active Kimera callback passes an immutable copy of the already
received legacy belief to the analyzer only after the active buffer operation.
The analyzer does not create a second legacy-belief subscriber. It subscribes
only to the separate Stage 2A metadata topic and recomputes Stage 2A's
canonical 128-bit SHA-256 payload prefix from:

- array header timestamp and frame ID;
- sender agent;
- sender endpoint indices and exact binary64 timestamp bits;
- six relative-mean values;
- 36 covariance values;
- relax factor;
- belief ordinal;
- Stage 2A publication sequence.

It accepts metadata only when schema/semantics, array identity, all endpoints,
stable-edge digests, and all payload digests agree. Repeated publications of
the same stable edge remain distinct belief instances. Ambiguous identical
legacy arrays are not arbitrarily paired. Exact duplicate metadata is counted;
a conflicting duplicate invalidates relative metadata for that instance.

No active belief waits for metadata. Out-of-order messages are joined in
bounded worker-owned caches and expire descriptively.

## 4. Shadow architecture

When enabled, Stage 2B has four bounded drop-newest SPSC queues:

```text
legacy belief callback -> immutable belief-array queue ----\
dedicated metadata callback queue -> immutable metadata ----+-> worker
active backend observer -> immutable receiver-match queue --/     |
                                                               output queue
                                                                    |
                                                         ROS topic + CSV
```

The active belief callback performs its existing conversion and buffer step
first, then copies the legacy array into the shadow queue. The backend observer
performs one non-blocking queue push. Association, Pose3 residuals, formatting,
ROS shadow publication, and CSV I/O happen only on the shadow worker.
Stage 2A transport callbacks run on a dedicated one-thread ROS callback queue,
not Kimera's estimator callback queue.

Queue overflow drops a shadow record and increments a counter. It cannot drop
or delay a belief or factor. Worker exceptions affect only shadow output.

The existing 20-second receive-delay gate is represented explicitly as an
active rejection with unavailable disagreement; it is not confused with a
missing backend observation. A failed TF conversion is likewise explicit and
does not fabricate an identity sender motion.

## 5. Output messages

New optional messages are:

- `liorf/DcregBeliefShadowAnalysis`;
- `liorf/DcregBeliefShadowAnalysisArray`.

Schema version 1 / semantics version 1 reports:

- exact Stage 2A identity and copied scalar metadata;
- exact active receiver match status and endpoint diagnostics;
- sender and receiver relative Pose3 Logmap vectors;
- the six-dimensional disagreement and separate norms;
- sender covariance in Kimera base frame;
- explicit receiver-covariance unavailability;
- health/reference/association state;
- numeric invalid-reason masks;
- queue, orphan, and latency counters.

No existing message is modified. In particular the legacy MD5 values remain:

```text
liorf/pose_odom_belief       ad44b731245f49faea905204e6eeb4d5
liorf/pose_odom_belief_array 170588018ad937e1dc0349d1bcb97cf8
```

Unknown shadow schemas are optional and safely ignorable.

## 6. Receiver covariance

An exact relative covariance would require a joint marginal for the matched
Kimera state pair followed by the correct `between` Jacobian. The current
read-only backend output exposes a current-state marginal, not that joint
object. Adding a marginal query to the optimization thread solely for Stage 2B
would violate passive scheduling isolation.

Therefore Stage 2B does not use endpoint-marginal addition and reports receiver
relative covariance unavailable. Runtime configuration rejects
`compute_receiver_relative_covariance=true` and
`compute_normalized_disagreement=true`. A pure, unit-tested controlled
pseudoinverse helper exists for future validated inputs, but it is unreachable
from the approved runtime configuration and emits no chi-square decision.

## 7. Public configuration

The optional private Kimera section is:

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

The section is absent by default. Absent or `enabled: false` constructs no
Stage 2B subscriber, cache, queue, worker, publisher, UUID, or file. Unknown
fields, invalid values, topic mismatch, or unsupported covariance modes disable
only Stage 2B with a startup diagnostic.

## 8. Descriptive states

Sender LiDAR-health labels are:

- `HEALTHY`;
- `DEGRADED`;
- `ABSOLUTELY_DEGENERATE`;
- `REFERENCE_UNAVAILABLE`;
- `METADATA_UNAVAILABLE`;
- `INVALID`.

Agreement is either available or unavailable. Stage 2B deliberately has no
online low/high-disagreement threshold. Missing metadata is never healthy.
Absolute degeneracy remains usable when the Stage 1 healthy reference is
unavailable.

## 9. Offline evaluation

`cbsms/tools/dcreg_shadow_stage2b_analysis.py` reads the dedicated CSV and
produces:

- exact matching and availability counts;
- disagreement distributions stratified by Stage 2A state;
- Spearman rank correlations;
- fixed-seed bootstrap 95% intervals;
- enriched per-edge CSV;
- an SVG timeline;
- an optional saved Rerun `.rrd` timeline.

Ground truth is never read online. Full edge evaluation requires both a
full-pose TUM trajectory and an explicit JSON transform marked `verified` with
`gt_body_T_kimera_base_xyz_xyzw`. Endpoints use linear translation
interpolation and quaternion SLERP. The evaluated base poses are

```text
T_world_base(t) = T_world_gt_body(t) * T_gt_body_base
Z_gt            = inverse(T_world_base(t_i)) * T_world_base(t_j).
```

Sender and receiver errors are separately

```text
e_sender   = Logmap(inverse(Z_gt) * Z_sender_base)
e_receiver = Logmap(inverse(Z_gt) * Z_receiver).
```

Dynamic01 and Wheel-float01 currently lack a verified tracked-UGV-to-Kimera
base transform. Outdoor01 provides RTK position with placeholder identity
orientation and lacks a verified antenna lever arm. The tool therefore marks
their exact 6-DoF edge errors unavailable rather than fitting an extrinsic or
silently approximating one. This is a ground-truth evaluation limitation, not
an online shadow limitation.

## 10. Scientific analysis boundary

Continuous health/disagreement analysis may use all descriptive records.
Any future low/high-disagreement classification must freeze thresholds from a
separate calibration sequence before held-out evaluation. Stage 2B does not
derive such thresholds.

It does not claim causality, covariance calibration, or an apples-to-apples
uncertainty scale. It does not fit a health-to-error law.

## 11. Explicit non-goals

Stage 2B does not change:

- GLIM or VGICP;
- Stage 1 health or reference logic;
- Stage 2A interval metadata;
- any CBS mean, covariance, serialization, factor, or robust kernel;
- Kimera state matching, factor insertion, optimization, or output;
- any sender or receiver decision.

Directional transport, interval Hessians, projectors, covariance inflation,
active coordination, and solver mitigation remain outside Stage 2B.

## 12. Validation status and limitations

The focused C++ suite passes 35 tests and the workspace's Stage 1/2A/2B plus
offline-reporting suites pass 123 tests in total. Four 25-second real-data
runs produced exact belief/metadata joins, raw receiver disagreements, and
offline Rerun recordings with zero queue drops or producer waits in the three
primary M3DGR cases.

The deterministic receiver replay uses a serialized-content-identical merged
bag, a primed simulated clock, installed-subscriber barriers, a sequential
Kimera pipeline, and single-thread numerical-library settings. It establishes:

- absent and explicit-disabled paths create no Stage 2B resources;
- after isolating the metadata callback queue, explicit-disabled and enabled
  runs selected the same initialization frame and emitted the same 324-row
  active CBS factor stream;
- the enabled/disabled output difference was at most `1.70e-9 m` in pose and
  `3.85e-10 rad`, matching a separately observed disabled/disabled replay
  envelope;
- these values exceed the task's strict `1e-12` deterministic acceptance
  tolerance.

Consequently this implementation is scientifically useful as passive shadow
infrastructure, but it is **not regression-approved under the strict 1e-12
criterion**. This is recorded as a test-harness/receiver-repeatability blocker,
not silently relabeled as exact equality.

The current datasets also lack a verified full 6-DoF ground-truth-body to
Kimera-base transform. Therefore the present scientific result concerns
health versus sender/receiver disagreement only. It does not yet establish
whether health predicts GLIM edge accuracy relative to ground truth.
