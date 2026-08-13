# GLIM DCReg Stage 2A Implementation and Validation Report

Date: 2026-08-04

Decision: **APPROVED for optional passive metadata-only experimentation.**

Stage 2A publishes a conservative descriptive summary of frozen Stage 1
scan-local LiDAR observability-health records associated with each already
published G-to-K CBS belief instance. It does not estimate interval
uncertainty and cannot control any estimator.

No commit or push was performed.

## 1. Git state and preservation

The state before edits was recorded in every involved nested repository.

| Repository | Branch | HEAD before and after | Initial state |
|---|---|---|---|
| `src/glim` | `cbs-gtsam43-noetic` | `6c4189e117c61015a79001388debb27413c78fa6f` | clean |
| `src/glim_ros1` | `cbs-gtsam43-noetic` | `2dea515e9e38743b6fc73e7dfdf7e45d3ae5bf6f` | clean |
| `src/liorf` | `cbsms/gtsam-4.3-develop` | `4286f982694dda550d1343ff21679f8ec2e708ba` | clean |
| `src/cbs` | `cbsms/gtsam-4.3-develop` | `994b1d6a5c05fb38dd1b0731c6430ec11d2f1ff0` | clean |
| `src/Kimera-VIO` | `cbsms/gtsam-4.3-develop` | `d0b2a31adf17ced8995994373f6b286679e26799` | clean |
| `src/Kimera-VIO-ROS` | `cbsms/gtsam-4.3-develop` | `09d7e17b5d27f97f622f285f0f23a61e6748278d` | pre-existing `scripts/__pycache__/` |
| `src/cbsms` | `cbsms/gtsam-4.3-develop` | `873f60fa748df97bf9f5f7f9dcdb825b4f5ed30c` | pre-existing Stage 1 documentation edits/untracked closeout and Stage 2A audit, plus `tools/__pycache__/` |

The active worktrees were never reset, stashed, committed, or switched. The
pre-edit safety files are under:

```text
stage2a_implementation/safety_before_edits_20260804/
```

Their SHA-256 values are:

```text
0c8bbea86ec68b553315d0983571677b0350faa3d465c33fce4b5f4550155706  cbsms_tracked.patch
125ad01c119da491d1106fdca53e66208cd860e382a3b008f3b0e3fe77473ce8  stage1_closeout_untracked.patch
1ec53052effd24d3da2bf93111fd32c86df91d8095e9b22e2ac137bc521b1d35  stage2a_audit_untracked.patch
```

Final `git diff --check` passes in `glim_ros1`, `liorf`, and `cbsms`.
`src/glim` remains clean, proving that frozen Stage 1 code was not changed.

## 2. Files added or changed

Stage 2A implementation in `src/glim_ros1`:

- `CMakeLists.txt`
- `include/glim_ros/cbs_bridge.hpp`
- `src/glim_ros/cbs_bridge.cpp`
- `include/glim_ros/bounded_spsc_queue.hpp`
- `include/glim_ros/dcreg_edge_metadata.hpp`
- `include/glim_ros/sha256.hpp`
- `src/glim_ros/dcreg_edge_metadata_core.cpp`
- `src/glim_ros/dcreg_edge_metadata_publisher.cpp`
- `src/glim_ros/sha256.cpp`
- `test/dcreg_edge_metadata_test.cpp`

New optional messages in `src/liorf`:

- `CMakeLists.txt`
- `msg/DcregNumericSummary.msg`
- `msg/DcregResolutionSupportSummary.msg`
- `msg/DcregEdgeHealthMetadata.msg`
- `msg/DcregEdgeHealthMetadataArray.msg`

New Stage 2A-only cbsms material:

- `config/glim_dcreg_edge_metadata_stage2a.example.json`
- `docs/2026-08-04_glim_dcreg_edge_metadata_stage2a.md`
- this report

Stage 2A experiment/analyzer artifacts are under `stage2a_implementation/`.
Pre-existing Stage 1 documentation changes and Python caches were preserved.

No files in CBS, Kimera, VGICP, the GLIM optimizer, or the Stage 1 monitor were
changed.

## 3. Frozen assumptions reconfirmed

Before editing, the current code reconfirmed:

- A Stage 1 record is keyed by scan/pose index `k` and the source cloud/GLIM
  pose timestamp.
- Its aggregate Hessian is the final-pose local unary VGICP
  `H_level_0 + H_level_1`.
- Its tangent is local/body, right perturbation, ordered rotation then
  translation.
- A G-to-K belief edge uses start pose `i`, end pose `j`, and
  `Z_ij = inverse(T_i) * T_j`.
- Its relative covariance is right-local in the same six-coordinate ordering.
- Rolling belief arrays can republish the same stable edge; updated payloads
  are different belief instances.

Frozen Stage 1 emits diagnostics at
`src/glim/src/glim/odometry/odometry_estimation_cpu.cpp:1078` after the final
scan pose is assigned and before smoother-factor insertion. Stage 2A reads the
existing callback; it neither copies nor recomputes a Hessian in CBS code.

## 4. Legacy ROS message compatibility

The legacy message source files were not edited. Their ROS MD5 values are
identical before and after:

```text
liorf/pose_odom_belief        ad44b731245f49faea905204e6eeb4d5
liorf/pose_odom_belief_array  170588018ad937e1dc0349d1bcb97cf8
```

The new optional array has:

```text
liorf/DcregEdgeHealthMetadataArray  90af5e860ae54a9939167ba56195ef6f
```

The deterministic regression further proved byte identity of the legacy
belief stream, not merely unchanged `.msg` text or MD5.

## 5. Message schemas and versions

Four additive messages were introduced:

1. `DcregNumericSummary`: validity, denominator, minimum, median, maximum,
   mean.
2. `DcregResolutionSupportSummary`: factor/resolution identity and separate
   support, matching-cost, Hessian-scale/rank/condition summaries.
3. `DcregEdgeHealthMetadata`: belief identity, exact association accounting,
   reference lineage/source, relative health, absolute DCReg occupancy,
   ambiguity, support/matching data, and operational counters.
4. `DcregEdgeHealthMetadataArray`: values common to one legacy belief-array
   publication, followed by entries in matching ordinal order.

Both `schema_version` and `semantics_version` are `1`.

Semantics version 1 fixes:

- association `i < k <= j`;
- actual pose-key registry expected counts;
- conservative scalar summaries;
- separate optional topic;
- no directions or axis labels;
- no Hessian/information aggregation;
- drop-newest, non-blocking overflow.

Unavailable numeric groups have explicit validity flags and quiet NaN values.
Unavailable relative health is never represented as `1.0`.

## 6. Public configuration

The optional section is inside `odometry_estimation`:

```json
"dcreg_edge_metadata": {
  "enabled": false,
  "topic": "/glim/cbs/dcreg_edge_health_metadata",
  "pose_timestamp_tolerance_sec": 1.0e-6,
  "minimum_valid_samples": 1,
  "maximum_samples_per_edge": 64,
  "lower_quantile": 0.10,
  "minimum_samples_for_lower_quantile": 10,
  "publish_unavailable": true,
  "include_absolute_degeneracy": true,
  "include_relative_health": true,
  "include_support_summary": true,
  "include_matching_summary": true,
  "publication_queue_capacity": 256,
  "overflow_policy": "drop_newest"
}
```

The complete file is
`src/cbsms/config/glim_dcreg_edge_metadata_stage2a.example.json`. Its default
is intentionally disabled.

The parser rejects unsupported fields and invalid ranges and disables only
Stage 2A. Enabling requires frozen Stage 1 `dcreg_health.mode=log_only`.
Stage 2A never enables Stage 1 itself.

Section absent or `enabled=false` constructs no Stage 2A callback, cache,
queue, worker, publisher, UUID, digest path, or topic.

## 7. Exact association implementation

The existing belief array is built and published at
`src/glim_ros1/src/glim_ros/cbs_bridge.cpp:2600-2692`. Only after
`belief_pub_.publish(msg)` does line 2694 enqueue the immutable Stage 2A
descriptor.

For every belief `i -> j`, `populateExpectedPoseKeys()` enumerates actual
finite entries in the accepted GLIM pose timestamp registry for `i < k <= j`.
It reports the numeric index span separately and marks whether keys are
contiguous. A missing numeric index is not a missing health sample if no GLIM
pose state existed there.

Association is by pose/frame index. Each chosen health timestamp must satisfy:

```text
abs(health_stamp[k] - pose_registry_stamp[k]) <= configured tolerance
```

Records are sorted by pose index and timestamp. Exact payload duplicates keep
the first record and increment a counter. Conflicting duplicates invalidate
the sample. Equal/non-monotonic timestamps for different keys invalidate the
temporal association.

An interval exceeding 64 expected keys by default is explicitly incomplete.
Health summaries are unavailable rather than silently computed from a
truncated prefix.

## 8. Async cutoff/high-watermark mechanism

The Stage 1 callback creates one immutable scalar record and non-blockingly
pushes it into a bounded SPSC queue. Successful enqueue advances an atomic
health high-watermark. A belief descriptor snapshots that watermark as its
cutoff.

The worker defers a descriptor until health processing reaches that cutoff.
Thus reversed worker processing cannot mislabel a record as late. A record
whose generation sequence is beyond the fixed cutoff cannot mutate already
published metadata and is counted as late only for that belief instance.

Belief publication never waits for health records, queue space, association,
digesting, serialization, or metadata publication.

## 9. Reference-lineage implementation

No Stage 1 reference formula or state was modified. Stage 2A derives an
additive diagnostics-only 128-bit lineage from:

- sender process/session UUID;
- complete odometry configuration fingerprint;
- reference generation;
- compatible reference-source family;
- initial rotational and translational reference-ratio bits.

The lineage persists through compatible slow adaptation and an
offline-to-hybrid-session-adapted label transition. It changes on a new
session reference, reset, incompatible profile/configuration, process restart,
or health-scale change.

One lineage permits relative summaries. Compatible mixed source labels become
`MIXED_COMPATIBLE`. Different lineages make relative health unavailable while
preserving absolute degeneracy, support, matching, and ambiguity.

## 10. Summary definitions

One scan scalar is the minimum of the three frozen Stage 1 smoothed mode-health
values for its rotation or translation group. Absolute/relative scan flags use
`any()` over the corresponding frozen Stage 1 masks. Cluster and low-alignment
flags remain separate ambiguity diagnostics.

Relative summaries use only valid, health-available records with one lineage:

- minimum and median;
- deterministic lower quantile when sufficiently populated;
- degraded numerator, denominator, and fraction;
- longest consecutive degraded run;
- worst-sample timestamp, with earliest pose/timestamp tie breaking.

The nearest-rank quantile is:

```text
rank = max(1, ceil(q*n))
```

It is invalid/NaN when `n < minimum_samples_for_lower_quantile` or when the
rank is one, preventing a duplicate minimum from masquerading as a distinct
quantile.

Denominators are independent:

- absolute occupancy: eligible valid Hessian records;
- relative occupancy: health-available, lineage-compatible records;
- ambiguity: eligible valid Hessian records;
- matching state: eligible matching records;
- each support/cost summary: its own finite-value count.

No combined score exists. Fractions are descriptive occupancy, not
probabilities.

## 11. Digest encoding and vectors

A small local SHA-256 implementation avoids a new large dependency.

The canonical encoding uses:

- versioned ASCII prefixes;
- big-endian fixed-width integers;
- big-endian exact IEEE-754 binary64 bits;
- 32-bit big-endian string lengths followed by raw UTF-8 bytes.

The stable edge digest covers session UUID, sender ID, endpoint pose indices,
and exact endpoint timestamp bits. The exact belief-payload digest covers the
array header timestamp/frame, sender, endpoints, six Logmap mean values, 36
covariance values, relax factor, ordinal, and publication sequence. Both are
the first 128 bits of SHA-256.

Tests pass the published SHA-256 empty-string and `abc` vectors. A locked
canonical belief-payload vector is:

```text
b0703565d09b0df452313300c32e8308
```

All 10,179 metadata entries in the three deterministic enabled runs were
independently recomputed from captured legacy beliefs; mismatches were zero.

## 12. Queue and cache architecture

Two preallocated bounded SPSC queues are used: one for immutable health
records and one for immutable belief-array descriptor pointers. Default
capacity is 256. Both producers use `tryPush`; overflow drops the newest
metadata item and increments counters. Producer waits are structurally zero.

The worker performs association, reference-lineage validation, scalar
summaries, SHA-256, ROS metadata construction, and publication. There is no
metadata file I/O on the belief thread.

The health cache prunes against the oldest actual expected pose key required
by the current rolling belief array and all pending descriptors—not the nominal
0.20-second horizon. The hard record bound is derived from queue capacity and
the maximum samples per edge. Anomalous forced pruning is explicitly counted.

Any producer, worker, cache, digest, or publisher exception affects metadata
only. CBS belief publication is outside and before this failure domain.

## 13. Build commands and products

Builds were performed inside the existing `cbsms_ws` Docker environment.

Focused message/bridge build:

```text
catkin build liorf glim_ros --no-deps --no-status --summarize
```

Full relevant workspace build:

```text
catkin build liorf glim glim_ros cbs cbsms kimera_vio kimera_vio_ros \
  --no-deps --no-status --summarize -j8 -p2
```

All 11 scheduled packages and dependencies succeeded. Warnings were
pre-existing/non-fatal.

The isolated Stage 1 control was built from the exact detached HEAD in
`stage2a_implementation/control_source/glim_ros`. Control and candidate build
and devel spaces were separate.

Representative binary hashes:

```text
control glim_rosbag    6e16985d1bb31bbc44f00328923fc50716c10a230fa75dfe77933512615279e6
control libglim_ros    a6c1753ff8c6150caf3fc2ca1135c6b9e30e2380abca26feeb0b362d64292ba1
candidate glim_rosbag  a1266cd2e53112d1ac8fdfde1dec41519e39fab7e7cca12bf7cc5de55b9d6810
candidate libglim_ros  9952aebcc04e88b7163e372e0b7c0bbe4fa3f24e26fcb65c847457888ad1ae8b
```

## 14. Unit and reporting tests

Commands:

```text
/workspace/cbs_gtsam4.3/devel/lib/glim_ros/dcreg_edge_metadata_test
/workspace/cbs_gtsam4.3/devel/scan_dcreg_diagnostics_test
python3 src/cbsms/tools/test_cbsms_experiment_runner.py
python3 src/cbsms/tools/test_glim_dcreg_calibration_protocol.py
python3 src/cbsms/tools/test_glim_dcreg_directional_audit.py
```

Results:

- Stage 2A: 31/31 gtests passed across SHA, association, reference,
  configuration, summaries, identity, queue, and compatibility suites.
- Frozen Stage 1: 39/39 gtests passed.
- cbsms reporting/runner: 4/4 + 3/3 + 4/4 passed.

The 31 Stage 2A test cases combine multiple requested scenarios. Together with
the deterministic bag regression they cover start/end boundaries, adjacent and
overlapping edges, gapped keys, missing/invalid records, timestamp mismatch,
worker-order reversal, true post-cutoff records, duplicate/conflicting records,
sample overflow, lineages, missing reference, deterministic statistics and
ties, quantile validity, no-direction semantics, restart/publication identity,
canonical digest vectors, non-blocking queue/drop-newest, immutable snapshots,
unknown schema handling, and legacy-byte immutability. Shutdown, rolling-cache
retention, publisher failure isolation, off-resource absence, and exact
estimator/CBS no-op are also exercised by the direct integration harness.

## 15. Deterministic no-op regression

The accepted preconverted Outdoor01 PointCloud2+IMU input path was used for
three interleaved repetitions of each case:

- A: frozen Stage 1 control, Stage 2A absent;
- B: candidate, Stage 2A section absent;
- C: candidate, `enabled=false`;
- D: candidate, Stage 2A enabled.

All 12 runs consumed the exact event sequence hash:

```text
9112b78205cde928c56f9c64dafa74eb77c37dc4e5447cbc408ff7c3d15c2d50
```

At the preregistered `1e-12` tolerance:

- every within-case and cross-case maximum numerical error was exactly `0.0`;
- scan registration, GLIM poses, consecutive increments, G-to-K means,
  G-to-K covariances, factor/output counts, and captured Kimera outputs matched;
- section absent and explicit disabled were exactly equal;
- metadata enabled was exactly equal to disabled for estimator/CBS outputs.

All 12 serialized legacy belief streams had the same length-framed SHA-256:

```text
a300601bfba9c275275667764d349d7e89caa1c1f1b144d9e54fca990b717d3f
```

This is the primary mathematical and serialization no-op proof.

Artifacts:

- `stage2a_implementation/deterministic_analysis.json`
- `stage2a_implementation/deterministic_run_summary.json`
- `stage2a_implementation/comparisons/`
- `stage2a_implementation/deterministic_runs/`

## 16. Metadata correctness in the deterministic runs

Each enabled repetition published:

- 568 metadata arrays;
- 3,393 entries in legacy belief ordinal order;
- zero unmatched arrays or entry/ordinal mismatches;
- zero stable-edge or exact-payload digest mismatches;
- zero health/metadata drops, cache anomalies, or orphan records;
- complete and valid association for all 3,393 entries;
- `directions_included=false` for every entry.

The same stable edge can recur in rolling arrays, while session UUID,
publication sequence, ordinal, and payload digest uniquely identify each
instance.

## 17. Real-data validation

### Dynamic01, session reference

- 567 metadata arrays, 3,387 entries;
- all 3,387 associations valid and complete;
- all 3,387 entries contain absolute, support, matching, and two-resolution
  summaries;
- 3,099 entries have relative health; the initial 288 explicitly report
  missing reference;
- reference becomes ready at frame 50;
- first maximum reference ratios: rotation `4.297`, translation `4.856`;
- median observed maximum condition: rotation `4.363`, translation `4.160`;
- absolute-degenerate occupancy: rotation `0/6774`, translation `0/6774`;
- median entry-minimum health: rotation `0.904`, translation `0.897`;
- relative-degraded occupancy: rotation `0/6192`, translation `0/6192`;
- zero directions, digest mismatches, drops, cache anomalies, or orphans.

This is the known-healthy validation.

### Outdoor01, session-only

- 568 metadata arrays, 3,393 entries;
- all 3,393 associations valid and complete;
- expected/observed/valid-Hessian descriptive sample total:
  `6786/6786/6786` across rolling belief instances;
- relative-health available sample total: `0`;
- every entry marks the reference missing; no health `1.0` placeholder exists;
- absolute rotation degeneracy: `6774/6786 = 99.823%`;
- absolute translation degeneracy: `6786/6786 = 100%`;
- absolute, support, matching, and two-resolution summaries remain available;
- zero directions, digest mismatches, drops, cache anomalies, or orphans.

The Stage 1 reference audit remains `0/569` accepted candidates for one run and
`0/8535` across 15 prior production repetitions. These denominators are
reference-candidate scans, while the Stage 2A `6786` denominator counts reused
descriptive records across published rolling belief instances.

### Outdoor01, compatible offline/hybrid

- 568 metadata arrays, 3,393 entries;
- all associations, relative/absolute/support/matching summaries available;
- all entries report the compatible hybrid-offline lineage;
- all 569 Stage 1 session adaptation candidates were rejected (568 combined
  rotational+translational absolute degeneracy, one translation-only);
- median entry-minimum health: rotation `0.0402`, translation `0.0193`;
- minimum entry health: rotation `0.0312`, translation `0.0144`;
- relative-degraded occupancy: `6768/6786 = 99.735%` for both groups;
- absolute occupancy agrees with session-only (`99.823%` rotation, `100%`
  translation);
- maximum interval degraded run is two, matching the two health samples per
  0.20-second edge in this deterministic run;
- zero directions, digest mismatches, drops, cache anomalies, or orphans.

Outdoor01 is the available planar/open-geometry degeneracy validation. No
physical axis is inferred. A separately downloaded named corridor dataset was
not required to establish these Stage 2A association and summary semantics.

### Artifacts and plots

- `stage2a_implementation/real_data_analysis/reference_validation_summary.json`
- `stage2a_implementation/real_data_analysis/real_data_summary.json`
- per-sequence `metadata_summary.json`, `metadata_entries.csv`,
  `metadata_array_timeline.csv`, and `metadata_timeline.svg`

The plots show edge publications, associated counts, relative health when
available, absolute occupancy, and ambiguity. Ground truth was not used to
construct or calibrate health.

## 18. Runtime, bandwidth, queue, and memory

Across the three deterministic enabled runs:

- belief-thread immutable descriptor copy: `1.008-1.039 us/array` mean;
- worker association/hash/message work: `68.8-82.0 us/array` mean;
- metadata rate: approximately `10.0 arrays/s`;
- mean serialized array size: `9,621.1 bytes` for approximately six rolling
  belief entries;
- serialized bandwidth: `96.31-96.38 kB/s`;
- maximum health queue depth: 1;
- maximum descriptor queue depth: 1;
- maximum pending descriptors: 1;
- maximum cached health records: 8;
- producer waits: 0;
- dropped metadata arrays: 0;
- dropped health records: 0;
- cache anomalies/orphans: 0;
- worker failures: 0.

Memory is bounded by the configured queue capacity and maximum samples per
edge. No file I/O occurs on the belief-publication thread. No scheduling effect
was suspected after exact deterministic equality and microsecond producer
cost, so Stage 1's already accepted natural-envelope experiment was not
reopened.

## 19. Full GLIM-CBS-Kimera and Rerun validation

A 60-s Outdoor01 run used the normal multithreaded production launch, CBS
enabled, the compatible hybrid profile, GLIM/Kimera factor graph inspectors,
and live Rerun. Roslaunch returned `0`; existing Kimera consumed the unchanged
legacy belief message without a metadata subscriber or code change.

Run report:

```text
stage2a_implementation/full_integration_runs/
20260804-153542_stage2a_outdoor01_hybrid_complete_visualization/summary.md
```

Final compacted Rerun recording:

```text
stage2a_implementation/rerun/
Outdoor01_stage2a_complete_visualization_final.rrd
```

The finalized recording verifies successfully and contains 398 entity paths
and 91,116 rows. It includes the GLIM/Kimera trajectories, synchronized camera
and point-cloud views, CBS traffic, factor-graph inspectors, covariance and
timing diagnostics. Its SHA-256 is:

```text
a8d3ff0bdb9fd1048e35c6ceed752e6bb10ee9630e7ba1adc1ca4124eab24dca
```

## 20. Backward-compatibility result

Passed:

- legacy message MD5s unchanged;
- legacy serialized belief bytes unchanged with metadata enabled;
- old publishers/subscribers connect normally;
- Kimera processes beliefs without metadata support;
- metadata topic is absent when disabled;
- metadata enabled adds only its optional topic and worker activity;
- unknown schema can be ignored by a test consumer;
- metadata errors/drops cannot alter or delay the belief.

Normal catkin regeneration is required because the message package gained new
optional message types, but a consumer that does not use the topic needs no
source or interface change.

## 21. Known limitations and remaining risks

- Stage 2A summaries describe associated scan-local records; they are not an
  interval covariance or independent evidence. Rolling/overlapping edges reuse
  samples by design.
- No direction, axis, basis, projector, subspace, Hessian, or information
  matrix is transported or published.
- The lower quantile is usually unavailable for the current two-scan 0.20-s
  edges because the default requires 10 samples. Minimum and median remain
  explicit.
- Metadata can be dropped independently under overload. Consumers must match
  the exact payload digest and tolerate missing metadata.
- The process-local UUID is intentionally random, so metadata bytes differ
  across restarts even when beliefs are deterministic.
- The full live Rerun raw recorder needed an offline compaction pass to add a
  valid footer after clients disconnected; the finalized RRD verifies.
- Stage 2A has no consumer in Kimera. Scientific usefulness of these
  descriptive summaries remains an experimental question, not an estimator
  guarantee.

None of these limitations invalidates the passive association or no-op proof.

## 22. Explicit Stage 2B/3 boundary

Not implemented or authorized:

- metadata consumption by Kimera or another coordinator;
- SE(3) direction/basis transport;
- projector or subspace aggregation;
- scan-Hessian or information summation;
- interval uncertainty construction;
- covariance inflation or replacement;
- belief weighting, gating, rejection, or delay;
- VGICP, GLIM, smoother, CBS-factor, or Kimera optimization changes;
- DCReg targeted PCG/solver mitigation;
- condition-ratio-to-centimetre/probability calibration.

Any Stage 2B/3 proposal requires a new reviewed specification.

## 23. Final approval

**Stage 2A is approved for engineering integration as an optional passive
metadata-only side channel.**

The approval is supported by unchanged legacy MD5s, byte-identical belief
serialization, exact estimator/CBS output equality at `1e-12`, independently
verified digests, complete real-data associations, explicit missing-reference
behavior, zero queue drops/waits, frozen Stage 1 reference behavior, and a
successful existing-Kimera integration run.

Stop here. Do not begin Stage 2B or Stage 3 without a new specification.
