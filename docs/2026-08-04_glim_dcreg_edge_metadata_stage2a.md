# GLIM DCReg Stage 2A: Conservative CBS-Edge Metadata

Date: 2026-08-04

Status: implemented as an optional, passive, metadata-only side channel.

Stage 1 remains frozen. This document defines only Stage 2A semantics.

## 1. Meaning

One `DcregEdgeHealthMetadata` entry is a conservative descriptive summary of
the frozen Stage 1 scan-local LiDAR observability-health records associated
with one already-published GLIM-to-Kimera CBS relative-belief instance.

It is not:

- interval covariance or information;
- accumulated independent evidence;
- a probability, confidence, or expected metric error;
- a physical weak-axis estimate;
- an instruction to weight, reject, or delay a CBS factor.

Stage 2A never changes the relative-pose mean, covariance, relax factor, CBS
factor, GLIM/VGICP/smoother state, or Kimera state.

## 2. Publication and interval semantics

The existing `liorf/pose_odom_belief_array` is published first. Only then is an
immutable descriptor copied into a bounded, non-blocking metadata queue.

For an edge from pose index `i` to pose index `j`, the fixed Stage 2A rule is:

```text
i < health_pose_index <= j
```

The start sample is excluded and the end sample is included. Adjacent
non-overlapping intervals therefore assign their shared boundary sample to the
earlier interval. Overlapping rolling CBS intervals may reuse the same health
sample, but the metadata never treats reuse as independent evidence.

Expected samples are derived from the actual finite GLIM pose-key/timestamp
registry at belief construction. `expected_sample_count` is not blindly set to
`j-i`. Gaps for pose states that never existed are not missing health records.
`pose_index_span` and `contiguous_pose_keys` expose the distinction.

Every health record is keyed by GLIM pose/frame index and is accepted for that
key only when its timestamp agrees with the registered GLIM pose timestamp
within `pose_timestamp_tolerance_sec`.

## 3. Asynchronous ordering

The health producer assigns a monotonic generation sequence and publishes an
atomic high-watermark after successfully enqueueing each immutable record.
Each belief descriptor snapshots that high-watermark as its health cutoff.

The metadata worker may receive a descriptor before it drains the earlier
health records. It defers that descriptor until it has processed records
through the cutoff. This makes worker scheduling delay different from a truly
late record. Deferral affects metadata only and never delays the belief.

Pending descriptors, both SPSC queues, and the health cache are bounded. Queue
overflow uses `drop_newest`; it increments a counter and discards diagnostics,
never CBS data. Shutdown either finalizes a ready descriptor or explicitly
marks unresolved metadata unavailable. Published beliefs and metadata are
never retrospectively mutated.

## 4. Association validity

Records are deterministically sorted by pose index and then timestamp.

- An exact duplicate key, timestamp, and canonical payload retains the first
  record and increments `duplicate_sample_count`.
- A duplicate key/timestamp with a different payload invalidates that sample
  and sets `REASON_CONFLICTING_DUPLICATE`.
- Equal or non-monotonic timestamps across different keys invalidate temporal
  association.
- A scan at `i` is excluded; a scan at `j` is included.
- No samples or only invalid samples are explicit unavailable states.
- Exceeding `maximum_samples_per_edge` marks the association incomplete and
  leaves health summaries unavailable; it never silently truncates a complete
  interval.

`publish_unavailable=true` keeps one metadata entry per belief ordinal even
when health is unavailable, making failures and missing evidence explicit.

## 5. Scan-local scalar reduction

No direction is published. For one valid aggregate Stage 1 record `k`:

```text
rot_health_scan(k)   = min(smoothed rotational mode health)
trans_health_scan(k) = min(smoothed translational mode health)

rot_absolute_degenerate(k)   = any rotational absolute-mask bit
trans_absolute_degenerate(k) = any translational absolute-mask bit

rot_relative_degraded(k)   = any rotational relative-degraded bit
trans_relative_degraded(k) = any translational relative-degraded bit
```

Clustered-spectrum and low-alignment states are reduced to separate ambiguity
flags. Eigenvectors, axis labels, projectors, subspaces, Hessians, and
information matrices are prohibited from the Stage 2A message.

## 6. Reference lineage

Relative health is summarized only when all health-available records share a
compatible reference lineage. The Stage 2A lineage digest contains the sender
session, complete configuration fingerprint, reference generation, compatible
source family, and initial reference ratios.

The lineage stays stable through slow adaptation and a compatible offline to
hybrid-session-adapted source transition. It changes on session-reference
initialization/reset, incompatible profile or configuration change, process
restart, or health-scale change. This is diagnostics-only and does not alter
Stage 1 reference values or equations.

Mixed source labels with one compatible lineage are reported as
`REFERENCE_MIXED_COMPATIBLE`. Different lineages set
`reference_consistent=false`; relative-health summaries become unavailable,
while absolute degeneracy, support, matching, and ambiguity may remain valid.

## 7. Interval summaries and denominators

Relative-health summaries use only health-available records with one
compatible lineage. Rotation and translation each publish minimum, median,
degraded numerator and denominator, occupancy fraction, longest consecutive
degraded run, and the worst-sample timestamp. Equal minima choose the earliest
pose index and then earliest timestamp.

An invalid, missing, conflicting, or health-unavailable record breaks a
consecutive degraded run.

The lower quantile uses deterministic nearest rank:

```text
rank = max(1, ceil(lower_quantile * n))
```

It is valid only when `n >= minimum_samples_for_lower_quantile` and its rank is
greater than one. Otherwise its validity flag is false and its value is quiet
NaN. This prevents a short interval minimum from being relabeled as a
separately informative quantile.

Denominators are explicit:

- absolute-degenerate fraction: valid Hessian records eligible for the
  absolute mask;
- relative-degraded fraction: health-available records sharing one lineage;
- ambiguity fractions: valid Hessian records eligible for interpretation;
- convergence and linear-solve fractions: records eligible for matching-state
  evaluation;
- every support/cost numeric summary: its own finite-value denominator.

Fractions are descriptive occupancy, never probabilities. Relative health,
absolute degeneracy, support, matching quality, and ambiguity remain separate;
no combined score exists.

The Stage 1 statement “Outdoor01 accepted 0/569 reference candidates” uses
scan-reference candidate rows as its denominator. Stage 2A entry denominators
instead count the records associated with each CBS belief instance. They answer
different questions and must not be compared as one rate.

## 8. Belief-instance identity

The existing belief message has no unique instance ID and rolling arrays may
republish an edge. Stage 2A generates a UUID v4 once per enabled sender process,
a monotonic belief-array publication sequence, and the ordinal in that array.

Two SHA-256-derived 128-bit values are published:

- stable edge digest: schema prefix, sender session UUID, sender ID,
  from/to indices, and exact endpoint timestamp bits;
- belief-payload digest: versioned prefix, array header timestamp and frame ID,
  sender ID, from/to indices, exact endpoint timestamp bits, six relative-mean
  values, 36 covariance values, relax factor, ordinal, and publication
  sequence.

Canonical encoding is big-endian, uses fixed-width integers, length-prefixed
UTF-8 byte strings, and exact IEEE-754 binary64 bit patterns. SHA-256 uses a
small local implementation tested against published standard vectors. A
consumer can recompute the payload digest from the original belief instance;
digest mismatch must prevent association.

## 9. ROS architecture and compatibility

Stage 2A adds three versioned optional messages and one helper summary:

- `liorf/DcregNumericSummary`
- `liorf/DcregResolutionSupportSummary`
- `liorf/DcregEdgeHealthMetadata`
- `liorf/DcregEdgeHealthMetadataArray`

The separate default topic is:

```text
/glim/cbs/dcreg_edge_health_metadata
```

`schema_version=1` describes the message representation.
`semantics_version=1` fixes conservative scalar aggregation, `i<k<=j`, no
directions, no information aggregation, separate-topic publication, and
drop-newest behavior.

The legacy files `pose_odom_belief.msg` and
`pose_odom_belief_array.msg` are unchanged, as are their ROS MD5 values.
Unknown metadata versions are optional and safely ignorable.

## 10. Configuration

The optional section belongs inside `odometry_estimation` in GLIM's odometry
configuration:

```json
"odometry_estimation": {
  "dcreg_health": { "mode": "log_only" },
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
}
```

The complete example is
`src/cbsms/config/glim_dcreg_edge_metadata_stage2a.example.json`.

The section is absent by default. When absent or disabled, no Stage 2A UUID,
callback, cache, queue, worker, publisher, digest computation, or topic exists.
Enabling requires frozen Stage 1 `dcreg_health.mode=log_only`; otherwise Stage
2A emits one warning and self-disables without enabling Stage 1.

Unsupported fields, invalid ranges, or any overflow policy other than
`drop_newest` disable only Stage 2A and preserve the estimator.

## 11. Cache, queue, and failure policy

The health cache retains every record required by the oldest edge in the
current rolling belief descriptor and any pending descriptor. Pruning follows
actual pose keys rather than the nominal 0.20-second edge horizon. The hard
record bound is derived from queue capacity and maximum samples per edge.

Health and belief-descriptor producers perform one non-blocking SPSC queue
push. No producer wait, file I/O, health aggregation, digest calculation, or
ROS metadata serialization happens on the belief-publication thread. The
background worker performs association, validation, lineage checks,
summarization, hashing, and publication.

Any callback, queue, cache, digest, worker, or publisher failure affects
metadata only. Existing belief publication continues. Cumulative drops,
orphan records, and cache anomalies are public operational counters.

## 12. Missing and ambiguous cases

- No reference: relative health is invalid/NaN; absolute/support/matching data
  may still be valid.
- Incompatible reference lineage: relative health is invalid; other groups are
  retained.
- No samples/no valid Hessians: explicit counts and reason bits; never health
  `1.0`.
- Clustered modes or low alignment: ambiguity occupancy only; never an axis.
- Queue/cache overflow: metadata is dropped or marked incomplete; the belief is
  untouched.
- Metadata after a dropped belief, or belief without metadata: digest/sequence
  identity lets a consumer reject the orphan side independently.
- Process restart: a new UUID prevents digest collisions despite sequence reset.

## 13. Stage 2B/3 boundary

Future work may investigate transported directions, projectors, timestamped
sample sets, or covariance-aware policies only after a separate specification
and validation. Stage 2A contains no SE(3) transport, interval Hessian,
information summation, covariance inflation, weighting, rejection, or Kimera
consumer. Nothing in this message is authorized for estimator control.
