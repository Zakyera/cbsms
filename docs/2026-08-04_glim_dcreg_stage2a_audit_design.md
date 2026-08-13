# GLIM DCReg Stage 2A audit and mathematical design

Date: 2026-08-04

Status: design audit complete; no Stage 2 implementation performed.

## 1. Executive decision

Stage 2A can proceed as a metadata-only implementation, but only with a
conservative interval summary.

The first implementation should:

- associate scan-health samples with CBS edges by pose key using
  `from_index < health_frame_index <= to_index`;
- publish a separate, versioned, optional ROS topic;
- summarize per-scan scalar health, validity, absolute degeneracy,
  persistence, support, and matching quality;
- publish no directional axis, eigenvector, projector, interval Hessian,
  information matrix, uncertainty, covariance, or sensor weighting;
- leave the existing G-to-K mean and covariance byte-for-byte unchanged;
- remain disabled by default.

Transported directional metadata is mathematically derivable, but it is not
ready for the first Stage 2 implementation. Full SE(3) transport mixes
rotation and translation, while Stage 1 health is split into rotational and
translational Schur spaces. In addition, scan Hessians are correlated through
the rolling map and cannot be summed as independent interval information.

No source, configuration, message, build, test, or documentation files were
modified during the Stage 2A audit itself. This memo was added afterward at
the user's request so that the complete audit could be read outside the chat
interface.

## 2. Repository and source state

The workspace is a multi-repository tree rather than one Git repository.

| Repository | Branch | HEAD | State during audit |
|---|---|---|---|
| `src/glim` | `cbs-gtsam43-noetic` | `6c4189e117c61015a79001388debb27413c78fa6` | clean |
| `src/glim_ros1` | `cbs-gtsam43-noetic` | `2dea515e9e38743b6fc73e7dfdf7e45d3ae5bf6f` | clean |
| `src/cbsms` | `cbsms/gtsam-4.3-develop` | `873f60fa748df97bf9f5f7f9dcdb825b4f5ed30c` | existing documentation changes and `tools/__pycache__/` |
| `src/cbs` | `cbsms/gtsam-4.3-develop` | `994b1d6a5c05fb38dd1b0731c6430ec11d2f1ff0` | clean |
| `src/Kimera-VIO` | `cbsms/gtsam-4.3-develop` | `d0b2a31adf17ced8995994373f6b286679e26799` | clean |
| `src/Kimera-VIO-ROS` | `cbsms/gtsam-4.3-develop` | `09d7e17b5d27f97f622f285f0f23a61e6748278d` | pre-existing `scripts/__pycache__/` |
| `src/liorf` | `cbsms/gtsam-4.3-develop` | `4286f982694dda550d1343ff21679f8ec2e708ba` | clean |

Existing `cbsms` changes at the start of the audit were:

```text
 M docs/2026-07-31_glim_dcreg_health_stage1.md
 M docs/2026-07-31_glim_dcreg_health_stage1_report.md
?? docs/2026-08-04_glim_dcreg_health_stage1_closeout.md
?? tools/__pycache__/
```

The tracked diff contained 9 insertions and 4 deletions. `git diff --check`
was clean. The audit did not alter those existing files.

The authoritative Stage 1 closeout is:

```text
src/cbsms/docs/2026-08-04_glim_dcreg_health_stage1_closeout.md
```

## 3. Exact audited code paths

### 3.1 GLIM scan and health production

- PointCloud2 header time becomes `RawPoints::stamp` in
  `src/glim/include/glim/util/ros_cloud_converter.hpp:230`.
- The same measurement stamp becomes the GLIM pose timestamp in
  `src/glim/src/glim/odometry/odometry_estimation_imu.cpp:329`.
- The same stamp is stored on `EstimationFrame` in
  `src/glim/src/glim/odometry/odometry_estimation_imu.cpp:364`.
- Final local scan-registration optimization occurs in
  `src/glim/src/glim/odometry/odometry_estimation_cpu.cpp:830`.
- The final scan pose and matching cost are obtained at
  `src/glim/src/glim/odometry/odometry_estimation_cpu.cpp:837`.
- The DCReg snapshot is constructed at
  `src/glim/src/glim/odometry/odometry_estimation_cpu.cpp:854`.
- Per-factor Hessians are extracted at the final local values at
  `src/glim/src/glim/odometry/odometry_estimation_cpu.cpp:887`.
- The passive monitor is invoked at
  `src/glim/src/glim/odometry/odometry_estimation_cpu.cpp:958`.
- The health callback executes before the smoother update at
  `src/glim/src/glim/odometry/odometry_estimation_cpu.cpp:1075`.
- The current scan enters the rolling maps only afterward, at
  `src/glim/src/glim/odometry/odometry_estimation_cpu.cpp:1118`.
- The frozen Stage 1 diagnostics structure is defined in
  `src/glim/include/glim/odometry/scan_dcreg_diagnostics.hpp:315`.
- Aggregate Hessian and Schur processing starts in
  `src/glim/src/glim/odometry/scan_dcreg_diagnostics.cpp:1910`.

### 3.2 CBS edge construction

- Pose timestamps are copied from the smoother update in
  `src/glim_ros1/src/glim_ros/cbs_bridge.cpp:1041`.
- Outgoing publication occurs after the smoother update in
  `src/glim_ros1/src/glim_ros/cbs_bridge.cpp:1267`.
- Outgoing belief message construction starts at
  `src/glim_ros1/src/glim_ros/cbs_bridge.cpp:2481`.
- Time-horizon pair selection is implemented at
  `src/glim_ros1/src/glim_ros/cbs_bridge.cpp:2705`.
- Relative means and marginal covariances are constructed at
  `src/glim_ros1/src/glim_ros/cbs_bridge.cpp:2782`.
- Relative covariance Schur processing is implemented in
  `src/cbs/include/cbs/utils/relative_pose_covariance.h:258`.
- The existing ROS belief message is
  `src/liorf/msg/pose_odom_belief.msg`.

### 3.3 GTSAM conventions

- Right-composed Lie retraction `T * Exp(delta)` is implemented in
  `src/gtsam/gtsam/base/Lie.h:141`.
- `between(a,b) = inverse(a) * b` and its Jacobians are implemented in
  `src/gtsam/gtsam/base/Lie.h:74`.
- BetweenFactor local error is implemented in
  `src/gtsam/gtsam/slam/BetweenFactor.h:112`.
- Pose3 rotation-first ordering is documented in
  `src/gtsam/gtsam/geometry/Pose3.h:270`.
- The built workspace has `GTSAM_POSE3_EXPMAP=ON`.

### 3.4 Kimera reception and frame conversion

- Incoming ROS belief conversion begins at
  `src/Kimera-VIO-ROS/src/KimeraVioRos.cpp:2330`.
- Relative-pose conjugation and covariance adjoint conversion occur at
  `src/Kimera-VIO-ROS/src/KimeraVioRos.cpp:2373`.
- The external-to-base TF direction is established at
  `src/Kimera-VIO-ROS/src/KimeraVioRos.cpp:2416`.
- Kimera timestamp matching is implemented at
  `src/Kimera-VIO/src/backend/VioBackend.cpp:1735`.
- Sender interval-duration validation starts at
  `src/Kimera-VIO/src/backend/VioBackend.cpp:2072`.

## 4. Exact G-to-K CBS edge semantics

### 4.1 Pose selection

The production M3DGR profile uses:

```text
cbs_odom_sender_mode = time_horizon_window
cbs_odom_horizon_sec = 0.20
cbs_odom_horizon_tolerance_sec = 0.06
cbs_odom_max_horizon_pairs_per_update = 25
```

For every candidate end pose `j`, GLIM searches all earlier available poses
and selects the start pose `i` minimizing:

```text
abs((t_j - t_i) - 0.20)
```

The pair is accepted only when:

```text
abs((t_j - t_i) - 0.20) <= 0.06
```

The allowed nominal duration is therefore approximately 0.14 to 0.26
seconds.

This is not the `glim_belief_timestamp_tolerance_sec=0.12` parameter. That
parameter applies to GLIM's reception of incoming K-to-G beliefs. Kimera's
G-to-K endpoint matching uses its own 0.20-second endpoint tolerance and a
0.06-second interval-duration tolerance.

### 4.2 Timestamp clock

The edge endpoint timestamps are GLIM pose-state timestamps. They originate
from:

```text
PointCloud2.header.stamp
  -> RawPoints::stamp
  -> PreprocessedFrame/EstimationFrame::stamp
  -> new_stamps[X(k)]
  -> pose_timestamps_sec_[k]
  -> belief.from_stamp_sec / belief.to_stamp_sec
```

They are measurement timestamps, not publication time or wall-clock time.

The belief-array header stamp is the latest GLIM state timestamp at
publication. An old edge republished in a rolling window can therefore have
endpoint timestamps older than the array header.

### 4.3 Relative mean

The mean is exactly:

```text
Z_ij = inverse(T_i) * T_j
```

through `from_pose.between(to_pose)`.

It is serialized as:

```text
mu_ij = Pose3::Logmap(Z_ij)
```

in GTSAM Pose3 ordering:

```text
[omega_x, omega_y, omega_z, v_x, v_y, v_z]
```

The receiver reconstructs the pose through
`Pose3::Expmap(relative_mu)`.

### 4.4 Relative covariance

The production mode is `schur_relative_between`.

The code constructs a zero-residual, unit-noise BetweenFactor whose local
error is:

```text
e(T_i, T_j) = Logmap(inverse(Z_ij) * (inverse(T_i) * T_j))
```

At the nominal estimate, `e = 0`. The joint marginal information of `T_i`
and `T_j` is transformed and marginalized to obtain covariance for this
relative local error.

The outgoing covariance is therefore:

- not the covariance of the start pose;
- not the covariance of the end pose;
- not directly the covariance of the absolute vector `Logmap(Z_ij)`;
- the covariance of a right-local perturbation around `Z_ij`.

Its coordinate order is rotation then translation, and its right tangent is
naturally expressed in the end-body `j` coordinates.

### 4.5 Overlap and repetition

With a 0.20-second horizon and scans arriving more frequently than 5 Hz:

- adjacent emitted intervals usually overlap;
- the same GLIM pose participates in multiple beliefs;
- a scan-health sample may belong to several overlapping intervals;
- the rolling outgoing window can republish the same sender edge on later
  updates;
- the mean and covariance of a repeated edge may change as the smoother
  changes.

The horizon sender does not apply `new_edge_once` duplicate suppression.

### 4.6 Missing or duplicate state behavior

- Poses with missing or non-finite timestamps are skipped.
- Poses missing from the active smoother are skipped.
- A covariance exception skips the belief.
- Equal timestamps cannot form a positive-duration start/end pair.
- There is no explicit error for distinct pose keys carrying equal
  timestamps.
- Startup publishes nothing until a valid pair exists.
- There is no retrospective shutdown publication for an interval never
  emitted.

## 5. Exact scan-health semantics

For scan `k`, the Stage 1 record uses:

```text
frame_index = k
stamp_sec = frames[k]->stamp
T_world_imu = pose derived from the final local scan correction
```

The Hessians are linearized at the final local VGICP scan-registration state.

The timestamp is the source scan's PointCloud2 measurement/start timestamp.
It is not:

- `scan_end_time`;
- the monitor execution time;
- the smoother update time;
- ROS publication time.

### 5.1 What the Hessian describes

Each VGICP factor is unary on `X(k)`, with the rolling voxel map treated as a
fixed target.

The Stage 1 object is therefore:

> Instantaneous current-scan-to-rolling-map registration curvature at the
> final local scan-registration solution.

It is not:

- an inter-scan motion factor;
- an interval factor from `i` to `j`;
- a marginal covariance for `X(k)`;
- a Hessian of the complete GLIM smoother;
- a direct Hessian of `inverse(T_i) * T_j`.

### 5.2 Tangent frame

The source points have already been deskewed and transformed into the IMU body
frame. The unary Pose3 variable uses GTSAM's right perturbation:

```text
T_k(delta_k) = T_k * Exp(delta_k)
```

Therefore:

- `H_level_0`, `H_level_1`, and `H_lidar` use the current IMU/body `k`
  local tangent;
- ordering is rotation then translation;
- rotational and translational Schur eigenvectors are expressed in this
  scan-local frame.

### 5.3 Timing relative to the smoother

The health Hessian is evaluated after the local scan-registration LM
optimization but before the resulting scan factors are inserted into the GLIM
smoother.

The outgoing CBS poses are calculated later from the smoother after its
update. Consequently:

- the sample key and timestamp correspond exactly to `X(k)`;
- the Hessian's body-coordinate axes remain the current sensor axes;
- the linearization pose is the final local scan-registration pose, not
  necessarily the post-smoother estimate used later for the CBS mean.

Any future directional transport must use the exact smoother pose snapshot
used for the CBS belief while documenting that the original curvature was
evaluated at the pre-smoother scan-registration solution.

### 5.4 Correlation and reused information

The current scan is inserted into the rolling map only after health capture.
That prevents direct self-matching within the same sample.

However:

- the target map contains overlapping historical scans;
- consecutive scans match against strongly overlapping rolling maps;
- historical points can affect multiple consecutive health samples;
- the two voxel resolutions use the same source scan;
- both resolutions are derived from related historical map content.

Consecutive health samples are statistically correlated, and the two
resolution Hessians are not independent measurements.

The Stage 1 sum:

```text
H_lidar = H_level_0 + H_level_1
```

is the curvature of GLIM's composite local matching objective. It must not be
interpreted as a sum of independent Fisher information.

## 6. Recommended interval association rule

For a CBS edge from pose `i` to pose `j`, associate scan-health records by
GLIM pose key:

```text
i < k <= j
```

Validate each record by checking:

```text
abs(health_stamp[k] - glim_pose_stamp[k]) <= timestamp_tolerance
```

Recommended timestamp tolerance:

```text
1.0e-6 seconds
```

Pose-key membership is primary. Timestamps are a consistency check.

### 6.1 Why this boundary is correct

The health sample for scan `i` describes the scan registration that created
the start state. It does not describe motion after that state, so it is
excluded.

The health sample for scan `j` participates in creating the end state, so it
is included.

This convention also makes adjacent non-overlapping intervals partition
boundary scans cleanly.

### 6.2 Rejected alternatives

| Rule | Problem |
|---|---|
| `t_i <= t_k < t_j` | Includes the scan that created the start state and excludes the scan that created the end state |
| Nearest scan to each endpoint | Drops interior degradation and persistence |
| Timestamp-only inclusion | Ambiguous under equal, non-monotonic, or rounded timestamps |
| Sum all contributing Hessians | Incorrectly implies independent interval information |

The recommended key rule includes all processed scan updates after `i`
through `j`, but only as descriptive samples, not as additive information.

### 6.3 Deterministic examples

Given:

```text
X10  10.0
X11  10.1
X12  10.2
X13  10.3
X14  10.4
```

Edge `X10 -> X12` includes:

```text
X11, X12
```

It excludes `X10`.

Edge `X12 -> X14` includes:

```text
X13, X14
```

`X12` belongs only to the first adjacent interval.

For overlapping edges:

```text
X10 -> X12
X11 -> X13
```

`X12` belongs to both. This reuse is allowed because metadata is descriptive;
it must not be treated as independent evidence across edges.

### 6.4 Missing, delayed, and duplicate records

- Expected sample count is `to_pose_index - from_pose_index`.
- Minimum valid sample count defaults to 1.
- Maximum samples per edge defaults to 64.
- If the maximum is exceeded, mark association incomplete; do not silently
  truncate and claim complete coverage.
- If no records exist, publish unavailable metadata when configured.
- If records exist but all are invalid, publish counts and invalid reasons
  with health unavailable.
- Exact duplicate `(frame_index, timestamp)` records retain the first and
  count later duplicates.
- Duplicate records with conflicting payloads invalidate the affected sample
  and set `conflicting_duplicate`.
- Distinct frame keys with identical or non-monotonic timestamps invalidate
  temporal association.
- Out-of-order records received before the edge cutoff are sorted by
  `(frame_index, timestamp)` before summarization.
- Records arriving after a belief instance is published do not generate a
  correction.
- A later repeated publication of the same stable edge is a new belief
  instance and may have a new metadata record.
- No retrospective mutation of a previously published belief or metadata
  instance is allowed.

### 6.5 Sample age

Define sample age relative to the edge end:

```text
age_k = t_j - t_k
```

The default maximum age should be automatic:

```text
maximum_sample_age_sec = -1
```

This means:

```text
0 <= age_k <= (t_j - t_i) + timestamp_tolerance
```

This is better than measuring age from publication time because the outgoing
rolling window republishes older edges.

Record retention is separate. The metadata buffer must retain scan records at
least as far back as the oldest edge eligible for the current outgoing CBS
window.

## 7. Common tangent frame and adjoint derivation

### 7.1 Pose convention

Let:

```text
T_k = T_world_from_body_k
```

map coordinates from body `k` into the world frame.

GTSAM applies a right perturbation:

```text
T_k' = T_k * Exp(delta_k)
```

The CBS relative pose is:

```text
Z_ij = inverse(T_i) * T_j
```

### 7.2 Relative covariance tangent

Perturb both endpoint poses on the right:

```text
T_i' = T_i * Exp(delta_i)
T_j' = T_j * Exp(delta_j)
```

Then:

```text
Z_ij' = Exp(-delta_i) * Z_ij * Exp(delta_j)
```

To first order:

```text
delta_Z = -Ad_{inverse(Z_ij)} * delta_i + delta_j
```

At zero residual, the BetweenFactor Jacobians are therefore:

```text
H_i = -Ad_{inverse(Z_ij)}
H_j = I
```

This confirms that `delta_Z` is the right tangent of `Z_ij`, expressed in the
end-body `j` coordinates.

### 7.3 Recommended common frame

For any future interval direction representation, use:

```text
relative_pose_right_tangent_at_Z_ij
```

equivalently:

```text
end_pose_j_body_right_tangent
```

This is preferable because:

- it matches the outgoing covariance's tangent;
- the scan at `j` already uses that body frame;
- it remains sender-local and does not depend on the receiver;
- it avoids global gauge dependence;
- Kimera's existing frame conversion applies the correct adjoint to this
  convention.

A start-pose tangent would require another conversion before comparison with
the covariance. A world tangent would introduce gauge dependence. A Kimera
frame cannot be known until receiver matching and TF conversion occur.

### 7.4 Transport from scan k to end frame j

Define:

```text
T_j_from_k = inverse(T_j) * T_k
A_j_from_k = Ad_{T_j_from_k}
```

The same infinitesimal physical motion expressed in the two right tangents
satisfies:

```text
delta_j = A_j_from_k * delta_k
```

For Pose3 ordering `[omega, v]`, if `T = (R, t)`:

```text
       [ R          0 ]
Ad_T = [              ]
       [ [t]x R     R ]
```

The lower-left block proves that rotational and translational components
generally mix.

### 7.5 Covariance transport

If a covariance were expressed in scan frame `k`:

```text
Sigma_j = A * Sigma_k * transpose(A)
```

This is only a coordinate-transport equation. Stage 1 does not produce a
calibrated covariance.

### 7.6 Information or quadratic-curvature transport

Because:

```text
delta_k = inverse(A) * delta_j
```

the corresponding quadratic form transforms as:

```text
H_j = inverse(transpose(A)) * H_k * inverse(A)
```

Transporting a Stage 1 Hessian does not make it statistically calibrated or
independent.

### 7.7 Directional-mode transport

A tangent direction transports as:

```text
v_j = A * v_k
```

It must not automatically be called an eigenvector of `H_j`. A non-orthogonal
adjoint changes the Euclidean metric and generally does not preserve the
eigendecomposition.

A Stage 1 rotational mode would first be embedded as:

```text
[v_R, 0]
```

and a translational mode as:

```text
[0, v_t]
```

After full SE(3) transport, either can become mixed.

### 7.8 Weak-subspace transport

For a source basis `B_k`:

```text
B_j_unormalized = A * B_k
```

To obtain an orthonormal basis under a declared metric, perform QR or SVD:

```text
A * B_k = Q * R
P_j = Q * transpose(Q)
```

`A * P * transpose(A)` is generally not an orthogonal projector.

A sign-invariant and ordering-invariant subspace projector is possible only
after defining the 6D metric used to combine radians and metres. No canonical
metric exists in Stage 1. Introducing a characteristic length would be a new
scientific design choice.

This is the central reason Stage 2A should not publish transported directions
or projector accumulations.

### 7.9 Kimera confirmation

Kimera converts a relative pose from an external body frame into its base
frame by conjugation:

```text
Z_base = C * Z_external * inverse(C)
```

For a right perturbation:

```text
delta_base = Ad_C * delta_external
```

The existing code applies:

```text
Sigma_base = Ad_C * Sigma_external * transpose(Ad_C)
```

This confirms the right-local convention.

In the current M3DGR shared-IMU profile, GLIM and Kimera both use
`camera_imu_link`, so this transform is normally identity.

## 8. Finite-difference validation specification

These tests belong in the future implementation. None were added during the
audit.

### 8.1 BetweenFactor endpoint Jacobians

Choose nontrivial poses `T_i` and `T_j`, including rotation and translation,
and define:

```text
Z = inverse(T_i) * T_j
```

For each canonical basis vector `e_m`, evaluate central differences of:

```text
f(T_i, T_j) = Logmap(inverse(Z) * (inverse(T_i) * T_j))
```

Expected:

```text
df / d(delta_i) = -Ad_{inverse(Z)}
df / d(delta_j) = I
```

Use `Pose3::retract` for perturbations. Test epsilon values `1e-5`, `1e-6`,
and `1e-7`, and require convergence to the analytic Jacobian. At the stable
step, target maximum absolute error `1e-8` and relative error `1e-7`.

### 8.2 Scan-to-end tangent transport

Create `T_k`, `T_j`, and a small `delta_k`. Define the equivalent world-frame
rigid displacement:

```text
D_world = T_k * Exp(delta_k) * inverse(T_k)
```

Apply it to `T_j`:

```text
T_j' = D_world * T_j
```

Numerically recover:

```text
delta_j_fd = Logmap(inverse(T_j) * T_j')
```

Expected:

```text
delta_j_fd = Ad_{inverse(T_j) * T_k} * delta_k + O(norm(delta_k)^2)
```

### 8.3 Known coupling case

Let:

```text
inverse(T_j) * T_k = (I, [1, 0, 0])
```

For:

```text
delta_k = [0, 1, 0, 0, 0, 0]
```

the expected result is:

```text
delta_j = [0, 1, 0, 0, 0, 1]
```

A pure y-axis rotational direction gains a positive z translational
component.

### 8.4 Ninety-degree orientation case

Let:

```text
T_k = identity
T_j = (R_z(90 degrees), 0)
```

Then:

```text
A = Ad_{R_z(-90 degrees)}
```

A pure local positive-x translational direction at `k`:

```text
[0, 0, 0, 1, 0, 0]
```

must become local negative-y at `j`:

```text
[0, 0, 0, 0, -1, 0]
```

### 8.5 Covariance and information congruence

Generate an SPD matrix:

```text
Sigma_k = B * transpose(B) + epsilon * I
```

Verify:

```text
Sigma_j = A * Sigma_k * transpose(A)
```

and:

```text
inverse(Sigma_j)
  = inverse(transpose(A)) * inverse(Sigma_k) * inverse(A)
```

Target relative Frobenius error `1e-12` in double precision for the direct
matrix identity.

## 9. Multi-scan aggregation analysis

### 9.1 Option A: conservative summary metadata

Recommendation: use this option.

For each scan, define permutation-invariant scalar summaries:

```text
s_R(k) = minimum over modes of smoothed rotational health
s_t(k) = minimum over modes of smoothed translational health
```

Then summarize across the interval using:

- minimum;
- deterministic lower quantile;
- median where useful;
- degraded fraction;
- longest degraded run;
- worst timestamp.

Advantages:

- no eigenvector averaging;
- no tangent transport required;
- sign and mode-order invariant;
- robust to clustered eigenspaces;
- makes no independence claim;
- directly describes which scan-health evidence occurred during the edge.

Limitations:

- does not state a common physical weak direction;
- does not produce interval uncertainty;
- minimum is sensitive to a single outlier, so minimum and lower quantile
  should both be published;
- correlated scans remain correlated, so fractions are descriptive occupancy,
  not binomial probabilities.

### 9.2 Option B: transported modes or projectors

Recommendation: do not use in the first implementation.

Sign-invariant projectors and clustered-subspace projectors are mathematically
preferable to raw eigenvector averaging. However:

- full adjoint transport mixes rotational and translational components;
- a 6D normalization metric is required to combine radians and metres;
- a transported weak vector is not generally an eigenvector of the
  transported Hessian;
- weighted projector sums would be directional occupancy, not information or
  covariance;
- one severe sample versus many mild samples requires an explicitly justified
  weighting policy.

This may become a later diagnostic research mode after a physical metric and
semantics are reviewed.

### 9.3 Option C: transported information aggregation

Recommendation: reject this option.

Even with correct adjoint transport, summing scan Hessians is not
probabilistically defensible because:

1. Consecutive scans use overlapping rolling maps.
2. Historical map points are reused.
3. The map is treated as fixed, omitting map uncertainty.
4. The two resolutions reuse the same source scan.
5. The scan Hessian constrains an instantaneous unary pose, not the complete
   edge `inverse(T_i) * T_j`.
6. Intermediate poses have motion, IMU, prior, CBS, and smoother dependencies.
7. Proper interval information would require the full factor graph, correct
   motion Jacobians, nuisance-variable marginalization, and an explicit
   correlation model.

A transported sum would double-count evidence and answer the wrong
mathematical question.

### 9.4 Option D: full timestamped sample set

This option is scientifically honest but is not the best first default.

A bounded list would preserve:

- timestamps;
- per-scan validity;
- health;
- ambiguity;
- optional future modes.

It increases message size and postpones rather than resolves interval
semantics. It is suitable as a future experimental or debugging topic, not the
coordinator-facing default.

## 10. Recommended Stage 2A representation

The metadata should represent:

> A conservative summary of the Stage 1 scan-local LiDAR
> observability-health records temporally associated with one emitted G-to-K
> CBS edge.

It must explicitly not represent:

- interval covariance;
- interval information;
- expected pose error;
- probability of correctness;
- calibrated confidence;
- a weighting command;
- a common physical weak axis;
- independent repeated evidence.

### 10.1 Direction policy

Stage 2A should include no physical direction:

```text
include_directional_modes = false
```

Clustered modes and low alignment are represented only by ambiguity fractions
and flags.

### 10.2 Relative-health summaries

For all health-available samples, publish:

- minimum rotational health;
- lower-quantile rotational health;
- minimum translational health;
- lower-quantile translational health;
- worst rotational sample timestamp;
- worst translational sample timestamp;
- rotational relative-degraded fraction;
- translational relative-degraded fraction;
- longest rotational degraded run;
- longest translational degraded run.

Use smoothed Stage 1 health as the primary interval signal. Raw-health
summaries may optionally be included but must be separately named.

Recommended deterministic lower quantile: nearest rank.

```text
rank = max(1, ceil(q * n))
```

Default:

```text
q = 0.10
```

### 10.3 Absolute degeneracy

Absolute degeneracy remains available without a healthy reference:

- fraction of valid scans having any absolutely degenerate rotational mode;
- fraction having any absolutely degenerate translational mode;
- total degenerate-mode count if desired;
- worst condition ratios;
- fraction of scans with any absolute mask.

### 10.4 Reference state

Include:

- `health_available`;
- `reference_ready`;
- `reference_consistent`;
- `reference_source`;
- counts by reference source;
- profile/configuration fingerprint.

Recommended source values:

```text
none
session
offline
hybrid_session_adapted
mixed
```

If samples within one interval use incompatible or mixed reference lineages,
relative interval health is unavailable. Absolute degeneracy and support can
still be reported.

### 10.5 Support and matching quality

These remain separate field groups.

Per VGICP resolution:

- source point count minimum/median;
- inlier count minimum/median;
- inlier fraction minimum/median;
- initial cost minimum/median;
- final cost minimum/median;
- cost reduction minimum/median;
- VGICP resolution.

Aggregate Hessian diagnostics may include descriptive minimum/median values
for:

- trace;
- normalized scale;
- rank;
- condition.

These values must not be combined into the health score.

### 10.6 Missing values

Every numeric field with possible absence needs an accompanying validity flag
or group-validity flag. When a fixed numeric representation is required, use
quiet NaN, consistent with Stage 1.

Unavailable must never be encoded as `1.0`.

## 11. Metadata publication architecture

### 11.1 Existing message extension

Recommendation: reject this option.

ROS1 message compatibility uses a message MD5. Adding fields to
`pose_odom_belief` would change the MD5 and prevent old publishers and
subscribers from connecting without recompilation.

It would also couple optional diagnostics to the authoritative belief path.

### 11.2 Recommended path

Create a new, versioned optional message and topic, conceptually:

```text
liorf/DcregEdgeHealthMetadata
liorf/DcregEdgeHealthMetadataArray
/glim/cbs/dcreg_edge_health_metadata
```

Using the existing `liorf` message package is the smallest future
message-build change because all CBS bridge participants already depend on
it. Adding a new message does not change the MD5 of existing belief messages.

Old GLIM, CBS, Kimera, LIO-RF, and visualization nodes remain unchanged
because they do not subscribe to the new topic.

### 11.3 Publication relationship

For each outgoing belief array:

1. Publish the existing belief array exactly as now.
2. Make a fixed-size immutable descriptor of each already-built belief.
3. Non-blockingly enqueue descriptors for metadata processing.
4. Build and publish the metadata array on a background worker.
5. On overflow or failure, drop metadata only.

The existing belief must be published first. Metadata computation must not
delay it to preserve diagnostics.

### 11.4 Edge and belief-instance association

The current message has no session UUID or publication sequence. Pose indices
reset after restart, and replaying a bag can reuse timestamps.

The new metadata needs both stable edge identity and exact belief-instance
association.

Stable edge identity:

```text
sender_session_uuid
source_agent
from_pose_index
to_pose_index
from_stamp_sec
to_stamp_sec
```

Exact belief-instance association:

```text
belief_publication_sequence
belief_ordinal
belief_array_header_stamp
belief_payload_digest
```

`belief_payload_digest` should be a 128-bit prefix of SHA-256 over a
versioned canonical byte encoding of:

- array header stamp and frame ID;
- source agent;
- from/to indices;
- exact IEEE-754 bit patterns of from/to timestamps;
- six mean values;
- 36 covariance values;
- relax factor.

The metadata worker receives a fixed immutable copy and computes the digest
off the odometry thread.

A metadata consumer can hash the received belief identically, preventing
metadata from attaching to the wrong belief payload.

A stable `edge_key_digest` may additionally group repeated publications of
the same sender edge within one session.

### 11.5 Proposed field groups

Identity:

- schema version;
- semantics version;
- sender session UUID;
- sender ID;
- belief publication sequence;
- belief ordinal;
- belief array header stamp;
- from/to pose index;
- edge start/end timestamp;
- belief payload digest;
- stable edge digest;
- configuration fingerprint.

Association:

- association valid;
- association complete;
- expected sample count;
- observed sample count;
- valid Hessian sample count;
- health-available sample count;
- unavailable sample count;
- duplicate sample count;
- late sample count;
- overflowed sample count;
- invalid reason code.

Reference:

- reference ready;
- reference consistent;
- reference source;
- per-source counts;
- profile fingerprint.

Relative health:

- minimum and lower-quantile rotational/translation health;
- degraded fractions;
- longest runs;
- worst timestamps.

Absolute DCReg:

- rotational and translational degenerate fractions;
- worst rotational and translational condition ratio.

Ambiguity:

- clustered-rotation fraction;
- clustered-translation fraction;
- low-rotation-alignment fraction;
- low-translation-alignment fraction;
- `directions_included=false`.

Support and matching quality:

- separate per-resolution summary structures;
- convergence and linear-solve fractions;
- aggregate Hessian scale/rank diagnostics.

Operational:

- metadata creation timestamp;
- queue drop counters;
- metadata sequence;
- source diagnostic schema version.

## 12. Invalid and ambiguous behavior

| Condition | Required behavior |
|---|---|
| No interval samples | Publish counts with incomplete association and unavailable health |
| Only invalid Hessians | Absolute and relative health unavailable; publish reasons and valid support fields |
| No healthy reference | Relative health unavailable/NaN; absolute DCReg and support may publish |
| Incompatible offline profile | Reference unavailable unless a valid session reference exists |
| Mixed/incompatible reference sources | Relative interval summary unavailable; absolute/support remain |
| Only absolutely degenerate samples | Publish absolute fractions; relative health only with a compatible reference |
| Clustered eigenspaces | Set ambiguity flags; no axis label or direction |
| Low alignment | Set ambiguity flags; no pure-axis statement |
| Health arrives after belief instance | No correction or mutation; increment late count |
| Metadata queue overflow | Drop newest metadata item, increment counter, never block belief |
| Duplicate metadata instance | Suppress by session, publication sequence, and ordinal |
| Belief dropped, metadata received | Treat metadata as orphan and expire it |
| Metadata dropped, belief received | Belief proceeds normally without metadata |
| Clock mismatch | Association invalid; do not select an arbitrary nearest sample |
| Process restart | New session UUID; sequence restarts within the new session |
| Unknown metadata schema | Ignore metadata; do not reject the belief |
| Metadata worker failure | Disable/drop metadata only; no estimator or CBS change |

## 13. Proposed public configuration

No configuration was added during this task. The proposed future section is:

```yaml
dcreg_edge_metadata:
  enabled: false
  publication_mode: separate_topic
  topic: /glim/cbs/dcreg_edge_health_metadata

  interval_association: open_start_closed_end_pose_key
  pose_timestamp_tolerance_sec: 1.0e-6
  common_frame: relative_pose_right_tangent_at_Z_ij
  aggregation_mode: conservative_summary

  minimum_valid_samples: 1
  maximum_samples_per_edge: 64
  maximum_sample_age_sec: -1.0
  lower_quantile: 0.10

  publish_unavailable: true
  include_absolute_degeneracy: true
  include_relative_health: true
  include_support_summary: true
  include_matching_summary: true
  include_directional_modes: false

  queue_capacity: 256
  overflow_policy: drop_newest
```

Validation rules:

- `enabled=false` creates no callback, queue, worker, publisher, buffer, or
  topic.
- `publication_mode` must be `separate_topic`.
- `interval_association` must be the audited pose-key open/closed rule.
- `aggregation_mode` must be `conservative_summary`.
- `common_frame` must use the audited relative/end right tangent, even though
  Stage 2A publishes no direction.
- `minimum_valid_samples >= 1`.
- `maximum_samples_per_edge >= minimum_valid_samples`.
- `0 < lower_quantile <= 0.5`.
- `pose_timestamp_tolerance_sec >= 0`.
- `maximum_sample_age_sec=-1` means the actual edge duration; another value
  must be at least the edge duration.
- `queue_capacity > 0`.
- Only `drop_newest` is accepted initially.
- `include_directional_modes=true` is invalid in Stage 2A.
- Metadata requires Stage 1 `dcreg_health.mode=log_only`.
- If Stage 1 is off, metadata self-disables with a clear startup diagnostic;
  it must not secretly enable the monitor.
- If relative health is unavailable, metadata may still publish absolute
  degeneracy and support summaries.

## 14. Synthetic validation plan

| Test | Expected result |
|---|---|
| Scan exactly at start | Excluded |
| Scan exactly at end | Included |
| Adjacent intervals | Boundary sample belongs to the earlier interval only |
| Overlapping intervals | Overlap samples appear in both; no independence claim |
| No samples | Counts zero; health unavailable |
| Only invalid samples | Observed count nonzero, valid count zero, health unavailable |
| Missing reference | Absolute fields available when Hessian is valid; relative fields unavailable |
| Delayed/out-of-order | Sort records before cutoff; no correction after cutoff |
| Duplicate timestamp | Exact duplicate suppressed; conflicting duplicate invalid; distinct keys with equal time invalidate temporal association |
| Weak local X translation, no rotation | Translation scalar health falls; rotation remains healthy; no direction published |
| Same world weak direction, robot rotates 90 degrees | Scalar degradation remains; future transported positive X maps to negative Y in the specified example |
| Eigenvector sign flip | Scalar summary and any future projector unchanged |
| Eigenvalue ordering swap | Minimum/quantile summary unchanged |
| Clustered 2D weak space | Cluster flag set; no axis claim |
| Rotating weak direction | Health and persistence reported; no averaged vector |
| One severe outlier | Minimum captures it; nearest-rank lower quantile is deterministic |
| Persistent mild degradation | Lower quantile and degraded occupancy reflect persistence |
| Full SE(3) coupling | With translation `[1,0,0]`, `omega_y` produces `v_z=+1` under the specified transform |
| Finite-difference adjoint | Matches analytic adjoint/Jacobians within specified tolerances |
| Metadata disabled | No metadata objects/topic; existing belief serialization unchanged |
| Metadata enabled | Existing mean and covariance bytes unchanged |
| Old consumers | Existing message/topic works without metadata support |
| Wrong-edge attachment | Payload digest mismatch rejects association |
| Restart/sequence reuse | New session UUID prevents collision |
| Queue overflow | Existing belief publishes; newest metadata is dropped and counted |

Additional regression tests must compare the serialized existing
`pose_odom_belief_array` bytes with metadata disabled and enabled.

## 15. Real-data validation plan

### 15.1 Dynamic01 or another healthy sequence

Expected:

- reference source becomes `session`;
- reference-ready state appears at the same Stage 1 frame as before;
- interval health summaries remain near 1;
- absolute-degenerate fractions remain low;
- metadata reproduces scan membership derived from raw Stage 1 records.

### 15.2 Outdoor01 with session-only reference

Expected:

- `reference_ready=false`;
- relative interval health unavailable;
- absolute DCReg fractions and support still publish;
- no `1.0` placeholder health;
- accepted reference candidate behavior remains 0/569.

### 15.3 Outdoor01 with compatible offline/hybrid reference

Expected:

- reference source begins as `offline`;
- relative health summaries are available;
- degraded scans do not adapt the reference;
- absolute and relative deterioration timelines align with the per-scan
  Stage 1 diagnostics.

### 15.4 Corridor or planar sequence

Expected:

- increased absolute-degenerate fractions;
- lower rotational and/or translational health quantiles when a valid
  reference exists;
- longer degraded runs;
- frozen reference adaptation.

### 15.5 Changing-orientation degeneration

Expected:

- scalar health summaries remain stable while degradation persists;
- ambiguity fractions describe clustered or low-alignment samples;
- no false fixed world-axis label is generated.

### 15.6 Required plots and tables

- CBS edge timeline with pose keys and endpoint timestamps;
- exact health scan keys assigned to each edge;
- expected, observed, valid, and available sample counts;
- reference source and readiness;
- minimum and 10th-percentile rotational/translation health;
- absolute-degenerate rotational/translation fractions;
- relative-degraded fractions and longest runs;
- worst scan timestamp;
- per-resolution support and matching-quality summaries;
- cluster and poor-alignment fractions;
- comparison against raw Stage 1 scan rows;
- metadata queue drops and orphan associations.

Ground truth may later be plotted against metadata to evaluate usefulness. It
must not be used to construct the metadata or convert health into metric
error.

## 16. Performance and storage estimate

At approximately 10 LiDAR updates per second:

- unique new horizon edges: approximately 10 per second;
- current outgoing array: up to 25 rolling edges per update;
- mirrored metadata entries: up to approximately 250 entries per second;
- typical scans per 0.20-second edge: 2 to 3;
- maximum configured samples per edge: 64.

Estimated entry size for conservative summaries is approximately 350 to 500
bytes. A 25-edge array is therefore approximately 9 to 13 KB.

Estimated bandwidth:

```text
90 to 130 KB/s at 10 outgoing arrays/s
```

Estimated serialized storage for a 403-second run:

```text
roughly 36 to 52 MB
```

CSV or JSON logging may be two to three times larger.

CPU cost should be small because Stage 2A performs no eigendecomposition or
Hessian aggregation. Metadata work consists mainly of:

- bounded record lookup;
- scalar minimum, quantile, and count calculations;
- fixed payload copying;
- digest calculation;
- ROS serialization.

Recommended architecture:

```text
Stage 1 health callback
    -> fixed immutable health-summary snapshot
    -> non-blocking bounded queue

CBS belief publication
    -> existing belief published first
    -> fixed immutable edge/belief descriptor
    -> non-blocking bounded queue

background metadata worker
    -> association and conservative summaries
    -> payload digest
    -> separate ROS metadata publication
```

Default queue capacity: 256 publication descriptors. Depending on the final
fixed record size, expected memory is approximately 2 to 4 MB.

Overflow policy:

```text
drop newest metadata
increment dropped count
never block GLIM or CBS
```

No metadata file I/O belongs on the odometry thread.

## 17. Required design decisions

| # | Decision |
|---:|---|
| 1 | Edge inclusion is `from_index < health_frame_index <= to_index` with timestamp consistency checks |
| 2 | Health time is the source PointCloud2/GLIM pose measurement timestamp |
| 3 | Future common frame is the right tangent of `Z_ij`, equivalent to end-body `j` |
| 4 | Transport uses `A = Ad_{inverse(T_j) * T_k}`; covariance uses `A Sigma A^T`; information uses `A^-T H A^-1` |
| 5 | No directions are transported or published in initial Stage 2A |
| 6 | Scan information matrices must not be aggregated |
| 7 | Projector aggregation is deferred pending an explicit 6D metric and occupancy semantics |
| 8 | Publish conservative health, absolute-degeneracy, validity, ambiguity, support, and matching summaries |
| 9 | Use a separate versioned optional metadata topic |
| 10 | Associate using session UUID, publication sequence/ordinal, edge tuple, and exact belief-payload digest |
| 11 | Missing reference means relative health unavailable; absolute degeneracy may still publish |
| 12 | Clustered eigenspaces produce ambiguity flags only, never pure-axis labels |
| 13 | Late records never mutate an already-published belief or metadata instance |
| 14 | Existing belief message/topic and consumers remain untouched; unknown metadata versions are ignored |
| 15 | Metadata defaults disabled; conservative summary; no directions; bounded drop-newest queue |
| 16 | Acceptance requires exact message no-op, deterministic association, adjoint finite differences, overflow isolation, and unchanged Stage 1 reference behavior |
| 17 | Stage 2 ends at optional diagnostics; covariance or factor weighting requires a separate future review |

## 18. Explicit Stage 2 boundary

Approved future Stage 2A work may:

- read frozen Stage 1 diagnostics;
- associate them with outgoing CBS edge instances;
- publish optional conservative metadata on a separate topic;
- report unavailable, ambiguous, invalid, and dropped metadata explicitly.

It may not:

- modify the G-to-K mean;
- modify the G-to-K covariance;
- modify any CBS message currently used for beliefs;
- weight, reject, or gate a CBS factor;
- change GLIM, VGICP, the smoother, or Kimera;
- sum scan Hessians into interval information;
- publish Stage 1 health as uncertainty;
- convert condition ratios into metric error;
- implement covariance inflation.

Future covariance weighting would require a separately validated
representation addressing correlation, interval factor semantics, full SE(3)
transport, and the rotational/translational unit metric. This memo does not
design or approve it.

## 19. Remaining mathematical risks

1. Stage 1's separate rotational and translational Schur modes are not closed
   under full SE(3) adjoint transport.
2. A 6D directional projector requires a deliberate metric relating radians
   and metres.
3. Scan Hessians omit rolling-map uncertainty and are correlated across time.
4. The scan Hessian is evaluated at the local scan-registration solution,
   while the CBS mean uses later smoother estimates.
5. A repeated sender edge may have updated mean/covariance on later
   publications; association must identify belief instances, not only stable
   edge keys.
6. The current belief message has no reliable session or publication ID; the
   separate metadata must solve this with session identity and an exact
   payload digest.
7. Metadata may be unavailable even when the belief is valid. This must remain
   normal pass-through behavior.

None of these risks invalidates conservative scalar summary metadata. They do
invalidate interval information aggregation and initial directional metadata.

## 20. Final recommendation

```text
GO:
    Stage 2A metadata-only implementation is sufficiently specified,
    provided the first implementation is restricted to the conservative
    separate-topic representation defined in this memo.
```
