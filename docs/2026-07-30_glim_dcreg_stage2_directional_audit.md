# GLIM DCReg Stage 2: Directional Belief Audit

Date: 2026-07-30

## Purpose

Stage 2 asks a narrower question than Stage 1:

> Do DCReg-weak directions actually have larger GLIM error, and does the
> covariance GLIM sends to Kimera represent that directional error?

This stage is still read-only. It does not modify GLIM optimization, scan
precision, factors, poses, the outgoing covariance, CBS messages, Kimera, or
Rerun geometry.

## Added Artifacts

GLIM now emits the complete local DCReg basis:

```text
GLIM_SCAN_DCREG_BASIS_ROW
```

The CBS bridge can optionally log the exact mean and 6-by-6 covariance after
sender-health processing and immediately before message publication:

```text
glim_dcreg_belief_shadow_enable:=true
GLIM_DCREG_BELIEF_SHADOW_ROW
```

The report writes:

```text
parsed/glim_dcreg_basis.csv
parsed/glim_dcreg_belief_shadow.csv
parsed/glim_dcreg_directional_audit.csv
parsed/glim_dcreg_directional_summary.csv
parsed/glim_dcreg_directional_threshold_comparison.csv
parsed/glim_dcreg_directional_correlation.csv
```

Repeated outgoing revisions are reduced to the latest parsed row for each
`(from_index, to_index)` edge.

## Tangent And Projection Convention

Both inputs use the GTSAM Pose3 local tangent order:

```text
[rx, ry, rz, tx, ty, tz]
```

A focused GTSAM test verifies that retracting GLIM's target pose by a local
increment gives the identical local increment in the corresponding
`from.between(to)` measurement. Therefore the DCReg basis at `to_index` and the
outgoing relative covariance are directly compatible.

For a translation eigenvector `v_i`, the GLIM-only projection is:

```text
e_i       = v_i^T e_translation
sigma_i^2 = v_i^T Sigma_tt v_i
z_i       = |e_i| / sigma_i
NIS_i     = e_i^2 / sigma_i^2
```

The report sweeps DCReg spectral-ratio thresholds:

```text
3, 10, 30, 50, 100, 200
```

## Outdoor01 Ground-Truth Restriction

Outdoor01 is not a full 6-DoF ground-truth pose source in the supplied TUM
file. Every quaternion in the evaluated interval is exactly identity. The
file therefore cannot identify rotational error or a ground-truth local body
frame.

The valid Outdoor01 audit is translation-only:

1. fit one SE2 yaw plus translation between the GLIM and RTK world
   trajectories;
2. transform each RTK endpoint displacement back into the GLIM world;
3. rotate it into the GLIM `from` pose local frame;
4. reuse the measured relative rotation so the residual contains no claimed
   rotation truth;
5. project only the translational residual and covariance.

Rotation rows are deliberately omitted.

The official M3DGR repository identifies Outdoor01 as RTK ground truth and
publishes RTK receiver accuracy of 0.008 m horizontal and 0.015 m vertical:

```text
https://github.com/sjtuyinjie/M3DGR
```

The report includes an explicitly labelled sensitivity calculation that adds
those values as independent endpoint noise. This is not an assertion that the
processed GT file has exactly that covariance.

## Isolated Outdoor01 Control

Final run:

```text
runs/20260730-144847_m3dgr_outdoor01_dcreg_directional_shadow_isolated_valid_60s_rate025
```

Rerun recording:

```text
m3dgr_outdoor01_dcreg_directional_shadow_isolated_valid_20260730_144833
```

The run used:

```text
bag_duration:=60
bag_rate:=0.25
glim_cbs_mode:=observe_only
glim_dcreg_belief_shadow_enable:=true
glim_dcreg_visualization_stride:=5
cbs_health_aware_enable:=false
Kimera N100
```

The bridge remained enabled so GLIM could publish the exact beliefs being
audited. GLIM matched incoming K2G traffic but injected zero K2G factors.
Therefore this is an isolated GLIM control, not a persistent-CBS GLIM run.

The run completed with return code zero:

- 569/569 valid DCReg rows;
- 569 basis rows;
- 3,387 shadow markers in the raw console log;
- 2,854 complete shadow revisions parsed;
- 567 unique outgoing GLIM edges;
- 566 edges with both estimator poses available;
- 1,698 valid translation mode projections;
- zero missing DCReg rows, basis rows, or GT endpoints;
- zero invalid covariance projections.

Rosbag's in-place console progress interrupted 533 redundant shadow revisions.
Because each edge was revised several times, all 567 unique edges were still
recovered; one leading edge lacked a recorded absolute estimator pose and was
excluded. Future long audits should record the outgoing belief topic or a
dedicated shadow file instead of relying only on the shared console stream.

An earlier run was deliberately stopped after the live log exposed
insufficient default stream precision for epoch timestamps:

```text
runs/20260730-144611_m3dgr_outdoor01_dcreg_directional_shadow_isolated_60s_rate025
```

The C++ logger was changed to 17-digit precision before the final run.

## Findings

At threshold 10, each evaluated edge contributed two weak translation modes
and one strong mode:

| class | mode samples | error RMS | GLIM sigma p50 | GLIM-only NIS mean |
|---|---:|---:|---:|---:|
| weak | 1,132 | 0.02386 m | 0.00369 m | 41.77 |
| strong | 566 | 0.00973 m | 0.00213 m | 20.87 |

The main observable result is independent of covariance calibration:

```text
weak / strong translation error RMS = 2.45
```

So DCReg weakness is not just an abstract Hessian label on this run. The two
weak translation eigenmodes had materially larger actual RTK-relative error
than the strong mode.

The continuous relationship is real but not deterministic:

```text
Spearman(log spectral ratio, log absolute error) = 0.339
```

Therefore the spectral ratio is a useful directional risk indicator, not an
error probability or a complete trust score.

Thresholds 3, 10, and 30 produce almost the same split on this sequence.
Threshold 50 gives the largest weak/strong RMS ratio, 2.59, while threshold
100 retains fewer weak samples and gives 2.50. The value 10 is not uniquely
calibrated by this experiment.

With GLIM covariance alone, both groups are much wider than their published
sigmas. This comparison includes RTK reference noise, so it must not be read
as a pure GLIM NEES result.

Under the independent-endpoint RTK accuracy sensitivity:

| class | reference-aware NIS mean | coverage `|z| <= 1.96` |
|---|---:|---:|
| weak | 4.01 | 82.9% |
| strong | 0.208 | 100% |

The sensitivity removes the apparent strong-mode inconsistency but not the
weak-mode tail. This supports directional covariance inflation as the next
shadow hypothesis, but not belief rejection and not a solver change.

## What This Does Not Establish

- It does not validate rotation covariance; Outdoor01 has no usable
  orientation truth.
- It does not prove the exact RTK covariance or temporal independence.
- It does not show that every weak direction should be rejected by Kimera.
- It does not calibrate a production inflation multiplier.
- It does not validate persistent-CBS feedback behavior.

## Recommended Next Stage

Keep estimator behavior unchanged and run one more shadow study:

1. choose a sequence with verified full-pose GT and a verified
   GT-body-to-MID360 extrinsic for rotation;
2. evaluate the same direction projections on a disjoint calibration/control
   split;
3. test a shadow-only covariance candidate that adds uncertainty in the DCReg
   weak translational subspace;
4. compare coverage and downstream Kimera factor NIS before enabling any
   message mutation;
5. only then decide whether to apply anisotropic inflation.

The present evidence argues against a binary rule such as “weak means discard
the GLIM belief.” It argues for retaining the belief and representing weak
directions with more uncertainty if the result repeats under a fully
calibrated reference.

## Validation

Build:

```bash
docker exec cbsms_ws bash -lc \
  'source /opt/ros/noetic/setup.bash; cd /workspace/cbs_gtsam4.3; \
   catkin build cbs glim glim_ros kimera_vio kimera_vio_ros gtsam_points \
     --no-status --summarize -j8 -p1'
```

All 19 packages succeeded. `glim_ros` retained the pre-existing OpenCV 3.4/4.2
link warning.

Focused C++ tests:

```bash
docker exec cbsms_ws bash -lc \
  'source /opt/ros/noetic/setup.bash; cd /workspace/cbs_gtsam4.3; \
   catkin run_tests glim --no-status --summarize'
```

All five focused tests pass, including target/between tangent equivalence and
offline SE3 Logmap parity.

Python audit tests:

```bash
cd src/cbsms/tools
python3 -m unittest -v test_glim_dcreg_directional_audit.py
```

All four tests pass, including the position-only GT path and reference-noise
projection.

## Source Map

```text
src/glim/src/glim/odometry/odometry_estimation_cpu.cpp
src/glim_ros1/include/glim_ros/cbs_bridge.hpp
src/glim_ros1/src/glim_ros/cbs_bridge.cpp
src/glim_ros1/launch/glim_rosnode.launch
src/glim_ros1/launch/m3dgr_glim_kimera_experiment.launch
src/glim_ros1/launch/m3dgr_glim_kimera_live_rerun_raw_cbs.launch
src/glim_ros1/launch/m3dgr_mid360_glim_kimera_live_rerun_raw_cbs.launch
src/cbsms/tools/glim_dcreg_directional_audit.py
src/cbsms/tools/test_glim_dcreg_directional_audit.py
src/cbsms/tools/cbsms_experiment.py
```

