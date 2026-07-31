# M3DGR CBS + GLIM + Kimera Persistent Handoff

Date: 2026-07-29

This is the authoritative handoff for continuing the M3DGR CBS work in a new
chat. It supersedes the July 9 handoff where the two disagree:

```text
/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/cbsms/docs/2026-07-09_m3dgr_cbs_handoff.md
```

The July 9 note is still useful as history, but its old active-window temporary
prompt is no longer the current experiment default.

## 1. Workspace And Environment

Host workspace:

```text
/home/yeranis/repos/V4RL/cbs_gtsam4.3
```

Docker container:

```text
cbsms_ws
```

Container workspace:

```text
/workspace/cbs_gtsam4.3
```

Start the container:

```bash
docker start cbsms_ws
```

Run ROS commands inside the container:

```bash
docker exec cbsms_ws bash -lc \
  'source /opt/ros/noetic/setup.bash; \
   source /workspace/cbs_gtsam4.3/devel/setup.bash; \
   <command>'
```

The source tree is shared between host and container. Host paths begin with
`/home/yeranis/repos/V4RL/cbs_gtsam4.3`; the same files appear in the container
under `/workspace/cbs_gtsam4.3`.

Important path rule:

- The experiment wrapper runs on the host.
- ROS launch runs in the container.
- `--bag-path` must therefore be a container path beginning with
  `/workspace/cbs_gtsam4.3`.
- `--gt-path` may be the host path used by the wrapper.

Do not pass a host `/home/yeranis/...bag` path to a container launch.

## 2. Project Goal

The project evaluates collaborative belief sharing between two independent
estimators on the same platform:

- GLIM: LiDAR-inertial odometry using the M3DGR MID360 LiDAR and IMU.
- Kimera-VIO: monocular visual-inertial odometry using the M3DGR camera and
  camera IMU.
- CBS: bidirectional exchange of short-horizon relative pose beliefs.

Directions:

```text
G2K = GLIM belief received by Kimera
K2G = Kimera belief received by GLIM
```

For endpoint poses `T_i` and `T_j`, the sender publishes:

```text
Z_ij = T_i^-1 T_j
mu_ij = Logmap(Z_ij)
Sigma_ij = relative pose covariance
```

The receiver creates a `gtsam::BetweenFactor<gtsam::Pose3>`. Its full
six-dimensional residual and full covariance participate in:

```text
cost = r^T Sigma^-1 r
```

The ordering is always GTSAM `Pose3` tangent ordering:

```text
[rot_x, rot_y, rot_z, trans_x, trans_y, trans_z]
```

Do not ROS-remap or reorder this covariance.

## 3. Current Decisions And Defaults

The current requested M3DGR CBS mode is persistent:

```text
glim_cbs_mode:=inject_persistent
cbs_odom_factor_mode:=persistent
cbs_use_temporary_cbs_linear_factors:=false
```

Persistent means received CBS factors remain ordinary nonlinear factors and
are allowed to contribute to the fixed-lag marginal prior.

Current common defaults:

```text
enable_cbs_bridge:=true
cbs_odom_covariance_mode:=schur_relative_between
cbs_health_aware_enable:=false
cbs_odom_duration_gate_enable:=true
cbs_odom_horizon_sec:=0.20
cbs_odom_horizon_tolerance_sec:=0.06
cbs_belief_receive_start_delay_sec:=20.0
glim_cbs_odom_receiver_match_mode:=duration_aware_edge
glim_cbs_odom_max_horizon_pairs_per_update:=25
kimera_cbs_odom_max_horizon_pairs_per_update:=6
glim_cbs_k2g_odom_factor_covariance_scale:=1.0
kimera_cbs_l2k_odom_factor_covariance_scale:=1.0
```

The current factor-graph inspection experiments override both outgoing caps to
`6` to control runtime.

The standard Kimera M3DGR profile has:

```text
src/Kimera-VIO/params/M3DGRMonoOriginal/BackendParams.yaml
nr_states: 25
```

The current Outdoor01 fixed-lag investigations use:

```text
src/Kimera-VIO/params/M3DGRMonoOriginalN100/BackendParams.yaml
nr_states: 100
```

Always state explicitly whether a run uses N25 or N100. The most recent
validated persistent factor-graph run used N100.

Historical modes remain available for controlled ablations:

```text
active_window_temporary
temporary_linear
persistent
```

Do not erase the fact that older M3DGR experiments used
`active_window_temporary` and the older S3E baseline used `temporary_linear`.

## 4. MID360 Sensor Setup

The current canonical M3DGR setup uses the MID360, not the older Livox Avia
path.

Topics:

```text
MID360 raw LiDAR: /livox/mid360/lidar
MID360 IMU:       /livox/mid360/imu
Converted cloud:  /m3dgr/mid360/points
GLIM body frame:  livox_frame
Kimera body frame: camera_imu_link
```

Canonical MID360 GLIM runtime config:

```text
/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/runtime_configs/config_m3dgr_mid360_native_imu_clean
```

Critical calibration fact:

```text
acc_scale = 9.80665
```

M3DGR MID360 IMU acceleration is stored in `g`. Without this scale, GLIM was
badly misconfigured. The successful clean full-Dynamic01 MID360 baseline used:

```text
runs/20260715-145152_m3dgr_dynamic01_clean_glim_mid360_accscale9p80665_full
evo APE RMSE: 0.1259 m
```

MID360 GLIM uses identity LiDAR-to-IMU in `livox_frame`. CBS converts GLIM
beliefs to Kimera's `camera_imu_link` using the M3DGR calibration:

```text
parent: camera_imu_link
child:  livox_frame

translation:
  x = -0.003497660
  y = -0.417688000
  z =  0.198242000

quaternion:
  qx =  0.367905538
  qy = -0.326718737
  qz =  0.586665212
  qw =  0.643214048
```

Canonical sensor note:

```text
/home/yeranis/repos/V4RL/cbs_gtsam4.3/notes/m3dgr_mid360_profile.md
```

## 5. Datasets Present

Container paths:

```text
/workspace/cbs_gtsam4.3/src/datasets/M3DGR/Dynamic01/Dynamic01.bag
/workspace/cbs_gtsam4.3/src/datasets/M3DGR/Dynamic01/Dynamic01.txt

/workspace/cbs_gtsam4.3/src/datasets/M3DGR/Outdoor01/Outdoor01.bag
/workspace/cbs_gtsam4.3/src/datasets/M3DGR/Outdoor01/Outdoor01.txt

/workspace/cbs_gtsam4.3/src/datasets/M3DGR/Wheel-float01/Wheel-float01.bag
/workspace/cbs_gtsam4.3/src/datasets/M3DGR/Wheel-float01/Wheel-float01.txt
```

Current interpretation:

- Dynamic01 is an easier indoor sequence for Kimera. Its active graph did not
  show the severe Outdoor01 backward revisions in the latest N100 tests.
- Outdoor01 is the main diagnostic sequence. It produces more realistic K2G
  covariance growth and exposes Kimera fixed-lag nonlinear behavior.
- Wheel-float01 is available but has not received the same detailed analysis.

## 6. Main Launches And Experiment Wrapper

Canonical MID360 live Rerun launch:

```text
/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/glim_ros1/launch/m3dgr_mid360_glim_kimera_live_rerun_raw_cbs.launch
```

Generic/legacy Avia live launch:

```text
/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/glim_ros1/launch/m3dgr_glim_kimera_live_rerun_raw_cbs.launch
```

Experiment wrapper:

```text
/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/cbsms/tools/cbsms_experiment.py
```

Canonical wrapper profile:

```text
m3dgr_mid360_glim_kimera_live_rerun
```

General setup note:

```text
/home/yeranis/repos/V4RL/cbs_gtsam4.3/notes/m3dgr_live_rerun_experiment_setup.md
```

## 7. Canonical Persistent Outdoor01 Run

Open Rerun on the host before launching the experiment.

This is the exact configuration of the latest validated 60 sensor-second
Outdoor01 run. At `bag_rate=0.25`, allow approximately four minutes of wall
time and use a large timeout padding:

```bash
/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/cbsms/tools/cbsms_experiment.py run \
  --workspace /home/yeranis/repos/V4RL/cbs_gtsam4.3 \
  --experiment-profile m3dgr_mid360_glim_kimera_live_rerun \
  --name m3dgr_outdoor01_n100_persistent_g2k_inspector_fix_valid_60s_rate025 \
  --bag-path /workspace/cbs_gtsam4.3/src/datasets/M3DGR/Outdoor01/Outdoor01.bag \
  --gt-path /home/yeranis/repos/V4RL/cbs_gtsam4.3/src/datasets/M3DGR/Outdoor01/Outdoor01.txt \
  --duration 60 \
  --timeout-padding 300 \
  --enable-cbs-bridge \
  --shutdown-on-bag-finish \
  --rerun-visualizer-enable \
  --rerun-world-alignment-enable \
  --use-kimera-rviz \
  --kimera-visualize \
  --extra-arg bag_rate:=0.25 \
  --extra-arg rerun_recording_id:=m3dgr_outdoor01_n100_persistent_g2k_inspector_fix_valid_60s_rate025 \
  --extra-arg glim_cbs_odom_max_horizon_pairs_per_update:=6 \
  --extra-arg kimera_cbs_odom_max_horizon_pairs_per_update:=6 \
  --extra-arg kimera_params_folder:=/workspace/cbs_gtsam4.3/src/Kimera-VIO/params/M3DGRMonoOriginalN100 \
  --extra-arg glim_factor_graph_inspector_enable:=true \
  --extra-arg glim_factor_graph_inspector_stride:=1 \
  --extra-arg glim_factor_graph_inspector_context_windows:=2 \
  --extra-arg kimera_rerun_factor_graph_inspector_enable:=true \
  --extra-arg kimera_rerun_factor_graph_inspector_stride:=1 \
  --extra-arg kimera_rerun_factor_graph_inspector_include_smart_factors:=true \
  --extra-arg kimera_rerun_factor_graph_inspector_max_smart_factors:=20
```

Use a new unique `--name` and `rerun_recording_id` for every new run.

For a lighter normal dashboard, set inspector strides to `5` and disable smart
factor expansion. Stride `1` is expensive and should remain at `bag_rate=0.25`.

For Dynamic01, use the same command without the Outdoor01 `--bag-path` and
`--gt-path` overrides because the profile defaults to Dynamic01. Change the run
name and recording ID.

The wrapper timeout is wall-clock based. For 60 sensor seconds at `0.25x`,
`--timeout-padding 90` is too short and terminates the run before 60 sensor
seconds. Use `300`.

## 8. Rerun Dashboard

Container-to-host Rerun proxy:

```text
rerun+http://172.17.0.1:9876/proxy
```

Host-side blueprint proxy:

```text
rerun+http://127.0.0.1:9876/proxy
```

Reapply and select the canonical dual factor-graph tab:

```bash
/home/yeranis/.local/share/pipx/venvs/rerun-sdk/bin/python \
  /home/yeranis/repos/V4RL/cbs_gtsam4.3/src/cbsms/tools/send_raw_cbs_dashboard_blueprint.py \
  --app-id cbsms \
  --url rerun+http://127.0.0.1:9876/proxy \
  --active-tab dual-factor-graph
```

Dashboard code:

```text
/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/cbsms/tools/send_raw_cbs_dashboard_blueprint.py
```

Canonical dashboard notes:

```text
/home/yeranis/repos/V4RL/cbs_gtsam4.3/notes/rerun_raw_cbs_dashboard.md
/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/cbsms/docs/2026-07-20_cbs_on_dual_factor_graph_rerun_setup.md
```

The `CBS-On Dual Factor Graph Inspector` contains:

- GLIM active factor graph with K2G factors.
- Kimera active factor graph with G2K factors.
- Synchronized camera.
- GLIM, Kimera, and ground-truth trajectories.
- Active external CBS factor counts.
- Outgoing relative rotation and translation covariance traces.
- Per-axis relative covariance diagonals.
- Receiver merge arrows for local-before, external, and final-after relative
  transformations.

The camera is published by a dedicated Rerun image process. Sparse dots on the
Rerun image timeline reflect the configured image publication rate, normally
5 Hz, and do not by themselves prove that rosbag playback is slowing.

## 9. Factor Graph Inspector Semantics

### GLIM

Received K2G factors:

```text
/glim/factor_graph_inspector/spatial/factors/cbs_k2g_pose_between
/glim/factor_graph_inspector/counts/cbs_k2g_pose_between_factors
```

Native scan pose-between factors are cyan. Received K2G factors are magenta.

Use the explicit latest-pose axes:

```text
/glim/factor_graph_inspector/spatial/states/latest_pose_axes/{x,y,z}
```

Do not use the retired detached `latest_pose_frame` entity to judge placement.

### Kimera

Persistent G2K factors:

```text
/kimera/factor_graph_inspector/spatial/factors/cbs_g2k_pose_between
/kimera/factor_graph_inspector/counts/cbs_g2k_pose_between_factors
```

Persistent G2K factors are now identified by BPSAM slot provenance and rendered
in magenta. They are no longer inferred only from generic factor type.

Kimera state colors:

```text
blue   = active pose states
red    = velocity variables, displayed with an offset
yellow = bias variables, displayed with an offset
```

The grey Kimera line is the append-only published incremental trajectory. It is
not the current active factor graph. The blue active states are the current
fixed-lag optimizer values and may be revised when new measurements arrive.

Kimera visual constraints are `SmartStereoProjectionPoseFactor` instances, not
native `BetweenFactor<Pose3>` edges. The inspector normally aggregates their
count. Enabling sampled smart factors draws only the configured bounded sample.

### Merge arrows

Kimera:

```text
/kimera/factor_graph_inspector/spatial/cbs_merge/arrows/local_before
/kimera/factor_graph_inspector/spatial/cbs_merge/arrows/external_g2k
/kimera/factor_graph_inspector/spatial/cbs_merge/arrows/final_after
```

GLIM:

```text
/glim/factor_graph_inspector/spatial/cbs_merge/arrows/local_before
/glim/factor_graph_inspector/spatial/cbs_merge/arrows/external_k2g
/glim/factor_graph_inspector/spatial/cbs_merge/arrows/final_after
```

These arrows describe the exact receiver edge shown in their labels. They do
not necessarily end at the newest active pose because transport and timestamp
matching introduce delay.

## 10. Persistent G2K Inspector Fix

Persistent G2K factors existed in Kimera's optimizer, but the inspector
originally received only the temporary CBS factor collection. Persistent
factors therefore appeared as generic pose-between factors or were missing.

The current implementation adds a validated read-only snapshot of tracked BPSAM
CBS factors:

```text
src/cbs/include/cbs/bpsam/bpsam.h
  BPSAM::trackedCbsOdomFactors()

src/cbs/src/bpsam/bpsam.cpp
  BPSAM::trackedCbsOdomFactors()

src/cbs/include/cbs/bpsam/incremental_fixed_lag_bpsam_smoother.h
  trackedCbsOdomFactors()

src/Kimera-VIO/src/backend/VioBackend.cpp
  passes tracked factors to the Rerun backend output
```

The latest valid N100 run produced:

```text
294 Kimera inspector frames
maximum active persistent G2K factors: 99

last inspector snapshot:
  SmartStereo:       3257
  IMU:                100
  bias random walk:   100
  marginal prior:       1
  generic pose-between: 0
  persistent G2K:      99
```

This validates the inspector classification. It does not change optimization.

## 11. Correct Relative Covariance Calculation

The old implementation used:

```text
Sigma_old = Lambda_tt^-1
```

This is the covariance of `to` conditional on `from` being fixed. It is not the
covariance of the nonlinear relative pose measurement.

The current default is the explicit Schur relative-between covariance.

Main implementation:

```text
/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/cbs/include/cbs/utils/relative_pose_covariance.h
```

Main function:

```text
cbs::relative_pose_covariance::compute(...)
```

Calculation:

```text
Lambda_y = jointMarginalInformation({from_key, to_key}).fullMatrix()
```

`Lambda_y` is the 12x12 joint information over
`[delta_from, delta_to]`.

The code obtains the exact GTSAM `BetweenFactor<Pose3>` error Jacobians:

```cpp
factor.evaluateError(from_pose, to_pose, H_from, H_to);
```

For:

```text
delta_rel ~= H_from delta_from + H_to delta_to
```

construct:

```text
M = [ I,                    0
     -H_to^-1 H_from, H_to^-1 ]
```

The implementation uses linear solves, not an explicit inverse operation:

```text
Lambda_u = M^T Lambda_y M

Lambda_u = [ A    B
             B^T  C ]

Lambda_rel = C - B^T A^-1 B
Sigma_rel  = Lambda_rel^-1
```

`A^-1 B` and `Lambda_rel^-1` use SPD solves with diagnostic jitter fallback.

The old conditional method remains available only as the
`conditional_to_pose` ablation. The default is:

```text
schur_relative_between
```

The implementation also computes the direct covariance propagation check:

```text
Sigma_y = Lambda_y^-1
J = [H_from H_to]
Sigma_rel_direct = J Sigma_y J^T
```

The Schur and direct propagated results match to numerical precision in the
sanity tests and recorded audits.

Diagnostics include:

```text
trace_old_conditional
trace_new_schur_relative
ratio_new_over_old
eig_old_min/max
eig_new_min/max
rotation_trace_old/new
translation_trace_old/new
H_to_rank
H_to_condition_estimate
schur_vs_direct_difference_norm
jitter_used
jitter_added
```

## 12. Covariance Audit Results

Important reports:

```text
src/cbsms/docs/2026-07-16_relative_pose_covariance_sanity_tests.md
src/cbsms/docs/2026-07-16_m3dgr_k2g_schur_full_matrix_audit_10samples.md
src/cbsms/docs/2026-07-16_m3dgr_k2g_x0_x24_first_window_covariance_audit.md
src/cbsms/docs/2026-07-16_m3dgr_kimera_k2g_relative_covariance_report.md
src/cbsms/docs/2026-07-16_m3dgr_g2k_relative_covariance_report.md
src/cbsms/docs/2026-07-17_m3dgr_outdoor01_k2g_schur_full_matrix_audit_10samples.md
```

The full 12x12 joint information, transformed information, Schur information,
and 6x6 covariance matrices are printed in the matrix audit reports.

First-window check for Kimera `x0 -> x24`:

```text
trace(absolute Sigma_x24) = 9.149505e-03
trace(relative Sigma_0_24) = 8.413203e-03
ratio = 0.9195
```

The two are close but not identical because `x0` is not fully fixed in
rotation and cross-covariance contributes to the relative result.

Dynamic01 short K2G edges often have trace around `1e-5`. The full-matrix
checks show this comes from large endpoint covariance terms that strongly
cancel through cross-covariance. The algebra is internally consistent. Whether
the resulting covariance is statistically calibrated remains a separate
question.

Outdoor01 produces significantly larger and more variable K2G translation
uncertainty, which is one reason it is the better calibration dataset.

## 13. Latest Valid Persistent Run

Valid run:

```text
/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/20260729-122355_m3dgr_outdoor01_n100_persistent_g2k_inspector_fix_valid_60s_rate025
```

Report:

```text
/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/20260729-122355_m3dgr_outdoor01_n100_persistent_g2k_inspector_fix_valid_60s_rate025/summary.md
```

Run facts:

```text
dataset: Outdoor01
sensor duration: 60 s
bag rate: 0.25
Kimera window: N100
CBS: bidirectional persistent
covariance scales: 1.0
roslaunch return code: 0
```

Evo:

```text
GLIM APE RMSE:   0.5957 m
GLIM RPE RMSE:   0.1575 m
Kimera APE RMSE: 0.5315 m
Kimera RPE RMSE: 0.2322 m
```

This run validates instrumentation and persistent factor visibility. It is not
claimed as the final tuned accuracy result.

Persistent factor applications:

```text
G2K factors applied in Kimera: 740
K2G factors injected in GLIM:  217
```

Inserted covariance medians:

```text
G2K:
  total trace p50       4.195e-05
  rotation trace p50    2.548e-05
  translation trace p50 2.135e-05
  total trace p95       8.947e-05

K2G:
  total trace p50       5.937e-04
  rotation trace p50    2.241e-08
  translation trace p50 5.937e-04
  total trace p95       2.924e-03
```

The full insertion rows are:

```text
runs/20260729-122355_m3dgr_outdoor01_n100_persistent_g2k_inspector_fix_valid_60s_rate025/parsed/cbs_odom_factor_covariance.csv
```

## 14. Latest Persistent Covariance Trend

The supervisor's narrow criterion was that persistent reuse is acceptable only
if the outgoing covariance does not collapse toward zero.

Over the latest valid 60-second Outdoor01 run:

```text
G2K, 740 inserted factors:
  trace min/p50/p95/max:
    7.09e-06 / 4.195e-05 / 8.947e-05 / 1.429e-04
  first 20 trace p50:
    2.363e-05
  last 20 trace p50:
    5.015e-05
  last/first:
    2.12

K2G, 217 inserted factors:
  trace min/p50/p95/max:
    7.955e-05 / 5.937e-04 / 2.924e-03 / 3.583e-03
  first 20 trace p50:
    1.048e-04
  last 20 trace p50:
    1.656e-03
  last/first:
    15.8
```

Conclusion:

- Neither direction collapsed toward zero in this 60-second run.
- K2G increased strongly, driven by translation uncertainty.
- K2G rotation remained extremely confident, around `2e-8` trace.
- This passes only the short-run non-collapse check. It is not proof that
  persistent recycling is statistically independent or safe over long runs.

Important graph-source distinction:

- Kimera's outgoing K2G local covariance graph filters tracked CBS factors, so
  the K2G sender covariance is not directly strengthened by recycled G2K.
- GLIM currently uses `glim_cbs_outgoing_marginal_source=direct` in the wrapper
  defaults. In persistent mode, verify whether its outgoing G2K covariance
  includes information previously received from K2G before drawing conclusions
  about long-run correlation or double counting.

## 15. Receiver Diagnostics And Metrics

Inserted factor covariance:

```text
rotation_trace_inserted
translation_trace_inserted
full 6x6 covariance diagnostics
minimum/maximum eigenvalues and condition estimate
```

Receiver consistency:

```text
pre-injection residual
whitened residual
NIS
```

The pre-injection residual compares the receiver's current relative transform
for the matched local endpoint pair against the received external relative
transform before optimization.

NIS is:

```text
NIS = r^T Sigma_inserted^-1 r
```

It was added by this project as a standard covariance-consistency diagnostic.
It is not a ground-truth metric and a large NIS may mean disagreement,
misalignment, timing error, or covariance overconfidence.

Runtime diagnostics include:

```text
GLIM incoming pending count
GLIM cbs_outgoing_total_ms
GLIM stage timings
Kimera belief_generation_ms
Kimera optimization_ms
```

Factor-strength diagnostics include per-factor:

```text
factor type
keys
category
error
Hessian contribution
```

Hessian Frobenius sums are scale diagnostics. They are not exact directional
stiffness or a direct statement that one factor is N times stronger in every
motion direction.

## 16. GLIM Scan-Health Diagnostics

The GLIM investigation added read-only diagnostics for:

```text
preprocessed point count
scan-vs-IMU translation
scan-vs-IMU rotation
scan_error_ratio
local target source
pose-stage differences
```

Interpretation:

- Low point count: weak geometric support.
- Scan-vs-IMU translation/rotation: disagreement between scan matching and IMU
  prediction for that update.
- `scan_error_ratio > 1`: scan optimization failed to improve its represented
  scan objective.
- Hessian rank alone is insufficient; a full-rank but low-scale Hessian may
  still be weak.

An experimental health rule was added:

```text
low points OR scan disagrees with IMU OR scan cost fails to improve
=> reduce local GLIM scan-factor precision
```

Treat this as an experiment, not a final production policy. Do not expand or
retune it without a controlled baseline comparison.

## 17. Kimera Fixed-Lag Findings

Main reports:

```text
src/cbsms/docs/2026-07-20_kimera_fixed_lag_marginalization_audit.md
src/cbsms/docs/2026-07-27_kimera_backward_update_parity_audit.md
src/cbsms/docs/2026-07-27_kimera_shadow_acceptance_audit.md
src/cbsms/docs/2026-07-28_m3dgr_outdoor01_kimera_n100_divergence_root_cause.md
```

Validated conclusions:

1. Outdoor01 active-window back-and-forth movement is a real revision of the
   current optimized active states. It is not only Rerun camera recentering.
2. Direct covariance preservation and Schur checks found no evidence that the
   custom BPSAM marginalization creates a malformed prior.
3. The same class of revision was reproduced in stock upstream Kimera/native
   GTSAM fixed-lag operation with CBS disabled. It is not uniquely caused by
   CBS.
4. Long SmartStereo tracks produce wide separators. Native iSAM2 fixes
   separator keys against relinearization because the marginal prior is linear.
5. One undamped Gauss-Newton update may be accepted even when the represented
   nonlinear objective increases.
6. Increasing the active window from 25 to 100 improved important
   marginalization-boundary behavior, but did not remove every large nonlinear
   revision.
7. Dynamic01 did not show the same visible severe revision in the latest N100
   run. Outdoor01 is the decisive stress case.

At Outdoor01 state 219 with N100:

```text
mean active-pose translation: 1.0996 m
common translation component: 1.0482 m
common component fraction:    95.32 percent

active graph:
  IMU:              100
  bias:             100
  marginal prior:     1
  SmartStereo:      5206
  G2K:                99
```

The jump happened inside the accepted iSAM2 update, before
`marginalizeLeaves()`.

The newest available G2K factor was one keyframe behind. More importantly,
short relative `BetweenFactor` constraints are invariant to a common left
transform:

```text
(S T_i)^-1 (S T_j) = T_i^-1 T_j
```

They cannot directly stop a common translation of the entire active window.
The fixed-lag marginal prior carries that absolute gauge.

No production solver change has been selected. Dogleg, repeated Gauss-Newton,
line search, and objective acceptance were diagnostics/possible future
directions only. The requested default remains Gaussian/Gauss-Newton.

Do not change SmartStereo track splitting at the marginalization boundary.
The user explicitly wants to preserve Kimera's original philosophy until a
clearly scoped decision is made.

## 18. Persistent Versus Temporary Marginalization

Temporary modes remove CBS factors before they are folded into the fixed-lag
prior. Persistent mode does not remove them for that reason; they are
marginalized like other factors.

Persistent mode intentionally permits recycled information. The user and
supervisor currently accept testing this behavior, provided outgoing
covariances do not collapse.

This is not the same as proving probabilistic independence. Keep these
questions separate:

1. Does the covariance numerically remain bounded away from zero?
2. Does persistent reuse introduce correlation/double counting?
3. Does the estimator improve in APE/RPE and remain stable?

The latest 60-second run answers only the first question positively.

## 19. Build And Compile Commands

After CBS/Kimera changes:

```bash
docker exec cbsms_ws bash -lc \
  'source /opt/ros/noetic/setup.bash; \
   cd /workspace/cbs_gtsam4.3; \
   catkin build cbs kimera_vio kimera_vio_ros \
     --no-status --summarize -j8 -p1'
```

If GLIM or the GLIM inspector changed:

```bash
docker exec cbsms_ws bash -lc \
  'source /opt/ros/noetic/setup.bash; \
   cd /workspace/cbs_gtsam4.3; \
   catkin build cbs glim glim_ros kimera_vio kimera_vio_ros gtsam_points \
     --no-status --summarize -j8 -p1'
```

Authoritative compile database:

```text
/home/yeranis/repos/V4RL/cbs_gtsam4.3/compile_commands.json
```

Compile-database note:

```text
/home/yeranis/repos/V4RL/cbs_gtsam4.3/notes/compile_commands_handoff.md
```

The compile database contains container-native paths. That is correct for the
real ROS/Catkin build environment.

## 20. Run Artifacts And Reporting

Each wrapper run creates:

```text
runs/<timestamp>_<name>/
  manifest.json
  roslaunch.log
  summary.md
  summary.json
  trajectories/
  parsed/
```

Some older runs use `roslaunch_stdout.log`.

Regenerate a report:

```bash
/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/cbsms/tools/cbsms_experiment.py report \
  /home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/<run-directory>
```

Useful parsed files:

```text
parsed/cbs_odom_factor_covariance.csv
parsed/cbs_odom_outgoing.csv
parsed/cbs_odom_relative_covariance_matrices.csv
parsed/cbs_preinjection_residuals.csv
parsed/glim_cbs_odom_inject.csv
parsed/kimera_factor_graph_audit.csv
parsed/kimera_factor_graph_factor_details.csv
parsed/glim_timing_rows.csv
parsed/kimera_timing_rows.csv
parsed/trajectory_metrics.csv
parsed/evo_metrics.csv
```

Always inspect `manifest.json` before comparing runs. It is the authoritative
record of launch arguments, bag rate, duration, config folder, and git state.

Known failed path run:

```text
runs/20260729-122158_m3dgr_outdoor01_n100_persistent_g2k_inspector_fix_60s_rate025
```

It used the wrong host bag path inside the container. Ignore it.

Use instead:

```text
runs/20260729-122355_m3dgr_outdoor01_n100_persistent_g2k_inspector_fix_valid_60s_rate025
```

## 21. Source Map

Relative covariance:

```text
src/cbs/include/cbs/utils/relative_pose_covariance.h
src/cbs/src/utils/relative_pose_covariance_sanity.cpp
```

Shared BPSAM belief generation, marginal graph, insertion, and tracked factors:

```text
src/cbs/include/cbs/bpsam/bpsam.h
src/cbs/include/cbs/bpsam/incremental_fixed_lag_bpsam_smoother.h
src/cbs/src/bpsam/bpsam.cpp
```

Kimera backend CBS integration and factor-graph output:

```text
src/Kimera-VIO/include/kimera-vio/backend/VioBackend.h
src/Kimera-VIO/include/kimera-vio/backend/VioBackend-definitions.h
src/Kimera-VIO/src/backend/VioBackend.cpp
src/Kimera-VIO/src/backend/CbsLocalBeliefCovariance.cpp
```

Kimera ROS transport and Rerun:

```text
src/Kimera-VIO-ROS/src/KimeraVioRos.cpp
src/Kimera-VIO-ROS/src/RosRerunVisualizer.cpp
src/Kimera-VIO-ROS/src/RerunTopicVisualizerNode.cpp
src/Kimera-VIO-ROS/src/RerunImageVisualizerNode.cpp
```

GLIM CBS bridge:

```text
src/glim_ros1/include/glim_ros/cbs_bridge.hpp
src/glim_ros1/src/glim_ros/cbs_bridge.cpp
```

GLIM factor graph inspector:

```text
src/glim_ros1/include/glim_ros/glim_factor_graph_inspector.hpp
src/glim_ros1/src/glim_ros/glim_factor_graph_inspector.cpp
```

GLIM odometry diagnostics:

```text
src/glim/include/glim/odometry/odometry_estimation_cpu.hpp
src/glim/src/glim/odometry/odometry_estimation_cpu.cpp
src/glim/src/glim/odometry/odometry_estimation_imu.cpp
```

GTSAM-points fixed-lag fallback:

```text
src/gtsam_points/include/gtsam_points/optimizers/incremental_fixed_lag_smoother_with_fallback.hpp
src/gtsam_points/src/gtsam_points/optimizers/incremental_fixed_lag_smoother_ext_with_fallback.cpp
```

Experiment/report parser and dashboard:

```text
src/cbsms/tools/cbsms_experiment.py
src/cbsms/tools/send_raw_cbs_dashboard_blueprint.py
```

## 22. Repository State

The following nested repositories are currently dirty:

```text
src/cbsms
src/cbs
src/Kimera-VIO
src/Kimera-VIO-ROS
src/glim_ros1
src/glim
src/gtsam_points
```

Branches:

```text
cbsms, cbs, Kimera-VIO, Kimera-VIO-ROS:
  cbsms/gtsam-4.3-develop tracking zak/cbsms/gtsam-4.3-develop

glim_ros1, glim, gtsam_points:
  cbs-gtsam43-noetic tracking zak/cbs-gtsam43-noetic
```

Do not reset, clean, checkout, or revert these repositories. Many diagnostics
and reports are uncommitted. Do not claim the latest changes are pushed without
checking each nested repository.

## 23. Do Not Change Without Asking

Do not change any of the following unless explicitly requested:

- The Schur relative covariance algorithm or GTSAM tangent ordering.
- Persistent CBS as the current M3DGR default.
- Receiver covariance scales of `1.0`.
- Health-aware sender scaling, currently disabled.
- The 0.20 s horizon and 0.06 s duration tolerance.
- The MID360 `acc_scale=9.80665`.
- The MID360-to-camera-IMU extrinsic.
- Kimera's Gaussian/Gauss-Newton production optimizer.
- SmartStereo track lifecycle or boundary splitting.
- Kimera's incremental published-trajectory semantics.
- Existing historical run artifacts or dirty worktree changes.

Diagnostics should remain default-off where they add substantial runtime cost.

## 24. Open Questions

The next work should remain focused. Important unresolved questions are:

1. Over longer persistent runs, do G2K and K2G covariances remain bounded away
   from zero?
2. Does GLIM's direct outgoing covariance graph include persistent K2G
   information, and if so, how much recycled information enters G2K?
3. Does persistent CBS improve Outdoor01 APE/RPE relative to matched CBS-off,
   K2G-only, and G2K-only controls at N100?
4. Is Kimera's extremely small K2G rotation covariance statistically
   calibrated on Outdoor01 after exact frame and timestamp matching?
5. Can an objective acceptance or damping policy address the Outdoor01
   objective-worsening update without changing normal Kimera behavior?
6. Is short relative CBS sufficient, or is an absolute/long-baseline belief
   needed to observe common active-window gauge motion?

Recommended immediate experiment:

- Keep N100, MID360, persistent, scales 1.0, horizon 0.20 s, bag rate 0.25.
- Run matched two-way CBS-on and CBS-off Outdoor01 controls.
- First extend duration beyond 60 sensor seconds to test covariance collapse.
- Compare covariance trends, factor counts, APE/RPE, objective changes, and
  active-window common-mode motion.
- Do not introduce a solver or SmartStereo change in the same experiment.

## 25. Paste Into The Next Chat

```text
We are working in:
/home/yeranis/repos/V4RL/cbs_gtsam4.3

Container:
cbsms_ws

Container workspace:
/workspace/cbs_gtsam4.3

Read this handoff first:
/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/cbsms/docs/2026-07-29_m3dgr_cbs_persistent_handoff.md

The current M3DGR setup uses MID360 GLIM plus Kimera and bidirectional
persistent CBS:
- glim_cbs_mode:=inject_persistent
- cbs_odom_factor_mode:=persistent
- cbs_odom_covariance_mode:=schur_relative_between
- health-aware scaling disabled
- receiver covariance scales 1.0
- horizon 0.20 s, tolerance 0.06 s
- current Outdoor01 factor-graph tests use Kimera N100
- bag_rate 0.25 for stride-1 dual Rerun inspection

Main launch:
src/glim_ros1/launch/m3dgr_mid360_glim_kimera_live_rerun_raw_cbs.launch

Experiment wrapper/profile:
src/cbsms/tools/cbsms_experiment.py
m3dgr_mid360_glim_kimera_live_rerun

Canonical dual factor-graph setup:
src/cbsms/docs/2026-07-20_cbs_on_dual_factor_graph_rerun_setup.md

Latest valid persistent run:
runs/20260729-122355_m3dgr_outdoor01_n100_persistent_g2k_inspector_fix_valid_60s_rate025

Persistent G2K factors now appear separately in the Kimera inspector through
BPSAM tracked-factor provenance. The latest 60 s covariance analysis showed no
collapse toward zero, but this is not yet a long-run proof and GLIM's direct
outgoing covariance source may recycle persistent K2G information.

Do not change the covariance algorithm, solver, SmartStereo lifecycle,
calibration, persistent default, horizon, or covariance scales without asking.
Start by checking the handoff, the latest run manifest/summary, and the dirty
nested repository state before making any changes.
```

## 26. 2026-07-30 DCReg Stage 1 Update

GLIM now has read-only DCReg-style LiDAR degeneracy diagnostics. The
implementation computes complementary rotation and translation Schur spectra
from the optimized LiDAR-only scan Hessian in GTSAM local tangent order:

```text
[rx, ry, rz, tx, ty, tz]
```

Stage 1 does not change LM, iSAM2, scan precision, factors, poses, CBS, or
Kimera. It adds:

- `GLIM_SCAN_DCREG_ROW`;
- `parsed/glim_dcreg.csv`;
- `parsed/glim_dcreg_summary.csv`;
- a `GLIM DCReg Stage 1` report section;
- a dedicated Rerun inspector tab and weak-direction arrows.

Full design, controls, interpretation, validation, and source map:

```text
src/cbsms/docs/2026-07-30_glim_dcreg_stage1.md
```

Latest validated 60 sensor-second Outdoor01 run:

```text
runs/20260730-132131_m3dgr_outdoor01_dcreg_stage1_stride5_60s_rate025_20260730_1304
```

The run completed with return code zero and produced 547/547 valid diagnostic
rows, full-rank eliminated 3-by-3 blocks, no indefinite Schur spectra, and no
DCReg visualization warnings.

The canonical MID360 runtime configuration enables Stage 1 diagnostics.
Rerun visualization defaults to every fifth scan through:

```text
glim_dcreg_visualization_stride:=5
```

Do not proceed to solver preconditioning, direction suppression, or factor
reweighting without a separate controlled Stage 2 decision.

## 27. 2026-07-30 DCReg Stage 2 Directional Audit

Stage 2 now logs the complete DCReg eigenbasis and, behind an opt-in flag, the
exact relative mean and 6-by-6 covariance GLIM publishes:

```text
GLIM_SCAN_DCREG_BASIS_ROW
glim_dcreg_belief_shadow_enable:=true
GLIM_DCREG_BELIEF_SHADOW_ROW
```

It remains read-only. No estimator, factor, pose, covariance, CBS message, or
Kimera behavior was changed.

Full implementation, mathematics, dataset limitation, validation, and
findings:

```text
src/cbsms/docs/2026-07-30_glim_dcreg_stage2_directional_audit.md
```

Final isolated GLIM Outdoor01 control:

```text
runs/20260730-144847_m3dgr_outdoor01_dcreg_directional_shadow_isolated_valid_60s_rate025
```

The run used `glim_cbs_mode:=observe_only`, completed with return code zero,
and injected zero K2G factors into GLIM. Rerun was live on the dedicated DCReg
tab at visualization stride five.

Outdoor01's supplied GT quaternions are all identity, so the valid audit is
translation-only. Rotation trust is not identifiable from this sequence.

At DCReg threshold 10:

```text
weak translation error RMS   = 0.02386 m
strong translation error RMS = 0.00973 m
weak / strong RMS            = 2.45
```

Thus DCReg weak translation modes do predict larger error, but the spectral
ratio is not a probability or binary belief-trust decision. With the published
M3DGR RTK accuracy added only as a sensitivity model, weak modes retain mean
directional NIS 4.01 and 82.9% 95-percent coverage, while the strong mode is
conservative.

Do not reject GLIM beliefs or enable directional inflation yet. The next
controlled step is a shadow-only anisotropic covariance candidate plus a
sequence with verified full-pose GT and verified GT-body-to-MID360 extrinsic.

## 28. 2026-07-30 GLIM DCReg Stage 3 Multi-Dataset Protocol

The first normalization scope is now explicitly limited to one fixed sender:

```text
GLIM + Livox MID-360 + m3dgr_mid360_native_imu_clean
```

Versioned manifest:

```text
src/cbsms/config/glim_dcreg_calibration_manifest_v1.json
```

Validator:

```text
src/cbsms/tools/glim_dcreg_calibration_protocol.py
```

Full protocol, run references, findings, and restrictions:

```text
src/cbsms/docs/2026-07-30_glim_dcreg_stage3_multidataset_protocol.md
```

The experiment runner accepts `--dcreg-sequence-id` and snapshots the selected
manifest record. It deliberately suppresses directional GT error projection
when the GT-body-to-GLIM transform is unverified while still collecting DCReg
spectra, bases, and exact outgoing shadow beliefs.

Runner deadlines now account for `bag_rate`: expected wall replay duration is
at least `sensor_duration / bag_rate`, plus timeout padding. This prevents slow
rate-0.25 experiments from being truncated at the nominal sensor duration.

Valid isolated 60-sensor-second, rate-0.25, live-Rerun collections:

```text
runs/20260731-105940_m3dgr_dynamic01_dcreg_multidataset_collection_valid_60sensorsec_rate025
runs/20260730-144847_m3dgr_outdoor01_dcreg_directional_shadow_isolated_valid_60s_rate025
runs/20260731-111028_m3dgr_wheel_float01_dcreg_multidataset_collection_valid_60sensorsec_rate025
```

Geometry summary:

| sequence | valid scans | rotation condition p50/p95/max | translation condition p50/p95/max | weak scans R/T |
|---|---:|---:|---:|---:|
| Dynamic01 | 567 | 4.388 / 5.581 / 6.017 | 4.187 / 6.622 / 8.940 | 0 / 0 |
| Outdoor01 | 569 | 29.881 / 44.529 / 60.528 | 131.667 / 184.591 / 230.259 | 568 / 569 |
| Wheel-float01 | 569 | 4.649 / 5.700 / 6.239 | 4.160 / 6.361 / 8.221 | 0 / 0 |

Dynamic01 and Wheel-float01 are healthy-baseline collections. Their GT is a
changing 6-DoF OptiTrack pose for a tracked rigid body named `UGV`, but the
rigid transform from that body origin to GLIM's MID360 pose is not documented
in the local or official calibration. They remain
`collect_pending_extrinsic`; do not use their directional errors for fitting.

Outdoor01 remains translation evidence only. The split is sequence-level,
requires at least 3/1/1 independent calibration/validation/test source
sequences, and is not frozen. Model fitting and covariance mutation remain
prohibited.
