# Newer College Maths-Easy edited GLIM + Kimera, CBS off, 80 seconds

## Outcome

The synchronized 80-second sensor window completed successfully. Edited GLIM
and edited Kimera consumed the same bag clock while every CBS injection path
was explicitly disabled. The open Rerun viewer was left running.

Recording ID:

`newer_college_maths_easy_edited_glim_kimera_cbs_off_80s_20260821`

## Estimator results

| Estimator | Poses | Evaluated sensor time | Translation APE RMSE | Height RMSE |
|---|---:|---:|---:|---:|
| Edited GLIM online | 792 | 79.096 s | 0.061716 m | not separately scored |
| Edited Kimera cam4 | 343 | 79.798 s | 1.860251 m | 0.049529 m |

Evaluation is Oxford Base frame, rigid SE(3) position alignment, scale fixed at
one. No time-offset fitting, Sim(3), trajectory deformation, or estimator
feedback was used.

Matched original-code controls:

| Estimator | Original control | Edited run | Edited - original |
|---|---:|---:|---:|
| GLIM | 0.061433 m | 0.061716 m | +0.000282 m |
| Kimera lag50 | 1.762948 m | 1.860251 m | +0.097303 m |

GLIM is effectively at original-code parity. Kimera completed the identical
343 timestamps but is 0.0973 m worse in ATE than the strict original lag-50
80-second control. Its aligned height RMSE improved from 0.065766 m to
0.049529 m, so the regression is predominantly horizontal/path-shape error.

## Configuration

- GLIM: optimized CUDA Newer College profile, `/os_cloud_node/points`,
  `/os_cloud_node/imu`, Oxford `T_os_imu_base` display/evaluation conversion.
- Kimera: physical cam4, Alphasense IMU, `nr_states=50`, `numOptimize=1`, skip
  the first two cam4 frames, strict profile, RViz visualization pipeline kept
  enabled.
- Bag replay: 80.0 sensor seconds at 0.75x.
- CBS off: GLIM bridge and health-aware path false; Kimera belief bridge,
  backend CBS path, and health-aware path false. No CBS activity marker was
  found in either log. CBS-named Rerun entities may still exist as zero/cleared
  inspector schema; they do not indicate an injected factor.

## Rerun validation

The recovered Stage-3A CBSMS layout supplies:

- GLIM and Kimera fixed-lag factor graphs;
- GLIM, Kimera, and GT in one aligned trajectory view;
- cam1 and physical cam4 video;
- LiDAR playback;
- factor composition and active-state counts;
- per-pose GLIM and Kimera translation APE;
- DCReg and covariance-audit panes, which remain empty/zero where the
  corresponding CBS-off diagnostic has no samples.

Observed streams:

- 793 GLIM factor-graph inspector snapshots;
- 69 Kimera factor-graph inspector snapshots (stride 5);
- 325 cam1 and 325 cam4 publication-count milestones;
- 81 independently verified, 10,000-point LiDAR playback frames, plus the
  aligned live scan stream produced during the estimator run.

The saved Rerun recording contains 412 entity paths and 59,672 rows. `rerun
rrd verify` completed without error.

### Synchronized presentation correction

The first post-run overlay logged the three completed trajectories as static
geometry. That made the final paths visible at every playback cursor position,
and its common trajectory frame did not match the raw LiDAR pane. Presentation
V2 fixes both issues without rerunning or changing either estimator:

- GT, GLIM, and Kimera are time-indexed growing prefixes (161, 159, and 172
  updates respectively), so the visible paths stop at the playback cursor;
- 79 retained LiDAR frames are placed with the recorded GLIM
  `T_world_lidar`, then transformed by the exact scored GLIM-to-GT SE(3) and
  the same display-origin translation as all three trajectories;
- every transformed LiDAR origin is 0.09101 m from the displayed GLIM Base
  pose, matching the physical Base-to-LiDAR extrinsic rather than a frame
  error;
- the Kimera presentation view whitelists its grey context trajectory, active
  states, and active factor edges, while excluding verbose factor-marker labels
  and CBS-merge overlays. The excluded entities remain recorded for forensic
  inspection.

The original live recording received the corrected overlay under
`/presentation_v2/**` and the corresponding blueprint was activated in the
already-open viewer.

### Kimera line-strip serialization correction

Visual inspection of the cleaned Kimera pane exposed a separate C++ lifetime
bug in `RosRerunVisualizer::drawLineStrips`: Rerun line-strip components kept
views into loop-local point buffers which were destroyed before serialization.
The original RRD contains corrupted first vertices (including coordinates in
the millions), explaining the collapsed/elongated graph.

The logger now retains all point buffers until after `rec()->log()`. Kimera was
rebuilt and replayed with CBS off using the same lag-50 profile. A 12-second
isolated validation recorded 51 snapshots and showed bounded vertices; for
example, IMU and bias midpoints around the origin were respectively
`[+0.059,-0.054,+0.031]` and `[-0.055,+0.058,-0.029]`. The complete archive
contains 343 corrected snapshots, one per backend update, rather than the
original stride-5 set of 69.

The live recording was repaired by replaying the corrected Kimera inspector
rows at the original sensor timestamps. Standalone V3 combined the
synchronized V2 trajectories/cloud/video with these corrected active factor
strips. V2 remains useful for trajectory inspection but must not be used to
present the Kimera factor graph.

### Kimera context-trajectory serialization correction

The first line-strip fix removed invalid factor vertices, but the grey history
still contained a separate serialization defect. It was assembled from
two-point buffers whose lifetime ended before Rerun serialized them.

The remaining fault was the same lifetime class one layer lower in
`aria_viz::VisualizerRerun::connectPositions3D`. Each trajectory edge was
wrapped in a non-owning Rerun collection backed by loop-local `p0` and `p1`
variables, then serialized only after those variables had expired. The logger
now retains every two-point buffer until `rec_->log()` returns.

A 20-second isolated replay produced 86 graph snapshots. Stored history edges
started at `[0,0,0]`, continued through the actual backend increments (the
first was `[-0.000819,+0.002221,+0.002804]` m), and shared exact endpoints
between consecutive segments. The complete CBS-off replay emitted 343
snapshots at the original sensor timestamps. It was streamed into the open
viewer and archived independently. V4 fixed the trajectory storage but retained
two routed Recording stores, which caused the live viewer to expose stale and
corrected sources together. V5 routed the data to one Recording ID, but it is
also superseded by the display-frame correction below.

### Kimera optimizer-state/display-frame correction

After both lifetime defects were fixed, the factor graph still did not follow
Kimera's smooth published trajectory. The actual remaining cause was a frame
mismatch, not another rendering artifact:

- the inspector drew active poses directly from backend optimizer `state_`;
- Kimera `/odometry` uses `W_State_Blkf_`, the incrementally chained published
  pose when `no_incremental_pose` is false;
- optimizer gauge corrections can be sharp between updates, so plotting raw
  optimizer states beside incremental output produced apparent graph jumps.

Before correction, the optimizer-state endpoint and published odometry had
5.763695 m SE(3)-aligned RMSE over 343 poses. Their path lengths were
118.760518 m and 83.520242 m, step-length correlation was -0.257638, and the
largest optimizer-state jump was 8.525443 m.

`KimeraVioRos::publishKimeraFactorGraphInspector` now computes one rigid
display transform per backend snapshot,
`display_T_state = W_State_Blkf.pose * state_current_pose.inverse()`, and
applies it to every active optimizer pose and CBS merge anchor. This preserves
exact within-snapshot factor geometry while placing the complete graph in the
same incremental frame as Kimera's published trajectory. Persistent grey
context uses `W_State_Blkf_` directly.

Full 80-second validation over all 343 snapshots gives:

- graph latest-pose versus published odometry direct RMSE: 0.000000699 m;
- maximum direct discrepancy: 0.000002186 m;
- graph path length: 83.520103 m; published path length: 83.520101 m;
- maximum step for both paths: 0.339106 m;
- step-length correlation: 1.000000000.

At the former failure around keyframe 187, both displayed graph and published
odometry now advance by 0.025317 m instead of the graph jumping 8.525 m. V7 is
therefore the first canonical artifact in which active Kimera factors, grey
history, and published Kimera trajectory share the same display frame.

## Artifacts

- Valid recording: `rerun_capture/newer_college_maths_easy_80s_complete.rrd`
  (SHA256 `e3129c4197b6f5d5fe8b6869fba5bd27578e1e99bffa2d0f139125066c469170`).
- `rerun_capture/output.partial.rrd` is the interrupted live download used to
  reconstruct the valid compact recording; do not use it for presentation.
- Recovered blueprints: `recovered_blueprints/cbsms_20260821.rbl` and
  `recovered_blueprints/geode_cbs_off_live60.rbl`.
- Corrected live overlay:
  `rerun_capture/live_overlay_presentation_v2.rrd` (SHA256
  `893ea36907e0867ff1190edb2f4594adcd319cb9687d012c2e0c3b916aeddad2`).
- Corrected self-contained recording:
  `rerun_capture/newer_college_maths_easy_80s_synchronized_v2.rrd` (SHA256
  `e05e6f6e24eb25b4b5a0783dde9fd795eca94518a873ad1b89b84aed15ca5fd5`).
  This pre-line-strip-fix version is superseded for Kimera graph presentation.
- Superseded V3 self-contained recording:
  `rerun_capture/newer_college_maths_easy_80s_synchronized_v3.rrd` (SHA256
  `b23e37678e1b7e52968bdea6b6e07496aac4bf3c737db5b426e264877d622a42`).
  Its active factor strips are valid, but its grey context trajectory is not.
- Superseded V4 corrected-data recording:
  `rerun_capture/newer_college_maths_easy_80s_synchronized_v4.rrd` (SHA256
  `6810c592048144bce1fb83a08f5d1fcec86933dbd5321f48e5c4a32e5f90fa00`).
  Its data is valid, but its two Recording stores make it unsuitable for the
  live presentation.
- Superseded V5 single-store recording:
  `rerun_capture/newer_college_maths_easy_80s_canonical_v5_clean.rrd` (SHA256
  `a403eb1264bd7fcc5ddb7e058b2553701cc5b0b80a40945e99b1cb6730cb26bf`).
  Its serialization is valid, but its Kimera graph remains in raw optimizer
  gauge rather than the published incremental display frame.
- **Canonical V7 single-store self-contained recording:**
  `rerun_capture/newer_college_maths_easy_80s_canonical_v7.rrd` (SHA256
  `42e3bad60bdbf69625c0d308936d3a80cbcfbb99cd494d040becd408ed16059d`).
- Live-overlay blueprint:
  `recovered_blueprints/newer_college_maths_easy_80s_synchronized_live_overlay_v2.rbl`.
- Standalone-recording blueprint:
  `recovered_blueprints/newer_college_maths_easy_80s_synchronized_standalone_v2.rbl`.
- Superseded V3 standalone blueprint:
  `recovered_blueprints/newer_college_maths_easy_80s_synchronized_standalone_v3.rbl`.
- Superseded V4 standalone blueprint:
  `recovered_blueprints/newer_college_maths_easy_80s_synchronized_standalone_v4.rbl`
  (SHA256
  `12b8fafc7e7c90d08cd98b1986c06d9b05335880a8d086174d0c216b976de188`).
- Superseded V5 standalone blueprint:
  `recovered_blueprints/newer_college_maths_easy_80s_canonical_v5.rbl`
  (SHA256
  `e340c2cc2e24fc1f2f8af543f8083f927cf14b47bac7f05820b933e67959d30c`).
- **Canonical V7 standalone blueprint:**
  `recovered_blueprints/newer_college_maths_easy_80s_canonical_v7.rbl`
  (SHA256
  `90afae9fc8708b13abbf47de4836ffc618090384e3302b86aeba93447480fde2`).
- Corrected 343-snapshot Kimera inspector archive:
  `fixed_kimera_factor_graph_archive/kimera_factor_graph_80s_compact.rrd`
  (SHA256
  `341e17a5031208e27635cae29c5101798db2747df8f67c27ea55437f5b514fad`).
- Kimera line-strip lifetime fix:
  `src/Kimera-VIO-ROS/src/RosRerunVisualizer.cpp`.
- Kimera/aria trajectory-edge lifetime fix:
  `src/aria_viz/src/visualizer_rerun.cpp`.
- Kimera optimizer-state to incremental-display-frame fix:
  `src/Kimera-VIO-ROS/src/KimeraVioRos.cpp`.
- Corrected V4 343-snapshot Kimera inspector archive:
  `context_trajectory_fixed_archive_v4/kimera_factor_graph_80s_compact.rrd`
  (SHA256
  `e5d950e366f6573452632672037b907b9cc1f20632a15277b60421f1cb3b92fc`).
- Corrected V7 343-snapshot Kimera inspector archive:
  `incremental_frame_archive_v7/kimera_factor_graph_80s_compact.rrd`
  (SHA256
  `76b8bbb616a6ab77a3ff12ebf99dd245f90fe1ce00692dfc77d6b2bb2bf88546`).
- Reproducible overlay builder: `build_synchronized_presentation.py`; its
  alignment/timeline audit is `live_synchronized_presentation_summary.json`.
- Canonical layout source: `src/cbsms/tools/geode_stage3a_rerun.py`.
- Reusable cross-sequence preset and acceptance contract:
  `src/cbsms/docs/2026-08-21_newer_college_cbs_off_rerun_preset.md`.
- Reusable preset sender:
  `src/cbsms/tools/send_newer_college_cbs_off_blueprint.py`.
- Machine-readable summary: `validation_summary.json`.
- Core scores: `combined_glim_kimera_cbs_off_80s/evaluation/metrics.json` and
  `combined_glim_kimera_cbs_off_80s/evaluation/kimera_metrics_base.json`.
- Immutable estimator configuration snapshots are under
  `combined_glim_kimera_cbs_off_80s/glim_config` and `kimera_profile`.

## Final status

**COMPLETE — READY FOR CBS-OFF 80-SECOND PRESENTATION AND COMPARISON.**
