# CBS-On Dual Factor Graph Rerun Setup

This is the canonical factor-graph visualization framework for future CBS
experiments. It renders the live GLIM and Kimera fixed-lag factor graphs in one
Rerun recording while two-way CBS belief exchange remains enabled. Preserve
this dashboard structure and its entity paths when extending diagnostics.

## Dashboard

Use the `CBS-On Dual Factor Graph Inspector` tab. It contains:

- GLIM factor graph with received K2G factors.
- Kimera factor graph with received G2K factors.
- Synchronized camera stream.
- GLIM, Kimera, and ground-truth trajectories.
- Active external-factor counts in both receivers.
- Separate outgoing relative rotation- and translation-covariance traces.
- Axis-resolved outgoing covariance diagonal entries for both directions.

GLIM native scan `BetweenFactor<Pose3>` edges are cyan. Received K2G pose
edges are magenta and published at:

```text
/glim/factor_graph_inspector/spatial/factors/cbs_k2g_pose_between
/glim/factor_graph_inspector/counts/cbs_k2g_pose_between_factors
```

The latest GLIM pose orientation is drawn using explicit world-coordinate RGB
axes from the same `Pose3` as the white latest-pose marker:

```text
/glim/factor_graph_inspector/spatial/states/latest_pose_axes/{x,y,z}
```

Do not use the retired `latest_pose_frame` transform entity to diagnose pose
placement; it could remain visually detached from the world-coordinate graph.

The GLIM distinction uses the factor noise-model type. Native scan surrogate
edges use `noiseModel::Isotropic::Precision`; received K2G edges use
`noiseModel::Gaussian::Covariance`. This remains valid when a 0.20 s K2G edge
connects adjacent GLIM pose indices because the estimator rate changes.

Kimera receives an explicit read-only snapshot of every validated, currently
tracked BPSAM G2K factor slot. This includes persistent and active-window
temporary modes. The factors are rendered in magenta separately from any
native pose-between factors:

```text
/kimera/factor_graph_inspector/spatial/factors/cbs_g2k_pose_between
/kimera/factor_graph_inspector/counts/cbs_g2k_pose_between_factors
```

This uses BPSAM factor provenance rather than factor-type inference. It remains
correct if Kimera also contains a native `BetweenFactor<Pose3>` or if the
active CBS factor is not present in the copied backend graph snapshot.

## Kimera G2K Merge Arrows

For the newest G2K edge accepted by Kimera, the Kimera spatial inspector also
compares three relative transformations using the same post-solve `from` pose
as their display origin:

- Orange: Kimera's relative pose immediately before the G2K factor is added.
- Magenta: the external relative pose received from GLIM.
- Green: Kimera's relative pose immediately after the smoother update.

The endpoints include coordinate frames, so orientation differences remain
visible when the translation arrows overlap. The matched receiver edge is
included in each endpoint label. Data is published below:

```text
/kimera/factor_graph_inspector/spatial/cbs_merge/arrows/local_before
/kimera/factor_graph_inspector/spatial/cbs_merge/arrows/external_g2k
/kimera/factor_graph_inspector/spatial/cbs_merge/arrows/final_after
/kimera/factor_graph_inspector/spatial/cbs_merge/frames/*
```

The backend logs one row per captured merge:

```text
KIMERA_CBS_POSE_MERGE_ROW,
optimization_frame,receiver_edge,
local_to_external_rotation_rad,local_to_external_translation_m,
final_to_external_rotation_rad,final_to_external_translation_m
```

The arrows are absent before the first accepted G2K factor and are cleared
after the captured receiver keys leave Kimera's active window.

### Reading The Kimera Inspector

The white `xN latest` label identifies the newest pose state in the Kimera
backend output. The merge arrows do **not** necessarily describe the edge that
ends at this latest state. Their endpoint labels identify the exact received
edge, for example `x130->x131`. Message transport and timestamp matching mean
that the newest accepted G2K edge commonly trails `xN latest` by one or more
keyframes. The arrows compare local, external, and final transformations only
for their labeled edge; they are not a prediction from `xN latest` to the next
state.

Only the blue points and blue connecting line are Kimera pose states. The red
velocity and yellow bias variables are drawn with display offsets so all state
types remain visible. They are not additional trajectories or alternative pose
estimates.

The blue active graph and grey trajectory history intentionally use different
Kimera pose paths with the default `no_incremental_pose=false`:

- Blue active graph: the current fixed-lag optimizer values in `state_`.
- Grey history and normal backend output: `W_Pose_B_lkf_from_increments_`, a
  smooth path obtained by chaining each newly optimized relative pose.

The grey history is append-only and is not retroactively changed when the
fixed-lag optimizer revises the absolute placement of its active window. The
blue window can therefore translate or rotate as a group while its internal
relative edges, the merge arrows, and the grey incremental output all continue
forward. A relative `BetweenFactor` cannot constrain this common-mode graph
shift because it constrains only `T_from^-1 * T_to`.

Outgoing K2G beliefs and the Kimera merge diagnostic are computed from pairs in
the current optimizer `state_`, not from the grey chained-output path. Setting
`no_incremental_pose=true` would expose the optimizer-state path as Kimera's
normal output, but it changes output-trajectory semantics and is not part of
this canonical setup. Rerun camera orbit, zoom, and auto-framing can add a
second screen-space ambiguity, so inspect edge labels and both pose paths when
diagnosing apparent reversals.

## GLIM K2G Merge Arrows

GLIM now provides the symmetric receiver-side diagnostic for the newest K2G
edge accepted in each smoother update. All three transformations use the same
post-solve GLIM `from` pose as their display origin:

- Orange: GLIM's relative pose immediately before the K2G factor is added.
- Magenta: the external relative pose received from Kimera.
- Green: GLIM's relative pose immediately after the smoother update.

The endpoint labels contain the matched GLIM receiver edge. Coordinate frames
at each endpoint expose rotational differences even when the translation
arrows overlap. Data is published below:

```text
/glim/factor_graph_inspector/spatial/cbs_merge/arrows/local_before
/glim/factor_graph_inspector/spatial/cbs_merge/arrows/external_k2g
/glim/factor_graph_inspector/spatial/cbs_merge/arrows/final_after
/glim/factor_graph_inspector/spatial/cbs_merge/frames/*
```

The bridge logs one row per captured GLIM merge:

```text
GLIM_CBS_POSE_MERGE_ROW,
optimization_pose,receiver_edge,
local_to_external_rotation_rad,local_to_external_translation_m,
final_to_external_rotation_rad,final_to_external_translation_m
```

The GLIM diagnostic is captured from the exact `BetweenFactor<Pose3>` that is
queued for insertion, then finalized from the fixed-lag smoother estimate after
that update. It is read-only and does not alter factor construction, weighting,
or marginalization.

Both inspectors are read-only observers. They do not add, remove, or alter
factors. The defaults remain disabled to avoid instrumentation overhead in
ordinary experiments.

The covariance plots use the full `6x6` covariance carried by each outgoing
belief in GTSAM `Pose3` ordering `[rot_x, rot_y, rot_z, trans_x, trans_y,
trans_z]`. Their scalar paths are:

```text
/metrics/relative_belief_covariance/glim_sent/latest_rotation_trace_rad2
/metrics/relative_belief_covariance/glim_sent/latest_translation_trace_m2
/metrics/relative_belief_covariance/kimera_sent/latest_rotation_trace_rad2
/metrics/relative_belief_covariance/kimera_sent/latest_translation_trace_m2
```

`glim_sent` is G2K and `kimera_sent` is K2G. Each rotation trace is the sum of
covariance diagonal entries `0..2`; each translation trace is the sum of
entries `3..5`.

The axis-resolved panels publish the corresponding six diagonal entries as:

```text
latest_rotation_variance_{x,y,z}_rad2
latest_translation_variance_{x,y,z}_m2
```

Consequently, the three rotation curves for one sender sum exactly to its
rotation trace, and the three translation curves sum exactly to its
translation trace.

## Synchronized Camera Publication

The canonical launch publishes the compressed camera through the dedicated
`rerun_image_visualizer_node` process. The main
`rerun_topic_visualizer_node` has its image stream disabled in this launch.
Both processes connect to the same Rerun recording ID.

This process boundary is intentional. Building trajectory histories and
factor-graph snapshots can take longer as a run progresses. Keeping the camera
callback in the heavy visualizer caused newer images to overwrite the pending
image before it was logged, which appeared in Rerun as progressively sparse
video events even though the rosbag camera remained at 29.97 Hz.

The dedicated process applies `image_max_hz` using the ROS message timestamp
and publishes:

```text
/video/rgb/image
/video/metadata/live_image_count
/video/metadata/live_image_interval_sec
```

It also logs `RERUN_IMAGE_TIMING_ROW` every 25 emitted images. In the validated
60-second Outdoor01 frame-by-frame run, the intended 5 Hz interval was about
`0.20016 s` and the maximum observed interval was `0.23325 s`. The run retained
567 GLIM and 295 Kimera trajectory samples without a visualizer process death.

## Dynamic01 Run

```bash
/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/cbsms/tools/cbsms_experiment.py run \
  --workspace /home/yeranis/repos/V4RL/cbs_gtsam4.3 \
  --experiment-profile m3dgr_mid360_glim_kimera_live_rerun \
  --name m3dgr_dynamic01_cbs_on_dual_factor_graphs_60s_rate05 \
  --duration 60 \
  --timeout-padding 90 \
  --enable-cbs-bridge \
  --rerun-visualizer-enable \
  --rerun-world-alignment-enable \
  --extra-arg bag_rate:=0.5 \
  --extra-arg rerun_recording_id:=m3dgr_dynamic01_cbs_on_dual_factor_graphs_60s \
  --extra-arg glim_factor_graph_inspector_enable:=true \
  --extra-arg glim_factor_graph_inspector_stride:=5 \
  --extra-arg glim_factor_graph_inspector_context_windows:=2 \
  --extra-arg kimera_rerun_factor_graph_inspector_enable:=true \
  --extra-arg kimera_rerun_factor_graph_inspector_stride:=5 \
  --extra-arg kimera_rerun_factor_graph_inspector_include_smart_factors:=false
```

With Rerun already open, select the combined tab automatically with:

```bash
/home/yeranis/.local/share/pipx/venvs/rerun-sdk/bin/python \
  /home/yeranis/repos/V4RL/cbs_gtsam4.3/src/cbsms/tools/send_raw_cbs_dashboard_blueprint.py \
  --app-id cbsms \
  --url rerun+http://127.0.0.1:9876/proxy \
  --active-tab dual-factor-graph
```

Keep smart-factor expansion disabled for live runs. Kimera still reports the
complete smart-factor count, but aggregates the landmark tracks instead of
drawing every visual factor.

For a frame-by-frame inspector, override both strides with `1`:

```text
glim_factor_graph_inspector_stride:=1
kimera_rerun_factor_graph_inspector_stride:=1
```

This changes only Rerun sampling. It does not change either estimator's update
rate, fixed-lag window, factor insertion, or optimization.

Stride `1` is substantially more expensive than the normal dashboard. In the
2026-07-27 Outdoor01 run at `0.5x`, GLIM's post-smoother inspector callback
averaged about `172 ms`, the points-to-IMU lag exceeded `10 s`, and G2K factors
eventually targeted poses outside Kimera's 25-state active window. For a
frame-by-frame dual inspector, use `bag_rate:=0.25`.

The experiment harness timeout is wall-clock based:
`duration + timeout_padding`. For `duration=60` and `bag_rate=0.25`, use at
least `--timeout-padding 240`; the canonical stress run uses `300`. A padding
of `90` ends the launch after roughly 35 sensor-seconds even though the
manifest still records the requested 60-second duration.

To inspect a bounded sample of Kimera's actual visual constraints, enable at
most 20 SmartStereo factors. Kimera does not normally contain native visual
`BetweenFactor<Pose3>` factors, so `pose_between_factors=0` is expected:

```text
kimera_rerun_factor_graph_inspector_include_smart_factors:=true
kimera_rerun_factor_graph_inspector_max_smart_factors:=20
```

The MID360 wrapper also forwards the existing per-update horizon-pair limits.
For a lower-traffic inspection run, use:

```text
glim_cbs_odom_max_horizon_pairs_per_update:=6
kimera_cbs_odom_max_horizon_pairs_per_update:=6
```

The validated Outdoor01 visualization recording is:

```text
m3dgr_outdoor01_cbs_on_dual_factor_graphs_visualfix_60s_rate025_20260727
m3dgr_outdoor01_n100_image_process_full2_60s_rate025_20260728
```

The first recording validated factor visibility. The second validated
process-isolated 5 Hz camera publication under the `nr_states=100`,
frame-by-frame dual-inspector load.
