# Kimera Factor Graph Rerun Setup

This is the saved diagnostic setup for inspecting Kimera's live fixed-lag
factor graph alongside synchronized rosbag video. It is visualization-only and
does not alter Kimera, GTSAM, or CBS optimization behavior.

## What The Inspector Shows

- muted gray trajectory: the most recent two active-window lengths of Kimera
  keyframe history, used only as local spatial context
- blue poses and chain: current active fixed-lag pose window
- white pose frame: latest active pose
- red edges and markers: IMU preintegration factors
- yellow edges and markers: IMU bias random-walk factors, spatially projected
  onto the corresponding pose endpoints
- cyan edges: `BetweenFactor<Pose3>`, including CBS factors when present
- purple marker and raised translucent rail: fixed-lag
  `LinearContainerFactor` marginal prior
- white markers: pose priors
- optional translucent green spans: sampled SmartStereo factors
- synchronized camera pane: `/camera/color/image_raw/compressed`

Bias variables are not spatial states. Their yellow edges are deliberately
drawn between the matching pose indices so their lifecycle can be inspected in
3D. Factor strength is not encoded by line width; use the Hessian, covariance,
residual, and NIS streams for numerical strength.

The inspector intentionally does not draw the complete trajectory history.
On long outdoor runs, the complete path makes Rerun auto-frame tens of meters
while the graph of interest contains only 26 active poses. The full trajectory
remains available in the main dashboard; the inspector keeps twice the active
pose count so archived playback retains a useful local scale.

## Required Recording Convention

The Kimera headless graph logger and `rerun_topic_visualizer_node` must use the
same `rerun_recording_id`. Otherwise the factor graph and video appear as
separate Rerun sources and cannot be inspected on one timeline.

Container publishers use:

```text
rerun+http://172.17.0.1:9876/proxy
```

The host-side blueprint sender uses:

```text
rerun+http://127.0.0.1:9876/proxy
```

## Canonical Dynamic01 Command

Open Rerun on the host first, then run:

```bash
/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/cbsms/tools/cbsms_experiment.py run \
  --workspace /home/yeranis/repos/V4RL/cbs_gtsam4.3 \
  --experiment-profile m3dgr_kimera_only \
  --name m3dgr_dynamic01_kimera_factor_graph_video_sync_60s \
  --duration 60 \
  --timeout-padding 90 \
  --rerun-visualizer-enable \
  --no-use-kimera-rviz \
  --no-kimera-visualize \
  --extra-arg bag_rate:=0.5 \
  --extra-arg rerun_geometry_enable:=false \
  --extra-arg rerun_factor_graph_inspector_enable:=true \
  --extra-arg rerun_factor_graph_inspector_stride:=5 \
  --extra-arg rerun_factor_graph_inspector_include_smart_factors:=false \
  --extra-arg kimera_factor_graph_audit_enable:=true \
  --extra-arg rerun_topic_visualizer_enable:=true \
  --extra-arg rerun_recording_id:=<unique_recording_id>
```

Apply the inspector-first blueprint:

```bash
/home/yeranis/.local/share/pipx/venvs/rerun-sdk/bin/python \
  /home/yeranis/repos/V4RL/cbs_gtsam4.3/src/cbsms/tools/send_raw_cbs_dashboard_blueprint.py \
  --app-id cbsms \
  --url rerun+http://127.0.0.1:9876/proxy \
  --active-tab factor-graph
```

## Sampling Tradeoff

`rerun_factor_graph_inspector_stride:=5` records one graph snapshot every five
Kimera keyframes. Rerun holds the most recent graph between snapshots. Use
`stride:=1` for frame-by-frame inspection at higher logging cost and recording
size. Keep SmartStereo geometry disabled for normal runs because the active
graph commonly contains hundreds of multi-pose tracks; their counts remain
visible even when their geometry is hidden.

## Verified Reference Run

```text
runs/20260717-174612_m3dgr_dynamic01_kimera_factor_graph_video_sync_60s
recording_id: m3dgr_kimera_factor_graph_video_sync_60s_20260717
bag rate: 0.5
graph snapshots: 56
keyframes: 0 through 275
trajectory history: 276 poses
active x/v/b states at steady state: 26 / 26 / 26
active IMU factors: 25
active bias factors: 25
active marginal priors: 1
missing key references: 0
camera decode failures: 0
```

The active graph is expected to remain bounded while raw ISAM2 factor slots
and tombstones grow. The graph inspector should be judged using live factors
and active states, not raw slot count alone.

The local-scale archived playback behavior was verified on Outdoor01:

```text
runs/20260720-105534_m3dgr_outdoor01_kimera_factor_graph_local_view_fixed_120s
recording_id: m3dgr_outdoor01_kimera_factor_graph_local_view_fixed_120s_20260720
bag rate: 0.5
graph snapshots: 116
trajectory states: 578
active x/v/b states at steady state: 26 / 26 / 26
camera/factor-graph publish failures: 0
```

## Reusing This On Another Dataset

Keep the instrumentation and shared-recording convention unchanged. Replace
only:

- experiment profile or launch file
- bag and ground-truth paths
- Kimera sensor/calibration parameter folder
- image topic if the dataset uses another camera topic
- unique run name and `rerun_recording_id`

For a pure-Kimera calibration study, keep CBS disabled. For a later CBS study,
enable the desired exchange direction and use the cyan pose-between layer to
inspect external factors inside the active Kimera graph.
