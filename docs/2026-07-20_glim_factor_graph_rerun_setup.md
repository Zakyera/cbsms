# GLIM Factor Graph Rerun Setup

This setup records a pure GLIM fixed-lag graph together with synchronized M3DGR
camera video and aligned GLIM/ground-truth trajectories. The inspector is
read-only: it observes GLIM odometry smoother callbacks after optimization and
does not add, remove, rescale, or modify factors.

## Implementation

- Inspector:
  `src/glim_ros1/src/glim_ros/glim_factor_graph_inspector.cpp`
- ROS integration:
  `src/glim_ros1/src/glim_ros/glim_ros.cpp`
- Launch plumbing:
  `src/glim_ros1/launch/glim_rosnode.launch`
  `src/glim_ros1/launch/m3dgr_glim_only_experiment.launch`
  `src/glim_ros1/launch/m3dgr_glim_only_live_rerun.launch`
  `src/glim_ros1/launch/m3dgr_mid360_glim_only_live_rerun.launch`
- Rerun layout:
  `src/cbsms/tools/send_raw_cbs_dashboard_blueprint.py`

The Rerun top-level tab is `GLIM Factor Graph Inspector`. It contains:

- Spatial GLIM fixed-lag factor graph
- Synchronized camera
- Aligned GLIM trajectory versus ground truth
- Factor slot, active state, and factor composition plots

## Spatial Legend

- Blue points/line: active GLIM pose states and pose chain
- White point/frame: latest active pose
- Gray line: short trajectory context, limited to two active-window lengths
- Red offset edges: IMU preintegration factors
- Yellow offset edges: IMU bias random-walk factors
- Cyan offset edges: scan-matching pose-between surrogate factors
- Orange markers: scan-matching pose-prior surrogate factors
- Purple raised rail: fixed-lag marginal prior (`LinearContainerFactor`)
- White/gray markers: initialization damping and uncategorized factors
- Green offset edges: velocity fallback factors used when IMU data is missing

GLIM's frontend VGICP/GICP factors are optimized during scan matching. The
fixed-lag smoother retains the resulting pose-between and pose-prior surrogate
factors, which are the cyan edges and orange markers shown here.

## Parameters

```text
glim_factor_graph_inspector_enable:=true
glim_factor_graph_inspector_stride:=5
glim_factor_graph_inspector_context_windows:=2
```

The inspector recording ID and host are automatically set to the same values as
the topic visualizer by `m3dgr_glim_only_live_rerun.launch`.

## Outdoor01 Command

```bash
/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/cbsms/tools/cbsms_experiment.py run \
  --workspace /home/yeranis/repos/V4RL/cbs_gtsam4.3 \
  --experiment-profile m3dgr_mid360_glim_only_live_rerun \
  --name m3dgr_outdoor01_glim_factor_graph_inspector_120s_rate05 \
  --bag-path /workspace/cbs_gtsam4.3/src/datasets/M3DGR/Outdoor01/Outdoor01.bag \
  --gt-path /home/yeranis/repos/V4RL/cbs_gtsam4.3/src/datasets/M3DGR/Outdoor01/Outdoor01.txt \
  --duration 120 \
  --timeout-padding 90 \
  --no-enable-cbs-bridge \
  --rerun-visualizer-enable \
  --rerun-world-alignment-enable \
  --extra-arg bag_rate:=0.5 \
  --extra-arg rerun_recording_id:=m3dgr_outdoor01_glim_factor_graph_inspector_120s \
  --extra-arg glim_factor_graph_inspector_enable:=true \
  --extra-arg glim_factor_graph_inspector_stride:=5 \
  --extra-arg glim_factor_graph_inspector_context_windows:=2
```

Send the inspector blueprint to an open viewer:

```bash
/home/yeranis/.local/share/pipx/venvs/rerun-sdk/bin/python \
  /home/yeranis/repos/V4RL/cbs_gtsam4.3/src/cbsms/tools/send_raw_cbs_dashboard_blueprint.py \
  --app-id cbsms \
  --url rerun+http://127.0.0.1:9876/proxy \
  --active-tab glim-factor-graph
```

## Validated Run

`runs/20260720-112828_m3dgr_outdoor01_glim_factor_graph_inspector_120s_rate05`

The run completed with return code 0 and recorded 205 graph snapshots. At the
end of the run, the graph had 50 active pose, velocity, and bias states; 49 IMU,
bias-between, and scan-between factors; 50 scan-prior factors; one marginal
prior; 198 live factors in 206 raw slots; and eight tombstones. This is bounded
fixed-lag behavior, not historical factor accumulation.
