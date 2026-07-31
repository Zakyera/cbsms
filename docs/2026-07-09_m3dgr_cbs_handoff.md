# M3DGR CBS + GLIM + Kimera Handoff

> Superseded for current work by
> `2026-07-29_m3dgr_cbs_persistent_handoff.md`. Keep this file as historical
> context; do not use its older final prompt as the current experiment default.

This note is for continuing the project in a fresh chat without replaying the
whole debugging history.

## Workspace

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

Start the container if needed:

```bash
docker start cbsms_ws
```

Run commands inside it with:

```bash
docker exec cbsms_ws bash -lc 'source /opt/ros/noetic/setup.bash; source /workspace/cbs_gtsam4.3/devel/setup.bash; <command>'
```

## Project Goal

We are evaluating CBS belief exchange between GLIM and Kimera. The CBS method
exchanges short-horizon relative odometry beliefs, not absolute priors.

Sender belief:

```text
T_i^-1 T_j
```

Receiver matches sender timestamps to local states and creates a BetweenFactor
style residual from the received relative pose and covariance.

The current preferred default for M3DGR CBS-on experiments, updated on
2026-07-29, is:

```text
persistent
```

This means CBS odometry factors are inserted as ordinary nonlinear factors and
are allowed to contribute to the fixed-lag marginal prior.

Important: the older successful S3E Square 1 baseline used
`temporary_linear`, and the earlier M3DGR experiments used
`active_window_temporary`. Do not erase those historical facts. For current
M3DGR work, the requested default is persistent beliefs.

## Current Default M3DGR CBS Setup

Main M3DGR GLIM+Kimera experiment launch:

```text
/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/glim_ros1/launch/m3dgr_glim_kimera_experiment.launch
```

Live Rerun raw-CBS launch:

```text
/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/glim_ros1/launch/m3dgr_glim_kimera_live_rerun_raw_cbs.launch
```

Current CBS defaults in the M3DGR GLIM+Kimera path:

```text
enable_cbs_bridge:=true
glim_cbs_mode:=inject_persistent
cbs_odom_factor_mode:=persistent
cbs_health_aware_enable:=false
cbs_enable_soft_reset:=true
cbs_d_reset:=0.1
cbs_odom_duration_gate_enable:=true
cbs_odom_horizon_sec:=0.20
cbs_odom_horizon_tolerance_sec:=0.06
glim_cbs_odom_receiver_match_mode:=duration_aware_edge
glim_cbs_odom_max_horizon_pairs_per_update:=25
kimera_cbs_odom_max_horizon_pairs_per_update:=6
glim_cbs_k2g_odom_factor_covariance_scale:=1.0
kimera_cbs_l2k_odom_factor_covariance_scale:=1.0
```

The active estimator/receiver window remains 25. The Kimera outgoing cap limits
only how many recent `time_horizon_window` relative beliefs are generated and
published per backend update.
The GLIM receiver also uses duration-aware edge matching for K->G odometry
beliefs, selecting a local GLIM edge by duration match rather than independent
nearest endpoint stamps.

The Python experiment wrapper was also updated:

```text
/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/cbsms/tools/cbsms_experiment.py
```

For M3DGR `m3dgr_glim_kimera`, it now defaults to:

```text
glim_cbs_mode=inject_persistent
cbs_odom_factor_mode=persistent
glim_cbs_odom_receiver_match_mode=duration_aware_edge
glim_cbs_odom_max_horizon_pairs_per_update=25
kimera_cbs_odom_max_horizon_pairs_per_update=6
```

## M3DGR Data Present

Available local sequences:

```text
/workspace/cbs_gtsam4.3/src/datasets/M3DGR/Dynamic01/Dynamic01.bag
/workspace/cbs_gtsam4.3/src/datasets/M3DGR/Dynamic01/Dynamic01.txt

/workspace/cbs_gtsam4.3/src/datasets/M3DGR/Outdoor01/Outdoor01.bag
/workspace/cbs_gtsam4.3/src/datasets/M3DGR/Outdoor01/Outdoor01.txt

/workspace/cbs_gtsam4.3/src/datasets/M3DGR/Wheel-float01/Wheel-float01.bag
/workspace/cbs_gtsam4.3/src/datasets/M3DGR/Wheel-float01/Wheel-float01.txt
```

Dataset topic mapping:

```text
Livox raw LiDAR: /livox/avia/lidar
Livox IMU:       /livox/avia/imu
Camera:          /camera/color/image_raw or compressed bridge output
Converted cloud: /m3dgr/avia/points
GT file:         sequence .txt file
```

The launch runs `m3dgr_tools` to convert Livox packets into PointCloud2 on:

```text
/m3dgr/avia/points
```

GLIM uses:

```text
points: /m3dgr/avia/points
imu:    /camera/imu
```

Kimera uses the M3DGR mono config:

```text
$(find kimera_vio)/params/M3DGRMonoOriginal
```

Current canonical M3DGR live-Rerun experiment setup note:

```text
/home/yeranis/repos/V4RL/cbs_gtsam4.3/notes/m3dgr_live_rerun_experiment_setup.md
```

## M3DGR GLIM-Kimera Body Extrinsic

The current M3DGR shared-IMU setup uses the D435i camera IMU as the body frame
for both GLIM and Kimera:

```text
kimera_body_frame_id:=camera_imu_link
glim_body_frame_id:=camera_imu_link
m3dgr_cbs_body_frame_tf_enable:=false
```

The GLIM camera-IMU runtime config also matches Kimera `M3DGRMonoOriginal` for
D435i IMU white-noise/preintegration confidence:

```text
imu_gyro_noise:=1.6968e-04
imu_acc_noise:=2.0000e-03
imu_int_noise:=1.0e-8
```

The legacy mixed-body transform is still available as an override if GLIM is
run in the Livox IMU body frame:

```text
x  = 0.039151162
y  = 0.111984435
z  = 0.236306953
qx = 0.507715982
qy = -0.497277044
qz = 0.477055852
qw = 0.517066473
```

Do not remove this. It was added so CBS relative odometry between GLIM and
Kimera is in compatible body frames.

## Main Live Rerun Run

Open Rerun on the host first, listening on the default proxy. Then run:

```bash
docker exec cbsms_ws bash -lc 'source /opt/ros/noetic/setup.bash; source /workspace/cbs_gtsam4.3/devel/setup.bash; roslaunch glim_ros m3dgr_glim_kimera_live_rerun_raw_cbs.launch bag_duration:=60.0 shutdown_on_bag_finish:=true rerun_recording_id:=m3dgr_dynamic01_raw_cbs_live'
```

For Outdoor01:

```bash
docker exec cbsms_ws bash -lc 'source /opt/ros/noetic/setup.bash; source /workspace/cbs_gtsam4.3/devel/setup.bash; roslaunch glim_ros m3dgr_glim_kimera_live_rerun_raw_cbs.launch bag_path:=/workspace/cbs_gtsam4.3/src/datasets/M3DGR/Outdoor01/Outdoor01.bag ground_truth_path:=/workspace/cbs_gtsam4.3/src/datasets/M3DGR/Outdoor01/Outdoor01.txt bag_duration:=60.0 shutdown_on_bag_finish:=true rerun_recording_id:=m3dgr_outdoor01_raw_cbs_live'
```

For Wheel-float01:

```bash
docker exec cbsms_ws bash -lc 'source /opt/ros/noetic/setup.bash; source /workspace/cbs_gtsam4.3/devel/setup.bash; roslaunch glim_ros m3dgr_glim_kimera_live_rerun_raw_cbs.launch bag_path:=/workspace/cbs_gtsam4.3/src/datasets/M3DGR/Wheel-float01/Wheel-float01.bag ground_truth_path:=/workspace/cbs_gtsam4.3/src/datasets/M3DGR/Wheel-float01/Wheel-float01.txt bag_duration:=60.0 shutdown_on_bag_finish:=true rerun_recording_id:=m3dgr_wheelfloat01_raw_cbs_live'
```

The launch already enables the raw-CBS Rerun dashboard blueprint:

```text
raw_cbs_dashboard_blueprint_enable=true
```

Container-to-host Rerun URL:

```text
rerun+http://172.17.0.1:9876/proxy
```

If manually reapplying the dashboard from the host:

```bash
/home/yeranis/.local/share/pipx/venvs/rerun-sdk/bin/python \
  /home/yeranis/repos/V4RL/cbs_gtsam4.3/src/cbsms/tools/send_raw_cbs_dashboard_blueprint.py \
  --app-id cbsms \
  --url rerun+http://127.0.0.1:9876/proxy
```

Canonical Rerun dashboard note:

```text
/home/yeranis/repos/V4RL/cbs_gtsam4.3/notes/rerun_raw_cbs_dashboard.md
```

Current Kimera covariance visualization requirements in this saved setup:

```text
src/Kimera-VIO/params/M3DGRMonoOriginal/flags/VioBackend.flags
src/Kimera-VIO/params/M3DGRMonoBackend0/flags/VioBackend.flags
  -> --compute_state_covariance=true

src/Kimera-VIO-ROS/src/RerunTopicVisualizerNode.cpp
  -> ignore unusable all-zero odometry covariance before publishing posterior metrics
```

## What To Watch In Rerun

For each raw CBS-on experiment, the most important dashboard windows are:

```text
Scene
Video
GLIM Posterior Covariance
Kimera Posterior Covariance
Relative Belief Covariance
Accumulated Covariance / Belief Chain State
G->K Counts
K->G Counts
GLIM CBS Updates
Kimera CBS Updates
Lag Timing
Alignment Quality
```

Primary interpretation:

- If GLIM struggles: LiDAR cloud/trajectory looks bad, GLIM posterior covariance
  grows or spikes, or GLIM runtime/stage timings become unstable.
- If Kimera struggles: video quality is poor, Kimera posterior covariance grows,
  keyframes/factor count stall or jump, or Kimera runtime grows.
- If CBS struggles: individual estimators look sane, but CBS update panels show
  many duration mismatches, duplicates, rejected beliefs, or no injected factors.
- If timing is the problem: `Lag Timing` shows timestamp drift or mismatch, and
  CBS updates show duration mismatch or retry/same-key behavior.

For the estimator posterior panels specifically:

- use `translation_trace_cm2` as the main scalar for absolute marginal translation covariance
- use `sigma_max_cm` as the conservative interpretable 1-sigma scalar
- do not treat `uncertainty_frobenius_norm` as the main covariance metric

## Historical Active-Window Test Evidence

A 60 s Dynamic01 active-window raw CBS live Rerun test was run:

```text
/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/20260709-113028_m3dgr_dynamic01_raw_cbs_active_window_live_rerun
```

Log:

```text
/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/20260709-113028_m3dgr_dynamic01_raw_cbs_active_window_live_rerun/roslaunch_stdout.log
```

It confirmed:

```text
GLIM mode=active_window_temporary
Kimera CBS odometry factor mode=active_window_temporary
```

Observed active-window lifecycle:

```text
GLIM active-window factors queued:        119
GLIM active-window removals committed:   106
Kimera active-window factors staged:     106
Kimera active-window removals scheduled: 106
```

Known issue still present:

```text
Kimera -> GLIM duration mismatch is high.
GLIM -> Kimera often retries/same local key.
```

So active-window lifetime is working, but it does not by itself fix timestamp
matching/keyframe-density issues.

## Saved Rerun Covariance Setup

On 2026-07-09 the live M3DGR raw-CBS Rerun experiment setup was saved as the
current canonical framework for this workspace.

Non-algorithmic setup changes kept in that framework:

```text
Kimera M3DGR flags:
  --compute_state_covariance=true

Rerun topic visualizer:
  do not publish posterior covariance metrics from unusable all-zero odometry covariance
```

Saved validation artifacts:

```text
live recording id: m3dgr_dynamic01_covfix_20260709_134508
probe run dir:     /home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/20260709-134900_m3dgr_dynamic01_covprobe
probe sample:      /home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/20260709-134900_m3dgr_dynamic01_covprobe/kimera_covariance_sample.txt
```

Derived Kimera posterior scalars from the saved probe sample:

```text
translation_trace_cm2 ~= 191.9
sigma_max_cm ~= 13.0
```

If a future live run shows a flat-zero `Kimera Posterior Covariance` panel
again, first treat it as a setup regression against this saved framework.

## Original / Clean Baselines

There are clean original workspaces/configs used for estimator-only baselines.
Do not confuse those with CBS-on runs.

Kimera clean original baseline exists under:

```text
/home/yeranis/repos/V4RL/cbs_gtsam4.3/clean_kimera_ws
```

GLIM clean original baseline/config was created so GLIM-only results can be
kept separate from CBS edits. The M3DGR GLIM original config used by the launch:

```text
/workspace/cbs_gtsam4.3/runs/runtime_configs/config_m3dgr_avia_original_glim_reference
```

Estimator-only baselines should remain CBS-off. CBS-on comparisons should be
reported against clean GLIM-only and clean Kimera-only metrics.

When reporting metrics, include:

```text
APE RMSE
RPE RMSE
algorithm trajectory traveled
ground-truth trajectory traveled
seconds traveled
```

## Do Not Change Without Asking

Do not casually change:

```text
CBS algorithmic code
relative-belief semantics
M3DGR body-frame extrinsic
persistent default for M3DGR CBS-on
health-aware modes
covariance scaling
```

Current raw CBS-on experiments should keep:

```text
cbs_health_aware_enable:=false
covariance scale:=1.0
duration-gated short-horizon odometry
persistent factor lifetime
```

## Suggested Opening Prompt For A New Chat

```text
We are working in /home/yeranis/repos/V4RL/cbs_gtsam4.3 on CBS + GLIM + Kimera.
Container: cbsms_ws. Container workspace: /workspace/cbs_gtsam4.3.

Current dataset: M3DGR. Local sequences:
- Dynamic01
- Outdoor01
- Wheel-float01

Current default for M3DGR CBS-on is persistent beliefs:
- glim_cbs_mode:=inject_persistent
- cbs_odom_factor_mode:=persistent
- cbs_health_aware_enable:=false
- covariance scales 1.0
- duration gate enabled
- horizon 0.20 s, tolerance 0.06 s

Main live Rerun CBS launch:
src/glim_ros1/launch/m3dgr_glim_kimera_live_rerun_raw_cbs.launch

Canonical Rerun dashboard:
notes/rerun_raw_cbs_dashboard.md
Canonical live Rerun experiment setup:
notes/m3dgr_live_rerun_experiment_setup.md
Canonical Kimera factor-graph Rerun setup:
src/cbsms/docs/2026-07-17_kimera_factor_graph_rerun_setup.md
Canonical two-way CBS factor-graph Rerun setup (use for future factor-graph experiments):
src/cbsms/docs/2026-07-20_cbs_on_dual_factor_graph_rerun_setup.md
Blueprint helper:
src/cbsms/tools/send_raw_cbs_dashboard_blueprint.py

Do not change algorithmic CBS code unless I explicitly ask.

First task: use the current M3DGR active-window CBS-on setup, run or inspect the
next experiment, and interpret the Rerun dashboard focusing on estimator
posterior covariance, relative belief covariance, belief traffic, injections,
and timing/alignment.
```
