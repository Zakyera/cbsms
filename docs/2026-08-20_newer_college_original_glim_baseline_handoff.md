# Newer College 2021 original-GLIM baseline handoff

**Date:** 2026-08-20
**Workspace:** `/home/yeranis/repos/V4RL/cbs_gtsam4.3`
**Validated sequence:** complete logical Maths-Easy (both official bag parts)
**Mode:** original standalone GLIM, CBS disabled
**Status:** frozen baseline configuration; full Maths-Easy run complete

This is the authoritative launch and evaluation handoff for the independent
GLIM LiDAR-inertial baseline on Oxford's
`2021-ouster-os0-128-alphasense` Multi-Camera Newer College release. Read this
before changing the GLIM configuration or launching another Newer College
sequence.

## Portability verdict

Use this same GLIM profile as the default starting point for all nine logical
sequences in Collections 1, 2, and 3.

That reuse is justified because:

- all three collections use the same Ouster OS0-128 LiDAR and its internal IMU;
- the required topics are `/os_cloud_node/points` and `/os_cloud_node/imu`;
- Oxford's `os_imu_lidar_transforms.yaml` explicitly says its transforms are the
  same for all three collections;
- the profile does not consume any camera stream; and
- Collection-specific camera calibration therefore does not alter the
  standalone GLIM estimate. The cameras shown in Rerun are passive bag video.

This is a portability claim about sensor configuration, not yet an accuracy
claim for every sequence. The optimized NAIVE initializer was selected with a
30-second Maths-Easy sweep and has passed one complete Maths-Easy run. Every
other logical sequence must first pass the validation gate below. In
particular, Stairs, Park, and the hard sequences stress different geometry and
motion, while Park also stresses long-run memory and global mapping.

Do not silently tune parameters per sequence. A changed parameter set is a new
named experimental profile, not this baseline.

## Frozen sensor contract

| Item | Frozen value |
|---|---|
| LiDAR topic | `/os_cloud_node/points` |
| LiDAR frame | `os_sensor` |
| LiDAR rate | approximately 10 Hz |
| IMU topic | `/os_cloud_node/imu` |
| IMU frame | `os_imu` |
| IMU rate | approximately 100 Hz |
| Accelerometer scale | `1.0` |
| GLIM base frame | `os_imu` |
| `T_lidar_imu` | `[-0.014, 0.012, 0.015, 0, 0, 0, 1]` |
| Camera input to GLIM | none |
| CBS | disabled |

Do not substitute `/alphasense_driver_ros/imu` in this frozen profile. That was
an earlier bring-up branch and requires a different cross-sensor extrinsic and
noise model. The successful official-GLIM baseline uses the Ouster IMU.

The LiDAR/IMU extrinsic is Oxford's `os_imu_to_os_sensor`, documented as
`T_sensor_imu`; it transforms IMU-frame points into the LiDAR frame. It is
shared across Collections 1-3.

## Frozen GLIM profile

Active profile on the host:

```text
/home/yeranis/repos/V4RL/cbs_gtsam4.3/clean_glim_ws/src/glim/
  config_newer_college_2021_gpu_ouster_optimized
```

Container path:

```text
/workspace/cbs_gtsam4.3/clean_glim_ws/src/glim/
  config_newer_college_2021_gpu_ouster_optimized
```

The active configuration chain is:

```text
config.json
  -> config_ros.json
  -> config_sensors.json
  -> config_preprocess.json
  -> config_odometry_gpu.json
  -> config_sub_mapping_gpu.json
  -> config_global_mapping_gpu.json
```

Important estimator values:

| Area | Frozen value |
|---|---|
| Odometry backend | `libodometry_estimation_gpu.so` |
| Initialization | `NAIVE`, `0.75 s` window |
| Fixed lag | `5.0 s` |
| IMU noise `(acc, gyro, integration, bias)` | `(0.05, 0.02, 0.001, 0.001)` |
| Bias estimation | enabled (`fix_imu_bias=false`) |
| ISAM2 | Gauss-Newton, relinearize skip `1`, threshold `0.1` |
| Odometry VGICP | CUDA, 0.25-0.5 m adaptive voxels, 2 levels |
| Odometry input points | random-grid target `10000` |
| Odometry keyframes | overlap strategy, maximum `15` |
| Fixed-lag full-connection window | `2` |
| Submapping | CUDA VGICP factors, submap optimization disabled |
| Global mapping | CUDA VGICP, optimization and IMU enabled |
| Global implicit-loop gate | distance `100 m`, overlap `0.2` |
| Global scale correction | none; estimator is metric |

The profile depends on the local one-line NAIVE-initializer correction in
`src/glim/odometry/initial_state_estimation.cpp`:

```cpp
this->stamp = stamp;
```

Without it, the member timestamp does not advance and the intended
initialization window is not honored. This correction does not alter the
fixed-lag estimator after initialization.

The paper-matched control profile remains
`config_newer_college_2021_gpu_ouster`. Do not overwrite it with the optimized
profile.

## Software and hardware identity

The validated run used:

- container: `cbsms_ws_gpu`;
- GPU: NVIDIA GeForce RTX 5070 Ti Laptop GPU, 12,227 MiB reported memory;
- clean GLIM clone HEAD: `88b3833229a9c3308e95065719a40acdd5f64c33`;
- clean GLIM ROS1 clone HEAD: `84e203f19d2f5d41890a02b258eac6c0047f339a`;
- GLIM ROS node SHA256:
  `7f824794053826f8f3b7b7eeed05ec4a93aae40cf25ae9c34656dad365415a61`.

These clean-clone repositories contain uncommitted local implementation and
configuration changes. Git HEAD alone is not a complete reproducer. The run
directory contains an exact copy of the full configuration, and the source
correction above must also be present. Its complete source-file SHA256 at the
validated run state is
`ab9412c895c28e70d0497175c2c6a4920cd65f64650a217bf1ee1a62200135d8`.

Active-file hashes:

| File | SHA256 |
|---|---|
| `config.json` | `7084990c4dadefa8629be430f65449eac23156a1df29e745ae43f5d0570360c3` |
| `config_ros.json` | `d861c3ad665fbdb620664f0608af73c34e9865969bf3caa0a167e44c6dad6446` |
| `config_sensors.json` | `37a7195a2677b1eadb87fb6713c28749f4a2e7d6f636e1f1e52a6fc6376ea62e` |
| `config_preprocess.json` | `6581aedba083e77aea5f7091e9ef5fdd0916b5b467f4566fa40f8e98b7fb9001` |
| `config_odometry_gpu.json` | `a9f52ab4d9c548e16d79cdb7268b6652212a46595ddc61d2e74e714da7c164d4` |
| `config_sub_mapping_gpu.json` | `ca61902a8482d1ace02edf8ae23787de7bf86748d49e6323fafc1bd8c2b57dd8` |
| `config_global_mapping_gpu.json` | `d97b8e26075fe88a73897ff94c7dce8b30fd61c38c70d048d8ac49450f82b372` |

## Gold reproduction command

The complete successful Maths-Easy runner is:

```text
/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/
  newer_college_maths_easy_glim_full_optimized_20260820/
  run_full_maths_easy.sh
```

Run it from the host with:

```bash
docker exec cbsms_ws_gpu bash \
  /workspace/cbs_gtsam4.3/runs/newer_college_maths_easy_glim_full_optimized_20260820/run_full_maths_easy.sh
```

The current completed run intentionally refuses to overwrite itself. A new
run needs a new run directory and a unique Rerun recording ID.

The runner is the behavioral template for every other sequence. Preserve these
details:

1. play every official split bag in chronological order without merging it;
2. use `--clock`, `bag_rate=1.0`, and `use_sim_time=true`;
3. keep `shutdown_on_bag_finish=false`;
4. wait for the rosbag player to exit;
5. drain GLIM's callback queues for 30 seconds before SIGINT;
6. reject a run whose final odometry is not within approximately one 10 Hz scan
   of the final point-cloud stamp;
7. preserve `/tmp/dump`, trajectories, graph, values, logs, exact config, bag
   manifest, and hashes.

The Maths-Easy launch accepts two bag paths. Other sequences have between one
and eight parts, so a future sequence runner must pass the correct complete
chronological list to one `rosbag play` process. Do not use only the first bag
part and do not concatenate the originals.

## Rerun contract

The validated recording is:

```text
newer_college_maths_easy_original_glim_optimized_full_v2
```

The viewer is a separately running Rerun server. The container endpoint used by
the run was:

```text
rerun+http://172.17.0.1:9876/proxy
```

Required live entities are:

- GLIM online trajectory;
- actual GLIM fixed-lag factor graph;
- current Ouster point cloud;
- Oxford ground truth and aligned trajectory comparison;
- passive left/right original camera video from cam1/cam0.

The camera streams are visualization only and must not be described as GLIM
measurement inputs. Use a unique recording ID for every run so several trials
do not collapse into one Rerun source.

## Output semantics

GLIM writes four trajectories:

| File | Meaning | Use |
|---|---|---|
| `odom_imu.txt` | online fixed-lag odometry in Ouster-IMU frame | no-loop-closure baseline and live Rerun |
| `traj_imu.txt` | globally optimized trajectory in Ouster-IMU frame | primary final GLIM result |
| `odom_lidar.txt` | online odometry in LiDAR frame | frame-chain cross-check |
| `traj_lidar.txt` | global trajectory in LiDAR frame | frame-chain cross-check |

Do not report `odom_imu.txt` as the loop-closed result. Do not report
`traj_imu.txt` as the online fixed-lag result.

## Ground-truth and ATE protocol

Oxford ground truth is the rig `Base` frame. First convert each GLIM Ouster-IMU
pose to the Base pose using the release transform chain:

```text
T_base_os_imu.translation = [-0.013, 0.012, 0.106] m
T_base_os_imu.rotation    = [0, 0, 0, 1] (qx, qy, qz, qw)

T_world_base = T_world_os_imu * inverse(T_base_os_imu)
```

Then interpolate Oxford ground truth to estimator timestamps and compute
translation ATE after Umeyama/Kabsch SE(3) alignment with scale fixed to one.

Report two timing conventions and never mix their labels:

- **primary paper-comparison value:** raw estimator timestamp;
- **secondary scan-centred diagnostic:** compare at timestamp `+0.050 s`.

The `+0.050 s` result corrects the validated Newer College convention that the
GLIM ICP state is scan-centred while the ROS point-cloud stamp denotes scan
start. It is useful internally, including for vertical-error diagnosis, but the
GLIM paper does not document the same offset. Therefore it is not the direct
Table-V comparison number.

## Definitive Maths-Easy result

The complete logical sequence contained two bags with a combined span of
215.999079 s. GLIM produced 2,152 poses and 2,152 graph updates. Its final
odometry was 0.099621 s behind the last LiDAR cloud, and no fatal, corruption,
connection, or segmentation error occurred.

| Output | Raw-time SE(3) ATE | `+0.050 s` SE(3) ATE | `+0.050 s` height RMSE |
|---|---:|---:|---:|
| Online `odom_imu` | 0.095647 m | 0.075386 m | 0.149508 m |
| Global `traj_imu` | **0.076543 m** | **0.049664 m** | 0.122644 m |

For context, GLIM Table V reports Maths-Easy ATE values of 0.096 m without
loop closure and 0.082 m with loop closure. The raw-time results therefore
essentially reproduce the published odometry and modestly outperform the
published global result. Do not compare the paper's 0.082 m directly with the
time-corrected 0.049664 m.

Authoritative run artifacts:

```text
/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/
  newer_college_maths_easy_glim_full_optimized_20260820/
```

Read `FULL_RUN_REPORT.md`, `evaluation/metrics.json`, and
`evaluation/scan_centered_metrics.json`. The first attempt under
`attempt1_trailing_backlog/` is deliberately preserved but is incomplete and
must never be used as the official result.

## Dataset availability as of this handoff

The configuration can serve all collections, but the local dataset is not yet
complete enough to run all of them officially.

| Sequence | Expected parts | Present parts | Local run status |
|---|---:|---:|---|
| Quad-Easy | 1 | 1 | available |
| Quad-Medium | 1 | 1 | available |
| Quad-Hard | 1 | 1 | available |
| Stairs | 1 | 1 | available |
| Cloister | 2 | 1 | incomplete |
| Park | 8 | 1 | incomplete |
| Maths-Easy | 2 | 2 | validated complete |
| Maths-Medium | 1 | 1 | available |
| Maths-Hard | 2 | 2 | available |

Missing Collection-2 parts are Cloister part 0 and Park parts 1-7. Do not run
or report full-sequence Cloister/Park benchmarks until they are present and
verified. The exact remote filenames are recorded in
`src/datasets/newer/newer_college_download_report.md` and
`newer_college_remote_manifest.tsv`.

## Validation gate for each new logical sequence

Before accepting another sequence as an official baseline:

1. verify every expected bag part and its exact byte size;
2. verify the Ouster points/IMU topics and timestamps;
3. run a 30-second smoke test with the frozen profile;
4. confirm finite gravity initialization and no NaN/GTSAM exception;
5. run the complete logical sequence with all parts in one chronological replay;
6. drain callbacks and enforce the final-stamp completeness check;
7. preserve `odom_*`, `traj_*`, `graph.bin`, `values.bin`, logs, config, and
   SHA256 hashes;
8. evaluate Base-frame raw-time ATE as the primary metric and `+0.050 s` as the
   separately labeled diagnostic;
9. inspect the trajectory, fixed-lag graph, point cloud, GT, and passive video
   in Rerun;
10. compare with the corresponding GLIM Table-V value before declaring the run
    accepted.

Only after these gates pass should this profile be called validated for that
sequence. CBS-on work must use the accepted independent GLIM baseline as its
control and must not mutate this frozen profile in place.
