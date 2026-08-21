# Newer College 2021 Maths-Easy: GLIM and Kimera with CBS off

Date: 2026-08-20

> **GLIM update:** The initial GLIM bring-up recorded below has been superseded
> by the complete Ouster-IMU/GPU baseline and its ground-truth evaluation. Use
> [the authoritative original-GLIM handoff](2026-08-20_newer_college_original_glim_baseline_handoff.md)
> for all subsequent GLIM runs. The Kimera notes in this document remain useful.

## Status

The independent GLIM and Kimera-VIO baselines are configured and smoke-tested on the complete logical Maths-Easy sequence. Both launch files include the two official bag segments in chronological order, default to their combined 216-second span, and stream to the project's already-running Rerun viewer by default.

This paragraph describes the original bring-up milestone. A later complete
GLIM run and ATE evaluation now exist in the authoritative handoff linked
above.

## Dataset inputs

Dataset root inside the container:

```text
/workspace/cbs_gtsam4.3/src/datasets/newer/2021-ouster-os0-128-alphasense
```

Maths-Easy inputs:

```text
collection 3 - maths institute/2021-04-07-13-49-03_0-math-easy.bag
collection 3 - maths institute/2021-04-07-13-52-31_1-math-easy.bag
collection 3 - maths institute/ground_truth/gt_state_easy.csv
```

The first bag spans 178.250388 seconds and the second spans 37.746627 seconds. The timestamp gap is 0.002064 seconds, so their combined logical duration is 215.999 seconds. Do not merge the bag files.

## Sensor assignment

GLIM uses:

- `/os_cloud_node/points` at 10 Hz, frame `os_sensor`
- `/alphasense_driver_ros/imu` at 200 Hz, frame `imu_sensor_frame`

Kimera uses the calibrated forward stereo pair and the same Alphasense IMU:

- left: `/alphasense_driver_ros/cam1/compressed`, 720x540 mono at 30 Hz
- right: `/alphasense_driver_ros/cam0/compressed`, 720x540 mono at 30 Hz
- IMU: `/alphasense_driver_ros/imu` at 200 Hz

Oxford identifies cam1 as the forward-left camera and cam0 as the forward-right camera. This order is also required by Kimera: reversing it produces a negative 0.111 m stereo baseline and Kimera rejects the calibration.

## Calibration provenance and conversions

The LiDAR/IMU transform comes from:

```text
os_imu_lidar_transforms.yaml
```

That file supplies `os_sensor_to_as_imu`, or `T_as_imu_os_sensor`. GLIM requires `T_lidar_imu`, which maps IMU-frame points into the LiDAR frame, so the configured value is its inverse:

```text
T_os_sensor_as_imu = [0.037, -0.008, -0.026, 1, 0, 0, 0]
```

The quaternion is in GLIM's `[qx, qy, qz, qw]` order. The transform is the Oxford release calibration, not the approximate Newer College value in GLIM's example comments.

The stereo calibration comes from:

```text
collection 3 - maths institute/cam_calibration/cam0-1/
  camchain-imucam-2021-03-31-15-56-22-cam0-1.yaml
```

Kalibr provides `T_cam_imu`; Kimera's `T_BS` is the inverse, `T_imu_cam`. The Kimera parameter files contain the full inverse matrices, Collection-3 intrinsics, equidistant distortion coefficients, 720x540 resolution, and the calibrated 0.111173535 m baseline.

The Alphasense noise values used in both profiles come from Oxford's supplied IMU-camera calibration report:

```text
accelerometer noise density: 0.019
gyroscope noise density:     0.019
accelerometer random walk:   0.0043
gyroscope random walk:       0.000266
```

Kimera also uses the cam1-to-IMU time shift of `0.0018438881109310357` seconds. The runtime message labels the number as `ns`, but the Kimera data-provider conversion treats the YAML value as seconds.

## GLIM-specific compatibility decisions

This checkout combines GLIM core v1.2.1-era code with the ROS1 wrapper at v1.1.0. Two compatibility issues were found during real replay:

1. `acc_scale: 0.0` is documented by the newer core as auto-detection, but this ROS1 wrapper directly multiplies acceleration by the value. It therefore converted every valid Oxford acceleration sample to zero. The dataset profile explicitly sets `acc_scale: 1.0`.
2. GLIM's NAIVE initializer had `stamp = stamp`, so its member timestamp never advanced. The source is corrected to `this->stamp = stamp`. NAIVE initialization now estimates gravity from the first three seconds instead of hard-coding an initial orientation. LOOSE initialization produced a NaN factor-graph error on this bag and is not used in this CPU profile.

The machine/container has no NVIDIA device or driver. The profile uses CPU odometry, CPU submapping, and CPU global mapping. The compiled `gtsam_points` library may still print `cudaErrorInsufficientDriver`; this warning was non-fatal in the tested CPU path.

## Kimera-specific decisions

- Stereo-IMU frontend (`frontend_type: 1`)
- standard `VioBackend` selected through `backend_type: 0` in this fork
- automatic IMU initialization enabled
- loop closure, RViz, and OpenCV/Pangolin display disabled
- headless Rerun trajectory, geometry, scalar metrics, and factor-graph inspector enabled
- two passive 5 Hz Rerun image streams for the original cam1/cam0 compressed images
- compressed images converted to raw `mono8` with two small ROS bridge nodes
- CBS ROS bridge disabled
- CBS backend computation disabled
- CBS health-aware weighting disabled

The launch sets `multirobot=true` only to suppress an optional `pose_graph_tools` RViz include that is unavailable in the current overlay; it does not enable multi-robot processing.

## Commands

Enter the existing development container:

```bash
docker exec -it cbsms_ws bash
source /opt/ros/noetic/setup.bash
source /workspace/cbs_gtsam4.3/devel/setup.bash
```

Run the full standalone GLIM baseline:

```bash
roslaunch glim_ros newer_college_2021_glim_only.launch
```

Run the full standalone Kimera baseline:

```bash
roslaunch glim_ros newer_college_2021_kimera_only.launch
```

Both commands expect an existing Rerun gRPC server and do not open a second viewer. The default container endpoint is:

```text
rerun+http://172.17.0.1:9876/proxy
```

Override `rerun_host` if the Docker gateway changes. Give every replay a unique recording ID, for example:

```bash
roslaunch glim_ros newer_college_2021_glim_only.launch \
  bag_duration:=60 \
  rerun_recording_id:=newer_college_maths_easy_glim_cbs_off_20260820

roslaunch glim_ros newer_college_2021_kimera_only.launch \
  bag_duration:=60 \
  rerun_recording_id:=newer_college_maths_easy_kimera_cbs_off_20260820
```

GLIM publishes its odometry, current Ouster scan, front-left image, uncertainty, and graph-inspector entities. Kimera publishes VIO trajectory/geometry, scalar metrics, graph-inspector entities, and both forward stereo images. All producers in one launch share its recording ID.

The live Rerun view deliberately does not parse `gt_state_easy.csv`; Oxford ground truth remains an offline evaluation input until a format-specific alignment/evaluation layer is added.

For a short reproducible smoke test, override the logical sensor-time duration and optionally slow replay:

```bash
roslaunch glim_ros newer_college_2021_glim_only.launch bag_duration:=10 bag_rate:=0.5
roslaunch glim_ros newer_college_2021_kimera_only.launch bag_duration:=15 bag_rate:=0.5
```

Useful live checks:

```bash
rostopic echo -n 1 /glim_ros/odom
rostopic echo -n 1 /kimera_vio_ros/odometry
```

## Smoke-test evidence

GLIM, 10 sensor-seconds at 0.5x:

- gravity initialization completed with finite state
- 69 post-initialization LiDAR states were processed
- finite odometry was observed on `/glim_ros/odom`
- no NaN, GTSAM exception, or estimator crash after the fixes
- the built-in validator reported much better IMU rotational prediction, but inconsistent translational improvement; retain this as a calibration/tuning item for the ground-truth evaluation

Kimera, 15 sensor-seconds at 0.5x:

- stereo/IMU initialization completed
- 449 left/right frames reached the data provider
- 64 backend updates completed
- finite odometry was observed on `/kimera_vio_ros/odometry`
- no CBS topic was advertised and the log explicitly reported `CBS backend computation paths: disabled`
- all VIO threads joined cleanly when rosbag ended

Kimera prints `Pipeline successful? No!` when roslaunch terminates it because the required rosbag process has ended. In this test it followed a clean, intentional manual shutdown and is not evidence of a VIO exception.

## Live Rerun replay evidence

On 2026-08-20, the already-open host viewer was reachable from `cbsms_ws` at `172.17.0.1:9876`. Two independent 60-sensor-second, 1.0x replays then completed:

- `newer_college_maths_easy_glim_cbs_off_20260820`: both Rerun producers connected, gravity initialization completed, 569 post-initialization LiDAR states were processed, and graph-inspector rows reached state 565.
- `newer_college_maths_easy_kimera_cbs_off_20260820`: the Kimera and two image producers connected to the same recording, stereo processing stayed near 30 Hz, the backend passed 200 updates, and both camera streams published at least 250 images each to Rerun.
- Both launch processes exited with code 0 when their required 60-second rosbag playback completed.
- No NaN, fatal exception, estimator crash, or Rerun connection failure was found in either replay.
- GLIM belief exchange, Kimera's ROS belief bridge, and Kimera's CBS backend remained disabled throughout.

## Files added for this profile

```text
glim/config_newer_college_2021_cpu/
glim_ros1/launch/newer_college_2021_glim_only.launch
glim_ros1/launch/newer_college_2021_kimera_only.launch
Kimera-VIO/params/NewerCollege2021Stereo/
Kimera-VIO-ROS/launch/kimera_vio_ros_newer_college_2021_stereo.launch
```

## Next validation gate

Before enabling CBS, run both complete 216-second standalone baselines, save their trajectories, convert the Oxford `gt_state_easy.csv` only in the evaluation layer, and compute identical ATE/RPE metrics in a declared common frame. Do not tune CBS against this sequence until the two independent baselines and their time/frame alignment pass that gate.
