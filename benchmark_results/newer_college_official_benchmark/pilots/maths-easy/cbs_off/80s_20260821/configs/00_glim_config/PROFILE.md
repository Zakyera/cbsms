# Newer College 2021 Ouster/GPU GLIM optimized profile

This is the frozen independent-GLIM accuracy profile for the official
`2021-ouster-os0-128-alphasense` release.

- LiDAR: `/os_cloud_node/points` (`os_sensor`, approximately 10 Hz)
- IMU: `/os_cloud_node/imu` (`os_imu`, approximately 100 Hz)
- Extrinsic: Oxford `os_imu_to_os_sensor` / `T_sensor_imu`
  `[-0.014, 0.012, 0.015, 0, 0, 0, 1]`
- Odometry: CUDA multi-resolution VGICP, 5 s fixed-lag smoother
- Submapping/global mapping: CUDA VGICP; global optimization enabled
- Initialization/noise: NAIVE 0.75 s, paper-era bias noise `1e-3`

The NAIVE initializer requires the local one-line correction in
`initial_state_estimation.cpp` (`this->stamp = stamp`). It is otherwise the
upstream GLIM initializer and does not change the fixed-lag estimator.

This choice was frozen after a 14-variant, 30-second Maths-Easy sweep. Two
repeat runs gave 0.0411 m and 0.0469 m Base-frame, gravity-preserving vertical
RMSE on the shared 24.602-second interval, compared with 0.1694 m for upstream
LOOSE initialization at 1 second. The conventional full-SE(3) ATE remained
essentially unchanged because full alignment can absorb the initial tilt.

Keep `config_newer_college_2021_gpu_ouster` as the paper-matched control. This
optimized profile is provisional until it passes full-sequence and held-out
sequence validation; do not tune CBS parameters from this initializer sweep.

The Alphasense cameras are replayed only for Rerun visualization. They are not
inputs to GLIM. Oxford ground truth is the `Base` frame, so evaluation must
right-compose GLIM's `os_imu` pose with `T_os_imu_base` before SE(3) alignment.
