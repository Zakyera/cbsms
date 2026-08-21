# Newer College CBS integration and experiment handoff

**Date:** 2026-08-19
**Workspace:** `/home/yeranis/repos/V4RL/cbs_gtsam4.3`
**Container:** `cbsms_ws`
**Container workspace:** `/workspace/cbs_gtsam4.3`
**Immediate research objective:** obtain clean, original CBS-on results between GLIM and Kimera-VIO on the Oxford Newer College Dataset.

This document is the starting context for the next Codex chat. Read it before changing or running anything.

> **2026-08-22 official Maths-Easy completion:** The complete full-sequence
> CBS-off/on result pair, frozen estimator/CBS settings, full 6x6
> sender-to-receiver covariance audit, Rerun hashes, and reuse rules are now
> recorded in
> [the official Maths-Easy results handoff](2026-08-22_newer_college_maths_easy_official_results.md).
> This newer document supersedes the early dataset assumptions and incomplete
> experiment status later in this handoff.

> **2026-08-21 Rerun preset update:** The synchronized Newer College CBS-off
> dashboard is now frozen for reuse across all nine sequences. Before creating
> or presenting a Newer College recording, read
> [the canonical Newer College CBS-off Rerun preset](2026-08-21_newer_college_cbs_off_rerun_preset.md).
> It fixes the layout, sensor-time behavior, factor-graph styling, and CBS-off
> validation contract while explicitly requiring sequence- and
> collection-specific bags, calibration, ground truth, and alignment.

> **2026-08-21 official-results update:** Every full Newer College run must be
> finalized through
> [the official results reporting contract](2026-08-21_newer_college_official_results_reporting.md).
> It creates one hashed result package per sequence and CBS mode, preserves
> trajectory figures and Rerun provenance, and generates the final
> Markdown/CSV/LaTeX paper tables without admitting pilots or mismatched
> off/on coverage.

> **2026-08-20 GLIM update:** The independent original-GLIM Maths-Easy baseline
> is now complete and evaluated. Before launching GLIM, read
> [the authoritative original-GLIM baseline handoff](2026-08-20_newer_college_original_glim_baseline_handoff.md).
> It freezes the successful Ouster-IMU/GPU configuration, Rerun contract,
> Base-frame evaluation protocol, complete-run safeguards, and cross-collection
> validation rules. It also records that Cloister part 0 and Park parts 1-7
> remain absent locally, so Collection-2 full benchmarks are not yet authorized.

## 1. Immediate scope

The current priority is no longer the GLIM DCReg degeneracy side algorithm. Preserve that completed work, but do not use it to alter the new experiments.

The next task is to bring up the original, unweighted CBS system on Newer College and compare:

1. GLIM local-only performance.
2. Kimera-VIO local-only performance.
3. CBS off with both estimators running on the same interval.
4. GLIM-to-Kimera CBS (`G2K`).
5. Kimera-to-GLIM CBS (`K2G`).
6. Bidirectional CBS.

Use the original CBS mathematics and configurations:

- no DCReg-based policy;
- no covariance inflation or rescaling;
- no belief rejection or gating beyond the existing CBS matching rules;
- no health-based weighting;
- no solver modification;
- sender covariance scale `1.0`;
- receiver covariance scale `1.0`;
- Schur relative covariance;
- persistent CBS `BetweenFactor<Pose3>` insertion where that is the existing mode.

DCReg may remain installed and available, but use `dcreg_health.mode=off` unless a passive diagnostic recording is explicitly requested. Stage 2A/2B topics should also remain disabled unless needed for a separate passive audit.

## 2. What this project is

The platform runs two independent estimators on the same sensor rig:

- **GLIM:** LiDAR-inertial odometry and mapping.
- **Kimera-VIO:** visual-inertial odometry.
- **CBS:** Consensus-Based Smoothing exchanges short-horizon relative-pose beliefs between their fixed-lag smoothers.

The two exchange directions are:

- `G2K`: GLIM sends a relative belief to Kimera.
- `K2G`: Kimera sends a relative belief to GLIM.

For endpoint poses `T_i` and `T_j`, the sender constructs

```text
Z_ij = inverse(T_i) * T_j
```

and sends its relative mean and covariance. The receiver inserts the result as a GTSAM `BetweenFactor<Pose3>` after the existing timestamp, body-frame, and covariance conversions.

The project uses GTSAM tangent ordering everywhere internally:

```text
[rot_x, rot_y, rot_z, trans_x, trans_y, trans_z]
```

Do not introduce a ROS-style covariance remapping into the internal CBS path.

The nominal CBS relative-belief horizon used in the recent GEODE work is `0.20 s`. The exact horizon and endpoint tolerance for Newer College must be resolved once, recorded in the run manifest, and then kept identical across the comparison matrix.

## 3. Frozen work that must be preserved

The following accepted work already exists and must not be reopened merely for the Newer College experiments:

- Stage 1: passive GLIM DCReg observability monitor.
- Stage 2A: passive conservative CBS-edge metadata side channel.
- Stage 2B: passive Kimera receiver-side shadow analysis.
- Stage 2C: GEODE adapter, frame-chain evaluator, ground-truth tools, and scientific analysis.
- Stage 3A: reference-free observability characterization and paper-facing diagnostic work in progress.

Important conclusions to retain:

- The passive Stage 1 monitor is mathematically a no-op when sensor-event order is fixed.
- No valid public GEODE Gamma healthy reference was found.
- Relative Gamma health therefore remained unavailable, correctly and deliberately.
- The CBS mean and covariance path has not been modified by DCReg.
- Active health-based weighting is not authorized.

Relevant documentation:

- [Stage 1 closeout](2026-08-04_glim_dcreg_health_stage1_closeout.md)
- [GEODE live Rerun preset](2026-08-17_geode_live_rerun_preset.md)
- [Stage 3A reference-free characterization](2026-08-14_glim_dcreg_stage3a_reference_free_characterization.md)
- [M3DGR persistent-CBS handoff](2026-07-29_m3dgr_cbs_persistent_handoff.md)

## 4. Git and workspace state at handoff

This is a multi-repository workspace. Do not treat the workspace root as one Git repository.

| Repository | Branch | HEAD at handoff | State requiring care |
|---|---|---|---|
| `src/glim` | `cbs-gtsam43-noetic` | `6c4189e117c61015a79001388debb27413c78fa6` | clean |
| `src/glim_ros1` | `cbs-gtsam43-noetic` | `3bc6927d18ec154214065699f2c2325561afa314` | modified CBS bridge/launch files from the passive covariance audit |
| `src/liorf` | `cbsms/gtsam-4.3-develop` | `4609767c6f740a7713e8813e64c44a834f716de5` | clean |
| `src/cbs` | `cbsms/gtsam-4.3-develop` | `994b1d6a5c05fb38dd1b0731c6430ec11d2f1ff0` | modified BPSAM files from the passive receiver covariance/factor-strength audit |
| `src/Kimera-VIO` | `cbsms/gtsam-4.3-develop` | `bcd158f15c45fd27f2054037472e3caeb87e45fe` | modified `VioBackend.cpp` from the passive audit |
| `src/Kimera-VIO-ROS` | `cbsms/gtsam-4.3-develop` | `13b0cc09038b16bcbd8576170dada04532c62068` | modified Rerun/launch/ROS files; untracked `__pycache__` |
| `src/cbsms` | `cbsms/gtsam-4.3-develop` | `e9a0104f0005f69a385baa856f9daa8e68f32b7e` | Stage 3A tools/docs and covariance-audit work remain dirty/untracked |

Before any next-stage edit, record in every relevant repository:

```bash
git branch --show-current
git rev-parse HEAD
git status --short
git diff --stat
git diff --check
```

Do not reset, restore, stash, clean, or overwrite these changes. In particular, do not delete `__pycache__` merely to make a status display cleaner.

The accepted frozen commits include:

- `src/glim`: `6c4189e Add passive DCReg observability health sidecar`
- `src/glim_ros1`: `09395e8 feat: publish passive DCReg metadata for CBS beliefs`
- `src/glim_ros1`: `3bc6927 chore: expose optional Stage 2B launch configuration`
- `src/liorf`: `7ac6ea8` Stage 2A messages and `4609767` Stage 2B messages
- `src/cbs`: `0e2df4 Add Schur CBS relative covariance diagnostics`
- `src/cbs`: `928f9ca Stabilize active-window CBS odometry factors`
- `src/cbs`: `994b1d6 Extend CBS covariance and smoother diagnostics`
- `src/Kimera-VIO`: `bcd158f1 feat: expose passive CBS receiver diagnostics`
- `src/Kimera-VIO-ROS`: `13b0cc0 feat: add passive DCReg CBS shadow analysis`
- `src/cbsms`: `31514b3` Stage 2A, `70bb999` Stage 2B, and `310357c`/`f8147c5`/`e9a0104` GEODE Stage 2C work

No commit or push is authorized by this handoff.

## 5. Verified covariance status before Newer College

The most recent passive G2K covariance audit is here:

- `/home/yeranis/repos/V4RL/cbs_gtsam4.3/stage3a_characterization/covariance_audit/geode_offroad7_g2k_final_20260819/TECHNICAL_REPORT.md`

It used a 60-second GEODE Offroad7 G2K run and ten audited belief samples. Key results:

- Schur covariance versus direct propagation maximum difference: `3.2245162453993414e-19`.
- Raw sender covariance, sender-processed covariance, receiver-frame covariance, scaled covariance, and inserted covariance were numerically identical in that configuration.
- Inserted information inverse relative error: `1.831654047889204e-16`.
- All selected covariances were SPD; no jitter was required.
- External/local endpoint-reduced information trace ratio: min `0.03739`, median `0.96166`, max `1.90854`.
- Directional relative strength varied much more widely: `0.005287` to `66.5413`.
- NIS ranged from `9.066` to `429.05`.

This proves the audited G2K covariance transport was internally consistent; it does **not** establish an optimal covariance scale. K2G has not yet received the same detailed end-to-end audit.

For the original Newer College results, do not tune from those numbers. Keep scale `1.0`, record the raw data, and compare directions separately.

## 6. Oxford Newer College Dataset facts

Use the official Oxford Robotics Institute release only:

- Main dataset: https://ori-drs.github.io/newer-college-dataset/
- Stereo-camera release: https://ori-drs.github.io/newer-college-dataset/stereo-cam/
- Calibration: https://ori-drs.github.io/newer-college-dataset/stereo-cam/calibration-stereo/
- Ground truth: https://ori-drs.github.io/newer-college-dataset/ground-truth/
- Usage/topics: https://ori-drs.github.io/newer-college-dataset/usage/
- Downloads: https://ori-drs.github.io/newer-college-dataset/download/

The first target should be the **original stereo-rig short experiment**, not the later multi-camera extension.

The original rig contains:

- Intel RealSense D435i stereo cameras and IMU;
- Ouster OS1 Gen1 64-beam LiDAR and its IMU.

The relevant published topics include:

```text
/camera/infra1/image_rect_raw          30 Hz
/camera/infra2/image_rect_raw          30 Hz
/camera/accel/sample                  250 Hz
/camera/gyro/sample                   400 Hz
/camera/imu                           250 Hz
/os1_cloud_node/points                 10 Hz
/os1_cloud_node/imu                   100 Hz
/tf_static
```

The official ground truth is a precise six-degree-of-freedom trajectory derived by registering Ouster scans to a prior Leica BLK360 map. Camera poses are supplied at approximately 30 Hz and LiDAR poses at approximately 10 Hz. For the original stereo setup, the documented ground-truth pose convention is associated with the `RS_C1` camera frame. Verify the exact file header and transform direction from the downloaded release before evaluation.

The Ouster is synchronized to the acquisition computer using PTP. The official documentation reports camera/Ouster clock drift and supplies a per-experiment `time_offset.csv`. Apply the released time correction deterministically:

```text
t_camera_corrected = t_camera - time_offset(t_camera)
```

Do not estimate a new offset from estimator output.

The existing GLIM configuration contains Newer College transform comments, but these must not be trusted without verification. Re-derive `T_lidar_imu` from the official calibration and confirm the transform direction used by GLIM.

No full Newer College bag was found locally at this handoff. Files named `newer_*` under GTSAM point-test data are tiny unit-test fixtures, not the dataset.

Recommended dataset location:

```text
/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/datasets/NewerCollege/
```

Keep bags and generated artifacts out of Git.

## 7. Files that must be downloaded and hashed

For the initial short-experiment bring-up, collect:

1. Official short-experiment ROS bag or bags.
2. Exact stereo-camera calibration files.
3. Exact camera-IMU calibration.
4. Exact Ouster-to-camera/rig extrinsics.
5. Ouster beam intrinsics and metadata required to interpret PointCloud2.
6. `time_offset.csv` for the selected experiment.
7. Timestamped six-DoF ground-truth pose files for the LiDAR and/or camera frame.
8. Any official `/tf_static` content supplied in the bag.

Create a provenance manifest before running:

```text
official URL
local path
file size
SHA-256
download date
sequence name
duration
topic names/types/counts
calibration hashes
ground-truth convention
known limitations
```

Run `rosbag info` and inspect actual message definitions, PointCloud2 fields, frame IDs, timestamp ranges, and `/tf_static`. Do not assume the website summary exactly matches every bag revision.

## 8. Required frame and clock audit

CBS can only compare and exchange beliefs correctly when the two estimators refer to one explicitly defined physical body.

For Newer College, define and document:

```text
L  = Ouster LiDAR body
I_L = Ouster IMU body used by GLIM
C1 = RealSense left/first camera body (RS_C1)
I_C = RealSense IMU body used by Kimera
B  = common CBS external body
W  = estimator world frame
```

Using `T_A_B` to map coordinates from `B` into `A`, audit the entire chain from official Kalibr files and `/tf_static`:

```text
T_L_I_L
T_C1_I_C
T_C1_L (or the equivalent chain)
T_B_I_L
T_B_I_C
```

Using `B = RS_C1` is the natural first choice because the official ground truth is documented in that frame, but this choice must be confirmed from the actual release.

Do not align the estimator trajectories to infer these extrinsics. A single constant **left** world alignment may be used for display/evaluation where appropriate, but the fixed body-frame conjugation must come from calibration.

The GLIM and Kimera sensor streams do not need to share an IMU. In fact, using the Ouster IMU for GLIM and the D435i IMU for Kimera preserves the intended independence of the estimators. What matters is correct time synchronization and body-frame conversion.

## 9. Dataset-specific runtime profiles

Do not silently modify the GEODE runtime profile to accept Newer College.

Create explicit Newer College profiles:

- GLIM/Ouster profile using `/os1_cloud_node/points` and `/os1_cloud_node/imu`.
- Kimera stereo/D435i profile using the two rectified infrared image topics and the corrected RealSense IMU stream.
- A common CBS bridge profile with the verified body extrinsics.
- A ground-truth/evaluation profile using the official `RS_C1` convention.

Prefer replaying the existing PointCloud2 topic directly into GLIM. Only create an offline adapter if field semantics or per-point time data are not accepted by the present GLIM preprocessing path. If an adapter is required, prove deterministic preservation of point ordering, ring/channel, reflectivity/intensity, and per-point time.

Apply the released camera time-offset correction before Kimera consumes the timestamps or via a deterministic test-only replay path. Do not alter estimator mathematics to compensate for clock drift.

The first debug window should be exactly `60 s` of actual sensor time after both estimators have valid input coverage. Record the bag start offset, sensor start/end timestamps, and initialization delays.

## 10. Exact experiment matrix

Use one frozen 60-second interval for all initial runs.

| Run | GLIM | Kimera | G2K | K2G | Purpose |
|---|---:|---:|---:|---:|---|
| `local_glim` | on | off | off | off | LiDAR-inertial baseline |
| `local_kimera` | off | on | off | off | stereo visual-inertial baseline |
| `cbs_off` | on | on | off | off | simultaneous scheduling baseline |
| `g2k` | on | on | on | off | effect of GLIM beliefs on Kimera |
| `k2g` | on | on | off | on | effect of Kimera beliefs on GLIM |
| `two_way` | on | on | on | on | original bidirectional CBS result |

For every run require:

- same bag and exact sensor-time interval;
- same replay rate;
- same calibration and clock correction;
- same estimator parameters;
- same initialization policy;
- covariance scales equal to `1.0`;
- health-aware covariance disabled;
- DCReg off;
- no manual trajectory edits;
- a dedicated run directory and manifest.

The existing GEODE live runner supports `off`, `g2k`, and `two_way`, but it is dataset-specific. Do not pretend it already provides a valid isolated `k2g` mode for Newer College. Audit the bridge enable/receive controls and implement an explicit dataset-specific `k2g` mode before claiming that result.

## 11. What to record from each run

At minimum save:

- resolved configurations and calibration hashes;
- Git branches, HEADs, and dirty state;
- bag/topic manifest and exact time window;
- GLIM pose trajectory;
- Kimera pose trajectory;
- official ground-truth trajectory;
- GLIM and Kimera fixed-lag factor snapshots;
- local and CBS factor counts by type;
- every CBS publication, match, rejection, and insertion;
- belief endpoint keys, timestamps, duration, mean, and covariance;
- final inserted covariance and information matrix;
- pre-injection residual/NIS where passive audit hooks are enabled;
- process logs, exit codes, CPU time, and wall time;
- Rerun recording ID, `.rbl`, `.rrd`, hashes, and verification status.

Primary metrics should remain separate:

- translation APE in metres;
- rotation APE in radians/degrees;
- translation RPE at an explicitly declared delta or time horizon;
- rotation RPE;
- endpoint error;
- valid GT coverage;
- estimator output/drop/failure counts;
- CBS published, associated, accepted, inserted, rejected, and expired counts.

Do not combine radians and metres into one unscaled score. Do not use display alignment as an estimator correction. For paired comparisons, evaluate identical timestamps and GT-valid intervals.

## 12. Existing experiment/reporting framework

The generic experiment runner is:

```text
/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/cbsms/tools/cbsms_experiment.py
```

It already knows how to create structured run directories, record Git state/configuration, capture trajectories/logs, parse CBS transport, and produce reports. Reuse it where practical, but create an explicit Newer College profile instead of overloading an M3DGR or GEODE profile.

The old M3DGR run/report pattern is documented in:

```text
/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/cbsms/docs/2026-07-29_m3dgr_cbs_persistent_handoff.md
```

The GEODE scripts are useful examples for live orchestration and Rerun, but their sensor topics, frame chain, GT parser, and sequence mappings are not valid for Newer College.

## 13. Canonical Rerun setup to reuse

The best existing Rerun dashboard is documented here:

```text
/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/cbsms/docs/2026-08-17_geode_live_rerun_preset.md
```

Core files:

```text
/home/yeranis/repos/V4RL/cbs_gtsam4.3/stage3a_characterization/live_geode_medium_cbs_off/run_live_synchronized_60s.sh
/home/yeranis/repos/V4RL/cbs_gtsam4.3/stage3a_characterization/live_geode_medium_cbs_off/send_blueprint.py
/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/cbsms/tools/geode_stage3a_rerun.py
```

Saved canonical blueprints:

```text
/home/yeranis/repos/V4RL/cbs_gtsam4.3/stage3a_characterization/experiment_registry/geode/cbs_on/blueprints/geode_cbs_g2k_live60.rbl
/home/yeranis/repos/V4RL/cbs_gtsam4.3/stage3a_characterization/experiment_registry/geode/cbs_off/blueprints/geode_cbs_off_live60.rbl
```

Use these as layout references. Create Newer College copies under a new registry, for example:

```text
stage4_newer_college/experiment_registry/
  cbs_off/blueprints/
  g2k/blueprints/
  k2g/blueprints/
  two_way/blueprints/
  experiments.csv
```

The dashboard should contain:

1. Actual GLIM active fixed-lag factor graph.
2. Actual Kimera active fixed-lag factor graph.
3. Ground truth, GLIM, and Kimera trajectories in one common frame.
4. Live Ouster PointCloud2.
5. Original left and right camera images.
6. Factor composition and active-state counts.
7. CBS publication/insertion diagnostics.
8. Optional covariance panels when auditing; no DCReg panels are required for the main Newer College experiments.

The canonical GEODE dashboard solved several previous visualization problems. Preserve these rules:

- One recording ID and application ID (`cbsms`) for all producers in one run.
- One common `sensor_time` timeline.
- Connect to the already open Rerun viewer; do not spawn a new viewer per producer.
- GLIM and Kimera factor-graph panels show only the active fixed-lag state/factors.
- A grey trajectory provides full-history context behind each active factor graph.
- Kimera's grey context must be transformed by the same display-only alignment as its active graph.
- The trajectory-comparison panel uses one frozen SE(3) alignment per estimator after a declared warm-up; no scale fitting.
- Alignment is visualization/evaluation only and never feeds back into an estimator.
- Do not render stale all-history factor edges as if they were active-window factors.

Rerun connection addresses used successfully:

```text
host:      rerun+http://127.0.0.1:9876/proxy
container: rerun+http://172.17.0.1:9876/proxy
```

If needed, start one viewer/server on the host:

```bash
rerun --bind 0.0.0.0 --port 9876
```

Then all publishers must connect to that viewer using the same recording ID. The existing blueprint sender uses:

```bash
/home/yeranis/.local/share/pipx/venvs/rerun-sdk/bin/python \
  /home/yeranis/repos/V4RL/cbs_gtsam4.3/stage3a_characterization/live_geode_medium_cbs_off/send_blueprint.py \
  --recording-id "$RECORDING_ID" \
  --connect rerun+http://127.0.0.1:9876/proxy \
  --cbs-label "CBS G-to-K"
```

Adapt the labels and entity paths for Newer College, but retain the proven layout behavior.

## 14. Saving a Rerun experiment correctly

Do not rely on a raw intermediary `output.rrd` that may lack a finalized recording footer.

For every completed experiment:

1. Save/export the finalized recording deliberately.
2. Save the matching `.rbl` blueprint beside it.
3. Run `rerun rrd verify <recording.rrd>`.
4. Calculate SHA-256 for both `.rrd` and `.rbl`.
5. Record the Rerun recording ID and application ID in the manifest.
6. Reopen the saved recording with the saved blueprint and visually confirm all panels.

The successful GEODE example is:

```text
/home/yeranis/repos/V4RL/cbs_gtsam4.3/stage3a_characterization/live_geode_medium_cbs_off/saved_recordings/geode_offroad7_live60_g2k_finalgraph_20260818_135600/
```

Its primary `.rrd` passed verification. Use its README and saved files as the expected packaging model.

## 15. Recommended first execution sequence

The next chat should proceed in this order.

### Phase 1: read-only audit

1. Record all nested Git state; do not clean anything.
2. Confirm whether Newer College data have since been downloaded.
3. Hash official bag, calibration, GT, and time-offset files.
4. Inspect actual bag topics, schemas, frame IDs, clocks, and coverage.
5. Derive and unit-test the Ouster/RealSense/common-body frame chain.
6. Verify the GT pose direction and `RS_C1` convention.

Stop rather than guess if any body transform or timestamp correction is ambiguous.

### Phase 2: dataset profiles

1. Add Newer College GLIM configuration without changing GLIM algorithms.
2. Add Newer College Kimera stereo configuration without changing Kimera algorithms.
3. Add explicit CBS `off`, `g2k`, `k2g`, and `two_way` launch modes.
4. Add a Newer College GT parser and passive display/evaluation conversion.
5. Add a Newer College Rerun runner/blueprint based on the canonical GEODE layout.

### Phase 3: 60-second smoke tests

1. Confirm direct Ouster PointCloud2 and IMU ingestion by GLIM.
2. Confirm stereo and corrected IMU ingestion by Kimera.
3. Run GLIM-only and Kimera-only.
4. Run simultaneous `cbs_off`.
5. Run `g2k`, `k2g`, and `two_way` on exactly the same interval.
6. Save structured outputs and verified Rerun recordings.

### Phase 4: validate before interpreting

For each CBS direction confirm:

- the intended sender publishes;
- the unintended sender direction is disabled in one-way modes;
- endpoint timestamps and durations match;
- body-frame conjugation is correct;
- inserted covariance equals transmitted covariance except for explicitly configured processing;
- factor counts show the expected inserted factors;
- CBS off contains zero CBS factors;
- no DCReg or health policy changes mean/covariance/factor behavior.

Only then compare trajectory accuracy.

### Phase 5: longer results

After the 60-second matrix is stable, run the complete selected sequence and at least repeated paired runs where production scheduling can vary. Keep the short debug results separate from full-sequence scientific results.

## 16. Builds and environment

Start the existing container if needed:

```bash
docker start cbsms_ws
```

The host and container share the workspace. Build only affected packages after inspecting the accepted environment and existing reports. A typical command is:

```bash
docker exec -w /workspace/cbs_gtsam4.3 cbsms_ws bash -lc \
  'source /opt/ros/noetic/setup.bash && \
   source devel/setup.bash 2>/dev/null || true; \
   catkin build cbs glim glim_ros kimera_vio kimera_vio_ros liorf \
     --no-status --summarize -j4 -p1'
```

Do not modify implementation just to hide an unrelated historical test failure. Report such a failure precisely.

## 17. Acceptance criteria for the first Newer College milestone

The first milestone is complete when:

- official data/calibration/GT provenance is recorded;
- GLIM and Kimera both complete the same 60-second interval;
- ground truth, GLIM, and Kimera are expressed in one verified physical frame;
- CBS off, G2K, K2G, and two-way modes are unambiguous;
- factor counts prove the intended direction(s);
- covariance scale remains `1.0` and health weighting is absent;
- GLIM/Kimera legacy behavior is unchanged in CBS-off mode;
- all six trajectories/conditions can be compared with official six-DoF GT;
- Rerun displays both true active fixed-lag graphs, full grey contexts, images, cloud, trajectories, and CBS diagnostics on one timeline;
- each run has a manifest, structured logs, `.rbl`, verified `.rrd`, and hashes;
- no tuning conclusion is made from one debug window.

## 18. Stop conditions

Stop and report instead of improvising if:

- official camera/LiDAR/body extrinsics cannot be proven;
- the GT pose direction or tracked frame is ambiguous;
- the camera time correction cannot be applied deterministically;
- the Ouster point schema loses timing needed for deskewing;
- GLIM or Kimera requires an estimator-algorithm change merely to ingest the dataset;
- a one-way CBS mode still inserts factors in the opposite direction;
- CBS off inserts any CBS factor;
- sender and inserted covariance differ without an explicit configured transform;
- Rerun producers use different recording IDs or timelines;
- display alignment would be mistaken for estimator correction;
- active DCReg weighting, covariance inflation, or gating appears necessary.

## 19. What the next chat should report

For each completed milestone, report:

1. Git state before and after.
2. Files added or changed.
3. Dataset provenance and hashes.
4. Bag/topic/timestamp audit.
5. Calibration and frame-chain equations.
6. Time-offset implementation.
7. Exact 60-second interval.
8. GLIM and Kimera configuration.
9. CBS direction and factor-count proof.
10. Covariance-path verification.
11. GLIM/Kimera/GT accuracy metrics.
12. Runtime and failure counts.
13. Rerun recording IDs, paths, hashes, and verification.
14. Remaining blockers.

## 20. Copy-paste prompt for the next chat

```text
Continue the V4RL CBS project using the handoff:

/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/cbsms/docs/2026-08-19_newer_college_cbs_handoff.md

The immediate objective is clean original CBS results between GLIM and
Kimera-VIO on the Oxford Newer College original stereo-rig short experiment.
Set the DCReg/health side path aside without deleting or modifying its frozen
implementation. Do not tune covariance, add weighting, gate beliefs, or alter
either estimator.

First audit and preserve every nested Git worktree. Then locate or obtain the
official Newer College bag, calibration, time_offset.csv, and 6-DoF ground
truth; hash them; inspect the actual ROS topics and frame conventions; and
prove the Ouster-IMU, RealSense, RS_C1, and common-CBS-body transform chain.

Create dataset-specific GLIM, Kimera stereo, CBS, evaluator, and Rerun profiles.
Use a single frozen 60-second sensor-time interval and run local GLIM, local
Kimera, CBS off, G2K, K2G, and two-way CBS with covariance scales 1.0 and no
health policy. Reuse the canonical GEODE Rerun dashboard layout and save a
verified .rrd plus .rbl for every experiment. Prove factor direction/counts
and covariance transport before interpreting trajectory results.

Do not commit or push unless I explicitly request it.
```

## 21. Bottom line

The project already has mature estimator integration, persistent CBS factor insertion, structured experiment tooling, covariance auditing, and a strong live Rerun dashboard. The Newer College task should be treated primarily as a careful dataset/frame/time integration and controlled CBS comparison—not as a reason to change CBS, GLIM, Kimera, or DCReg mathematics.

The first defensible result is a paired, same-window, official-calibration comparison of CBS off, G2K, K2G, and bidirectional CBS against Newer College six-DoF ground truth, with the exact active factor graphs and trajectories visible and saved in Rerun.
