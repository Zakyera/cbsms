# Canonical Newer College CBS-Off Rerun Preset

**Date frozen:** 2026-08-21
**Workspace:** `/home/yeranis/repos/V4RL/cbs_gtsam4.3`
**Reference run:** Newer College Maths-Easy, edited GLIM + edited Kimera,
CBS off, 80 sensor-seconds

This is the default Rerun presentation contract for Newer College CBS-off
experiments. Reuse it for all nine logical sequences instead of constructing a
new layout for every run:

- Collection 1: Quad-Easy, Quad-Medium, Quad-Hard, Stairs.
- Collection 2: Cloister, Park.
- Collection 3: Maths-Easy, Maths-Medium, Maths-Hard.

The preset standardizes visualization and synchronization. It does **not**
authorize reusing a bag, calibration, ground-truth file, alignment, or time
window from another sequence or collection.

## Frozen dashboard contract

The active `Live estimators, factor graphs and trajectories` tab contains:

1. The actual GLIM fixed-lag factor graph.
2. The actual Kimera fixed-lag factor graph in Kimera's published incremental
   display frame, with active states/factors over a grey trajectory history.
3. Oxford ground truth, GLIM, and Kimera in one aligned trajectory view.
4. Synchronized GLIM PointCloud2 playback.
5. Two original camera streams used for visual context. The Kimera processing
   stream must be identified in the run manifest; the Maths-Easy reference uses
   physical cam4.
6. GLIM active factor composition and active-state counts.
7. Frozen DCReg condition ratios and absolute masks in separate plots.
8. The existing observability/error tab and CBS covariance-audit tab. Empty or
   zero CBS audit entities are valid in CBS-off recordings.

The layout source of truth is:

`src/cbsms/tools/geode_stage3a_rerun.py::make_blueprint`

Apply it with:

`src/cbsms/tools/send_newer_college_cbs_off_blueprint.py`

## Time and trajectory contract

- Bag video, LiDAR, factor graphs, estimator trajectories, and ground truth use
  one ROS sensor-time timeline.
- Trajectories are logged as time-indexed growing prefixes. Never log the final
  complete path as static geometry, because that exposes future poses while the
  playback cursor is still behind.
- GLIM and Kimera are each aligned to Oxford ground truth for display and
  evaluation with one rigid SE(3) transform. Scale is fixed at one.
- No Sim(3), time-offset fitting, path deformation, or estimator feedback is
  allowed.
- The displayed LiDAR scan uses the recorded GLIM pose plus the verified
  Oxford Base/LiDAR extrinsic and the same display alignment as the GLIM
  trajectory.
- All scoring remains in the declared Oxford evaluation frame. Display
  alignment must never replace physical sensor extrinsics.

## Factor-graph presentation contract

- GLIM factor types remain separate where classified: IMU preintegration, bias
  random walk, marginal prior, and other factors. In the current Newer College
  build, the grey `other` edges are the unclassified
  `IntegratedVGICPFactorGPU` LiDAR scan-matching constraints.
- Kimera uses `nr_states=50` for this benchmark family. Its active optimizer
  graph is transformed once per snapshot into the incremental published pose
  frame; the transform preserves all within-snapshot factor geometry.
- The Kimera pane records factor-marker labels and CBS-merge overlays for
  forensic inspection, but the presentation view hides them so they do not
  obscure the active graph.
- A marginal prior drawn across many active states is a support/hyperedge
  visualization of a dense `LinearContainerFactor`. It is not a chain of
  pairwise old-to-new factors and must not be interpreted as such.

## CBS-off contract

CBS off means all estimator-coupling injection paths are disabled:

- GLIM belief bridge and health-aware injection path: false.
- Kimera belief bridge, backend CBS path, and health-aware path: false.
- No CBS activity marker and no accepted CBS factor insertion.

CBS-named schema entities may still be present with zero or cleared values.
Their existence alone does not mean CBS was enabled. Every run report must
record the launch flags and verify zero CBS activity.

## What changes for every sequence

The following are run inputs, not preset defaults:

- all ROS bag segment paths and their chronological order;
- the exact Collection-1, Collection-2, or Collection-3 calibration set;
- camera/IMU/LiDAR extrinsics, time offsets, and Ouster metadata;
- the sequence's official Oxford six-DoF ground truth;
- bag start, duration, replay rate, and evaluated coverage;
- Kimera physical camera topic and any justified frame skips;
- the rigid display/evaluation alignments computed from that run;
- recording ID, recording name, output paths, and file hashes.

Do not reuse Collection-1 calibration for Collections 2 or 3. Park and any
other split sequence must include every official bag segment before a full-run
result can be called complete.

## Applying the preset

With an existing Rerun viewer listening on port 9876, apply the raw/live entity
layout to one recording:

```bash
/home/yeranis/.local/share/pipx/venvs/rerun-sdk/bin/python \
  src/cbsms/tools/send_newer_college_cbs_off_blueprint.py \
  --recording-id newer_college_<sequence>_cbs_off_<run-id> \
  --sequence maths-easy \
  --duration-sec 80 \
  --connect rerun+http://127.0.0.1:9876/proxy
```

For a canonical post-run recording whose synchronized overlay is under
`/presentation_v2`, save a reusable blueprint file with:

```bash
/home/yeranis/.local/share/pipx/venvs/rerun-sdk/bin/python \
  src/cbsms/tools/send_newer_college_cbs_off_blueprint.py \
  --recording-id newer_college_<sequence>_cbs_off_<run-id> \
  --sequence maths-easy \
  --duration-sec 80 \
  --presentation-root presentation_v2 \
  --output runs/<run>/recovered_blueprints/<sequence>_cbs_off.rbl
```

Change `--sequence` and `--duration-sec` for the real experiment. Do not edit
the blueprint source only to rename a sequence.

## Canonical validated reference

Reference report:

`runs/newer_college_maths_easy_edited_cbs_off_80s_20260821/REPORT.md`

Canonical self-contained Rerun recording:

`runs/newer_college_maths_easy_edited_cbs_off_80s_20260821/rerun_capture/newer_college_maths_easy_80s_canonical_v7.rrd`

SHA256:
`42e3bad60bdbf69625c0d308936d3a80cbcfbb99cd494d040becd408ed16059d`

Canonical standalone blueprint:

`runs/newer_college_maths_easy_edited_cbs_off_80s_20260821/recovered_blueprints/newer_college_maths_easy_80s_canonical_v7.rbl`

SHA256:
`90afae9fc8708b13abbf47de4836ffc618090384e3302b86aeba93447480fde2`

Corrected Kimera factor-graph archive:

`runs/newer_college_maths_easy_edited_cbs_off_80s_20260821/incremental_frame_archive_v7/kimera_factor_graph_80s_compact.rrd`

SHA256:
`76b8bbb616a6ab77a3ff12ebf99dd245f90fe1ce00692dfc77d6b2bb2bf88546`

## Per-run acceptance checklist

- Correct logical sequence and every required split bag are present.
- Correct collection-specific calibration, time offset, Ouster metadata, and
  official ground truth are recorded in the manifest with hashes.
- GLIM, Kimera, video, LiDAR, and GT share the intended sensor-time interval.
- Growing trajectories stop at the playback cursor.
- GLIM scan and trajectory remain physically colocated after the verified
  Base/LiDAR extrinsic.
- Kimera latest graph pose follows published Kimera odometry and its factor
  strips are bounded and smooth.
- Both fixed-lag graphs show only the intended active window plus explicitly
  styled history/context.
- Alignment is rigid SE(3), scale fixed, with no time fitting or deformation.
- All CBS injection flags are false and activity counts remain zero.
- The `.rrd` and `.rbl` pass `rerun rrd verify`; their SHA256 hashes are saved.

## Status

**FROZEN AS THE DEFAULT NEWER COLLEGE CBS-OFF RERUN PRESET.**

The dashboard contract is reusable across all nine sequences. Scientific
completion and accuracy remain sequence-specific and require the checklist
above.

Paper-facing metrics, trajectory figures, hashes, and the cross-sequence table
must be stored using the
[official Newer College results reporting contract](2026-08-21_newer_college_official_results_reporting.md).
