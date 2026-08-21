# Canonical GEODE Live Rerun Preset

This is the default visualization setup for interactive GEODE debugging and
experiments. Reuse it instead of constructing a new Rerun layout for each
sequence.

## Preset contract

- Play a bounded **60-second** bag window.
- Use one continuous bag playback and one ROS sensor-time timeline.
- Start Kimera after GLIM publishes its first odometry state so the displayed
  GLIM/Kimera interval overlaps.
- Stream into the already-open Rerun viewer through
  `rerun+http://127.0.0.1:9876/proxy` on the host and
  `rerun+http://172.17.0.1:9876/proxy` from the container.
- Do not start another Rerun viewer.
- Keep visualization passive: it must not change GLIM, Kimera, VGICP, CBS
  factors, covariances, optimization, or DCReg.

## Dashboard

The `Live estimators, factor graphs and trajectories` tab contains:

1. Actual GLIM fixed-lag factor graph.
2. Actual Kimera fixed-lag factor graph, with colored states/factors restricted
   to the active window and a grey historical trajectory for spatial context.
3. Ground truth, GLIM, and Kimera trajectories in one comparison view.
4. Live GLIM PointCloud2 playback.
5. Original left and right camera images.
6. GLIM active-factor composition and state counts.
7. Frozen DCReg condition ratios in their own plot.
8. Frozen DCReg absolute masks in a separate 0/1 plot.

The trajectory comparison waits for at least 100 timestamp-matched poses and
10 metres of motion, estimates one rigid **SE(3) position alignment** for each
estimator, and freezes it. The complete trajectory history is displayed only
after this warm-up, using the immutable transform. This provides a robust
common displayed rotation without letting the trajectory float as later
samples arrive. It uses rotation and translation only: no scale correction,
trajectory deformation, time-offset fitting, or estimator feedback. Real
path-shape and scale errors therefore remain visible.

## CBS modes

The runner supports:

- `off`: GLIM and Kimera run independently; no CBS factors are injected.
- `g2k`: persistent GLIM-to-Kimera factors are enabled; Kimera-to-GLIM
  injection remains disabled. This is the preferred mode for isolating the
  effect of healthy/degraded GLIM beliefs on Kimera.
- `two_way`: persistent bidirectional CBS. Use only when explicitly requested.

Default to `off` for estimator comparisons. Always put the selected mode in
the recording label.

All CBS modes use the fixed 0.20-second time-horizon sender policy. The Kimera
receiver duration gate is enabled with a 0.03-second absolute tolerance and a
sender/receiver duration-ratio interval of `[0.75, 1.25]`. A short adjacent
GLIM increment must never be inserted across a longer Kimera keyframe interval.

## Canonical files

- Runtime runner:
  `stage3a_characterization/live_geode_medium_cbs_off/run_live_synchronized_60s.sh`
- Blueprint sender:
  `stage3a_characterization/live_geode_medium_cbs_off/send_blueprint.py`
- Blueprint definition:
  `src/cbsms/tools/geode_stage3a_rerun.py`
- Saved G-to-K blueprint:
  `stage3a_characterization/experiment_registry/geode/cbs_on/blueprints/geode_cbs_g2k_live60.rbl`
- GEODE CBS-on experiment catalog:
  `stage3a_characterization/experiment_registry/geode/cbs_on/experiments.csv`

## Running the preset

From the workspace root, with the Rerun viewer already open:

```bash
recording_id="geode_<sequence>_live60_<cbs-mode>_<timestamp>"

docker exec -w /workspace/cbs_gtsam4.3 cbsms_ws \
  bash stage3a_characterization/live_geode_medium_cbs_off/run_live_synchronized_60s.sh \
  "$recording_id" <sequence> <off|g2k|two_way>
```

Send or refresh the dashboard in the existing viewer:

```bash
/home/yeranis/.local/share/pipx/venvs/rerun-sdk/bin/python \
  stage3a_characterization/live_geode_medium_cbs_off/send_blueprint.py \
  --recording-id "$recording_id" \
  --connect rerun+http://127.0.0.1:9876/proxy \
  --cbs-label "CBS off"
```

The current runner has explicit dataset mappings for `medium`, `flat_smooth`,
and `offroad7`. Add future GEODE sequences only by adding their official bag
and GT paths to the sequence switch; do not change the visualization contract.

## Quick validation after every run

- All panels use the same sensor-time interval.
- The Kimera graph shows its current fixed-lag states/factors over a grey
  historical trajectory context and follows the latest active pose.
- Each estimator alignment becomes visible after the fixed warm-up and does
  not move afterward.
- No Sim(3) scale alignment is used.
- DCReg ratios and binary masks remain in separate plots.
- With `g2k`, active Kimera CBS factors are visible and missing state-key
  references remain zero.
- With `off`, both CBS injection paths remain disabled.

Last validated interval-matched G-to-K recording:

`geode_offroad7_live60_g2k_finalgraph_20260818_135600`

The saved `.rbl` is the canonical dashboard for future 60-second GEODE G-to-K
experiments. Open it alongside a recording or use `send_blueprint.py` when
streaming into an already-open viewer.
