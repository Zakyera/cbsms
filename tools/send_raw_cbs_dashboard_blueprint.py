#!/usr/bin/env python3

import argparse
import sys


def ts_view(rrb, name, origin, contents):
  return rrb.TimeSeriesView(
      origin=origin,
      contents=[*contents, "- /__properties/**"],
      name=name,
  )


def explicit_paths(prefix, names):
  return [f"+ {prefix}/{name}" for name in names]


def cbs_count_paths(prefix):
  return [
      f"+ {prefix}/**",
      f"- {prefix}/message_stamp_sec",
      f"- {prefix}/message_interval_sec",
      f"- {prefix}/edge_duration_mean_sec",
      f"- {prefix}/edge_duration_min_sec",
      f"- {prefix}/edge_duration_max_sec",
      f"- {prefix}/covariance_trace_mean",
      f"- {prefix}/covariance_frobenius_mean",
      f"- {prefix}/latest_source_agent",
      f"- {prefix}/latest_from_stamp_sec",
      f"- {prefix}/latest_to_stamp_sec",
  ]


def build_blueprint(rrb):
  scene = rrb.Spatial3DView(
      origin="/",
      contents=[
          "+ /aligned/**",
          "+ /ground_truth/**",
          "- /aligned/metadata/**",
          "- /__properties/**",
      ],
      name="Scene",
  )

  video = rrb.Spatial2DView(
      origin="/video",
      contents=[
          "+ /video/**",
          "- /__properties/**",
      ],
      name="Video",
  )

  g2k_counts = ts_view(
      rrb,
      "G->K Counts",
      "/cbs/g2k",
      cbs_count_paths("/cbs/g2k"),
  )

  g2k_quality = ts_view(
      rrb,
      "G->K Quality",
      "/cbs/g2k",
      [
          "+ /cbs/g2k/message_interval_sec",
          "+ /cbs/g2k/edge_duration_mean_sec",
          "+ /cbs/g2k/edge_duration_min_sec",
          "+ /cbs/g2k/edge_duration_max_sec",
          "+ /cbs/g2k/covariance_trace_mean",
          "+ /cbs/g2k/covariance_frobenius_mean",
      ],
  )

  k2g_counts = ts_view(
      rrb,
      "K->G Counts",
      "/cbs/k2g",
      cbs_count_paths("/cbs/k2g"),
  )

  k2g_quality = ts_view(
      rrb,
      "K->G Quality",
      "/cbs/k2g",
      [
          "+ /cbs/k2g/message_interval_sec",
          "+ /cbs/k2g/edge_duration_mean_sec",
          "+ /cbs/k2g/edge_duration_min_sec",
          "+ /cbs/k2g/edge_duration_max_sec",
          "+ /cbs/k2g/covariance_trace_mean",
          "+ /cbs/k2g/covariance_frobenius_mean",
      ],
  )

  glim_cbs_updates = ts_view(
      rrb,
      "GLIM CBS Updates",
      "/glim/cbs/beliefs",
      [
          "+ /glim/cbs/beliefs/received_per_update",
          "+ /glim/cbs/beliefs/matched_per_update",
          "+ /glim/cbs/beliefs/added_to_factor_graph_per_update",
          "+ /glim/cbs/beliefs/duplicate_per_update",
          "+ /glim/cbs/beliefs/rejected_per_update",
          "+ /glim/cbs/beliefs/dropped_by_receive_gate_per_update",
          "+ /glim/cbs/beliefs/retried_per_update",
          "+ /glim/cbs/beliefs/duration_mismatch_per_update",
          "+ /glim/cbs/beliefs/published_per_update",
          "+ /glim/cbs/incoming/pending",
          "+ /glim/cbs/incoming/pending_deferred",
      ],
  )

  glim_cbs_totals = ts_view(
      rrb,
      "GLIM CBS Totals",
      "/glim/cbs",
      [
          "+ /glim/cbs/received_total",
          "+ /glim/cbs/matched_total",
          "+ /glim/cbs/injected_total",
          "+ /glim/cbs/duplicate_total",
          "+ /glim/cbs/rejected_total",
          "+ /glim/cbs/dropped_total",
          "+ /glim/cbs/retried_total",
          "+ /glim/cbs/duration_mismatch_total",
          "+ /glim/cbs/outgoing_total",
          "+ /glim/cbs/beliefs/published_total",
          "+ /glim/cbs/incoming/pending",
          "+ /glim/cbs/incoming/pending_deferred",
      ],
  )

  kimera_cbs_updates = ts_view(
      rrb,
      "Kimera CBS Updates",
      "/kimera/cbs/beliefs",
      [
          "+ /kimera/cbs/beliefs/published_per_update",
          "+ /kimera/cbs/beliefs/received_per_update",
          "+ /kimera/cbs/beliefs/dropped_by_receive_gate_per_update",
          "+ /kimera/cbs/beliefs/added_to_factor_graph_per_update",
          "+ /kimera/cbs/beliefs/rejected_first_message_per_update",
          "+ /kimera/cbs/beliefs/rejected_update_status_per_update",
          "+ /kimera/cbs/beliefs/rejected_inactive_window_per_update",
          "+ /kimera/cbs/beliefs/rejected_shape_per_update",
          "+ /kimera/cbs/beliefs/rejected_exception_per_update",
      ],
  )

  kimera_state = ts_view(
      rrb,
      "Kimera Graph State",
      "/kimera",
      [
          "+ /kimera/keyframe_id",
          "+ /kimera/factor_graph/factors_total",
          "+ /kimera/cbs/marginalization_graph/factor_count",
      ],
  )

  glim_runtime_summary = ts_view(
      rrb,
      "GLIM Runtime Summary",
      "/glim/timing",
      [
          "+ /glim/timing/total_ms",
          "+ /glim/timing/cpu_frame_ms",
          "+ /glim/cbs/timing/update_finish_total_ms",
      ],
  )

  glim_runtime_stages = ts_view(
      rrb,
      "GLIM Runtime Stages",
      "/glim/timing",
      [
          "+ /glim/timing/state_lookup_ms",
          "+ /glim/timing/inter_scan_imu_ms",
          "+ /glim/timing/imu_factor_ms",
          "+ /glim/timing/intra_scan_imu_ms",
          "+ /glim/timing/deskew_ms",
          "+ /glim/timing/point_covariance_ms",
          "+ /glim/timing/create_frame_ms",
          "+ /glim/timing/create_factors_ms",
          "+ /glim/timing/pre_smoother_callback_ms",
          "+ /glim/timing/smoother_update_ms",
          "+ /glim/timing/post_smoother_callback_ms",
          "+ /glim/timing/marginalization_ms",
          "+ /glim/timing/update_frames_ms",
          "+ /glim/timing/imu_validation_ms",
          "+ /glim/timing/update_callbacks_ms",
          "+ /glim/cbs/timing/temporary_factor_ms",
          "+ /glim/cbs/timing/publish_odometry_ms",
          "+ /glim/cbs/timing/publish_outgoing_ms",
          "+ /glim/cbs/timing/update_finish_total_ms",
          "+ /glim/cbs/incoming/pending_oldest_age_sec",
      ],
  )

  glim_state = ts_view(
      rrb,
      "GLIM State",
      "/glim/timing",
      [
          "+ /glim/timing/frame_index",
          "+ /glim/timing/point_count",
          "+ /glim/timing/imu_integrated_count",
          "+ /glim/timing/new_factor_count",
          "+ /glim/timing/active_frame_count",
          "+ /glim/timing/marginalized_frame_count",
      ],
  )

  kimera_runtime = ts_view(
      rrb,
      "Kimera Runtime Timing",
      "/kimera/timing",
      [
          "+ /kimera/timing/optimization_ms",
          "+ /kimera/timing/optimize_total_ms",
          "+ /kimera/timing/smoother_update_ms",
          "+ /kimera/timing/compute_state_covariance_ms",
          "+ /kimera/cbs/timing/belief_generation_ms",
          "+ /kimera/cbs/timing/collect_external_beliefs_ms",
          "+ /kimera/cbs/timing/outgoing_total_ms",
          "+ /kimera/cbs/timing/set_marginalization_graph_ms",
          "+ /kimera/cbs/timing/get_odometry_beliefs_ms",
      ],
  )

  glim_posterior_covariance = ts_view(
      rrb,
      "GLIM Posterior Covariance",
      "/glim",
      explicit_paths(
          "/metrics/posterior_covariance/glim",
          [
              "uncertainty_frobenius_norm",
              "sigma_mean_cm",
              "sigma_max_cm",
              "sigma_x_cm",
              "sigma_y_cm",
              "sigma_z_cm",
              "translation_trace_cm2",
              "translation_frobenius_cm2",
          ],
      ),
  )

  kimera_posterior_covariance = ts_view(
      rrb,
      "Kimera Posterior Covariance",
      "/kimera",
      explicit_paths(
          "/metrics/posterior_covariance/kimera",
          [
              "uncertainty_frobenius_norm",
              "sigma_mean_cm",
              "sigma_max_cm",
              "sigma_x_cm",
              "sigma_y_cm",
              "sigma_z_cm",
              "translation_trace_cm2",
              "translation_frobenius_cm2",
          ],
      ),
  )

  relative_covariance = ts_view(
      rrb,
      "Relative Belief Covariance",
      "/metrics/relative_belief_covariance",
      explicit_paths(
          "/metrics/relative_belief_covariance/glim_sent",
          [
              "latest_sigma_x_cm",
              "latest_sigma_y_cm",
              "latest_sigma_z_cm",
              "latest_sigma_mean_cm",
              "latest_sigma_min_cm",
              "latest_sigma_max_cm",
              "latest_translation_trace_cm2",
              "latest_translation_frobenius_cm2",
          ],
      ) + explicit_paths(
          "/metrics/relative_belief_covariance/kimera_sent",
          [
              "latest_sigma_x_cm",
              "latest_sigma_y_cm",
              "latest_sigma_z_cm",
              "latest_sigma_mean_cm",
              "latest_sigma_min_cm",
              "latest_sigma_max_cm",
              "latest_translation_trace_cm2",
              "latest_translation_frobenius_cm2",
          ],
      ),
  )

  accumulated_covariance = ts_view(
      rrb,
      "Accumulated Covariance",
      "/metrics/accumulated_covariance",
      explicit_paths(
          "/metrics/accumulated_covariance/glim_sent",
          [
              "sigma_x_cm",
              "sigma_y_cm",
              "sigma_z_cm",
              "sigma_mean_cm",
              "sigma_min_cm",
              "sigma_max_cm",
              "translation_trace_cm2",
              "translation_frobenius_cm2",
          ],
      ) + explicit_paths(
          "/metrics/accumulated_covariance/kimera_sent",
          [
              "sigma_x_cm",
              "sigma_y_cm",
              "sigma_z_cm",
              "sigma_mean_cm",
              "sigma_min_cm",
              "sigma_max_cm",
              "translation_trace_cm2",
              "translation_frobenius_cm2",
          ],
      ),
  )

  belief_chain_state = ts_view(
      rrb,
      "Belief Chain State",
      "/metrics",
      explicit_paths(
          "/metrics/relative_belief_covariance/glim_sent",
          [
              "latest_from_index",
              "latest_to_index",
              "latest_edge_duration_sec",
          ],
      ) + explicit_paths(
          "/metrics/relative_belief_covariance/kimera_sent",
          [
              "latest_from_index",
              "latest_to_index",
              "latest_edge_duration_sec",
          ],
      ) + explicit_paths(
          "/metrics/accumulated_covariance/glim_sent",
          [
              "initialized",
              "edge_count",
              "chain_duration_sec",
              "latest_to_index",
              "skipped_duplicate_total",
              "skipped_noncontiguous_total",
              "skipped_invalid_total",
          ],
      ) + explicit_paths(
          "/metrics/accumulated_covariance/kimera_sent",
          [
              "initialized",
              "edge_count",
              "chain_duration_sec",
              "latest_to_index",
              "skipped_duplicate_total",
              "skipped_noncontiguous_total",
              "skipped_invalid_total",
          ],
      ),
  )

  alignment_quality = ts_view(
      rrb,
      "Alignment Quality",
      "/aligned/metadata",
      explicit_paths(
          "/aligned/metadata",
          [
              "glim_alignment_pairs",
              "glim_alignment_rmse_m",
              "kimera_alignment_pairs",
              "kimera_alignment_rmse_m",
              "common_alignment_glim_to_gt",
              "common_frame_ready",
              "metric_per_estimator_alignment",
          ],
      ),
  )

  lag_timing = ts_view(
      rrb,
      "Lag Timing",
      "/visualization",
      [
          "+ /visualization/lag/**",
          "+ /visualization/rate/**",
          "+ /aligned/metadata/origin_gt_diff_sec",
      ],
  )

  runtime_group = rrb.Horizontal(
      rrb.Vertical(
          glim_runtime_summary,
          glim_state,
          row_shares=[1.0, 1.0],
          name="GLIM Summary and State",
      ),
      glim_runtime_stages,
      rrb.Vertical(
          kimera_runtime,
          kimera_state,
          row_shares=[1.0, 1.0],
          name="Kimera Timing and State",
      ),
      column_shares=[1.0, 1.35, 1.0],
      name="Runtime",
  )

  covariance_group = rrb.Horizontal(
      rrb.Vertical(
          glim_posterior_covariance,
          kimera_posterior_covariance,
          row_shares=[1.0, 1.0],
          name="Estimator Uncertainty",
      ),
      rrb.Vertical(
          relative_covariance,
          accumulated_covariance,
          row_shares=[1.1, 1.0],
          name="Belief Uncertainty",
      ),
      rrb.Vertical(
          belief_chain_state,
          alignment_quality,
          row_shares=[1.0, 1.0],
          name="Belief State and Alignment",
      ),
      lag_timing,
      column_shares=[1.0, 1.0, 1.0, 1.0],
      name="Covariance and Alignment",
  )

  root = rrb.Vertical(
      rrb.Horizontal(
          scene,
          video,
          column_shares=[1.7, 1.0],
          name="Media",
      ),
      rrb.Horizontal(
          g2k_counts,
          g2k_quality,
          k2g_counts,
          k2g_quality,
          column_shares=[1.0, 1.0, 1.0, 1.0],
          name="Directional Traffic",
      ),
      rrb.Horizontal(
          glim_cbs_updates,
          glim_cbs_totals,
          kimera_cbs_updates,
          column_shares=[1.0, 1.0, 1.0],
          name="Local CBS",
      ),
      runtime_group,
      covariance_group,
      row_shares=[1.7, 1.0, 1.0, 1.2, 1.2],
      name="Raw CBS Dashboard",
  )

  return rrb.Blueprint(
      root,
      rrb.BlueprintPanel(state="collapsed"),
      rrb.SelectionPanel(state="collapsed"),
      rrb.TimePanel(state="expanded", play_state="following", loop_mode="off"),
      auto_layout=False,
      auto_views=False,
  )


def parse_args():
  parser = argparse.ArgumentParser(
      description="Send the Raw CBS dashboard blueprint to a running Rerun viewer.")
  parser.add_argument(
      "--app-id",
      default="cbsms",
      help="Rerun application id to target.")
  parser.add_argument(
      "--url",
      default="rerun+http://127.0.0.1:9876/proxy",
      help="Rerun viewer grpc url.")
  parser.add_argument(
      "--no-make-default",
      action="store_true",
      help="Send as active only, without setting it as the default blueprint.")
  return parser.parse_args()


def main():
  args = parse_args()

  try:
    import rerun.blueprint as rrb
  except ImportError as exc:
    print(
        "Failed to import rerun.blueprint. Run this script with the rerun-sdk Python environment.",
        file=sys.stderr)
    raise SystemExit(2) from exc

  blueprint = build_blueprint(rrb)
  blueprint.connect_grpc(
      application_id=args.app_id,
      url=args.url,
      make_active=True,
      make_default=not args.no_make_default,
  )
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
