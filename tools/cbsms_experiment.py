#!/usr/bin/env python3
"""Run CBSMS experiments and generate reproducible reports.

The runner is intentionally host-side: it starts the existing Docker container,
captures roslaunch output, records the estimator odometry topics as CSV, then
parses the resulting artifacts into a compact report.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import datetime as dt
import json
import math
import os
from pathlib import Path
import re
import shutil
import shlex
import signal
import subprocess
import sys
import time
from collections import Counter, defaultdict
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple


DEFAULT_CONTAINER = "cbsms_ws"
DEFAULT_CONTAINER_WORKSPACE = "/workspace/cbs_gtsam4.3"
DEFAULT_CLEAN_GLIM_CONTAINER_WORKSPACE = (
    DEFAULT_CONTAINER_WORKSPACE + "/clean_glim_ws"
)
DEFAULT_BAG_PATH = (
    DEFAULT_CONTAINER_WORKSPACE
    + "/src/datasets/S3E/S3E_Square_1/S3E_Square_1_alpha_ros1.bag"
)
DEFAULT_GT_RELATIVE = "src/datasets/S3E/S3E_Square_1/alpha_gt.txt"
DEFAULT_RERUN_HOST = "rerun+http://172.17.0.1:9876/proxy"
CBS_MODE_PRESETS: Dict[str, Dict[str, str]] = {
    "off": {
        "enable_cbs_bridge": "false",
        "cbs_health_aware_enable": "false",
        "cbs_health_relative_trust_enable": "false",
        "cbs_health_relative_trust_calibrated_enable": "false",
        "glim_cbs_wait_for_receiver_covariance_linearization_enable": "false",
    },
    "raw_cbs": {
        "enable_cbs_bridge": "true",
        "cbs_health_aware_enable": "false",
        "cbs_health_relative_trust_enable": "false",
        "cbs_health_relative_trust_calibrated_enable": "false",
        "glim_cbs_wait_for_receiver_covariance_linearization_enable": "false",
    },
    "health_default": {
        "enable_cbs_bridge": "true",
        "cbs_health_aware_enable": "true",
        "cbs_health_sender_enable": "true",
        "cbs_health_receiver_nis_enable": "true",
        "cbs_health_relative_trust_enable": "false",
        "cbs_health_relative_trust_calibrated_enable": "false",
        "glim_cbs_wait_for_receiver_covariance_linearization_enable": "false",
    },
    "health_reltrust": {
        "enable_cbs_bridge": "true",
        "cbs_health_aware_enable": "true",
        "cbs_health_sender_enable": "true",
        "cbs_health_receiver_nis_enable": "true",
        "cbs_health_relative_trust_enable": "true",
        "cbs_health_relative_trust_calibrated_enable": "false",
        "glim_cbs_wait_for_receiver_covariance_linearization_enable": "false",
    },
    "health_reltrust_calibrated": {
        "enable_cbs_bridge": "true",
        "cbs_health_aware_enable": "true",
        "cbs_health_sender_enable": "true",
        "cbs_health_receiver_nis_enable": "true",
        "cbs_health_relative_trust_enable": "true",
        "cbs_health_relative_trust_calibrated_enable": "true",
        "glim_cbs_wait_for_receiver_covariance_linearization_enable": "false",
    },
    "health_reltrust_calibrated_waitlin": {
        "enable_cbs_bridge": "true",
        "cbs_health_aware_enable": "true",
        "cbs_health_sender_enable": "true",
        "cbs_health_receiver_nis_enable": "true",
        "cbs_health_relative_trust_enable": "true",
        "cbs_health_relative_trust_calibrated_enable": "true",
        "glim_cbs_wait_for_receiver_covariance_linearization_enable": "true",
    },
}
CBS_MODE_PRESET_PROFILES = {
    "glim_kimera",
    "m3dgr_glim_kimera",
}
CORE_REPOS = [
    "cbsms",
    "cbs",
    "liorf",
    "Kimera-VIO",
    "Kimera-VIO-ROS",
    "glim",
    "glim_ros1",
    "gtsam_points",
]
EXPERIMENT_PROFILES: Dict[str, Dict[str, Any]] = {
    "liorf_kimera": {
        "launch_package": "kimera_vio_ros",
        "launch_file": "s3e_alpha_liorf_kimera_experiment.launch",
        "trajectory_topics": {
            "kimera": "/kimera_vio_ros/odometry",
            "liorf": "/liorf/mapping/odometry",
        },
    },
    "glim_kimera": {
        "launch_package": "glim_ros",
        "launch_file": "s3e_alpha_glim_kimera_experiment.launch",
        "trajectory_topics": {
            "kimera": "/kimera_vio_ros/odometry",
            "glim": "/glim/cbs/odometry",
        },
    },
    "glim_only": {
        "launch_package": "glim_ros",
        "launch_file": "boreas_glim_only_experiment.launch",
        "trajectory_topics": {
            "glim": "/glim/cbs/odometry",
        },
    },
    "m3dgr_glim_only": {
        "launch_package": "glim_ros",
        "launch_file": "m3dgr_glim_only_experiment.launch",
        "trajectory_topics": {
            "glim": "/glim/cbs/odometry",
        },
    },
    "m3dgr_glim_kimera": {
        "launch_package": "glim_ros",
        "launch_file": "m3dgr_glim_kimera_experiment.launch",
        "default_bag_path": (
            DEFAULT_CONTAINER_WORKSPACE
            + "/src/datasets/M3DGR/Dynamic01/Dynamic01.bag"
        ),
        "default_gt_relative": "src/datasets/M3DGR/Dynamic01/Dynamic01.txt",
        "trajectory_topics": {
            "kimera": "/kimera_vio_ros/odometry",
            "glim": "/glim/cbs/odometry",
        },
        "launch_args": {
            "shutdown_on_bag_finish": "true",
            "enable_cbs_bridge": "true",
            "use_kimera_rviz": "false",
            "kimera_visualize": "false",
            "rerun_visualizer_enable": "false",
            "rerun_world_alignment_enable": "true",
            "glim_config_path": (
                DEFAULT_CONTAINER_WORKSPACE
                + "/runs/runtime_configs/config_m3dgr_camera_imu_experimental"
            ),
            "kimera_params_folder": (
                DEFAULT_CONTAINER_WORKSPACE
                + "/src/Kimera-VIO/params/M3DGRMonoOriginal"
            ),
            "kimera_body_frame_id": "camera_imu_link",
            "glim_body_frame_id": "camera_imu_link",
            "m3dgr_cbs_body_frame_tf_enable": "false",
            "m3dgr_kimera_T_glim_x": "0.039151162",
            "m3dgr_kimera_T_glim_y": "0.111984435",
            "m3dgr_kimera_T_glim_z": "0.236306953",
            "m3dgr_kimera_T_glim_qx": "0.507715982",
            "m3dgr_kimera_T_glim_qy": "-0.497277044",
            "m3dgr_kimera_T_glim_qz": "0.477055852",
            "m3dgr_kimera_T_glim_qw": "0.517066473",
            "cbs_health_aware_enable": "false",
            "glim_cbs_mode": "active_window_temporary",
            "glim_cbs_outgoing_marginal_source": "direct",
            "kimera_cbs_odom_sender_mode": "time_horizon_window",
            "glim_cbs_odom_sender_mode": "time_horizon_window",
            "glim_cbs_odom_receiver_match_mode": "duration_aware_edge",
            "glim_cbs_odom_max_horizon_pairs_per_update": "25",
            "kimera_cbs_odom_max_horizon_pairs_per_update": "6",
            "cbs_odom_duration_gate_enable": "true",
            "cbs_odom_horizon_sec": "0.20",
            "cbs_odom_horizon_tolerance_sec": "0.06",
            "cbs_use_temporary_cbs_linear_factors": "true",
            "cbs_odom_factor_mode": "active_window_temporary",
            "cbs_odom_covariance_mode": "schur_relative_between",
            "cbs_temporary_linear_already_applied_gate_enable": "true",
            "cbs_temporary_linear_already_applied_metric_threshold": "0.01",
            "cbs_temporary_linear_already_applied_dmu_threshold": "0.001",
            "cbs_temporary_linear_already_applied_cov_rel_threshold": "0.001",
            "glim_cbs_k2g_odom_factor_covariance_scale": "1.0",
            "kimera_cbs_l2k_odom_factor_covariance_scale": "1.0",
            "cbs_enable_soft_reset": "true",
            "cbs_d_reset": "0.1",
        },
    },
    "m3dgr_glim_only_live_rerun": {
        "launch_package": "glim_ros",
        "launch_file": "m3dgr_glim_only_live_rerun.launch",
        "default_bag_path": (
            DEFAULT_CONTAINER_WORKSPACE
            + "/src/datasets/M3DGR/Dynamic01/Dynamic01.bag"
        ),
        "default_gt_relative": "src/datasets/M3DGR/Dynamic01/Dynamic01.txt",
        "ground_truth_launch_arg": "ground_truth_path",
        "trajectory_topics": {
            "glim": "/glim_ros/odom",
        },
        "launch_args": {
            "shutdown_on_bag_finish": "true",
            "enable_cbs_bridge": "false",
            "use_kimera_rviz": "false",
            "kimera_visualize": "false",
            "rerun_visualizer_enable": "true",
            "rerun_world_alignment_enable": "true",
        },
    },
    "m3dgr_mid360_glim_only_live_rerun": {
        "launch_package": "glim_ros",
        "launch_file": "m3dgr_mid360_glim_only_live_rerun.launch",
        "default_bag_path": (
            DEFAULT_CONTAINER_WORKSPACE
            + "/src/datasets/M3DGR/Dynamic01/Dynamic01.bag"
        ),
        "default_gt_relative": "src/datasets/M3DGR/Dynamic01/Dynamic01.txt",
        "ground_truth_launch_arg": "ground_truth_path",
        "trajectory_topics": {
            "glim": "/glim_ros/odom",
        },
        "launch_args": {
            "shutdown_on_bag_finish": "true",
            "enable_cbs_bridge": "false",
            "rerun_visualizer_enable": "true",
            "rerun_world_alignment_enable": "true",
        },
    },
    "m3dgr_mid360_glim_kimera_live_rerun": {
        "launch_package": "glim_ros",
        "launch_file": "m3dgr_mid360_glim_kimera_live_rerun_raw_cbs.launch",
        "default_bag_path": (
            DEFAULT_CONTAINER_WORKSPACE
            + "/src/datasets/M3DGR/Dynamic01/Dynamic01.bag"
        ),
        "default_gt_relative": "src/datasets/M3DGR/Dynamic01/Dynamic01.txt",
        "ground_truth_launch_arg": "ground_truth_path",
        "trajectory_topics": {
            "kimera": "/kimera_vio_ros/odometry",
            "glim": "/glim/cbs/odometry",
        },
        "launch_args": {
            "shutdown_on_bag_finish": "true",
            "enable_cbs_bridge": "true",
            "rerun_visualizer_enable": "true",
            "rerun_world_alignment_enable": "true",
        },
    },
    "m3dgr_clean_glim_original": {
        "launch_package": "glim_ros",
        "launch_file": "m3dgr_clean_glim_only_experiment.launch",
        "default_bag_path": (
            DEFAULT_CONTAINER_WORKSPACE
            + "/src/datasets/M3DGR/Dynamic01/Dynamic01.bag"
        ),
        "default_gt_relative": "src/datasets/M3DGR/Dynamic01/Dynamic01.txt",
        "trajectory_topics": {
            "glim": "/glim_ros/odom",
        },
        "ros_env": "main_with_clean_glim",
        "launch_args": {
            "shutdown_on_bag_finish": "true",
            "enable_cbs_bridge": "false",
            "use_kimera_rviz": "false",
            "kimera_visualize": "false",
            "rerun_visualizer_enable": "false",
            "rerun_world_alignment_enable": "true",
            "glim_config_path": (
                DEFAULT_CONTAINER_WORKSPACE
                + "/runs/runtime_configs/config_m3dgr_avia_original_glim_reference"
            ),
        },
    },
    "m3dgr_kimera_only": {
        "launch_package": "glim_ros",
        "launch_file": "m3dgr_kimera_only_experiment.launch",
        "default_bag_path": (
            DEFAULT_CONTAINER_WORKSPACE
            + "/src/datasets/M3DGR/Dynamic01/Dynamic01.bag"
        ),
        "default_gt_relative": "src/datasets/M3DGR/Dynamic01/Dynamic01.txt",
        "trajectory_topics": {
            "kimera": "/kimera_vio_ros/odometry",
        },
        "launch_args": {
            "shutdown_on_bag_finish": "true",
            "enable_cbs_bridge": "false",
            "use_kimera_rviz": "false",
            "kimera_visualize": "false",
            "rerun_visualizer_enable": "false",
            "rerun_world_alignment_enable": "true",
        },
    },
    "m3dgr_kimera_only_backend0": {
        "launch_package": "glim_ros",
        "launch_file": "m3dgr_kimera_only_experiment.launch",
        "default_bag_path": (
            DEFAULT_CONTAINER_WORKSPACE
            + "/src/datasets/M3DGR/Dynamic01/Dynamic01.bag"
        ),
        "default_gt_relative": "src/datasets/M3DGR/Dynamic01/Dynamic01.txt",
        "trajectory_topics": {
            "kimera": "/kimera_vio_ros/odometry",
        },
        "launch_args": {
            "shutdown_on_bag_finish": "true",
            "enable_cbs_bridge": "false",
            "use_kimera_rviz": "false",
            "kimera_visualize": "false",
            "rerun_visualizer_enable": "false",
            "rerun_world_alignment_enable": "true",
            "kimera_params_folder": (
                DEFAULT_CONTAINER_WORKSPACE
                + "/src/Kimera-VIO/params/M3DGRMonoBackend0"
            ),
        },
    },
    "m3dgr_clean_kimera_original": {
        "launch_package": "kimera_vio_ros",
        "launch_file": "kimera_vio_ros_m3dgr_mono_clean.launch",
        "default_bag_path": (
            DEFAULT_CONTAINER_WORKSPACE
            + "/src/datasets/M3DGR/Dynamic01/Dynamic01.bag"
        ),
        "default_gt_relative": "../src/datasets/M3DGR/Dynamic01/Dynamic01.txt",
        "trajectory_topics": {
            "kimera": "/kimera_vio_ros/odometry",
        },
        "launch_args": {
            "shutdown_on_bag_finish": "true",
            "enable_cbs_bridge": "false",
            "use_kimera_rviz": "true",
            "kimera_visualize": "true",
            "rerun_visualizer_enable": "false",
            "rerun_world_alignment_enable": "false",
        },
    },
    "m3dgr_clean_kimera_backend0": {
        "launch_package": "kimera_vio_ros",
        "launch_file": "kimera_vio_ros_m3dgr_mono_clean.launch",
        "default_bag_path": (
            DEFAULT_CONTAINER_WORKSPACE
            + "/src/datasets/M3DGR/Dynamic01/Dynamic01.bag"
        ),
        "default_gt_relative": "../src/datasets/M3DGR/Dynamic01/Dynamic01.txt",
        "trajectory_topics": {
            "kimera": "/kimera_vio_ros/odometry",
        },
        "launch_args": {
            "shutdown_on_bag_finish": "true",
            "enable_cbs_bridge": "false",
            "use_kimera_rviz": "true",
            "kimera_visualize": "true",
            "rerun_visualizer_enable": "false",
            "rerun_world_alignment_enable": "false",
            "kimera_params_folder": (
                DEFAULT_CONTAINER_WORKSPACE
                + "/clean_kimera_ws/src/Kimera-VIO/params/M3DGRMonoBackend0"
            ),
        },
    },
    "kimera_only": {
        "launch_package": "glim_ros",
        "launch_file": "boreas_kimera_only_experiment.launch",
        "trajectory_topics": {
            "kimera": "/kimera_vio_ros/odometry",
        },
    },
    "glim_kimera_live_rerun": {
        "launch_package": "glim_ros",
        "launch_file": "s3e_alpha_glim_kimera_live_rerun.launch",
        "trajectory_topics": {
            "kimera": "/kimera_vio_ros/odometry",
            "glim": "/glim/cbs/odometry",
        },
        "launch_args": {
            "shutdown_on_bag_finish": "false",
            "enable_cbs_bridge": "true",
            "use_kimera_rviz": "false",
            "kimera_visualize": "false",
            "rerun_visualizer_enable": "true",
            "rerun_world_alignment_enable": "true",
        },
    },
}
ANSI_RE = re.compile(
    r"\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~]|\][^\a]*(?:\a|\x1b\\))"
)
ROW_MARKERS = (
    "CBS_TRANSPORT_ROW_L2K",
    "CBS_TRANSPORT_ROW_K2L",
    "CBS_ROUNDTRIP_ROW_L2K",
    "CBS_ROUNDTRIP_ROW_K2L",
    "CBS_MERGE_ROW_L2K",
    "CBS_MERGE_ROW_K2L",
    "CBS_BPSAM_ADD_ROW_L2K",
    "CBS_BPSAM_ADD_ROW_K2L",
    "CBS_OUTGOING_FILTER_ROW",
    "CBS_RECEIVER_DIAGNOSTIC_ROW",
    "CBS_TEMPORARY_LINEAR_ACCOUNTING_ROW",
    "CBS_PREINJECTION_RESIDUAL_ROW",
    "CBS_BELIEF_ODOM_ROW",
    "CBS_ODOM_OUTGOING_ROW",
    "CBS_ODOM_RELATIVE_COVARIANCE_ROW",
    "CBS_ODOM_MATCH_ROW_L2K",
    "CBS_ODOM_MATCH_ROW_K2L",
    "CBS_ODOM_MATCH_ROW_G2K",
    "CBS_ODOM_MATCH_ROW_K2G",
    "CBS_ODOM_RETRY_ROW_L2K",
    "CBS_ODOM_RETRY_ROW_K2L",
    "CBS_ODOM_RETRY_ROW_G2K",
    "CBS_ODOM_RETRY_ROW_K2G",
    "CBS_BPSAM_ODOM_ADD_ROW_L2K",
    "CBS_BPSAM_ODOM_ADD_ROW_K2L",
    "CBS_BPSAM_ODOM_ADD_ROW_G2K",
    "CBS_BPSAM_ODOM_ADD_ROW_K2G",
    "CBS_ODOM_PREINJECTION_RESIDUAL_ROW",
    "CBS_ODOM_TEMPORARY_POSTSOLVE_RESIDUAL_ROW",
    "CBS_ODOM_FACTOR_COVARIANCE_ROW",
    "CBS_HEALTH_AWARE_SENDER_ROW",
    "CBS_HEALTH_AWARE_NIS_ROW",
    "GLIM_CBS_RECEIVER_COVARIANCE_ROW",
    "CBS_TEMPORARY_LINEARIZATION_RESIDUAL_ROW",
    "CBS_KIMERA_OUTGOING_PROVENANCE_ROW",
    "CBS_MARGINALIZATION_GRAPH_ROW",
    "GLIM_CBS_ODOM_INJECT_ROW",
    "GLIM_CBS_ACTIVE_FACTOR_DIAGNOSTIC_ROW",
    "GLIM_CBS_ACTIVE_FACTOR_DETAIL_ROW",
    "KIMERA_CBS_ACTIVE_FACTOR_DIAGNOSTIC_ROW",
    "GLIM_POSE_STAGE_ROW",
    "GLIM_TARGET_UPDATE_ROW",
    "GLIM_ROS_INPUT_TIMING_ROW",
    "GLIM_ASYNC_ODOM_TIMING_ROW",
    "GLIM_ODOM_IMU_TIMING_ROW",
    "GLIM_GPU_TIMING_ROW",
    "GLIM_CBS_TIMING_ROW",
    "GLIM_SCAN_HEALTH_ROW",
    "KIMERA_BACKEND_SPINONCE_TIMING_ROW",
    "KIMERA_CBS_OUTGOING_TIMING_ROW",
    "KIMERA_OPTIMIZE_TIMING_ROW",
    "KIMERA_BACKEND_CALLBACK_TIMING_ROW",
    "KIMERA_RERUN_CALLBACK_TIMING_ROW",
)
VALID_BPSAM_STATUSES = {
    "accepted",
    "accepted_but_skipped_already_applied",
    "rejected_shape",
    "rejected_first_message",
    "rejected_update_status",
    "rejected_inactive_window",
    "rejected_exception",
    "unknown",
}
BPSAM_UPDATE_NUMBER_RE = re.compile(
    r"[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?"
)
MERGE_STATUSES = {
    "bpsam_added",
    "bpsam_accepted_but_skipped_already_applied",
    "bpsam_rejected_first_message",
    "bpsam_rejected_update_status",
    "bpsam_rejected_shape",
    "bpsam_rejected_inactive_window",
    "bpsam_rejected_exception",
    "bpsam_rejected_root_size",
    "bpsam_rejected",
    "dropped_no_local_timestamp",
    "dropped_invalid_stamp",
    "dropped_timestamp_mismatch",
    "dropped_window",
    "dropped_missing_state",
    "dropped_bad_covariance",
}
OUTGOING_FILTER_STATUSES = {
    "sent_filter_disabled",
    "sent_first_message",
    "sent_metric_error",
    "sent_changed",
    "filtered_similar",
}
TEMPORARY_LINEAR_ACCOUNTING_ACTIONS = {
    "temporary_linear_applied",
    "temporary_linear_odom_applied",
    "accepted_but_skipped_already_applied",
    "applied_incremental",
}
PREINJECTION_RESIDUAL_ACTIONS = {
    "temporary_linear_odom_applied",
    "temporary_odom_factor_applied",
    "persistent_odom_applied",
    "active_window_temporary_odom_applied",
    "active_window_temporary_graph_queued",
    "temporary_linear_queued",
}
RECEIVER_METRIC_STATUSES = {
    "ok",
    "no_previous",
    "no_peer",
}


Point = Tuple[float, float, float]
Trajectory = List[Tuple[float, Point]]
PoseRow = Dict[str, float]
PoseTrajectory = List[PoseRow]


def workspace_root_from_script() -> Path:
    return Path(__file__).resolve().parents[3]


def iso_now() -> str:
    return dt.datetime.now(dt.timezone.utc).astimezone().isoformat(timespec="seconds")


def slugify(text: str) -> str:
    text = text.strip().lower()
    text = re.sub(r"[^a-z0-9_.-]+", "-", text)
    text = text.strip("-")
    return text or "run"


def run_command(
    args: Sequence[str],
    cwd: Optional[Path] = None,
    env: Optional[Dict[str, str]] = None,
) -> subprocess.CompletedProcess:
    return subprocess.run(
        list(args),
        cwd=str(cwd) if cwd else None,
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )


def git_info(repo: Path) -> Dict[str, Any]:
    def git(*args: str) -> str:
        result = run_command(["git", "-C", str(repo), *args])
        return result.stdout.strip()

    if not (repo / ".git").exists():
        return {"present": False}

    status = git("status", "--short")
    return {
        "present": True,
        "path": str(repo),
        "branch": git("branch", "--show-current"),
        "commit": git("rev-parse", "HEAD"),
        "commit_short": git("rev-parse", "--short", "HEAD"),
        "remote": git("remote", "get-url", "origin"),
        "dirty": bool(status),
        "status_short": status.splitlines(),
    }


def write_json(path: Path, payload: Dict[str, Any]) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")


def container_path(workspace: Path, host_path: Path, container_workspace: str) -> str:
    host_path = host_path.resolve()
    workspace = workspace.resolve()
    try:
        rel = host_path.relative_to(workspace)
    except ValueError:
        raise ValueError(f"{host_path} is not under workspace {workspace}")
    return container_workspace.rstrip("/") + "/" + str(rel).replace(os.sep, "/")


def docker_bash(container: str, command: str) -> List[str]:
    return ["docker", "exec", container, "bash", "-lc", command]


def ros_env_command(
    command: str,
    container_workspace: str = "/workspace",
    ros_env: str = "default",
) -> str:
    quoted_workspace = shlex.quote(container_workspace)
    if ros_env == "main_with_clean_glim":
        clean_ws = shlex.quote(DEFAULT_CLEAN_GLIM_CONTAINER_WORKSPACE)
        main_ws = quoted_workspace
        return (
            "source /opt/ros/noetic/setup.bash && "
            f"cd {main_ws} && "
            "source devel/setup.bash && "
            f"source {clean_ws}/devel/setup.bash && "
            "export ROS_PACKAGE_PATH="
            f"{clean_ws}/src/glim_ros1:"
            f"{clean_ws}/devel/share:"
            f"{main_ws}/src:"
            f"{main_ws}/devel/share:"
            "/opt/ros/noetic/share:${ROS_PACKAGE_PATH} && "
            "export LD_LIBRARY_PATH="
            f"{clean_ws}/devel/lib:"
            f"{main_ws}/devel/lib:"
            "/opt/ros/noetic/lib:${LD_LIBRARY_PATH} && "
            "export PYTHONPATH="
            f"{main_ws}/devel/lib/python3/dist-packages:"
            "/opt/ros/noetic/lib/python3/dist-packages:${PYTHONPATH} && "
            f"{command}"
        )
    return (
        "source /opt/ros/noetic/setup.bash && "
        f"cd {quoted_workspace} && "
        "source devel/setup.bash && "
        f"{command}"
    )


def list_container_processes(container: str) -> List[Tuple[int, str]]:
    result = run_command(
        ["docker", "exec", container, "ps", "-eo", "pid=", "-o", "cmd="]
    )
    processes: List[Tuple[int, str]] = []
    if result.returncode != 0:
        return processes
    for line in result.stdout.splitlines():
        stripped = line.strip()
        if not stripped:
            continue
        pid_text, _, command = stripped.partition(" ")
        pid = to_int(pid_text)
        if pid > 0:
            processes.append((pid, command.strip()))
    return processes


def interrupt_container_processes(container: str, needles: Sequence[str]) -> None:
    pids = [
        str(pid)
        for pid, command in list_container_processes(container)
        if any(needle in command for needle in needles)
    ]
    if pids:
        run_command(["docker", "exec", container, "kill", "-INT", *pids])


def stop_existing_experiment(container: str) -> None:
    interrupt_container_processes(
        container,
        [
            "s3e_alpha_liorf_kimera_experiment.launch",
            "s3e_alpha_liorf_kimera_topics.launch",
            "s3e_alpha_glim_kimera_experiment.launch",
            "s3e_alpha_glim_kimera_live_rerun.launch",
            "m3dgr_glim_only_experiment.launch",
            "m3dgr_glim_kimera_experiment.launch",
            "m3dgr_glim_only_live_rerun.launch",
            "m3dgr_mid360_glim_only_live_rerun.launch",
            "m3dgr_mid360_glim_kimera_live_rerun_raw_cbs.launch",
            "m3dgr_clean_glim_only_experiment.launch",
            "m3dgr_kimera_only_experiment.launch",
            "boreas_glim_kimera_experiment.launch",
            "rerun_topic_visualizer_node",
            "rostopic echo -p /kimera_vio_ros/odometry",
            "rostopic echo -p /liorf/mapping/odometry",
            "rostopic echo -p /glim/cbs/odometry",
            "rostopic echo -p /glim_ros/odom",
        ],
    )


def wait_for_ros_master(
    container: str,
    timeout_sec: float,
    container_workspace: str,
    ros_env: str = "default",
) -> bool:
    deadline = time.time() + timeout_sec
    probe = ros_env_command(
        "rostopic list >/dev/null 2>&1",
        container_workspace,
        ros_env,
    )
    while time.time() < deadline:
        result = run_command(docker_bash(container, probe))
        if result.returncode == 0:
            return True
        time.sleep(0.5)
    return False


def start_rostopic_csv(
    container: str,
    topic: str,
    output_container_path: str,
    stderr_path: Path,
    container_workspace: str,
    ros_env: str = "default",
) -> subprocess.Popen:
    command = ros_env_command(
        f"rostopic echo -p {shlex.quote(topic)} > {shlex.quote(output_container_path)}",
        container_workspace,
        ros_env,
    )
    err = stderr_path.open("w", encoding="utf-8")
    return subprocess.Popen(
        docker_bash(container, command),
        stdout=subprocess.DEVNULL,
        stderr=err,
        text=True,
    )


def terminate_process(proc: subprocess.Popen, timeout_sec: float = 5.0) -> None:
    if proc.poll() is not None:
        return
    proc.terminate()
    try:
        proc.wait(timeout=timeout_sec)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait(timeout=timeout_sec)


def selected_profile(args: argparse.Namespace) -> Dict[str, Any]:
    profile = EXPERIMENT_PROFILES.get(args.experiment_profile)
    if profile is None:
        raise ValueError(f"unknown experiment profile: {args.experiment_profile}")
    return profile


def parse_trajectory_topic_specs(
    profile_topics: Dict[str, str],
    specs: Sequence[str],
) -> Dict[str, str]:
    topics = dict(profile_topics)
    for spec in specs:
        if "=" not in spec:
            raise ValueError(f"trajectory topic must look like label=/topic: {spec}")
        label, topic = spec.split("=", 1)
        label = slugify(label).replace("-", "_")
        topic = topic.strip()
        if not label or not topic.startswith("/"):
            raise ValueError(f"trajectory topic must look like label=/topic: {spec}")
        topics[label] = topic
    return topics


def odometry_csv_name(label: str) -> str:
    return label if label.endswith("_odometry") else f"{label}_odometry"


def odometry_estimator_name(path: Path) -> str:
    stem = path.stem
    suffix = "_odometry"
    return stem[: -len(suffix)] if stem.endswith(suffix) else stem


def apply_cbs_mode_preset(
    launch_args: Dict[str, str],
    preset_name: Optional[str],
) -> None:
    if not preset_name:
        return
    preset = CBS_MODE_PRESETS.get(preset_name)
    if preset is None:
        raise ValueError(f"unknown CBS mode preset: {preset_name}")
    launch_args.update(preset)


def run_experiment(args: argparse.Namespace) -> Path:
    profile = selected_profile(args)
    if args.cbs_mode_preset and args.experiment_profile not in CBS_MODE_PRESET_PROFILES:
        supported = ", ".join(sorted(CBS_MODE_PRESET_PROFILES))
        raise ValueError(
            "CBS mode presets are currently supported only for profiles: "
            f"{supported}"
        )
    ros_env = str(profile.get("ros_env", "default"))
    launch_package = args.launch_package or str(profile["launch_package"])
    launch_file = args.launch_file or str(profile["launch_file"])
    trajectory_topics = parse_trajectory_topic_specs(
        profile["trajectory_topics"], args.trajectory_topic
    )

    workspace = args.workspace.resolve()
    runs_root = args.output_root.resolve() if args.output_root else workspace / "runs"
    stamp = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
    run_id = f"{stamp}_{slugify(args.name)}" if args.name else stamp
    run_dir = runs_root / run_id
    traj_dir = run_dir / "trajectories"
    run_dir.mkdir(parents=True, exist_ok=False)
    traj_dir.mkdir(parents=True, exist_ok=True)

    git_repos = {
        name: git_info(workspace / "src" / name)
        for name in CORE_REPOS
    }

    default_gt_relative = str(profile.get("default_gt_relative", DEFAULT_GT_RELATIVE))
    gt_path = args.gt_path.resolve() if args.gt_path else workspace / default_gt_relative
    bag_path = args.bag_path
    if bag_path == DEFAULT_BAG_PATH and "default_bag_path" in profile:
        bag_path = str(profile["default_bag_path"])
    launch_args = {
        "bag_path": bag_path,
        "bag_duration": str(args.duration),
        "shutdown_on_bag_finish": str(args.shutdown_on_bag_finish).lower(),
        "enable_cbs_bridge": str(args.enable_cbs_bridge).lower(),
        "use_kimera_rviz": str(args.use_kimera_rviz).lower(),
        "kimera_visualize": str(args.kimera_visualize).lower(),
        "rerun_visualizer_enable": str(args.rerun_visualizer_enable).lower(),
        "rerun_world_alignment_enable": str(args.rerun_world_alignment_enable).lower(),
        "rerun_host": args.rerun_host,
    }
    if args.experiment_profile == "liorf_kimera":
        launch_args["use_liorf_rviz"] = str(args.use_liorf_rviz).lower()
    launch_args.update(
        {key: str(value) for key, value in profile.get("launch_args", {}).items()}
    )
    apply_cbs_mode_preset(launch_args, args.cbs_mode_preset)
    if "ground_truth_launch_arg" in profile:
        launch_gt_key = str(profile["ground_truth_launch_arg"])
        try:
            launch_args[launch_gt_key] = container_path(
                workspace, gt_path, args.container_workspace
            )
        except ValueError:
            launch_args[launch_gt_key] = str(gt_path)
    for item in args.extra_arg:
        if ":=" not in item:
            raise ValueError(f"extra launch arg must look like key:=value: {item}")
        key, value = item.split(":=", 1)
        launch_args[key] = value

    launch_target = (
        f"{launch_package} {launch_file}".strip()
        if launch_package
        else launch_file
    )

    manifest: Dict[str, Any] = {
        "run_id": run_id,
        "created_at": iso_now(),
        "experiment_profile": args.experiment_profile,
        "workspace": str(workspace),
        "container_workspace": args.container_workspace,
        "container": args.container,
        "ros_env": ros_env,
        "launch_file": launch_target,
        "launch_args": launch_args,
        "cbs_mode_preset": args.cbs_mode_preset,
        "duration_sec": args.duration,
        "timeout_padding_sec": args.timeout_padding,
        "ground_truth": str(gt_path),
        "git": git_repos,
        "recorded_topics": {},
    }
    write_json(run_dir / "manifest.json", manifest)

    if args.clean_start:
        stop_existing_experiment(args.container)
        time.sleep(1.0)

    launch_arg_text = " ".join(
        f"{key}:={shlex.quote(value)}" for key, value in launch_args.items()
    )
    if launch_package:
        roslaunch_target = (
            f"{shlex.quote(launch_package)} {shlex.quote(launch_file)}"
        )
    else:
        roslaunch_target = shlex.quote(launch_file)
    launch_command = ros_env_command(
        f"roslaunch {roslaunch_target} " + launch_arg_text,
        args.container_workspace,
        ros_env,
    )
    roslaunch_log = run_dir / "roslaunch.log"
    with roslaunch_log.open("w", encoding="utf-8", errors="replace") as log_file:
        proc = subprocess.Popen(
            docker_bash(args.container, launch_command),
            stdout=log_file,
            stderr=subprocess.STDOUT,
            text=True,
        )

        topic_procs: List[subprocess.Popen] = []
        if args.record_trajectories and wait_for_ros_master(
            args.container, 30.0, args.container_workspace, ros_env
        ):
            for label, topic in trajectory_topics.items():
                csv_path = traj_dir / f"{odometry_csv_name(label)}.csv"
                err_path = traj_dir / f"{label}.stderr.log"
                topic_procs.append(
                    start_rostopic_csv(
                        args.container,
                        topic,
                        container_path(workspace, csv_path, args.container_workspace),
                        err_path,
                        args.container_workspace,
                        ros_env,
                    )
                )
                manifest["recorded_topics"][label] = {
                    "topic": topic,
                    "csv": str(csv_path),
                }
            write_json(run_dir / "manifest.json", manifest)

        deadline = time.time() + args.duration + args.timeout_padding
        while proc.poll() is None and time.time() < deadline:
            time.sleep(1.0)

        if proc.poll() is None:
            stop_existing_experiment(args.container)
            try:
                proc.wait(timeout=30.0)
            except subprocess.TimeoutExpired:
                proc.send_signal(signal.SIGINT)
                try:
                    proc.wait(timeout=10.0)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait(timeout=5.0)

        for topic_proc in topic_procs:
            terminate_process(topic_proc)

    manifest["finished_at"] = iso_now()
    manifest["roslaunch_returncode"] = proc.returncode
    write_json(run_dir / "manifest.json", manifest)

    generate_report(run_dir, gt_path)
    return run_dir


def strip_ansi(text: str) -> str:
    return ANSI_RE.sub("", text)


def parse_csv_rows_from_line(line: str, marker: str) -> List[List[str]]:
    rows: List[List[str]] = []
    start = 0
    while True:
        idx = line.find(marker, start)
        if idx < 0:
            break
        next_idx = min(
            (
                pos
                for row_marker in ROW_MARKERS
                if (pos := line.find(row_marker, idx + len(marker))) >= 0
            ),
            default=len(line),
        )
        payload = line[idx:next_idx].strip()
        try:
            rows.append(next(csv.reader([payload])))
        except csv.Error:
            pass
        start = idx + len(marker)
    return rows


def to_float(value: str) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def timestamp_to_seconds(value: float) -> float:
    if math.isfinite(value) and value > 1.0e12:
        return value * 1.0e-9
    return value


def to_int(value: str) -> int:
    try:
        return int(float(value))
    except (TypeError, ValueError):
        return 0


def parse_vector_token(token: str) -> List[float]:
    if not token.startswith("v[") or not token.endswith("]"):
        return []
    return [to_float(part) for part in token[2:-1].split(";")]


def parse_matrix_token(token: str) -> List[List[float]]:
    if not token.startswith("m[") or not token.endswith("]"):
        return []
    rows: List[List[float]] = []
    for row in token[2:-1].split("|"):
        rows.append([to_float(part) for part in row.split(";")])
    return rows


def matrix_trace(matrix: List[List[float]]) -> float:
    if not matrix:
        return math.nan
    n = min(len(matrix), min((len(row) for row in matrix), default=0))
    return sum(matrix[i][i] for i in range(n))


def matrix_frobenius(matrix: List[List[float]]) -> float:
    if not matrix:
        return math.nan
    total = 0.0
    for row in matrix:
        for value in row:
            if math.isfinite(value):
                total += value * value
    return math.sqrt(total)


def matrix_diag_min(matrix: List[List[float]]) -> float:
    if not matrix:
        return math.nan
    n = min(len(matrix), min((len(row) for row in matrix), default=0))
    vals = [matrix[i][i] for i in range(n) if math.isfinite(matrix[i][i])]
    return min(vals) if vals else math.nan


def matrix_diag_max(matrix: List[List[float]]) -> float:
    if not matrix:
        return math.nan
    n = min(len(matrix), min((len(row) for row in matrix), default=0))
    vals = [matrix[i][i] for i in range(n) if math.isfinite(matrix[i][i])]
    return max(vals) if vals else math.nan


def numeric_stats(values: Iterable[float]) -> Dict[str, Any]:
    clean = [value for value in values if math.isfinite(value)]
    if not clean:
        return {"count": 0, "mean": math.nan, "min": math.nan, "max": math.nan, "sum": 0.0}
    return {
        "count": len(clean),
        "mean": sum(clean) / len(clean),
        "min": min(clean),
        "max": max(clean),
        "sum": sum(clean),
    }


def percentile(values: Iterable[float], q: float) -> float:
    clean = sorted(value for value in values if math.isfinite(value))
    if not clean:
        return math.nan
    if len(clean) == 1:
        return clean[0]
    rank = (len(clean) - 1) * q
    low = int(math.floor(rank))
    high = int(math.ceil(rank))
    if low == high:
        return clean[low]
    return clean[low] * (high - rank) + clean[high] * (rank - low)


def parse_log_artifacts(log_paths: Sequence[Path]) -> Dict[str, Any]:
    transport_rows: List[Dict[str, Any]] = []
    roundtrip_rows: List[Dict[str, Any]] = []
    merge_rows: List[Dict[str, Any]] = []
    bpsam_rows: List[Dict[str, Any]] = []
    outgoing_filter_rows: List[Dict[str, Any]] = []
    receiver_diagnostic_rows: List[Dict[str, Any]] = []
    temporary_linear_accounting_rows: List[Dict[str, Any]] = []
    preinjection_residual_rows: List[Dict[str, Any]] = []
    belief_odom_rows: List[Dict[str, Any]] = []
    odom_outgoing_rows: List[Dict[str, Any]] = []
    odom_relative_covariance_rows: List[Dict[str, Any]] = []
    odom_match_rows: List[Dict[str, Any]] = []
    odom_retry_rows: List[Dict[str, Any]] = []
    bpsam_odom_add_rows: List[Dict[str, Any]] = []
    glim_odom_inject_rows: List[Dict[str, Any]] = []
    odom_factor_covariance_rows: List[Dict[str, Any]] = []
    health_sender_rows: List[Dict[str, Any]] = []
    health_nis_rows: List[Dict[str, Any]] = []
    glim_receiver_covariance_rows: List[Dict[str, Any]] = []
    glim_active_factor_diagnostic_rows: List[Dict[str, Any]] = []
    glim_active_factor_detail_rows: List[Dict[str, Any]] = []
    kimera_active_factor_diagnostic_rows: List[Dict[str, Any]] = []
    glim_pose_stage_rows: List[Dict[str, Any]] = []
    glim_target_update_rows: List[Dict[str, Any]] = []
    odom_temporary_postsolve_residual_rows: List[Dict[str, Any]] = []
    temporary_linearization_residual_rows: List[Dict[str, Any]] = []
    provenance_rows: List[Dict[str, Any]] = []
    marginalization_graph_rows: List[Dict[str, Any]] = []
    timing_rows: List[Dict[str, Any]] = []
    glim_timing_rows: List[Dict[str, Any]] = []
    glim_scan_health_rows: List[Dict[str, Any]] = []
    kimera_flow_rows: List[Dict[str, int]] = []
    kimera_odom_flow_rows: List[Dict[str, int]] = []
    liorf_odom_flow_rows: List[Dict[str, int]] = []
    glim_odom_flow_rows: List[Dict[str, int]] = []
    alignment_rows: List[Dict[str, float]] = []
    skipped_rows: Counter[str] = Counter()

    kimera_flow_re = re.compile(
        r"Kimera CBS incoming flow: pending=(?P<pending>\d+) "
        r"resolved=(?P<resolved>\d+) selected=(?P<selected>\d+) "
        r"bpsam_added=(?P<bpsam_added>\d+) rejected=(?P<rejected>\d+) "
        r"rejected_by\(window=(?P<window>\d+),timestamp=(?P<timestamp>\d+),"
        r"state=(?P<state>\d+),covariance=(?P<covariance>\d+),"
        r"bpsam=(?P<bpsam>\d+),bpsam_first_message=(?P<bpsam_first_message>\d+),"
        r"bpsam_update_status=(?P<bpsam_update_status>\d+),"
        r"bpsam_inactive_window=(?P<bpsam_inactive_window>\d+),"
        r"bpsam_shape=(?P<bpsam_shape>\d+),"
        r"bpsam_exception=(?P<bpsam_exception>\d+)"
    )
    kimera_odom_flow_re = re.compile(
        r"Kimera CBS incoming odometry flow: pending_odom=(?P<pending_odom>\d+) "
        r"resolved=(?P<resolved>\d+) bpsam_added=(?P<bpsam_added>\d+) "
        r"rejected=(?P<rejected>\d+)(?: retried=(?P<retried>\d+))? "
        r"rejected_by\(window=(?P<window>\d+),timestamp=(?P<timestamp>\d+),"
        r"state=(?P<state>\d+),covariance=(?P<covariance>\d+),"
        r"bpsam=(?P<bpsam>\d+),bpsam_first_message=(?P<bpsam_first_message>\d+),"
        r"bpsam_update_status=(?P<bpsam_update_status>\d+),"
        r"bpsam_inactive_window=(?P<bpsam_inactive_window>\d+),"
        r"bpsam_shape=(?P<bpsam_shape>\d+),"
        r"bpsam_exception=(?P<bpsam_exception>\d+)"
    )
    liorf_odom_flow_re = re.compile(
        r"LiORF CBS incoming odometry flow: dequeued=(?P<dequeued>\d+) "
        r"matched=(?P<matched>\d+) dropped=(?P<dropped>\d+)"
        r"(?: retried=(?P<retried>\d+))? "
        r"bpsam\(added=(?P<added>\d+),rejected=(?P<rejected>\d+),"
        r"inactive_window=(?P<inactive_window>\d+),shape=(?P<shape>\d+),"
        r"exception=(?P<exception>\d+)\)"
    )
    glim_odom_flow_re = re.compile(
        r"GLIM CBS incoming flow: matched=(?P<matched>\d+) "
        r"injected=(?P<injected>\d+) duplicate=(?P<duplicate>\d+) "
        r"rejected=(?P<rejected>\d+)(?: deferred=(?P<deferred>\d+))? "
        r"pending=(?P<pending>\d+)(?: pending_deferred=(?P<pending_deferred>\d+))?"
    )
    alignment_re = re.compile(
        r"Kimera Rerun world alignment initialized .* dt=(?P<dt_ms>[-+0-9.eE]+) ms"
    )

    for path in log_paths:
        if not path.exists():
            continue
        with path.open("r", encoding="utf-8", errors="replace") as stream:
            for raw_line in stream:
                line = strip_ansi(raw_line)

                flow_match = kimera_flow_re.search(line)
                if flow_match:
                    kimera_flow_rows.append(
                        {key: int(value) for key, value in flow_match.groupdict().items()}
                    )
                kimera_odom_flow_match = kimera_odom_flow_re.search(line)
                if kimera_odom_flow_match:
                    kimera_odom_flow_rows.append(
                        {
                            key: int(value) if value is not None else 0
                            for key, value in kimera_odom_flow_match.groupdict().items()
                        }
                    )
                liorf_odom_flow_match = liorf_odom_flow_re.search(line)
                if liorf_odom_flow_match:
                    liorf_odom_flow_rows.append(
                        {
                            key: int(value) if value is not None else 0
                            for key, value in liorf_odom_flow_match.groupdict().items()
                        }
                    )
                glim_odom_flow_match = glim_odom_flow_re.search(line)
                if glim_odom_flow_match:
                    glim_odom_flow_rows.append(
                        {
                            key: int(value) if value is not None else 0
                            for key, value in glim_odom_flow_match.groupdict().items()
                        }
                    )

                alignment_match = alignment_re.search(line)
                if alignment_match:
                    alignment_rows.append({"dt_ms": to_float(alignment_match.group("dt_ms"))})

                for marker in ROW_MARKERS:
                    if marker not in line:
                        continue
                    for row in parse_csv_rows_from_line(line, marker):
                        if not row or row[0] != marker:
                            skipped_rows[f"{marker}:malformed_marker"] += 1
                            continue
                        if marker.startswith("CBS_TRANSPORT_ROW") and (
                            len(row) < 17 or row[16] != "ok"
                        ):
                            skipped_rows[f"{marker}:malformed_transport"] += 1
                            continue
                        if marker.startswith("CBS_ROUNDTRIP_ROW") and (
                            len(row) < 6 or row[5] != "ok"
                        ):
                            skipped_rows[f"{marker}:malformed_roundtrip"] += 1
                            continue
                        if marker.startswith("CBS_BPSAM_ADD_ROW") and (
                            len(row) < 14 or row[3] not in VALID_BPSAM_STATUSES
                        ):
                            skipped_rows[f"{marker}:malformed_bpsam"] += 1
                            continue
                        if marker == "CBS_OUTGOING_FILTER_ROW" and (
                            len(row) < 14 or row[13] not in OUTGOING_FILTER_STATUSES
                        ):
                            skipped_rows[f"{marker}:malformed_outgoing_filter"] += 1
                            continue
                        if marker == "CBS_RECEIVER_DIAGNOSTIC_ROW" and (
                            len(row) < 25 or row[5] not in VALID_BPSAM_STATUSES
                        ):
                            skipped_rows[f"{marker}:malformed_receiver_diagnostic"] += 1
                            continue
                        if marker == "CBS_TEMPORARY_LINEAR_ACCOUNTING_ROW" and (
                            len(row) < 18 or row[5] not in TEMPORARY_LINEAR_ACCOUNTING_ACTIONS
                        ):
                            skipped_rows[f"{marker}:malformed_temporary_linear_accounting"] += 1
                            continue
                        if marker == "CBS_PREINJECTION_RESIDUAL_ROW" and (
                            len(row) < 22 or row[5] not in PREINJECTION_RESIDUAL_ACTIONS
                        ):
                            skipped_rows[f"{marker}:malformed_preinjection_residual"] += 1
                            continue
                        if marker == "CBS_BELIEF_ODOM_ROW" and len(row) < 10:
                            skipped_rows[f"{marker}:malformed_belief_odom"] += 1
                            continue
                        if marker == "CBS_ODOM_OUTGOING_ROW" and len(row) < 10:
                            skipped_rows[f"{marker}:malformed_odom_outgoing"] += 1
                            continue
                        if marker == "CBS_ODOM_RELATIVE_COVARIANCE_ROW" and len(row) < 15:
                            skipped_rows[f"{marker}:malformed_odom_relative_covariance"] += 1
                            continue
                        if marker.startswith("CBS_ODOM_MATCH_ROW") and len(row) < 13:
                            skipped_rows[f"{marker}:malformed_odom_match"] += 1
                            continue
                        if marker.startswith("CBS_ODOM_RETRY_ROW") and len(row) < 8:
                            skipped_rows[f"{marker}:malformed_odom_retry"] += 1
                            continue
                        if marker.startswith("CBS_BPSAM_ODOM_ADD_ROW") and len(row) < 6:
                            skipped_rows[f"{marker}:malformed_bpsam_odom_add"] += 1
                            continue
                        if marker == "CBS_ODOM_PREINJECTION_RESIDUAL_ROW" and (
                            len(row) < 14 or row[6] not in PREINJECTION_RESIDUAL_ACTIONS
                        ):
                            skipped_rows[f"{marker}:malformed_odom_preinjection_residual"] += 1
                            continue
                        if marker == "CBS_ODOM_TEMPORARY_POSTSOLVE_RESIDUAL_ROW" and (
                            len(row) < 14 or row[6] not in PREINJECTION_RESIDUAL_ACTIONS
                        ):
                            skipped_rows[
                                f"{marker}:malformed_odom_temporary_postsolve_residual"
                            ] += 1
                            continue
                        if marker == "CBS_ODOM_FACTOR_COVARIANCE_ROW" and len(row) < 21:
                            skipped_rows[f"{marker}:malformed_odom_factor_covariance"] += 1
                            continue
                        if marker == "CBS_HEALTH_AWARE_SENDER_ROW" and len(row) < 17:
                            skipped_rows[f"{marker}:malformed_health_sender"] += 1
                            continue
                        if marker == "CBS_HEALTH_AWARE_NIS_ROW" and len(row) < 18:
                            skipped_rows[f"{marker}:malformed_health_nis"] += 1
                            continue
                        if marker == "CBS_TEMPORARY_LINEARIZATION_RESIDUAL_ROW" and (
                            len(row) < 23 or row[5] not in PREINJECTION_RESIDUAL_ACTIONS
                        ):
                            skipped_rows[
                                f"{marker}:malformed_temporary_linearization_residual"
                            ] += 1
                            continue
                        if marker == "CBS_MARGINALIZATION_GRAPH_ROW" and len(row) < 12:
                            skipped_rows[f"{marker}:malformed_marginalization_graph"] += 1
                            continue
                        if marker == "GLIM_CBS_ODOM_INJECT_ROW" and len(row) < 7:
                            skipped_rows[f"{marker}:malformed_glim_odom_inject"] += 1
                            continue
                        if marker == "GLIM_CBS_ACTIVE_FACTOR_DIAGNOSTIC_ROW" and len(row) < 42:
                            skipped_rows[
                                f"{marker}:malformed_glim_active_factor_diagnostic"
                            ] += 1
                            continue
                        if marker == "GLIM_CBS_ACTIVE_FACTOR_DETAIL_ROW" and len(row) < 20:
                            skipped_rows[
                                f"{marker}:malformed_glim_active_factor_detail"
                            ] += 1
                            continue
                        if marker == "KIMERA_CBS_ACTIVE_FACTOR_DIAGNOSTIC_ROW" and len(row) < 42:
                            skipped_rows[
                                f"{marker}:malformed_kimera_active_factor_diagnostic"
                            ] += 1
                            continue
                        if marker == "GLIM_POSE_STAGE_ROW" and len(row) < 26:
                            skipped_rows[f"{marker}:malformed_glim_pose_stage"] += 1
                            continue
                        if marker == "GLIM_TARGET_UPDATE_ROW" and len(row) < 21:
                            skipped_rows[f"{marker}:malformed_glim_target_update"] += 1
                            continue
                        if marker == "GLIM_SCAN_HEALTH_ROW" and len(row) < 28:
                            skipped_rows[f"{marker}:malformed_glim_scan_health"] += 1
                            continue
                        if marker.startswith("GLIM_") and marker.endswith("_TIMING_ROW"):
                            timing = parse_glim_timing_row(row)
                            if timing:
                                glim_timing_rows.append(timing)
                            else:
                                skipped_rows[f"{marker}:malformed_glim_timing"] += 1
                            continue

                        if marker.startswith("CBS_TRANSPORT_ROW"):
                            direction = row[0].replace("CBS_TRANSPORT_ROW_", "")
                            sender_cov = parse_matrix_token(row[10])
                            received_cov = parse_matrix_token(row[11])
                            reconstructed_cov = parse_matrix_token(row[12])
                            transport_rows.append(
                                {
                                    "direction": direction,
                                    "key": row[1],
                                    "sender_timestamp_ns": row[2],
                                    "sender_frame": row[3],
                                    "receiver_expected_frame": row[4],
                                    "transform_label": row[5],
                                    "sender_mu": row[6],
                                    "received_mu": row[7],
                                    "reconstructed_mu": row[8],
                                    "mean_error_norm": to_float(row[9]),
                                    "sender_cov_trace": matrix_trace(sender_cov),
                                    "sender_cov_frobenius": matrix_frobenius(sender_cov),
                                    "sender_cov_diag_min": matrix_diag_min(sender_cov),
                                    "sender_cov_diag_max": matrix_diag_max(sender_cov),
                                    "received_cov_trace": matrix_trace(received_cov),
                                    "received_cov_frobenius": matrix_frobenius(received_cov),
                                    "received_cov_diag_min": matrix_diag_min(received_cov),
                                    "received_cov_diag_max": matrix_diag_max(received_cov),
                                    "reconstructed_cov_trace": matrix_trace(reconstructed_cov),
                                    "cov_error_fro": to_float(row[13]),
                                    "cov_symmetry_error": to_float(row[14]),
                                    "received_cov_min_eigenvalue": to_float(row[15]),
                                    "status": row[16],
                                }
                            )
                        elif marker.startswith("CBS_ROUNDTRIP_ROW"):
                            roundtrip_rows.append(
                                {
                                    "direction": row[0].replace("CBS_ROUNDTRIP_ROW_", ""),
                                    "key": row[1],
                                    "sender_timestamp_ns": row[2],
                                    "mean_roundtrip_error": to_float(row[3]),
                                    "cov_roundtrip_error": to_float(row[4]),
                                    "status": row[5],
                                }
                            )
                        elif marker.startswith("CBS_MERGE_ROW") and len(row) >= 24:
                            if row[15] not in MERGE_STATUSES:
                                skipped_rows[f"{marker}:malformed_merge"] += 1
                                continue
                            merge_rows.append(
                                {
                                    "direction": row[0].replace("CBS_MERGE_ROW_", ""),
                                    "sample_idx": row[1],
                                    "sender_frame": row[2],
                                    "sender_timestamp_ns": row[3],
                                    "sender_key": row[4],
                                    "receiver_key": row[5],
                                    "sent_trace": to_float(row[6]),
                                    "received_trace": to_float(row[7]),
                                    "receiver_local_before_trace": to_float(row[8]),
                                    "receiver_merged_trace": to_float(row[9]),
                                    "receiver_posterior_pre_trace": to_float(row[10]),
                                    "receiver_posterior_post_trace": to_float(row[11]),
                                    "hellinger_local_incoming": to_float(row[12]),
                                    "hellinger_local_merged": to_float(row[13]),
                                    "mahal_local_incoming": to_float(row[14]),
                                    "status": row[15],
                                    "receiver_local_source": row[16],
                                    "receiver_local_raw_trace": to_float(row[17]),
                                    "receiver_local_anchored_trace": to_float(row[18]),
                                    "dmu_local_incoming": to_float(row[19]),
                                    "dmu_local_merged": to_float(row[20]),
                                    "step": to_float(row[21]),
                                    "dxycurr": to_float(row[22]),
                                    "dxy_target": to_float(row[23]),
                                }
                            )
                        elif marker.startswith("CBS_MERGE_ROW"):
                            skipped_rows[f"{marker}:malformed_merge"] += 1
                        elif marker.startswith("CBS_BPSAM_ADD_ROW"):
                            bpsam_rows.append(
                                {
                                    "direction": row[0].replace("CBS_BPSAM_ADD_ROW_", ""),
                                    "sender_key": row[1],
                                    "receiver_key": row[2],
                                    "status": row[3],
                                    "enable_soft_reset": row[4],
                                    "metric_type": row[5],
                                    "d_reset": to_float(row[6]),
                                    "contract_alpha": to_float(row[7]),
                                    "existing_gbp_var": row[8],
                                    "is_first_message": row[9],
                                    "dxycurr": to_float(row[10]),
                                    "dxy": to_float(row[11]),
                                    "contraction_step_size": to_float(row[12]),
                                    "message": row[13],
                                }
                            )
                        elif marker == "CBS_OUTGOING_FILTER_ROW":
                            outgoing_filter_rows.append(
                                {
                                    "direction": row[1],
                                    "sender_robot": row[2],
                                    "receiver_robot": row[3],
                                    "producing_agent": row[4],
                                    "belief_key": row[5],
                                    "metric_type": row[6],
                                    "threshold": to_float(row[7]),
                                    "has_last": row[8],
                                    "metric_distance": to_float(row[9]),
                                    "last_trace": to_float(row[10]),
                                    "current_trace": to_float(row[11]),
                                    "dmu": to_float(row[12]),
                                    "status": row[13],
                                }
                            )
                        elif marker == "CBS_RECEIVER_DIAGNOSTIC_ROW":
                            if len(row) >= 40:
                                raw_gate_enabled = row[12]
                                raw_gate_checked = row[13]
                                raw_gate_rejected = row[14]
                                temporary_linear_gate_enabled = row[15]
                                temporary_linear_gate_checked = row[16]
                                accepted_but_skipped_already_applied = row[17]
                                repeated_impulse_prevented = row[18]
                                applied_incremental = row[19]
                                has_last_applied = row[20]
                                last_applied_metric_status = row[21]
                                last_applied_metric_distance = to_float(row[22])
                                last_applied_dmu = to_float(row[23])
                                last_applied_trace = to_float(row[24])
                                last_applied_cov_rel_frobenius = to_float(row[25])
                                has_gbp_peer = row[26]
                                gbp_peer_metric_status = row[27]
                                gbp_peer_metric_distance = to_float(row[28])
                                gbp_peer_dmu = to_float(row[29])
                                gbp_peer_trace = to_float(row[30])
                                incoming_trace = to_float(row[31])
                                d_reset = to_float(row[32])
                                enable_soft_reset = row[33]
                                gbp_update_soft_reset_effective = row[34]
                                contract_alpha = to_float(row[35])
                                is_first_message = row[36]
                                existing_gbp_var = row[37]
                                dxycurr = to_float(row[38])
                                contraction_step_size = to_float(row[39])
                            elif len(row) >= 29:
                                raw_gate_enabled = row[12]
                                raw_gate_checked = row[13]
                                raw_gate_rejected = row[14]
                                temporary_linear_gate_enabled = "false"
                                temporary_linear_gate_checked = "false"
                                accepted_but_skipped_already_applied = "false"
                                repeated_impulse_prevented = "false"
                                applied_incremental = "false"
                                has_last_applied = "false"
                                last_applied_metric_status = "no_previous"
                                last_applied_metric_distance = math.nan
                                last_applied_dmu = math.nan
                                last_applied_trace = math.nan
                                last_applied_cov_rel_frobenius = math.nan
                                has_gbp_peer = row[15]
                                gbp_peer_metric_status = row[16]
                                gbp_peer_metric_distance = to_float(row[17])
                                gbp_peer_dmu = to_float(row[18])
                                gbp_peer_trace = to_float(row[19])
                                incoming_trace = to_float(row[20])
                                d_reset = to_float(row[21])
                                enable_soft_reset = row[22]
                                gbp_update_soft_reset_effective = row[23]
                                contract_alpha = to_float(row[24])
                                is_first_message = row[25]
                                existing_gbp_var = row[26]
                                dxycurr = to_float(row[27])
                                contraction_step_size = to_float(row[28])
                            else:
                                raw_gate_enabled = "false"
                                raw_gate_checked = "false"
                                raw_gate_rejected = "false"
                                temporary_linear_gate_enabled = "false"
                                temporary_linear_gate_checked = "false"
                                accepted_but_skipped_already_applied = "false"
                                repeated_impulse_prevented = "false"
                                applied_incremental = "false"
                                has_last_applied = "false"
                                last_applied_metric_status = "no_previous"
                                last_applied_metric_distance = math.nan
                                last_applied_dmu = math.nan
                                last_applied_trace = math.nan
                                last_applied_cov_rel_frobenius = math.nan
                                has_gbp_peer = row[12]
                                gbp_peer_metric_status = row[13]
                                gbp_peer_metric_distance = to_float(row[14])
                                gbp_peer_dmu = to_float(row[15])
                                gbp_peer_trace = to_float(row[16])
                                incoming_trace = to_float(row[17])
                                d_reset = to_float(row[18])
                                enable_soft_reset = row[19]
                                gbp_update_soft_reset_effective = row[19]
                                contract_alpha = to_float(row[20])
                                is_first_message = row[21]
                                existing_gbp_var = row[22]
                                dxycurr = to_float(row[23])
                                contraction_step_size = to_float(row[24])
                            receiver_diagnostic_rows.append(
                                {
                                    "direction": row[1],
                                    "receiver_robot": row[2],
                                    "source_agent": row[3],
                                    "belief_key": row[4],
                                    "status": row[5],
                                    "metric_type": row[6],
                                    "has_raw_previous": row[7],
                                    "raw_previous_metric_status": row[8],
                                    "raw_previous_metric_distance": to_float(row[9]),
                                    "raw_previous_dmu": to_float(row[10]),
                                    "raw_previous_trace": to_float(row[11]),
                                    "raw_previous_gate_enabled": raw_gate_enabled,
                                    "raw_previous_gate_checked": raw_gate_checked,
                                    "raw_previous_gate_rejected": raw_gate_rejected,
                                    "temporary_linear_gate_enabled": temporary_linear_gate_enabled,
                                    "temporary_linear_gate_checked": temporary_linear_gate_checked,
                                    "accepted_but_skipped_already_applied": accepted_but_skipped_already_applied,
                                    "repeated_impulse_prevented": repeated_impulse_prevented,
                                    "applied_incremental": applied_incremental,
                                    "has_last_applied": has_last_applied,
                                    "last_applied_metric_status": last_applied_metric_status,
                                    "last_applied_metric_distance": last_applied_metric_distance,
                                    "last_applied_dmu": last_applied_dmu,
                                    "last_applied_trace": last_applied_trace,
                                    "last_applied_cov_rel_frobenius": last_applied_cov_rel_frobenius,
                                    "has_gbp_peer": has_gbp_peer,
                                    "gbp_peer_metric_status": gbp_peer_metric_status,
                                    "gbp_peer_metric_distance": gbp_peer_metric_distance,
                                    "gbp_peer_dmu": gbp_peer_dmu,
                                    "gbp_peer_trace": gbp_peer_trace,
                                    "incoming_trace": incoming_trace,
                                    "d_reset": d_reset,
                                    "enable_soft_reset": enable_soft_reset,
                                    "gbp_update_soft_reset_effective": gbp_update_soft_reset_effective,
                                    "contract_alpha": contract_alpha,
                                    "is_first_message": is_first_message,
                                    "existing_gbp_var": existing_gbp_var,
                                    "dxycurr": dxycurr,
                                    "contraction_step_size": contraction_step_size,
                                }
                            )
                        elif marker == "CBS_TEMPORARY_LINEAR_ACCOUNTING_ROW":
                            temporary_linear_accounting_rows.append(
                                {
                                    "direction": row[1],
                                    "receiver_robot": row[2],
                                    "source_agent": row[3],
                                    "belief_key": row[4],
                                    "action": row[5],
                                    "repeated_impulse_prevented": row[6],
                                    "applied_incremental": row[7],
                                    "has_last_applied": row[8],
                                    "last_applied_metric_status": row[9],
                                    "last_applied_metric_distance": to_float(row[10]),
                                    "last_applied_dmu": to_float(row[11]),
                                    "last_applied_trace": to_float(row[12]),
                                    "incoming_trace": to_float(row[13]),
                                    "last_applied_cov_rel_frobenius": to_float(row[14]),
                                    "metric_threshold": to_float(row[15]),
                                    "dmu_threshold": to_float(row[16]),
                                    "cov_rel_threshold": to_float(row[17]),
                                }
                            )
                        elif marker == "CBS_PREINJECTION_RESIDUAL_ROW":
                            preinjection_residual_rows.append(
                                {
                                    "direction": row[1],
                                    "receiver_robot": row[2],
                                    "source_agent": row[3],
                                    "belief_key": row[4],
                                    "action": row[5],
                                    "receiver_pose_source": row[6],
                                    "residual_norm": to_float(row[7]),
                                    "rot_norm": to_float(row[8]),
                                    "trans_norm": to_float(row[9]),
                                    "dx": to_float(row[10]),
                                    "dy": to_float(row[11]),
                                    "dz": to_float(row[12]),
                                    "yaw_error_rad": to_float(row[13]),
                                    "yaw_error_deg": to_float(row[14]),
                                    "incoming_trace": to_float(row[15]),
                                    "receiver_x": to_float(row[16]),
                                    "receiver_y": to_float(row[17]),
                                    "receiver_z": to_float(row[18]),
                                    "incoming_x": to_float(row[19]),
                                    "incoming_y": to_float(row[20]),
                                    "incoming_z": to_float(row[21]),
                                }
                            )
                        elif marker == "CBS_BELIEF_ODOM_ROW":
                            belief_odom_rows.append(
                                {
                                    "direction": row[1],
                                    "receiver_robot": row[2],
                                    "source_agent": row[3],
                                    "from_key": row[4],
                                    "to_key": row[5],
                                    "status": row[6],
                                    "from_trace": to_float(row[7]),
                                    "to_trace": to_float(row[8]),
                                    "odom_trace": to_float(row[9]),
                                }
                            )
                        elif marker == "CBS_ODOM_OUTGOING_ROW":
                            odom_outgoing_rows.append(
                                {
                                    "direction": row[1],
                                    "sender_robot": row[2],
                                    "receiver_robot": row[3],
                                    "from_key": row[4],
                                    "to_key": row[5],
                                    "from_trace": to_float(row[6]),
                                    "to_trace": to_float(row[7]),
                                    "odom_trace": to_float(row[8]),
                                    "status": row[9],
                                }
                            )
                        elif marker == "CBS_ODOM_RELATIVE_COVARIANCE_ROW":
                            parsed_row = {
                                "direction": row[1],
                                "sender_robot": row[2],
                                "receiver_robot": row[3],
                                "from_key": row[4],
                                "to_key": row[5],
                                "mode": row[6],
                                "trace_old_conditional": to_float(row[7]),
                                "trace_new_schur_relative": to_float(row[8]),
                                "ratio_new_over_old": to_float(row[9]),
                            }
                            if len(row) >= 26:
                                parsed_row.update(
                                    {
                                        "eig_old_min": to_float(row[10]),
                                        "eig_old_max": to_float(row[11]),
                                        "eig_new_min": to_float(row[12]),
                                        "eig_new_max": to_float(row[13]),
                                        "rotation_trace_old": to_float(row[14]),
                                        "translation_trace_old": to_float(row[15]),
                                        "rotation_trace_new": to_float(row[16]),
                                        "translation_trace_new": to_float(row[17]),
                                        "H_to_rank": to_int(row[18]),
                                        "H_to_condition_estimate": to_float(row[19]),
                                        "schur_vs_direct_difference_norm": to_float(row[20]),
                                        "jitter_used": to_int(row[21]),
                                        "jitter_added": to_float(row[22]),
                                        "lambda_rel_eigenvalues": row[23],
                                        "sigma_rel_eigenvalues": row[24],
                                        "status": row[25],
                                    }
                                )
                            else:
                                parsed_row.update(
                                    {
                                        "lambda_rel_eigenvalues": row[10],
                                        "sigma_rel_eigenvalues": row[11],
                                        "jitter_used": to_int(row[12]),
                                        "jitter_added": to_float(row[13]),
                                        "status": row[14],
                                    }
                                )
                            odom_relative_covariance_rows.append(parsed_row)
                        elif marker.startswith("CBS_ODOM_MATCH_ROW"):
                            parsed_row = {
                                "direction": row[0].replace("CBS_ODOM_MATCH_ROW_", ""),
                                "sender_edge": row[1],
                                "from_stamp_sec": to_float(row[2]),
                                "to_stamp_sec": to_float(row[3]),
                                "receiver_edge": row[4],
                                "from_best_stamp_sec": to_float(row[5]),
                                "to_best_stamp_sec": to_float(row[6]),
                                "from_abs_dt": to_float(row[7]),
                                "to_abs_dt": to_float(row[8]),
                                "tolerance_sec": to_float(row[9]),
                                "from_reason": row[10],
                                "to_reason": row[11],
                                "decision": row[12],
                            }
                            if len(row) >= 17:
                                parsed_row.update(
                                    {
                                        "sender_dt": to_float(row[13]),
                                        "receiver_dt": to_float(row[14]),
                                        "duration_error": to_float(row[15]),
                                        "duration_ratio": to_float(row[16]),
                                    }
                                )
                            odom_match_rows.append(parsed_row)
                        elif marker.startswith("CBS_ODOM_RETRY_ROW"):
                            odom_retry_rows.append(
                                {
                                    "direction": row[0].replace("CBS_ODOM_RETRY_ROW_", ""),
                                    "sender_edge": row[1],
                                    "from_stamp_sec": to_float(row[2]),
                                    "to_stamp_sec": to_float(row[3]),
                                    "age_sec": to_float(row[4]),
                                    "max_age_sec": to_float(row[5]),
                                    "reason": row[6],
                                    "decision": row[7],
                                }
                            )
                        elif marker.startswith("CBS_BPSAM_ODOM_ADD_ROW"):
                            bpsam_odom_add_rows.append(
                                {
                                    "direction": row[0].replace("CBS_BPSAM_ODOM_ADD_ROW_", ""),
                                    "sender_edge": row[1],
                                    "receiver_edge": row[2],
                                    "status_code": row[3],
                                    "covariance_trace": to_float(row[4]),
                                    "message": row[5],
                                }
                            )
                        elif marker == "CBS_ODOM_PREINJECTION_RESIDUAL_ROW":
                            parsed_row = {
                                "direction": row[1],
                                "receiver_robot": row[2],
                                "source_agent": row[3],
                                "belief_key": f"{row[4]}->{row[5]}",
                                "from_key": row[4],
                                "to_key": row[5],
                                "action": row[6],
                                "receiver_pose_source": row[7],
                                "residual_norm": to_float(row[8]),
                                "rot_norm": to_float(row[9]),
                                "trans_norm": to_float(row[10]),
                                "yaw_error_rad": to_float(row[11]),
                                "yaw_error_deg": to_float(row[12]),
                                "incoming_trace": to_float(row[13]),
                            }
                            if len(row) >= 16:
                                parsed_row.update(
                                    {
                                        "whitened_norm": to_float(row[14]),
                                        "nis": to_float(row[15]),
                                    }
                                )
                            preinjection_residual_rows.append(parsed_row)
                        elif marker == "CBS_ODOM_TEMPORARY_POSTSOLVE_RESIDUAL_ROW":
                            odom_temporary_postsolve_residual_rows.append(
                                {
                                    "direction": row[1],
                                    "receiver_robot": row[2],
                                    "source_agent": row[3],
                                    "belief_key": f"{row[4]}->{row[5]}",
                                    "from_key": row[4],
                                    "to_key": row[5],
                                    "action": row[6],
                                    "receiver_pose_source": row[7],
                                    "residual_norm": to_float(row[8]),
                                    "rot_norm": to_float(row[9]),
                                    "trans_norm": to_float(row[10]),
                                    "yaw_error_rad": to_float(row[11]),
                                    "yaw_error_deg": to_float(row[12]),
                                    "incoming_trace": to_float(row[13]),
                                }
                            )
                        elif marker == "CBS_ODOM_FACTOR_COVARIANCE_ROW":
                            odom_factor_covariance_rows.append(
                                {
                                    "direction": row[1],
                                    "receiver_robot": row[2],
                                    "source_agent": row[3],
                                    "from_key": row[4],
                                    "to_key": row[5],
                                    "belief_key": f"{row[4]}->{row[5]}",
                                    "action": row[6],
                                    "trace": to_float(row[7]),
                                    "rot_trace": to_float(row[8]),
                                    "trans_trace": to_float(row[9]),
                                    "diag0": to_float(row[10]),
                                    "diag1": to_float(row[11]),
                                    "diag2": to_float(row[12]),
                                    "diag3": to_float(row[13]),
                                    "diag4": to_float(row[14]),
                                    "diag5": to_float(row[15]),
                                    "min_eigenvalue": to_float(row[16]),
                                    "max_eigenvalue": to_float(row[17]),
                                    "condition": to_float(row[18]),
                                    "asym_frobenius": to_float(row[19]),
                                    "asym_relative": to_float(row[20]),
                                }
                            )
                        elif marker == "CBS_HEALTH_AWARE_SENDER_ROW":
                            health_sender_rows.append(
                                {
                                    "direction": row[1],
                                    "sender_agent": row[2],
                                    "receiver_agent": row[3],
                                    "from_key": row[4],
                                    "to_key": row[5],
                                    "belief_key": f"{row[4]}->{row[5]}",
                                    "enabled": to_int(row[6]),
                                    "status": row[7],
                                    "raw_rel_trace": to_float(row[8]),
                                    "abs_trace": to_float(row[9]),
                                    "u": to_float(row[10]),
                                    "u0": to_float(row[11]),
                                    "g_det": to_float(row[12]),
                                    "g_tr": to_float(row[13]),
                                    "alpha_health": to_float(row[14]),
                                    "floor_trace": to_float(row[15]),
                                    "final_trace": to_float(row[16]),
                                }
                            )
                        elif marker == "CBS_HEALTH_AWARE_NIS_ROW":
                            health_nis_rows.append(
                                {
                                    "direction": row[1],
                                    "receiver_agent": row[2],
                                    "source_agent": row[3],
                                    "from_key": row[4],
                                    "to_key": row[5],
                                    "belief_key": f"{row[4]}->{row[5]}",
                                    "enabled": to_int(row[6]),
                                    "status": row[7],
                                    "residual_norm": to_float(row[8]),
                                    "rot_norm": to_float(row[9]),
                                    "trans_norm": to_float(row[10]),
                                    "sender_trace": to_float(row[11]),
                                    "receiver_trace": to_float(row[12]),
                                    "s_trace": to_float(row[13]),
                                    "nu": to_float(row[14]),
                                    "chi2": to_float(row[15]),
                                    "alpha_cons": to_float(row[16]),
                                    "final_trace": to_float(row[17]),
                                    "raw_alpha_cons": to_float(row[18]) if len(row) > 18 else math.nan,
                                    "relative_trust_enabled": to_int(row[19]) if len(row) > 19 else 0,
                                    "relative_trust_status": row[20] if len(row) > 20 else "legacy",
                                    "sender_uncertainty": to_float(row[21]) if len(row) > 21 else math.nan,
                                    "receiver_uncertainty": to_float(row[22]) if len(row) > 22 else math.nan,
                                    "trust_balance": to_float(row[23]) if len(row) > 23 else math.nan,
                                    "trust_weight": to_float(row[24]) if len(row) > 24 else math.nan,
                                    "relative_trust_calibrated": to_int(row[25]) if len(row) > 25 else 0,
                                    "sender_uncertainty_baseline": to_float(row[26]) if len(row) > 26 else math.nan,
                                    "receiver_uncertainty_baseline": to_float(row[27]) if len(row) > 27 else math.nan,
                                    "sender_uncertainty_growth": to_float(row[28]) if len(row) > 28 else math.nan,
                                    "receiver_uncertainty_growth": to_float(row[29]) if len(row) > 29 else math.nan,
                                }
                            )
                        elif marker == "GLIM_CBS_RECEIVER_COVARIANCE_ROW":
                            glim_receiver_covariance_rows.append(
                                {
                                    "direction": row[1],
                                    "receiver_agent": row[2],
                                    "source_agent": row[3],
                                    "from_key": row[4],
                                    "to_key": row[5],
                                    "belief_key": f"{row[4]}->{row[5]}",
                                    "from_in_new_values": to_int(row[6]),
                                    "to_in_new_values": to_int(row[7]),
                                    "from_in_linearization": to_int(row[8]),
                                    "to_in_linearization": to_int(row[9]),
                                    "joint_info_attempted": to_int(row[10]),
                                    "joint_info_ok": to_int(row[11]),
                                    "conditional_covariance_ok": to_int(row[12]),
                                    "status": row[13],
                                }
                            )
                        elif marker == "CBS_TEMPORARY_LINEARIZATION_RESIDUAL_ROW":
                            temporary_linearization_residual_rows.append(
                                {
                                    "direction": row[1],
                                    "receiver_robot": row[2],
                                    "source_agent": row[3],
                                    "belief_key": row[4],
                                    "action": row[5],
                                    "linearization_source": row[6],
                                    "linearized_key": row[7],
                                    "residual_norm": to_float(row[8]),
                                    "rot_norm": to_float(row[9]),
                                    "trans_norm": to_float(row[10]),
                                    "dx": to_float(row[11]),
                                    "dy": to_float(row[12]),
                                    "dz": to_float(row[13]),
                                    "yaw_error_rad": to_float(row[14]),
                                    "yaw_error_deg": to_float(row[15]),
                                    "incoming_trace": to_float(row[16]),
                                    "receiver_x": to_float(row[17]),
                                    "receiver_y": to_float(row[18]),
                                    "receiver_z": to_float(row[19]),
                                    "incoming_x": to_float(row[20]),
                                    "incoming_y": to_float(row[21]),
                                    "incoming_z": to_float(row[22]),
                                }
                            )
                        elif marker == "CBS_KIMERA_OUTGOING_PROVENANCE_ROW" and len(row) >= 8:
                            provenance_rows.append(
                                {
                                    "key": row[1],
                                    "timestamp_ns": row[2],
                                    "source": row[3],
                                    "has_local_marginal": row[4],
                                    "has_bpsam": row[5],
                                    "is_external": row[6],
                                    "frame_semantic": row[7],
                                    "cov_semantic": row[8] if len(row) > 8 else "",
                                }
                            )
                        elif marker == "CBS_KIMERA_OUTGOING_PROVENANCE_ROW":
                            skipped_rows[f"{marker}:malformed_provenance"] += 1
                        elif marker == "CBS_MARGINALIZATION_GRAPH_ROW":
                            robot = row[1]
                            direction = (
                                "K2L"
                                if robot == "k"
                                else "L2K"
                                if robot == "l"
                                else "G2K"
                                if robot == "g"
                                else f"{robot.upper()}2?"
                            )
                            marginalization_graph_rows.append(
                                {
                                    "direction": direction,
                                    "robot": robot,
                                    "type": row[2],
                                    "active_filter": row[3] == "true",
                                    "active_key_count": to_float(row[4]),
                                    "input_factor_slots": to_float(row[5]),
                                    "kept_factor_count": to_float(row[6]),
                                    "removed_total": to_float(row[7]),
                                    "removed_outside_active": to_float(row[8]),
                                    "removed_tracked_cbs_factor": to_float(row[9]),
                                    "removed_anchor_belief": to_float(row[10]),
                                    "tmp_marginals": row[11] == "true",
                                }
                            )
                        elif marker == "GLIM_CBS_ODOM_INJECT_ROW":
                            glim_odom_inject_rows.append(
                                {
                                    "direction": "K2G",
                                    "sender_edge": row[1],
                                    "receiver_edge": row[2],
                                    "from_abs_dt": to_float(row[3]),
                                    "to_abs_dt": to_float(row[4]),
                                    "covariance_trace": to_float(row[5]),
                                    "status": row[6],
                                }
                            )
                        elif marker == "GLIM_CBS_ACTIVE_FACTOR_DIAGNOSTIC_ROW":
                            glim_active_factor_diagnostic_rows.append(
                                {
                                    "direction": row[1],
                                    "receiver_robot": row[2],
                                    "source_agent": row[3],
                                    "local_from_key": row[4],
                                    "local_to_key": row[5],
                                    "source_edge": row[6],
                                    "factor_index": row[7],
                                    "active_update_count": to_int(row[8]),
                                    "active_age_sec": to_float(row[9]),
                                    "from_abs_dt": to_float(row[10]),
                                    "to_abs_dt": to_float(row[11]),
                                    "covariance_trace": to_float(row[12]),
                                    "covariance_rotation_trace": to_float(row[13]),
                                    "covariance_translation_trace": to_float(row[14]),
                                    "cbs_error": to_float(row[15]),
                                    "residual_norm": to_float(row[16]),
                                    "rot_norm": to_float(row[17]),
                                    "trans_norm": to_float(row[18]),
                                    "whitened_norm": to_float(row[19]),
                                    "nis": to_float(row[20]),
                                    "cbs_hessian_frobenius": to_float(row[21]),
                                    "same_edge_count": to_int(row[22]),
                                    "same_edge_error_count": to_int(row[23]),
                                    "same_edge_error_sum": to_float(row[24]),
                                    "same_edge_hessian_count": to_int(row[25]),
                                    "same_edge_hessian_sum": to_float(row[26]),
                                    "unary_count": to_int(row[27]),
                                    "unary_error_count": to_int(row[28]),
                                    "unary_error_sum": to_float(row[29]),
                                    "unary_hessian_count": to_int(row[30]),
                                    "unary_hessian_sum": to_float(row[31]),
                                    "other_local_count": to_int(row[32]),
                                    "other_local_error_count": to_int(row[33]),
                                    "other_local_error_sum": to_float(row[34]),
                                    "other_local_hessian_count": to_int(row[35]),
                                    "other_local_hessian_sum": to_float(row[36]),
                                    "other_external_count": to_int(row[37]),
                                    "other_external_error_count": to_int(row[38]),
                                    "other_external_error_sum": to_float(row[39]),
                                    "other_external_hessian_count": to_int(row[40]),
                                    "other_external_hessian_sum": to_float(row[41]),
                                }
                            )
                        elif marker == "GLIM_CBS_ACTIVE_FACTOR_DETAIL_ROW":
                            glim_active_factor_detail_rows.append(
                                {
                                    "direction": row[1],
                                    "receiver_robot": row[2],
                                    "source_agent": row[3],
                                    "local_from_key": row[4],
                                    "local_to_key": row[5],
                                    "source_edge": row[6],
                                    "active_update_count": to_int(row[7]),
                                    "active_age_sec": to_float(row[8]),
                                    "cbs_factor_index": row[9],
                                    "graph_factor_index": row[10],
                                    "category": row[11],
                                    "is_external": to_int(row[12]),
                                    "touches_from": to_int(row[13]),
                                    "touches_to": to_int(row[14]),
                                    "key_count": to_int(row[15]),
                                    "keys": row[16],
                                    "factor_type": row[17],
                                    "error": to_float(row[18]),
                                    "hessian_frobenius": to_float(row[19]),
                                }
                            )
                        elif marker == "KIMERA_CBS_ACTIVE_FACTOR_DIAGNOSTIC_ROW":
                            kimera_active_factor_diagnostic_rows.append(
                                {
                                    "direction": row[1],
                                    "receiver_robot": row[2],
                                    "source_agent": row[3],
                                    "local_from_key": row[4],
                                    "local_to_key": row[5],
                                    "source_edge": row[6],
                                    "factor_index": row[7],
                                    "active_update_count": to_int(row[8]),
                                    "active_age_sec": to_float(row[9]),
                                    "from_abs_dt": to_float(row[10]),
                                    "to_abs_dt": to_float(row[11]),
                                    "covariance_trace": to_float(row[12]),
                                    "covariance_rotation_trace": to_float(row[13]),
                                    "covariance_translation_trace": to_float(row[14]),
                                    "cbs_error": to_float(row[15]),
                                    "residual_norm": to_float(row[16]),
                                    "rot_norm": to_float(row[17]),
                                    "trans_norm": to_float(row[18]),
                                    "whitened_norm": to_float(row[19]),
                                    "nis": to_float(row[20]),
                                    "cbs_hessian_frobenius": to_float(row[21]),
                                    "same_edge_count": to_int(row[22]),
                                    "same_edge_error_count": to_int(row[23]),
                                    "same_edge_error_sum": to_float(row[24]),
                                    "same_edge_hessian_count": to_int(row[25]),
                                    "same_edge_hessian_sum": to_float(row[26]),
                                    "unary_count": to_int(row[27]),
                                    "unary_error_count": to_int(row[28]),
                                    "unary_error_sum": to_float(row[29]),
                                    "unary_hessian_count": to_int(row[30]),
                                    "unary_hessian_sum": to_float(row[31]),
                                    "other_local_count": to_int(row[32]),
                                    "other_local_error_count": to_int(row[33]),
                                    "other_local_error_sum": to_float(row[34]),
                                    "other_local_hessian_count": to_int(row[35]),
                                    "other_local_hessian_sum": to_float(row[36]),
                                    "other_external_count": to_int(row[37]),
                                    "other_external_error_count": to_int(row[38]),
                                    "other_external_error_sum": to_float(row[39]),
                                    "other_external_hessian_count": to_int(row[40]),
                                    "other_external_hessian_sum": to_float(row[41]),
                                }
                            )
                        elif marker == "GLIM_POSE_STAGE_ROW":
                            glim_pose_stage_rows.append(
                                {
                                    "stamp": to_float(row[1]),
                                    "frame_id": to_int(row[2]),
                                    "last_frame_id": to_int(row[3]),
                                    "imu_integrated_count": to_int(row[4]),
                                    "last_to_imu_pred_translation_m": to_float(row[5]),
                                    "last_to_imu_pred_rotation_deg": to_float(row[6]),
                                    "last_to_scan_translation_m": to_float(row[7]),
                                    "last_to_scan_rotation_deg": to_float(row[8]),
                                    "last_to_smoother_translation_m": to_float(row[9]),
                                    "last_to_smoother_rotation_deg": to_float(row[10]),
                                    "scan_minus_imu_translation_m": to_float(row[11]),
                                    "scan_minus_imu_rotation_deg": to_float(row[12]),
                                    "smoother_minus_scan_translation_m": to_float(row[13]),
                                    "smoother_minus_scan_rotation_deg": to_float(row[14]),
                                    "smoother_minus_imu_translation_m": to_float(row[15]),
                                    "smoother_minus_imu_rotation_deg": to_float(row[16]),
                                    "imu_pred_x": to_float(row[17]),
                                    "imu_pred_y": to_float(row[18]),
                                    "imu_pred_z": to_float(row[19]),
                                    "scan_x": to_float(row[20]),
                                    "scan_y": to_float(row[21]),
                                    "scan_z": to_float(row[22]),
                                    "smoother_x": to_float(row[23]),
                                    "smoother_y": to_float(row[24]),
                                    "smoother_z": to_float(row[25]),
                                }
                            )
                        elif marker == "GLIM_TARGET_UPDATE_ROW":
                            glim_target_update_rows.append(
                                {
                                    "stamp": to_float(row[1]),
                                    "frame_id": to_int(row[2]),
                                    "action": row[3],
                                    "pose_source": row[4],
                                    "scan_health_enable": to_int(row[5]),
                                    "scan_health_reference_ready": to_int(row[6]),
                                    "scan_health_raw": to_float(row[7]),
                                    "scan_health_clamped": to_float(row[8]),
                                    "preprocessed_points": to_float(row[9]),
                                    "scan_error_ratio": to_float(row[10]),
                                    "scan_minus_imu_translation_m": to_float(row[11]),
                                    "scan_minus_imu_rotation_deg": to_float(row[12]),
                                    "scan_correction_translation_m": to_float(row[13]),
                                    "scan_correction_rotation_deg": to_float(row[14]),
                                    "target_update_x": to_float(row[15]),
                                    "target_update_y": to_float(row[16]),
                                    "target_update_z": to_float(row[17]),
                                    "previous_target_x": to_float(row[18]),
                                    "previous_target_y": to_float(row[19]),
                                    "previous_target_z": to_float(row[20]),
                                }
                            )
                        elif marker == "GLIM_SCAN_HEALTH_ROW":
                            parsed_row = parse_glim_scan_health_row(row)
                            if parsed_row:
                                glim_scan_health_rows.append(parsed_row)
                            else:
                                skipped_rows[f"{marker}:malformed_glim_scan_health"] += 1
                        elif marker in {
                            "KIMERA_BACKEND_SPINONCE_TIMING_ROW",
                            "KIMERA_CBS_OUTGOING_TIMING_ROW",
                            "KIMERA_OPTIMIZE_TIMING_ROW",
                            "KIMERA_BACKEND_CALLBACK_TIMING_ROW",
                            "KIMERA_RERUN_CALLBACK_TIMING_ROW",
                        }:
                            timing = parse_kimera_timing_row(row)
                            if timing:
                                timing_rows.append(timing)
                            else:
                                skipped_rows[f"{marker}:malformed_timing"] += 1

    return {
        "transport": transport_rows,
        "roundtrip": roundtrip_rows,
        "merge": merge_rows,
        "bpsam_add": bpsam_rows,
        "outgoing_filter": outgoing_filter_rows,
        "receiver_diagnostic": receiver_diagnostic_rows,
        "temporary_linear_accounting": temporary_linear_accounting_rows,
        "preinjection_residual": preinjection_residual_rows,
        "belief_odom": belief_odom_rows,
        "odom_outgoing": odom_outgoing_rows,
        "odom_relative_covariance": odom_relative_covariance_rows,
        "odom_match": odom_match_rows,
        "odom_retry": odom_retry_rows,
        "bpsam_odom_add": bpsam_odom_add_rows,
        "glim_odom_inject": glim_odom_inject_rows,
        "odom_factor_covariance": odom_factor_covariance_rows,
        "health_sender": health_sender_rows,
        "health_nis": health_nis_rows,
        "glim_receiver_covariance": glim_receiver_covariance_rows,
        "glim_active_factor_diagnostic": glim_active_factor_diagnostic_rows,
        "glim_active_factor_detail": glim_active_factor_detail_rows,
        "kimera_active_factor_diagnostic": kimera_active_factor_diagnostic_rows,
        "glim_pose_stage": glim_pose_stage_rows,
        "glim_target_update": glim_target_update_rows,
        "odom_temporary_postsolve_residual": odom_temporary_postsolve_residual_rows,
        "temporary_linearization_residual": temporary_linearization_residual_rows,
        "provenance": provenance_rows,
        "marginalization_graph": marginalization_graph_rows,
        "timing": timing_rows,
        "glim_timing": glim_timing_rows,
        "glim_scan_health": glim_scan_health_rows,
        "kimera_flow": kimera_flow_rows,
        "kimera_odom_flow": kimera_odom_flow_rows,
        "liorf_odom_flow": liorf_odom_flow_rows,
        "glim_odom_flow": glim_odom_flow_rows,
        "alignment": alignment_rows,
        "skipped_rows": dict(skipped_rows),
    }


def parse_kimera_timing_row(row: List[str]) -> Dict[str, Any]:
    marker = row[0] if row else ""
    common = {
        "marker": marker,
        "stage": marker.replace("KIMERA_", "").replace("_TIMING_ROW", "").lower(),
    }
    if marker == "KIMERA_BACKEND_SPINONCE_TIMING_ROW" and len(row) >= 10:
        return {
            **common,
            "cur_kf_id": to_int(row[1]),
            "total_ms": to_float(row[2]),
            "backend_state_process_ms": to_float(row[3]),
            "output_landmark_map_ms": to_float(row[4]),
            "map_update_callback_ms": to_float(row[5]),
            "backend_output_construct_ms": to_float(row[6]),
            "backend_logger_output_ms": to_float(row[7]),
            "backend_state": to_int(row[8]),
            "backend_status_ok": to_int(row[9]),
        }
    if marker == "KIMERA_CBS_OUTGOING_TIMING_ROW" and len(row) >= 8:
        return {
            **common,
            "cur_kf_id": to_int(row[1]),
            "total_ms": to_float(row[2]),
            "set_marginalization_graph_ms": to_float(row[3]),
            "get_odometry_beliefs_ms": to_float(row[4]),
            "outgoing_odom_beliefs": to_int(row[5]),
            "marginalization_graph_factor_count": to_int(row[6]),
            "request_keys": to_int(row[7]),
        }
    if marker == "KIMERA_OPTIMIZE_TIMING_ROW" and len(row) >= 18:
        return {
            **common,
            "cur_kf_id": to_int(row[1]),
            "total_ms": to_float(row[2]),
            "factor_preparation_ms": to_float(row[3]),
            "collect_external_beliefs_ms": to_float(row[4]),
            "delete_slots_sort_ms": to_float(row[5]),
            "smoother_update_ms": to_float(row[6]),
            "slot_bookkeeping_ms": to_float(row[7]),
            "extra_iterations_ms": to_float(row[8]),
            "update_states_ms": to_float(row[9]),
            "cbs_outgoing_total_ms": to_float(row[10]),
            "cbs_set_marginalization_graph_ms": to_float(row[11]),
            "cbs_get_odometry_beliefs_ms": to_float(row[12]),
            "compute_state_covariance_ms": to_float(row[13]),
            "post_debug_ms": to_float(row[14]),
            "outgoing_odom_beliefs": to_int(row[15]),
            "marginalization_graph_factor_count": to_int(row[16]),
            "smoother_ok": to_int(row[17]),
        }
    if marker == "KIMERA_BACKEND_CALLBACK_TIMING_ROW" and len(row) >= 7:
        return {
            **common,
            "cur_kf_id": to_int(row[1]),
            "total_ms": to_float(row[2]),
            "odometry_publish_ms": to_float(row[3]),
            "rerun_publish_ms": to_float(row[4]),
            "odometry_belief_publish_ms": to_float(row[5]),
            "use_rviz": to_int(row[6]),
        }
    if marker == "KIMERA_RERUN_CALLBACK_TIMING_ROW" and len(row) >= 10:
        return {
            **common,
            "cur_kf_id": to_int(row[1]),
            "total_ms": to_float(row[2]),
            "current_pose_ms": to_float(row[3]),
            "trajectory_ms": to_float(row[4]),
            "landmarks_ms": to_float(row[5]),
            "factor_graph_ms": to_float(row[6]),
            "landmark_count": to_int(row[7]),
            "factor_graph_factor_count": to_int(row[8]),
            "factor_graph_enabled": to_int(row[9]),
        }
    return {}


def parse_glim_timing_row(row: List[str]) -> Dict[str, Any]:
    marker = row[0] if row else ""
    common = {"marker": marker}
    if marker == "GLIM_ROS_INPUT_TIMING_ROW" and len(row) >= 13:
        return {
            **common,
            "stage": "ros_input",
            "stamp": to_float(row[1]),
            "raw_points": to_int(row[2]),
            "preprocessed_points": to_int(row[3]),
            "time_keeper_ms": to_float(row[4]),
            "preprocess_ms": to_float(row[5]),
            "workload_wait_ms": to_float(row[6]),
            "enqueue_ms": to_float(row[7]),
            "total_ms": to_float(row[8]),
            "workload_before_wait": to_int(row[9]),
            "workload_after_wait": to_int(row[10]),
            "wait_loops": to_int(row[11]),
            "skipped": to_int(row[12]),
        }
    if marker == "GLIM_ASYNC_ODOM_TIMING_ROW" and len(row) >= 14:
        return {
            **common,
            "stage": "async_odom",
            "stamp": to_float(row[1]),
            "scan_end_time": to_float(row[2]),
            "last_imu_time": to_float(row[3]),
            "raw_queue": to_int(row[4]),
            "imu_batch": to_int(row[5]),
            "frame_batch": to_int(row[6]),
            "imu_wait_ms": to_float(row[7]),
            "odom_insert_ms": to_float(row[8]),
            "total_ms": to_float(row[9]),
            "internal_queue_after": to_int(row[10]),
            "produced_state": to_int(row[11]),
            "marginalized_count": to_int(row[12]),
            "status": row[13],
        }
    if marker == "GLIM_ODOM_IMU_TIMING_ROW" and len(row) >= 26:
        return {
            **common,
            "stage": "odom_imu",
            "stamp": to_float(row[1]),
            "frame_id": to_int(row[2]),
            "points": to_int(row[3]),
            "num_imu_integrated": to_int(row[4]),
            "new_factors": to_int(row[5]),
            "active_frames": to_int(row[6]),
            "marginalized_frames": to_int(row[7]),
            "state_lookup_ms": to_float(row[8]),
            "inter_scan_imu_ms": to_float(row[9]),
            "imu_factor_ms": to_float(row[10]),
            "intra_scan_imu_ms": to_float(row[11]),
            "deskew_ms": to_float(row[12]),
            "point_covariance_ms": to_float(row[13]),
            "cpu_frame_ms": to_float(row[14]),
            "create_frame_ms": to_float(row[15]),
            "create_factors_ms": to_float(row[16]),
            "pre_smoother_callback_ms": to_float(row[17]),
            "smoother_update_ms": to_float(row[18]),
            "post_smoother_callback_ms": to_float(row[19]),
            "marginalization_ms": to_float(row[20]),
            "update_frames_ms": to_float(row[21]),
            "imu_validation_ms": to_float(row[22]),
            "update_callbacks_ms": to_float(row[23]),
            "total_ms": to_float(row[24]),
            "status": row[25],
        }
    if marker == "GLIM_GPU_TIMING_ROW" and len(row) >= 18:
        substage = row[1]
        return {
            **common,
            "stage": f"gpu_{substage}",
            "substage": substage,
            "stamp": to_float(row[2]),
            "frame_id": to_int(row[3]),
            "points": to_int(row[4]),
            "keyframes_before": to_int(row[5]),
            "keyframes_after": to_int(row[6]),
            "median_ms": to_float(row[7]),
            "clone_ms": to_float(row[8]),
            "voxelmap_ms": to_float(row[9]),
            "factor_create_ms": to_float(row[10]),
            "keyframe_update_ms": to_float(row[11]),
            "total_ms": to_float(row[12]),
            "voxelmap_count": to_int(row[13]),
            "binary_factors": to_int(row[14]),
            "unary_factors": to_int(row[15]),
            "overlap_calls": to_int(row[16]),
            "status": row[17],
        }
    if marker == "GLIM_CBS_TIMING_ROW" and len(row) >= 18:
        substage = row[1]
        parsed: Dict[str, Any] = {
            **common,
            "stage": f"cbs_{substage}",
            "substage": substage,
            "last_pose_index": to_int(row[2]),
            "matched": to_int(row[3]),
            "injected": to_int(row[4]),
            "duplicate": to_int(row[5]),
            "rejected": to_int(row[6]),
            "pending": to_int(row[7]),
            "pending_oldest_age_sec": to_float(row[8]),
            "update_timestamps_ms": to_float(row[9]),
            "consume_incoming_ms": to_float(row[10]),
            "inject_loop_ms": to_float(row[11]),
            "sidecar_update_ms": to_float(row[12]),
            "publish_odometry_ms": to_float(row[13]),
            "outgoing_ms": to_float(row[14]),
            "scalar_ms": to_float(row[15]),
            "outgoing_count": to_int(row[16]),
            "total_ms": to_float(row[17]),
        }
        if substage == "publish_outgoing" and len(row) >= 24:
            parsed.update(
                {
                    "request_build_ms": to_float(row[18]),
                    "set_marginalization_graph_ms": to_float(row[19]),
                    "get_odometry_beliefs_ms": to_float(row[20]),
                    "message_build_ms": to_float(row[21]),
                    "publish_ms": to_float(row[22]),
                    "request_keys": to_int(row[23]),
                }
            )
        elif substage == "publish_odometry" and len(row) >= 23:
            parsed.update(
                {
                    "estimate_ms": to_float(row[18]),
                    "marginal_covariance_ms": to_float(row[19]),
                    "odom_publish_ms": to_float(row[20]),
                    "rerun_ms": to_float(row[21]),
                    "covariance_ok": to_int(row[22]),
                }
            )
        return parsed
    return {}


def parse_glim_scan_health_row(row: List[str]) -> Dict[str, Any]:
    if len(row) < 28:
        return {}
    parsed = {
        "marker": row[0],
        "stamp": to_float(row[1]),
        "frame_id": to_int(row[2]),
        "registration_type": row[3],
        "preprocessed_points": to_int(row[4]),
        "matching_factor_count": to_int(row[5]),
        "scan_initial_error": to_float(row[6]),
        "scan_final_error": to_float(row[7]),
        "scan_error_ratio": to_float(row[8]),
        "lm_iterations": to_int(row[9]),
        "lm_inner_iterations": to_int(row[10]),
        "lm_error": to_float(row[11]),
        "lm_cost_change": to_float(row[12]),
        "lm_lambda": to_float(row[13]),
        "lm_solve_success": to_int(row[14]),
        "lm_linearization_time_sec": to_float(row[15]),
        "lm_linear_solver_time_sec": to_float(row[16]),
        "imu_pred_delta_translation_m": to_float(row[17]),
        "imu_pred_delta_rotation_deg": to_float(row[18]),
        "scan_delta_translation_m": to_float(row[19]),
        "scan_delta_rotation_deg": to_float(row[20]),
        "scan_minus_imu_translation_m": to_float(row[21]),
        "scan_minus_imu_rotation_deg": to_float(row[22]),
        "scan_hessian_min_eigenvalue": to_float(row[23]),
        "scan_hessian_max_eigenvalue": to_float(row[24]),
        "scan_hessian_condition_estimate": to_float(row[25]),
        "scan_hessian_rank_estimate": to_int(row[26]),
        "scan_hessian_frobenius_norm": to_float(row[27]),
    }
    if len(row) >= 48:
        parsed.update(
            {
                "scan_health_enable": to_int(row[28]),
                "scan_health_apply_to_local_scan_precision": to_int(row[29]),
                "scan_health_reference_ready": to_int(row[30]),
                "scan_health_reference_count": to_int(row[31]),
                "scan_health_point_reference": to_float(row[32]),
                "scan_health_hessian_min_reference": to_float(row[33]),
                "scan_health_hessian_frobenius_reference": to_float(row[34]),
                "scan_health_point_ratio": to_float(row[35]),
                "scan_health_point_score": to_float(row[36]),
                "scan_health_translation_score": to_float(row[37]),
                "scan_health_rotation_score": to_float(row[38]),
                "scan_health_error_ratio_score": to_float(row[39]),
                "scan_health_hessian_min_ratio": to_float(row[40]),
                "scan_health_hessian_min_score": to_float(row[41]),
                "scan_health_hessian_frobenius_ratio": to_float(row[42]),
                "scan_health_hessian_frobenius_score": to_float(row[43]),
                "scan_health_raw": to_float(row[44]),
                "scan_health_clamped": to_float(row[45]),
                "scan_health_base_scan_precision": to_float(row[46]),
                "scan_health_effective_scan_precision": to_float(row[47]),
                "scan_health_hessian_enable": to_int(row[48]) if len(row) >= 49 else 0,
            }
        )
    return parsed


def write_dicts_csv(path: Path, rows: List[Dict[str, Any]]) -> None:
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fieldnames: List[str] = []
    for row in rows:
        for key in row.keys():
            if key not in fieldnames:
                fieldnames.append(key)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def load_ground_truth(path: Path) -> Trajectory:
    return [(pose["t"], (pose["x"], pose["y"], pose["z"])) for pose in load_ground_truth_poses(path)]


def load_ground_truth_poses(path: Path) -> PoseTrajectory:
    points: Trajectory = []
    if not path.exists():
        return points
    with path.open("r", encoding="utf-8", errors="replace") as stream:
        for line in stream:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.replace(",", " ").split()
            if len(parts) < 8:
                continue
            pose = {
                "t": timestamp_to_seconds(to_float(parts[0])),
                "x": to_float(parts[1]),
                "y": to_float(parts[2]),
                "z": to_float(parts[3]),
                "qx": to_float(parts[4]),
                "qy": to_float(parts[5]),
                "qz": to_float(parts[6]),
                "qw": to_float(parts[7]),
            }
            if all(math.isfinite(value) for value in pose.values()):
                points.append(normalize_pose_quaternion(pose))
    return sorted(points, key=lambda item: item["t"])


def first_existing_key(row: Dict[str, str], candidates: Sequence[str]) -> Optional[str]:
    for key in candidates:
        if key in row:
            return key
    for key in row:
        for candidate in candidates:
            if key.endswith(candidate):
                return key
    return None


def load_odometry_csv(path: Path) -> Trajectory:
    return [(pose["t"], (pose["x"], pose["y"], pose["z"])) for pose in load_odometry_poses(path)]


def normalize_pose_quaternion(pose: PoseRow) -> PoseRow:
    norm = math.sqrt(
        pose["qx"] ** 2 + pose["qy"] ** 2 + pose["qz"] ** 2 + pose["qw"] ** 2
    )
    if not math.isfinite(norm) or norm <= 0.0:
        pose["qx"] = 0.0
        pose["qy"] = 0.0
        pose["qz"] = 0.0
        pose["qw"] = 1.0
        return pose
    pose["qx"] /= norm
    pose["qy"] /= norm
    pose["qz"] /= norm
    pose["qw"] /= norm
    return pose


def load_odometry_poses(path: Path) -> PoseTrajectory:
    if not path.exists() or path.stat().st_size == 0:
        return []
    poses: PoseTrajectory = []
    with path.open("r", encoding="utf-8", errors="replace", newline="") as stream:
        reader = csv.DictReader(stream)
        for row in reader:
            if not row:
                continue
            x_key = first_existing_key(row, ("field.pose.pose.position.x", "pose.pose.position.x"))
            y_key = first_existing_key(row, ("field.pose.pose.position.y", "pose.pose.position.y"))
            z_key = first_existing_key(row, ("field.pose.pose.position.z", "pose.pose.position.z"))
            if not x_key or not y_key or not z_key:
                continue
            qx_key = first_existing_key(
                row, ("field.pose.pose.orientation.x", "pose.pose.orientation.x")
            )
            qy_key = first_existing_key(
                row, ("field.pose.pose.orientation.y", "pose.pose.orientation.y")
            )
            qz_key = first_existing_key(
                row, ("field.pose.pose.orientation.z", "pose.pose.orientation.z")
            )
            qw_key = first_existing_key(
                row, ("field.pose.pose.orientation.w", "pose.pose.orientation.w")
            )

            sec_key = first_existing_key(row, ("field.header.stamp.secs", "header.stamp.secs"))
            nsec_key = first_existing_key(row, ("field.header.stamp.nsecs", "header.stamp.nsecs"))
            stamp_key = first_existing_key(row, ("field.header.stamp", "header.stamp", "%time"))
            if sec_key and nsec_key:
                t = to_float(row[sec_key]) + to_float(row[nsec_key]) * 1e-9
            elif stamp_key:
                t = to_float(row[stamp_key])
            else:
                t = math.nan
            t = timestamp_to_seconds(t)

            pose = {
                "t": t,
                "x": to_float(row[x_key]),
                "y": to_float(row[y_key]),
                "z": to_float(row[z_key]),
                "qx": to_float(row[qx_key]) if qx_key else 0.0,
                "qy": to_float(row[qy_key]) if qy_key else 0.0,
                "qz": to_float(row[qz_key]) if qz_key else 0.0,
                "qw": to_float(row[qw_key]) if qw_key else 1.0,
            }
            if all(math.isfinite(value) for value in pose.values()):
                poses.append(normalize_pose_quaternion(pose))
    return sorted(poses, key=lambda item: item["t"])


def interpolate_trajectory(traj: Trajectory, query_t: float) -> Optional[Point]:
    if not traj:
        return None
    times = [item[0] for item in traj]
    if query_t < times[0] or query_t > times[-1]:
        return None
    idx = bisect.bisect_left(times, query_t)
    if idx < len(traj) and abs(times[idx] - query_t) < 1e-9:
        return traj[idx][1]
    if idx == 0 or idx >= len(traj):
        return None
    t0, p0 = traj[idx - 1]
    t1, p1 = traj[idx]
    if t1 <= t0:
        return p0
    alpha = (query_t - t0) / (t1 - t0)
    return (
        p0[0] + alpha * (p1[0] - p0[0]),
        p0[1] + alpha * (p1[1] - p0[1]),
        p0[2] + alpha * (p1[2] - p0[2]),
    )


def pair_trajectories(
    source: Trajectory,
    target: Trajectory,
) -> Tuple[List[Point], List[Point], List[float]]:
    src_points: List[Point] = []
    dst_points: List[Point] = []
    matched_times: List[float] = []
    for t, point in source:
        target_point = interpolate_trajectory(target, t)
        if target_point is None:
            continue
        src_points.append(point)
        dst_points.append(target_point)
        matched_times.append(t)
    return src_points, dst_points, matched_times


def path_length(traj: Trajectory) -> float:
    if len(traj) < 2:
        return 0.0
    total = 0.0
    last = traj[0][1]
    for _, point in traj[1:]:
        total += distance(point, last)
        last = point
    return total


def point_path_length(points: Sequence[Point]) -> float:
    if len(points) < 2:
        return 0.0
    total = 0.0
    last = points[0]
    for point in points[1:]:
        total += distance(point, last)
        last = point
    return total


def distance(lhs: Point, rhs: Point) -> float:
    return math.sqrt(
        (lhs[0] - rhs[0]) ** 2 + (lhs[1] - rhs[1]) ** 2 + (lhs[2] - rhs[2]) ** 2
    )


def mean_point(points: Sequence[Point]) -> Point:
    n = float(len(points))
    return (
        sum(point[0] for point in points) / n,
        sum(point[1] for point in points) / n,
        sum(point[2] for point in points) / n,
    )


def align_se2(source: Sequence[Point], target: Sequence[Point]) -> Tuple[float, Point, List[Point]]:
    if len(source) != len(target) or len(source) < 2:
        return 0.0, (0.0, 0.0, 0.0), list(source)

    src_mean = mean_point(source)
    dst_mean = mean_point(target)
    a = 0.0
    b = 0.0
    for src, dst in zip(source, target):
        sx = src[0] - src_mean[0]
        sy = src[1] - src_mean[1]
        dx = dst[0] - dst_mean[0]
        dy = dst[1] - dst_mean[1]
        a += sx * dx + sy * dy
        b += sx * dy - sy * dx
    yaw = math.atan2(b, a)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    tx = dst_mean[0] - (cos_yaw * src_mean[0] - sin_yaw * src_mean[1])
    ty = dst_mean[1] - (sin_yaw * src_mean[0] + cos_yaw * src_mean[1])
    tz = dst_mean[2] - src_mean[2]

    aligned = [
        (
            cos_yaw * point[0] - sin_yaw * point[1] + tx,
            sin_yaw * point[0] + cos_yaw * point[1] + ty,
            point[2] + tz,
        )
        for point in source
    ]
    return yaw, (tx, ty, tz), aligned


def rmse(errors: Sequence[float]) -> float:
    if not errors:
        return math.nan
    return math.sqrt(sum(error * error for error in errors) / len(errors))


def trajectory_metric_row(name: str, traj: Trajectory, reference: Trajectory) -> Dict[str, Any]:
    src, dst, matched_times = pair_trajectories(traj, reference)
    yaw, translation, aligned = align_se2(src, dst)
    errors = [distance(lhs, rhs) for lhs, rhs in zip(aligned, dst)]
    return {
        "name": name,
        "samples": len(traj),
        "duration_sec": (
            matched_times[-1] - matched_times[0]
            if len(matched_times) >= 2
            else math.nan
        ),
        "path_length_m": point_path_length(src),
        "reference_path_length_m": point_path_length(dst),
        "reference_matches": len(errors),
        "alignment": "SE2 yaw+translation",
        "alignment_yaw_deg": math.degrees(yaw),
        "alignment_tx_m": translation[0],
        "alignment_ty_m": translation[1],
        "alignment_tz_m": translation[2],
        "position_rmse_m": rmse(errors),
        "position_mean_error_m": numeric_stats(errors)["mean"],
        "position_max_error_m": numeric_stats(errors)["max"],
    }


def load_estimator_odometry(run_dir: Path) -> Dict[str, PoseTrajectory]:
    trajectories: Dict[str, PoseTrajectory] = {}
    traj_dir = run_dir / "trajectories"
    for path in sorted(traj_dir.glob("*_odometry.csv")):
        estimator = odometry_estimator_name(path)
        poses = load_odometry_poses(path)
        if poses:
            trajectories[estimator] = poses
    return trajectories


def compute_trajectory_metrics(run_dir: Path, gt_path: Path) -> List[Dict[str, Any]]:
    gt = load_ground_truth(gt_path)
    estimators = {
        name: [(pose["t"], (pose["x"], pose["y"], pose["z"])) for pose in poses]
        for name, poses in load_estimator_odometry(run_dir).items()
    }

    rows: List[Dict[str, Any]] = []
    if gt:
        for name, traj in sorted(estimators.items()):
            rows.append(trajectory_metric_row(f"{name}_vs_ground_truth", traj, gt))
    names = sorted(estimators)
    for i, lhs in enumerate(names):
        for rhs in names[i + 1 :]:
            rows.append(
                trajectory_metric_row(
                    f"{lhs}_vs_{rhs}",
                    estimators[lhs],
                    estimators[rhs],
                )
            )
    return rows


def pose_index_from_token(token: Any) -> Optional[int]:
    match = re.fullmatch(r"[A-Za-z](\d+)", str(token))
    if not match:
        return None
    return int(match.group(1))


def edge_to_pose_indices(edge: Any) -> Tuple[Optional[int], Optional[int]]:
    indices = [int(item) for item in re.findall(r"[A-Za-z](\d+)", str(edge))]
    if len(indices) < 2:
        return None, None
    return indices[0], indices[1]


def nearest_row_by_stamp(
    rows: Sequence[Dict[str, Any]],
    query_t: float,
    stamp_key: str = "stamp",
) -> Optional[Dict[str, Any]]:
    if not rows or not math.isfinite(query_t):
        return None
    times = [to_float(str(row.get(stamp_key, math.nan))) for row in rows]
    idx = bisect.bisect_left(times, query_t)
    candidates: List[int] = []
    if idx < len(rows):
        candidates.append(idx)
    if idx > 0:
        candidates.append(idx - 1)
    if not candidates:
        return None
    best = min(candidates, key=lambda item: abs(times[item] - query_t))
    return rows[best]


def trajectory_error_timeline(
    run_dir: Path,
    gt_path: Path,
) -> List[Dict[str, Any]]:
    gt = load_ground_truth(gt_path)
    estimators = load_estimator_odometry(run_dir)
    if not gt or not estimators:
        return []

    first_stamps = [poses[0]["t"] for poses in estimators.values() if poses]
    if not first_stamps:
        return []
    first_stamp = min(first_stamps)
    rows: List[Dict[str, Any]] = []
    for estimator, poses in sorted(estimators.items()):
        matched: List[Tuple[PoseRow, Point]] = []
        for pose in poses:
            gt_point = interpolate_trajectory(gt, pose["t"])
            if gt_point is not None:
                matched.append((pose, gt_point))
        if len(matched) < 2:
            continue
        est_points = [(pose["x"], pose["y"], pose["z"]) for pose, _ in matched]
        gt_points = [point for _, point in matched]
        yaw, translation, aligned = align_se2(est_points, gt_points)
        for (pose, gt_point), aligned_point in zip(matched, aligned):
            rows.append(
                {
                    "estimator": estimator,
                    "stamp": pose["t"],
                    "rel_sec": pose["t"] - first_stamp,
                    "error_m": distance(aligned_point, gt_point),
                    "alignment": "SE2 yaw+translation",
                    "alignment_yaw_deg": math.degrees(yaw),
                    "alignment_tx_m": translation[0],
                    "alignment_ty_m": translation[1],
                    "alignment_tz_m": translation[2],
                    "est_x": pose["x"],
                    "est_y": pose["y"],
                    "est_z": pose["z"],
                    "aligned_x": aligned_point[0],
                    "aligned_y": aligned_point[1],
                    "aligned_z": aligned_point[2],
                    "gt_x": gt_point[0],
                    "gt_y": gt_point[1],
                    "gt_z": gt_point[2],
                }
            )
    return sorted(rows, key=lambda row: (str(row.get("estimator", "")), row["stamp"]))


def median_or_nan(values: Iterable[float]) -> float:
    return percentile(values, 0.50)


def glim_frame_timeline(
    parsed: Dict[str, Any],
    trajectory_error_rows: Sequence[Dict[str, Any]],
) -> List[Dict[str, Any]]:
    scan_rows = sorted(parsed.get("glim_scan_health", []), key=lambda row: row.get("stamp", 0.0))
    pose_rows = {
        int(row["frame_id"]): row
        for row in parsed.get("glim_pose_stage", [])
        if row.get("frame_id") is not None
    }
    target_rows = {
        int(row["frame_id"]): row
        for row in parsed.get("glim_target_update", [])
        if row.get("frame_id") is not None
    }
    if not scan_rows and pose_rows:
        scan_rows = sorted(pose_rows.values(), key=lambda row: row.get("stamp", 0.0))
    if not scan_rows:
        return []

    estimator_errors: Dict[str, List[Dict[str, Any]]] = defaultdict(list)
    for row in trajectory_error_rows:
        estimator_errors[str(row.get("estimator", ""))].append(row)
    for rows in estimator_errors.values():
        rows.sort(key=lambda row: row.get("stamp", 0.0))

    active_by_to_frame: Dict[int, List[Dict[str, Any]]] = defaultdict(list)
    active_near_by_frame: Dict[int, List[Dict[str, Any]]] = defaultdict(list)
    for row in parsed.get("glim_active_factor_diagnostic", []):
        if row.get("direction") != "K2G":
            continue
        to_index = pose_index_from_token(row.get("local_to_key"))
        if to_index is None:
            continue
        active_by_to_frame[to_index].append(row)
        for near_index in range(to_index - 2, to_index + 3):
            active_near_by_frame[near_index].append(row)

    inject_by_to_frame: Counter[int] = Counter()
    for row in parsed.get("glim_odom_inject", []):
        if row.get("direction") != "K2G":
            continue
        _, to_index = edge_to_pose_indices(row.get("receiver_edge"))
        if to_index is not None:
            inject_by_to_frame[to_index] += 1

    start_stamp = min(to_float(str(row.get("stamp", math.nan))) for row in scan_rows)
    rows: List[Dict[str, Any]] = []
    for scan in scan_rows:
        stamp = to_float(str(scan.get("stamp", math.nan)))
        frame_id = int(scan.get("frame_id")) if scan.get("frame_id") is not None else -1
        pose = pose_rows.get(frame_id, {})
        target = target_rows.get(frame_id, {})
        glim_error = nearest_row_by_stamp(estimator_errors.get("glim", []), stamp)
        kimera_error = nearest_row_by_stamp(estimator_errors.get("kimera", []), stamp)
        active_exact = active_by_to_frame.get(frame_id, [])
        active_near = active_near_by_frame.get(frame_id, [])

        def active_stat(key: str, rows_: Sequence[Dict[str, Any]]) -> float:
            return median_or_nan(to_float(str(row.get(key, math.nan))) for row in rows_)

        rows.append(
            {
                "stamp": stamp,
                "rel_sec": stamp - start_stamp,
                "frame_id": frame_id,
                "glim_ape_m": (
                    to_float(str(glim_error.get("error_m", math.nan)))
                    if glim_error
                    else math.nan
                ),
                "kimera_nearest_ape_m": (
                    to_float(str(kimera_error.get("error_m", math.nan)))
                    if kimera_error
                    else math.nan
                ),
                "kimera_nearest_dt_sec": (
                    abs(stamp - to_float(str(kimera_error.get("stamp", math.nan))))
                    if kimera_error
                    else math.nan
                ),
                "preprocessed_points": scan.get("preprocessed_points", math.nan),
                "scan_error_ratio": scan.get("scan_error_ratio", math.nan),
                "scan_minus_imu_translation_m": scan.get(
                    "scan_minus_imu_translation_m", math.nan
                ),
                "scan_minus_imu_rotation_deg": scan.get(
                    "scan_minus_imu_rotation_deg", math.nan
                ),
                "scan_hessian_min_eigenvalue": scan.get(
                    "scan_hessian_min_eigenvalue", math.nan
                ),
                "scan_hessian_condition_estimate": scan.get(
                    "scan_hessian_condition_estimate", math.nan
                ),
                "scan_health_enable": scan.get("scan_health_enable", math.nan),
                "scan_health_clamped": scan.get("scan_health_clamped", math.nan),
                "effective_scan_precision": scan.get(
                    "scan_health_effective_scan_precision", math.nan
                ),
                "smoother_minus_scan_translation_m": pose.get(
                    "smoother_minus_scan_translation_m", math.nan
                ),
                "smoother_minus_scan_rotation_deg": pose.get(
                    "smoother_minus_scan_rotation_deg", math.nan
                ),
                "smoother_minus_imu_translation_m": pose.get(
                    "smoother_minus_imu_translation_m", math.nan
                ),
                "smoother_minus_imu_rotation_deg": pose.get(
                    "smoother_minus_imu_rotation_deg", math.nan
                ),
                "target_update_action": target.get("action", ""),
                "target_update_pose_source": target.get("pose_source", ""),
                "target_update_scan_health_clamped": target.get(
                    "scan_health_clamped", math.nan
                ),
                "target_update_scan_correction_translation_m": target.get(
                    "scan_correction_translation_m", math.nan
                ),
                "target_update_scan_correction_rotation_deg": target.get(
                    "scan_correction_rotation_deg", math.nan
                ),
                "k2g_injected_to_frame_count": inject_by_to_frame.get(frame_id, 0),
                "k2g_active_to_frame_rows": len(active_exact),
                "k2g_active_near_frame_rows": len(active_near),
                "k2g_active_trans_residual_p50_m": active_stat(
                    "trans_norm", active_exact
                ),
                "k2g_active_nis_p50": active_stat("nis", active_exact),
                "k2g_active_cbs_hessian_p50": active_stat(
                    "cbs_hessian_frobenius", active_exact
                ),
                "k2g_active_other_local_hessian_p50": active_stat(
                    "other_local_hessian_sum", active_exact
                ),
            }
        )
    return rows


def glim_frame_timeline_event_summary(rows: Sequence[Dict[str, Any]]) -> List[Dict[str, Any]]:
    conditions = [
        (
            "points_lt_3000",
            lambda row: to_float(str(row.get("preprocessed_points", math.nan))) < 3000.0,
        ),
        (
            "scan_imu_translation_gt_0p15m",
            lambda row: to_float(str(row.get("scan_minus_imu_translation_m", math.nan))) > 0.15,
        ),
        (
            "scan_imu_rotation_gt_5deg",
            lambda row: to_float(str(row.get("scan_minus_imu_rotation_deg", math.nan))) > 5.0,
        ),
        (
            "scan_error_ratio_gt_1p05",
            lambda row: to_float(str(row.get("scan_error_ratio", math.nan))) > 1.05,
        ),
        (
            "k2g_not_active_near_frame",
            lambda row: to_float(str(row.get("k2g_active_near_frame_rows", math.nan))) <= 0.0,
        ),
    ]
    summary: List[Dict[str, Any]] = []
    for name, predicate in conditions:
        matches = [row for row in rows if predicate(row)]
        if not matches:
            summary.append({"condition": name, "count": 0})
            continue
        first = min(matches, key=lambda row: row.get("rel_sec", math.inf))
        summary.append(
            {
                "condition": name,
                "count": len(matches),
                "first_rel_sec": first.get("rel_sec", math.nan),
                "first_frame_id": first.get("frame_id", math.nan),
                "first_glim_ape_m": first.get("glim_ape_m", math.nan),
                "first_kimera_nearest_ape_m": first.get(
                    "kimera_nearest_ape_m", math.nan
                ),
                "first_preprocessed_points": first.get(
                    "preprocessed_points", math.nan
                ),
                "first_scan_imu_translation_m": first.get(
                    "scan_minus_imu_translation_m", math.nan
                ),
                "first_scan_imu_rotation_deg": first.get(
                    "scan_minus_imu_rotation_deg", math.nan
                ),
                "first_scan_error_ratio": first.get("scan_error_ratio", math.nan),
                "first_target_update_pose_source": first.get(
                    "target_update_pose_source", ""
                ),
                "first_k2g_injected_to_frame_count": first.get(
                    "k2g_injected_to_frame_count", 0
                ),
                "first_k2g_active_near_frame_rows": first.get(
                    "k2g_active_near_frame_rows", 0
                ),
            }
        )
    return summary


def write_tum_poses(path: Path, poses: PoseTrajectory) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as stream:
        for pose in sorted(poses, key=lambda item: item["t"]):
            stream.write(
                f"{pose['t']:.9f} {pose['x']:.9f} {pose['y']:.9f} "
                f"{pose['z']:.9f} {pose['qx']:.9f} {pose['qy']:.9f} "
                f"{pose['qz']:.9f} {pose['qw']:.9f}\n"
            )


def nearest_pose_by_time(poses: PoseTrajectory, query_t: float) -> Optional[PoseRow]:
    if not poses:
        return None
    times = [pose["t"] for pose in poses]
    idx = bisect.bisect_left(times, query_t)
    if idx <= 0:
        return poses[0]
    if idx >= len(poses):
        return poses[-1]
    before = poses[idx - 1]
    after = poses[idx]
    return before if abs(before["t"] - query_t) <= abs(after["t"] - query_t) else after


def interpolate_ground_truth_for_estimator(
    gt_poses: PoseTrajectory,
    estimator_poses: PoseTrajectory,
) -> Tuple[PoseTrajectory, PoseTrajectory]:
    gt_points = [(pose["t"], (pose["x"], pose["y"], pose["z"])) for pose in gt_poses]
    gt_eval: PoseTrajectory = []
    est_eval: PoseTrajectory = []
    for est_pose in estimator_poses:
        point = interpolate_trajectory(gt_points, est_pose["t"])
        nearest = nearest_pose_by_time(gt_poses, est_pose["t"])
        if point is None or nearest is None:
            continue
        gt_eval.append(
            {
                "t": est_pose["t"],
                "x": point[0],
                "y": point[1],
                "z": point[2],
                "qx": nearest["qx"],
                "qy": nearest["qy"],
                "qz": nearest["qz"],
                "qw": nearest["qw"],
            }
        )
        est_eval.append(est_pose)
    return gt_eval, est_eval


def export_tum_artifacts(run_dir: Path, gt_path: Path) -> Dict[str, Path]:
    tum_dir = run_dir / "trajectories" / "tum"
    gt_poses = load_ground_truth_poses(gt_path)
    estimators = load_estimator_odometry(run_dir)

    paths = {"ground_truth": tum_dir / "ground_truth.tum"}
    write_tum_poses(paths["ground_truth"], gt_poses)
    for estimator, poses in sorted(estimators.items()):
        paths[estimator] = tum_dir / f"{estimator}.tum"
        paths[f"ground_truth_{estimator}"] = tum_dir / f"ground_truth_for_{estimator}.tum"
        paths[f"{estimator}_eval"] = tum_dir / f"{estimator}_eval.tum"
        write_tum_poses(paths[estimator], poses)
        gt_eval, est_eval = interpolate_ground_truth_for_estimator(gt_poses, poses)
        write_tum_poses(paths[f"ground_truth_{estimator}"], gt_eval)
        write_tum_poses(paths[f"{estimator}_eval"], est_eval)
    return paths


def find_executable(name: str) -> Optional[str]:
    found = shutil.which(name)
    if found:
        return found
    pipx_candidate = Path.home() / ".local" / "bin" / name
    if pipx_candidate.exists():
        return str(pipx_candidate)
    return None


def parse_evo_stats(output: str) -> Dict[str, float]:
    stats: Dict[str, float] = {}
    stat_re = re.compile(r"^\s*(max|mean|median|min|rmse|sse|std)\s+([-+0-9.eE]+)\s*$")
    for line in strip_ansi(output).splitlines():
        match = stat_re.match(line)
        if match:
            stats[match.group(1)] = to_float(match.group(2))
    return stats


def run_evo_command(
    command: Sequence[str],
    output_path: Path,
    evo_home: Path,
) -> subprocess.CompletedProcess:
    evo_home.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env["HOME"] = str(evo_home)
    env["MPLBACKEND"] = "Agg"
    result = run_command(command, env=env)
    output_path.write_text(
        "$ " + " ".join(shlex.quote(part) for part in command) + "\n\n" + result.stdout,
        encoding="utf-8",
    )
    return result


def run_evo_metrics(run_dir: Path, gt_path: Path) -> List[Dict[str, Any]]:
    evo_ape = find_executable("evo_ape")
    evo_rpe = find_executable("evo_rpe")
    if not evo_ape or not evo_rpe:
        return [
            {
                "estimator": "all",
                "metric": "evo",
                "status": "unavailable",
                "message": "evo_ape/evo_rpe not found",
            }
        ]

    tum_paths = export_tum_artifacts(run_dir, gt_path)
    evo_dir = run_dir / "parsed" / "evo"
    evo_dir.mkdir(parents=True, exist_ok=True)
    evo_home = run_dir / ".evo_home"
    rows: List[Dict[str, Any]] = []

    estimators = sorted(
        key
        for key in tum_paths
        if key != "ground_truth"
        and not key.startswith("ground_truth_")
        and not key.endswith("_eval")
    )
    specs = [
        (
            estimator,
            tum_paths[f"ground_truth_{estimator}"],
            tum_paths[f"{estimator}_eval"],
        )
        for estimator in estimators
    ]
    for estimator, reference, estimate in specs:
        if not reference.exists() or not estimate.exists() or estimate.stat().st_size == 0:
            rows.append(
                {
                    "estimator": estimator,
                    "metric": "evo",
                    "status": "skipped",
                    "message": "missing TUM trajectory data",
                }
            )
            continue

        commands = [
            (
                "ape",
                [
                    evo_ape,
                    "tum",
                    str(reference),
                    str(estimate),
                    "--align",
                    "--pose_relation",
                    "trans_part",
                    "--no_warnings",
                    "--save_results",
                    str(evo_dir / f"ape_{estimator}.zip"),
                ],
            ),
            (
                "rpe",
                [
                    evo_rpe,
                    "tum",
                    str(reference),
                    str(estimate),
                    "--align",
                    "--pose_relation",
                    "trans_part",
                    "--delta",
                    "1",
                    "--delta_unit",
                    "f",
                    "--no_warnings",
                    "--save_results",
                    str(evo_dir / f"rpe_{estimator}.zip"),
                ],
            ),
        ]
        for metric, command in commands:
            output_path = evo_dir / f"{metric}_{estimator}.txt"
            result = run_evo_command(command, output_path, evo_home)
            stats = parse_evo_stats(result.stdout)
            row: Dict[str, Any] = {
                "estimator": estimator,
                "metric": metric,
                "status": "ok" if result.returncode == 0 else "failed",
                "returncode": result.returncode,
                "pose_relation": "trans_part",
                "alignment": "Umeyama SE3 no scale",
                "delta": "1 frame" if metric == "rpe" else "",
                "log": str(output_path),
                "result_zip": str(evo_dir / f"{metric}_{estimator}.zip"),
            }
            for key in ("rmse", "mean", "median", "std", "min", "max", "sse"):
                row[key] = stats.get(key, math.nan)
            rows.append(row)
    return rows


def counter_dict(rows: Iterable[Dict[str, Any]], key: str) -> Dict[str, int]:
    return dict(Counter(str(row.get(key, "")) for row in rows if row.get(key, "") != ""))


def direction_counts(rows: Iterable[Dict[str, Any]]) -> Dict[str, int]:
    return dict(Counter(str(row.get("direction", "")) for row in rows if row.get("direction", "")))


def sum_kimera_flow(rows: List[Dict[str, int]]) -> Dict[str, int]:
    totals: Dict[str, int] = defaultdict(int)
    for row in rows:
        for key, value in row.items():
            totals[key] += value
    return dict(totals)


def rate_summary(
    rows: List[Dict[str, Any]],
    group_key: str,
    duration_sec: float,
) -> List[Dict[str, Any]]:
    by_group: Dict[str, List[Dict[str, Any]]] = defaultdict(list)
    for row in rows:
        by_group[str(row.get(group_key, ""))].append(row)
    out: List[Dict[str, Any]] = []
    for group, group_rows in sorted(by_group.items()):
        if not group:
            continue
        count = len(group_rows)
        out.append(
            {
                group_key: group,
                "count": count,
                "approx_hz_over_bag_duration": count / duration_sec
                if duration_sec > 0.0
                else math.nan,
            }
        )
    return out


def timing_summary(rows: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    by_stage: Dict[str, List[Dict[str, Any]]] = defaultdict(list)
    for row in rows:
        by_stage[str(row.get("stage", ""))].append(row)
    summary: List[Dict[str, Any]] = []
    for stage, stage_rows in sorted(by_stage.items()):
        metric_keys = sorted(
            {
                key
                for row in stage_rows
                for key, value in row.items()
                if key.endswith("_ms") and isinstance(value, (int, float))
            }
        )
        for key in metric_keys:
            values = [float(row.get(key, math.nan)) for row in stage_rows]
            summary.append(
                {
                    "stage": stage,
                    "metric": key,
                    "count": numeric_stats(values)["count"],
                    "mean_ms": numeric_stats(values)["mean"],
                    "p50_ms": percentile(values, 0.50),
                    "p95_ms": percentile(values, 0.95),
                    "max_ms": numeric_stats(values)["max"],
                }
            )
    return summary


def glim_scan_health_summary(rows: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    if not rows:
        return []

    start_stamp = min(
        (to_float(str(row.get("stamp", math.nan))) for row in rows),
        default=math.nan,
    )

    def rel_sec(row: Dict[str, Any], key: str) -> float:
        stamp = to_float(str(row.get(key, math.nan)))
        if not math.isfinite(stamp) or not math.isfinite(start_stamp):
            return math.nan
        return stamp - start_stamp

    def rel_at_min(key: str) -> float:
        clean = [row for row in rows if math.isfinite(to_float(str(row.get(key, math.nan))))]
        if not clean:
            return math.nan
        return rel_sec(min(clean, key=lambda row: to_float(str(row.get(key, math.nan)))), "stamp")

    def rel_at_max(key: str) -> float:
        clean = [row for row in rows if math.isfinite(to_float(str(row.get(key, math.nan))))]
        if not clean:
            return math.nan
        return rel_sec(max(clean, key=lambda row: to_float(str(row.get(key, math.nan)))), "stamp")

    points = [row.get("preprocessed_points", math.nan) for row in rows]
    error_ratios = [row.get("scan_error_ratio", math.nan) for row in rows]
    scan_imu_trans = [row.get("scan_minus_imu_translation_m", math.nan) for row in rows]
    scan_imu_rot = [row.get("scan_minus_imu_rotation_deg", math.nan) for row in rows]
    h_min = [row.get("scan_hessian_min_eigenvalue", math.nan) for row in rows]
    h_cond = [row.get("scan_hessian_condition_estimate", math.nan) for row in rows]
    h_rank = [row.get("scan_hessian_rank_estimate", math.nan) for row in rows]
    health = [row.get("scan_health_clamped", math.nan) for row in rows]
    raw_health = [row.get("scan_health_raw", math.nan) for row in rows]
    effective_precision = [
        row.get("scan_health_effective_scan_precision", math.nan) for row in rows
    ]
    base_precision = [
        row.get("scan_health_base_scan_precision", math.nan) for row in rows
    ]

    return [
        {
            "count": len(rows),
            "preprocessed_points_p50": percentile(points, 0.50),
            "preprocessed_points_p05": percentile(points, 0.05),
            "preprocessed_points_min": numeric_stats(points)["min"],
            "preprocessed_points_below_3000_count": sum(
                1 for value in points if math.isfinite(value) and value < 3000.0
            ),
            "min_preprocessed_points_rel_sec": rel_at_min("preprocessed_points"),
            "scan_error_ratio_p50": percentile(error_ratios, 0.50),
            "scan_error_ratio_p95": percentile(error_ratios, 0.95),
            "scan_error_ratio_above_1_count": sum(
                1 for value in error_ratios if math.isfinite(value) and value > 1.0
            ),
            "scan_minus_imu_translation_p50_m": percentile(scan_imu_trans, 0.50),
            "scan_minus_imu_translation_p95_m": percentile(scan_imu_trans, 0.95),
            "scan_minus_imu_translation_max_m": numeric_stats(scan_imu_trans)["max"],
            "scan_minus_imu_translation_above_0p15m_count": sum(
                1 for value in scan_imu_trans if math.isfinite(value) and value > 0.15
            ),
            "max_scan_minus_imu_translation_rel_sec": rel_at_max(
                "scan_minus_imu_translation_m"
            ),
            "scan_minus_imu_rotation_p50_deg": percentile(scan_imu_rot, 0.50),
            "scan_minus_imu_rotation_p95_deg": percentile(scan_imu_rot, 0.95),
            "scan_minus_imu_rotation_max_deg": numeric_stats(scan_imu_rot)["max"],
            "scan_minus_imu_rotation_above_5deg_count": sum(
                1 for value in scan_imu_rot if math.isfinite(value) and value > 5.0
            ),
            "max_scan_minus_imu_rotation_rel_sec": rel_at_max(
                "scan_minus_imu_rotation_deg"
            ),
            "scan_hessian_min_eigenvalue_p50": percentile(h_min, 0.50),
            "scan_hessian_min_eigenvalue_min": numeric_stats(h_min)["min"],
            "scan_hessian_condition_p50": percentile(h_cond, 0.50),
            "scan_hessian_condition_p95": percentile(h_cond, 0.95),
            "scan_hessian_condition_max": numeric_stats(h_cond)["max"],
            "scan_hessian_rank_min": numeric_stats(h_rank)["min"],
            "scan_health_enabled_count": sum(
                1 for row in rows if to_int(str(row.get("scan_health_enable", 0))) != 0
            ),
            "scan_health_apply_local_count": sum(
                1
                for row in rows
                if to_int(str(row.get("scan_health_apply_to_local_scan_precision", 0))) != 0
            ),
            "scan_health_hessian_enabled_count": sum(
                1 for row in rows if to_int(str(row.get("scan_health_hessian_enable", 0))) != 0
            ),
            "scan_health_raw_p50": percentile(raw_health, 0.50),
            "scan_health_raw_p05": percentile(raw_health, 0.05),
            "scan_health_clamped_p50": percentile(health, 0.50),
            "scan_health_clamped_min": numeric_stats(health)["min"],
            "effective_scan_precision_p50": percentile(effective_precision, 0.50),
            "effective_scan_precision_min": numeric_stats(effective_precision)["min"],
            "base_scan_precision_p50": percentile(base_precision, 0.50),
        }
    ]


def glim_active_factor_detail_summary(
    rows_in: List[Dict[str, Any]]
) -> List[Dict[str, Any]]:
    groups: Dict[Tuple[str, str, str], List[Dict[str, Any]]] = defaultdict(list)
    for row in rows_in:
        groups[
            (
                str(row.get("direction", "")),
                str(row.get("category", "")),
                str(row.get("factor_type", "")),
            )
        ].append(row)

    rows: List[Dict[str, Any]] = []
    for (direction, category, factor_type), values in sorted(groups.items()):
        errors = [to_float(str(row.get("error", math.nan))) for row in values]
        hessians = [
            to_float(str(row.get("hessian_frobenius", math.nan))) for row in values
        ]
        key_counts = [to_float(str(row.get("key_count", math.nan))) for row in values]
        rows.append(
            {
                "direction": direction,
                "category": category,
                "factor_type": factor_type,
                "count": len(values),
                "key_count_p50": percentile(key_counts, 0.50),
                "error_p50": percentile(errors, 0.50),
                "error_p95": percentile(errors, 0.95),
                "error_sum": numeric_stats(errors)["sum"],
                "hessian_frobenius_p50": percentile(hessians, 0.50),
                "hessian_frobenius_p95": percentile(hessians, 0.95),
                "hessian_frobenius_sum": numeric_stats(hessians)["sum"],
            }
        )
    return rows


def glim_pose_stage_summary(rows: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    if not rows:
        return []
    return [
        {
            "count": len(rows),
            "scan_minus_imu_translation_p50_m": percentile(
                (row.get("scan_minus_imu_translation_m", math.nan) for row in rows),
                0.50,
            ),
            "scan_minus_imu_translation_p95_m": percentile(
                (row.get("scan_minus_imu_translation_m", math.nan) for row in rows),
                0.95,
            ),
            "scan_minus_imu_translation_max_m": numeric_stats(
                row.get("scan_minus_imu_translation_m", math.nan) for row in rows
            )["max"],
            "scan_minus_imu_rotation_p95_deg": percentile(
                (row.get("scan_minus_imu_rotation_deg", math.nan) for row in rows),
                0.95,
            ),
            "scan_minus_imu_rotation_max_deg": numeric_stats(
                row.get("scan_minus_imu_rotation_deg", math.nan) for row in rows
            )["max"],
            "smoother_minus_scan_translation_p50_m": percentile(
                (row.get("smoother_minus_scan_translation_m", math.nan) for row in rows),
                0.50,
            ),
            "smoother_minus_scan_translation_p95_m": percentile(
                (row.get("smoother_minus_scan_translation_m", math.nan) for row in rows),
                0.95,
            ),
            "smoother_minus_scan_translation_max_m": numeric_stats(
                row.get("smoother_minus_scan_translation_m", math.nan) for row in rows
            )["max"],
            "smoother_minus_imu_translation_p50_m": percentile(
                (row.get("smoother_minus_imu_translation_m", math.nan) for row in rows),
                0.50,
            ),
            "smoother_minus_imu_translation_p95_m": percentile(
                (row.get("smoother_minus_imu_translation_m", math.nan) for row in rows),
                0.95,
            ),
            "smoother_minus_imu_rotation_p95_deg": percentile(
                (row.get("smoother_minus_imu_rotation_deg", math.nan) for row in rows),
                0.95,
            ),
        }
    ]


def glim_active_factor_diagnostic_summary(
    rows: List[Dict[str, Any]]
) -> List[Dict[str, Any]]:
    by_direction: Dict[str, List[Dict[str, Any]]] = defaultdict(list)
    for row in rows:
        by_direction[str(row.get("direction", ""))].append(row)
    summary: List[Dict[str, Any]] = []
    for direction, values in sorted(by_direction.items()):
        summary.append(
            {
                "direction": direction,
                "count": len(values),
                "active_age_p50_sec": percentile(
                    (row.get("active_age_sec", math.nan) for row in values), 0.50
                ),
                "trans_residual_p50_m": percentile(
                    (row.get("trans_norm", math.nan) for row in values), 0.50
                ),
                "trans_residual_p95_m": percentile(
                    (row.get("trans_norm", math.nan) for row in values), 0.95
                ),
                "rot_residual_p95_rad": percentile(
                    (row.get("rot_norm", math.nan) for row in values), 0.95
                ),
                "whitened_norm_p50": percentile(
                    (row.get("whitened_norm", math.nan) for row in values), 0.50
                ),
                "nis_p50": percentile(
                    (row.get("nis", math.nan) for row in values), 0.50
                ),
                "cbs_hessian_p50": percentile(
                    (row.get("cbs_hessian_frobenius", math.nan) for row in values),
                    0.50,
                ),
                "other_local_hessian_p50": percentile(
                    (row.get("other_local_hessian_sum", math.nan) for row in values),
                    0.50,
                ),
                "other_local_hessian_p95": percentile(
                    (row.get("other_local_hessian_sum", math.nan) for row in values),
                    0.95,
                ),
            }
        )
    return summary


def covariance_summary(transport_rows: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    by_direction: Dict[str, List[Dict[str, Any]]] = defaultdict(list)
    for row in transport_rows:
        by_direction[str(row["direction"])].append(row)
    for direction, values in sorted(by_direction.items()):
        rows.append(
            {
                "direction": direction,
                "count": len(values),
                "sent_trace_mean": numeric_stats(row["sender_cov_trace"] for row in values)["mean"],
                "sent_trace_min": numeric_stats(row["sender_cov_trace"] for row in values)["min"],
                "sent_trace_max": numeric_stats(row["sender_cov_trace"] for row in values)["max"],
                "received_trace_mean": numeric_stats(row["received_cov_trace"] for row in values)["mean"],
                "received_trace_min": numeric_stats(row["received_cov_trace"] for row in values)["min"],
                "received_trace_max": numeric_stats(row["received_cov_trace"] for row in values)["max"],
                "received_min_eigen_min": numeric_stats(
                    row["received_cov_min_eigenvalue"] for row in values
                )["min"],
                "cov_error_fro_max": numeric_stats(row["cov_error_fro"] for row in values)["max"],
                "cov_symmetry_error_max": numeric_stats(
                    row["cov_symmetry_error"] for row in values
                )["max"],
            }
        )
    return rows


def marginalization_graph_summary(rows_in: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    by_group: Dict[Tuple[str, str, str], List[Dict[str, Any]]] = defaultdict(list)
    for row in rows_in:
        by_group[
            (
                str(row.get("direction", "")),
                str(row.get("robot", "")),
                str(row.get("type", "")),
            )
        ].append(row)
    for (direction, robot, graph_type), values in sorted(by_group.items()):
        rows.append(
            {
                "direction": direction,
                "robot": robot,
                "type": graph_type,
                "calls": len(values),
                "active_filter_calls": sum(1 for row in values if row.get("active_filter")),
                "tmp_marginals_calls": sum(1 for row in values if row.get("tmp_marginals")),
                "active_keys_mean": numeric_stats(
                    row.get("active_key_count") for row in values
                )["mean"],
                "input_factors_mean": numeric_stats(
                    row.get("input_factor_slots") for row in values
                )["mean"],
                "kept_factors_mean": numeric_stats(
                    row.get("kept_factor_count") for row in values
                )["mean"],
                "removed_total_mean": numeric_stats(
                    row.get("removed_total") for row in values
                )["mean"],
                "removed_outside_active_mean": numeric_stats(
                    row.get("removed_outside_active") for row in values
                )["mean"],
                "removed_tracked_cbs_factor_sum": numeric_stats(
                    row.get("removed_tracked_cbs_factor") for row in values
                )["sum"],
                "removed_anchor_belief_sum": numeric_stats(
                    row.get("removed_anchor_belief") for row in values
                )["sum"],
            }
        )
    return rows


def roundtrip_summary(roundtrip_rows: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    by_direction: Dict[str, List[Dict[str, Any]]] = defaultdict(list)
    for row in roundtrip_rows:
        by_direction[str(row["direction"])].append(row)
    for direction, values in sorted(by_direction.items()):
        rows.append(
            {
                "direction": direction,
                "count": len(values),
                "mean_pose_roundtrip_error": numeric_stats(
                    row["mean_roundtrip_error"] for row in values
                )["mean"],
                "max_pose_roundtrip_error": numeric_stats(
                    row["mean_roundtrip_error"] for row in values
                )["max"],
                "mean_cov_roundtrip_error": numeric_stats(
                    row["cov_roundtrip_error"] for row in values
                )["mean"],
                "max_cov_roundtrip_error": numeric_stats(
                    row["cov_roundtrip_error"] for row in values
                )["max"],
            }
        )
    return rows


def merge_quality_summary(merge_rows: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    by_direction_status: Dict[Tuple[str, str], List[Dict[str, Any]]] = defaultdict(list)
    for row in merge_rows:
        by_direction_status[
            (str(row.get("direction", "")), str(row.get("status", "")))
        ].append(row)

    for (direction, status), values in sorted(by_direction_status.items()):
        hellinger_values = [row.get("hellinger_local_incoming", math.nan) for row in values]
        dmu_values = [row.get("dmu_local_incoming", math.nan) for row in values]
        mahal_values = [row.get("mahal_local_incoming", math.nan) for row in values]
        step_values = [row.get("step", math.nan) for row in values]
        rows.append(
            {
                "direction": direction,
                "status": status,
                "count": len(values),
                "hellinger_count": numeric_stats(hellinger_values)["count"],
                "hellinger_p50": percentile(hellinger_values, 0.50),
                "hellinger_p95": percentile(hellinger_values, 0.95),
                "hellinger_max": numeric_stats(hellinger_values)["max"],
                "dmu_p95": percentile(dmu_values, 0.95),
                "dmu_max": numeric_stats(dmu_values)["max"],
                "mahalanobis_p95": percentile(mahal_values, 0.95),
                "mahalanobis_max": numeric_stats(mahal_values)["max"],
                "step_mean": numeric_stats(step_values)["mean"],
            }
        )
    return rows


def outgoing_filter_summary(rows_in: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    by_direction_status: Dict[Tuple[str, str], List[Dict[str, Any]]] = defaultdict(list)
    for row in rows_in:
        by_direction_status[
            (str(row.get("direction", "")), str(row.get("status", "")))
        ].append(row)

    for (direction, status), values in sorted(by_direction_status.items()):
        metric_values = [row.get("metric_distance", math.nan) for row in values]
        dmu_values = [row.get("dmu", math.nan) for row in values]
        current_trace_values = [row.get("current_trace", math.nan) for row in values]
        rows.append(
            {
                "direction": direction,
                "status": status,
                "count": len(values),
                "metric_count": numeric_stats(metric_values)["count"],
                "metric_p50": percentile(metric_values, 0.50),
                "metric_p95": percentile(metric_values, 0.95),
                "metric_max": numeric_stats(metric_values)["max"],
                "dmu_p95": percentile(dmu_values, 0.95),
                "dmu_max": numeric_stats(dmu_values)["max"],
                "current_trace_mean": numeric_stats(current_trace_values)["mean"],
                "threshold_mean": numeric_stats(row.get("threshold", math.nan) for row in values)[
                    "mean"
                ],
            }
        )
    return rows


def receiver_diagnostic_summary(rows_in: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    by_direction_status: Dict[Tuple[str, str], List[Dict[str, Any]]] = defaultdict(list)
    for row in rows_in:
        by_direction_status[
            (str(row.get("direction", "")), str(row.get("status", "")))
        ].append(row)

    for (direction, status), values in sorted(by_direction_status.items()):
        raw_values = [row.get("raw_previous_metric_distance", math.nan) for row in values]
        raw_dmu_values = [row.get("raw_previous_dmu", math.nan) for row in values]
        gbp_values = [row.get("gbp_peer_metric_distance", math.nan) for row in values]
        gbp_dmu_values = [row.get("gbp_peer_dmu", math.nan) for row in values]
        gaps = []
        for row in values:
            raw = row.get("raw_previous_metric_distance", math.nan)
            gbp = row.get("gbp_peer_metric_distance", math.nan)
            if math.isfinite(raw) and math.isfinite(gbp):
                gaps.append(gbp - raw)
        incoming_traces = [row.get("incoming_trace", math.nan) for row in values]
        rows.append(
            {
                "direction": direction,
                "status": status,
                "count": len(values),
                "raw_gate_enabled_count": sum(
                    1 for row in values if str(row.get("raw_previous_gate_enabled", "")) == "true"
                ),
                "raw_gate_checked_count": sum(
                    1 for row in values if str(row.get("raw_previous_gate_checked", "")) == "true"
                ),
                "raw_gate_rejected_count": sum(
                    1 for row in values if str(row.get("raw_previous_gate_rejected", "")) == "true"
                ),
                "raw_metric_count": numeric_stats(raw_values)["count"],
                "raw_metric_p50": percentile(raw_values, 0.50),
                "raw_metric_p95": percentile(raw_values, 0.95),
                "raw_metric_max": numeric_stats(raw_values)["max"],
                "raw_dmu_p95": percentile(raw_dmu_values, 0.95),
                "gbp_metric_count": numeric_stats(gbp_values)["count"],
                "gbp_metric_p50": percentile(gbp_values, 0.50),
                "gbp_metric_p95": percentile(gbp_values, 0.95),
                "gbp_metric_max": numeric_stats(gbp_values)["max"],
                "gbp_dmu_p95": percentile(gbp_dmu_values, 0.95),
                "gbp_minus_raw_p50": percentile(gaps, 0.50),
                "gbp_minus_raw_p95": percentile(gaps, 0.95),
                "incoming_trace_mean": numeric_stats(incoming_traces)["mean"],
            }
        )
    return rows


def temporary_linear_accounting_summary(rows_in: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    by_direction_action: Dict[Tuple[str, str], List[Dict[str, Any]]] = defaultdict(list)
    for row in rows_in:
        by_direction_action[
            (str(row.get("direction", "")), str(row.get("action", "")))
        ].append(row)

    for (direction, action), values in sorted(by_direction_action.items()):
        last_metric_values = [
            row.get("last_applied_metric_distance", math.nan) for row in values
        ]
        last_dmu_values = [row.get("last_applied_dmu", math.nan) for row in values]
        cov_rel_values = [
            row.get("last_applied_cov_rel_frobenius", math.nan) for row in values
        ]
        incoming_traces = [row.get("incoming_trace", math.nan) for row in values]
        rows.append(
            {
                "direction": direction,
                "action": action,
                "count": len(values),
                "repeated_impulse_prevented_count": sum(
                    1 for row in values if str(row.get("repeated_impulse_prevented", "")) == "true"
                ),
                "applied_incremental_count": sum(
                    1 for row in values if str(row.get("applied_incremental", "")) == "true"
                ),
                "has_last_applied_count": sum(
                    1 for row in values if str(row.get("has_last_applied", "")) == "true"
                ),
                "last_H_p50": percentile(last_metric_values, 0.50),
                "last_H_p95": percentile(last_metric_values, 0.95),
                "last_dmu_p95": percentile(last_dmu_values, 0.95),
                "last_cov_rel_p95": percentile(cov_rel_values, 0.95),
                "incoming_trace_mean": numeric_stats(incoming_traces)["mean"],
            }
        )
    return rows


def preinjection_residual_summary(rows_in: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    by_direction_action: Dict[Tuple[str, str], List[Dict[str, Any]]] = defaultdict(list)
    for row in rows_in:
        by_direction_action[
            (str(row.get("direction", "")), str(row.get("action", "")))
        ].append(row)

    for (direction, action), values in sorted(by_direction_action.items()):
        residuals = [row.get("residual_norm", math.nan) for row in values]
        rot_values = [row.get("rot_norm", math.nan) for row in values]
        trans_values = [row.get("trans_norm", math.nan) for row in values]
        yaw_values = [abs(row.get("yaw_error_deg", math.nan)) for row in values]
        trace_values = [row.get("incoming_trace", math.nan) for row in values]
        whitened_values = [row.get("whitened_norm", math.nan) for row in values]
        nis_values = [row.get("nis", math.nan) for row in values]
        rows.append(
            {
                "direction": direction,
                "action": action,
                "count": len(values),
                "residual_p50": percentile(residuals, 0.50),
                "residual_p95": percentile(residuals, 0.95),
                "residual_max": numeric_stats(residuals)["max"],
                "rot_p95": percentile(rot_values, 0.95),
                "trans_p50": percentile(trans_values, 0.50),
                "trans_p95": percentile(trans_values, 0.95),
                "trans_max": numeric_stats(trans_values)["max"],
                "abs_yaw_deg_p95": percentile(yaw_values, 0.95),
                "incoming_trace_mean": numeric_stats(trace_values)["mean"],
                "whitened_norm_p50": percentile(whitened_values, 0.50),
                "whitened_norm_p95": percentile(whitened_values, 0.95),
                "nis_p50": percentile(nis_values, 0.50),
                "nis_p95": percentile(nis_values, 0.95),
            }
        )
    return rows


def belief_odom_summary(rows_in: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    by_direction_status: Dict[Tuple[str, str], List[Dict[str, Any]]] = defaultdict(list)
    for row in rows_in:
        by_direction_status[
            (str(row.get("direction", "")), str(row.get("status", "")))
        ].append(row)

    for (direction, status), values in sorted(by_direction_status.items()):
        odom_traces = [row.get("odom_trace", math.nan) for row in values]
        rows.append(
            {
                "direction": direction,
                "status": status,
                "count": len(values),
                "odom_trace_mean": numeric_stats(odom_traces)["mean"],
                "odom_trace_p95": percentile(odom_traces, 0.95),
            }
        )
    return rows


def odom_relative_covariance_summary(rows_in: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    by_key: Dict[Tuple[str, str, str], List[Dict[str, Any]]] = defaultdict(list)
    for row in rows_in:
        by_key[
            (
                str(row.get("direction", "")),
                str(row.get("mode", "")),
                str(row.get("status", "")),
            )
        ].append(row)

    for (direction, mode, status), values in sorted(by_key.items()):
        old_traces = [row.get("trace_old_conditional", math.nan) for row in values]
        new_traces = [row.get("trace_new_schur_relative", math.nan) for row in values]
        ratios = [row.get("ratio_new_over_old", math.nan) for row in values]
        eig_old_min = [row.get("eig_old_min", math.nan) for row in values]
        eig_new_min = [row.get("eig_new_min", math.nan) for row in values]
        h_to_rank = [row.get("H_to_rank", math.nan) for row in values]
        h_to_condition = [row.get("H_to_condition_estimate", math.nan) for row in values]
        schur_direct = [
            row.get("schur_vs_direct_difference_norm", math.nan) for row in values
        ]
        jitter_count = sum(1 for row in values if row.get("jitter_used", 0))
        rows.append(
            {
                "direction": direction,
                "mode": mode,
                "status": status,
                "count": len(values),
                "trace_old_p50": percentile(old_traces, 0.50),
                "trace_new_p50": percentile(new_traces, 0.50),
                "ratio_p50": percentile(ratios, 0.50),
                "ratio_p95": percentile(ratios, 0.95),
                "eig_old_min": numeric_stats(eig_old_min)["min"],
                "eig_new_min": numeric_stats(eig_new_min)["min"],
                "H_to_rank_min": numeric_stats(h_to_rank)["min"],
                "H_to_condition_p95": percentile(h_to_condition, 0.95),
                "schur_vs_direct_norm_max": numeric_stats(schur_direct)["max"],
                "jitter_count": jitter_count,
            }
        )
    return rows


def odom_factor_covariance_summary(rows_in: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    by_direction_action: Dict[Tuple[str, str], List[Dict[str, Any]]] = defaultdict(list)
    for row in rows_in:
        by_direction_action[
            (str(row.get("direction", "")), str(row.get("action", "")))
        ].append(row)

    for (direction, action), values in sorted(by_direction_action.items()):
        traces = [row.get("trace", math.nan) for row in values]
        rot_traces = [row.get("rot_trace", math.nan) for row in values]
        trans_traces = [row.get("trans_trace", math.nan) for row in values]
        min_eigs = [row.get("min_eigenvalue", math.nan) for row in values]
        max_eigs = [row.get("max_eigenvalue", math.nan) for row in values]
        conditions = [row.get("condition", math.nan) for row in values]
        asym_values = [row.get("asym_relative", math.nan) for row in values]
        rows.append(
            {
                "direction": direction,
                "action": action,
                "count": len(values),
                "trace_p50": percentile(traces, 0.50),
                "trace_p95": percentile(traces, 0.95),
                "rot_trace_p50": percentile(rot_traces, 0.50),
                "trans_trace_p50": percentile(trans_traces, 0.50),
                "min_eigenvalue_min": numeric_stats(min_eigs)["min"],
                "max_eigenvalue_p95": percentile(max_eigs, 0.95),
                "condition_p95": percentile(conditions, 0.95),
                "asym_relative_max": numeric_stats(asym_values)["max"],
            }
        )
    return rows


def health_sender_summary(rows_in: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    by_direction_status: Dict[Tuple[str, str], List[Dict[str, Any]]] = defaultdict(list)
    for row in rows_in:
        by_direction_status[
            (str(row.get("direction", "")), str(row.get("status", "")))
        ].append(row)

    for (direction, status), values in sorted(by_direction_status.items()):
        alphas = [row.get("alpha_health", math.nan) for row in values]
        raw_traces = [row.get("raw_rel_trace", math.nan) for row in values]
        abs_traces = [row.get("abs_trace", math.nan) for row in values]
        final_traces = [row.get("final_trace", math.nan) for row in values]
        u_values = [row.get("u", math.nan) for row in values]
        u0_values = [row.get("u0", math.nan) for row in values]
        g_det_values = [row.get("g_det", math.nan) for row in values]
        rows.append(
            {
                "direction": direction,
                "status": status,
                "count": len(values),
                "alpha_p50": percentile(alphas, 0.50),
                "alpha_p95": percentile(alphas, 0.95),
                "alpha_max": numeric_stats(alphas)["max"],
                "raw_rel_trace_p50": percentile(raw_traces, 0.50),
                "raw_rel_trace_p95": percentile(raw_traces, 0.95),
                "abs_trace_p50": percentile(abs_traces, 0.50),
                "abs_trace_p95": percentile(abs_traces, 0.95),
                "abs_trace_max": numeric_stats(abs_traces)["max"],
                "final_trace_p50": percentile(final_traces, 0.50),
                "final_trace_p95": percentile(final_traces, 0.95),
                "u_p95": percentile(u_values, 0.95),
                "u0_p50": percentile(u0_values, 0.50),
                "g_det_p95": percentile(g_det_values, 0.95),
            }
        )
    return rows


def health_flow_summary(parsed: Dict[str, Any]) -> List[Dict[str, Any]]:
    directions = sorted(
        set(
            str(row.get("direction", ""))
            for key in (
                "health_sender",
                "health_nis",
                "belief_odom",
                "glim_odom_inject",
                "odom_match",
                "bpsam_odom_add",
                "odom_outgoing",
            )
            for row in parsed.get(key, [])
            if str(row.get("direction", ""))
        )
    )
    rows: List[Dict[str, Any]] = []
    applied_statuses = {
        "temporary_linear_odom_applied",
        "persistent_odom_applied",
        "active_window_temporary_odom_applied",
        "temporary_linear_queued",
    }
    for direction in directions:
        health_sender = [
            row for row in parsed.get("health_sender", [])
            if row.get("direction") == direction
        ]
        odom_outgoing = [
            row for row in parsed.get("odom_outgoing", [])
            if row.get("direction") == direction
        ]
        sent = len(health_sender) if health_sender else len(odom_outgoing)
        sender_status = Counter(
            str(row.get("status", "")) for row in health_sender
        )
        health_nis = [
            row for row in parsed.get("health_nis", [])
            if row.get("direction") == direction
        ]
        belief_odom = [
            row for row in parsed.get("belief_odom", [])
            if row.get("direction") == direction
        ]
        glim_odom_inject = [
            row for row in parsed.get("glim_odom_inject", [])
            if row.get("direction") == direction
        ]
        bpsam_odom_add = [
            row for row in parsed.get("bpsam_odom_add", [])
            if row.get("direction") == direction
        ]
        odom_match = [
            row for row in parsed.get("odom_match", [])
            if row.get("direction") == direction
        ]
        injected = sum(
            1
            for row in belief_odom
            if str(row.get("status", "")) in applied_statuses
        ) + sum(
            1
            for row in glim_odom_inject
            if str(row.get("status", "")) in applied_statuses
        )
        duplicate_refused = sum(
            1
            for row in bpsam_odom_add
            if "already_applied" in str(row.get("message", ""))
        )
        if duplicate_refused == 0:
            duplicate_refused = sum(
                1
                for row in belief_odom
                if "already_applied" in str(row.get("status", ""))
            )
        match_decisions = Counter(str(row.get("decision", "")) for row in odom_match)
        dropped = sum(
            count
            for decision, count in match_decisions.items()
            if decision.startswith("dropped") or decision == "duration_mismatch"
        )
        retry = sum(
            count
            for decision, count in match_decisions.items()
            if decision.startswith("retry")
        )
        superseded = match_decisions.get(
            "receiver_edge_superseded_by_better_candidate", 0
        )
        rows.append(
            {
                "direction": direction,
                "sent_or_produced": sent,
                "sender_ok": sender_status.get("ok", 0),
                "sender_warmup": sender_status.get("warmup", 0),
                "receiver_nis_checked": len(health_nis),
                "injected": injected,
                "duplicate_refused": duplicate_refused,
                "dropped": dropped,
                "matcher_retry": retry,
                "superseded_candidate": superseded,
                "injection_ratio_pct": 100.0 * injected / sent if sent else math.nan,
            }
        )
    return rows


def health_nis_summary(rows_in: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    by_direction_status: Dict[Tuple[str, str], List[Dict[str, Any]]] = defaultdict(list)
    for row in rows_in:
        by_direction_status[
            (str(row.get("direction", "")), str(row.get("status", "")))
        ].append(row)

    for (direction, status), values in sorted(by_direction_status.items()):
        nus = [row.get("nu", math.nan) for row in values]
        alphas = [row.get("alpha_cons", math.nan) for row in values]
        raw_alphas = [row.get("raw_alpha_cons", math.nan) for row in values]
        trust_weights = [row.get("trust_weight", math.nan) for row in values]
        trust_balances = [row.get("trust_balance", math.nan) for row in values]
        sender_growth = [row.get("sender_uncertainty_growth", math.nan) for row in values]
        receiver_growth = [row.get("receiver_uncertainty_growth", math.nan) for row in values]
        residuals = [row.get("residual_norm", math.nan) for row in values]
        trans = [row.get("trans_norm", math.nan) for row in values]
        final_traces = [row.get("final_trace", math.nan) for row in values]
        rows.append(
            {
                "direction": direction,
                "status": status,
                "count": len(values),
                "nu_p50": percentile(nus, 0.50),
                "nu_p95": percentile(nus, 0.95),
                "nu_max": numeric_stats(nus)["max"],
                "alpha_cons_p50": percentile(alphas, 0.50),
                "alpha_cons_p95": percentile(alphas, 0.95),
                "alpha_cons_max": numeric_stats(alphas)["max"],
                "raw_alpha_cons_p95": percentile(raw_alphas, 0.95),
                "trust_weight_p50": percentile(trust_weights, 0.50),
                "trust_weight_p95": percentile(trust_weights, 0.95),
                "trust_balance_p50": percentile(trust_balances, 0.50),
                "sender_growth_p50": percentile(sender_growth, 0.50),
                "receiver_growth_p50": percentile(receiver_growth, 0.50),
                "residual_p95": percentile(residuals, 0.95),
                "trans_p95": percentile(trans, 0.95),
                "final_trace_p95": percentile(final_traces, 0.95),
            }
        )
    return rows


def factor_whitened_norm_iso(row: Dict[str, Any]) -> float:
    residual = row.get("residual_norm", math.nan)
    trace = row.get("final_trace", math.nan)
    if not math.isfinite(residual) or not math.isfinite(trace) or trace <= 0.0:
        return math.nan
    return residual / math.sqrt(trace / 6.0)


def factor_whitened_trans_norm_nominal(row: Dict[str, Any]) -> float:
    trans = row.get("trans_norm", math.nan)
    trace = row.get("final_trace", math.nan)
    if not math.isfinite(trans) or not math.isfinite(trace) or trace <= 0.0:
        return math.nan
    floor_rot_trace = 3.0 * (0.017453292519943295 ** 2)
    floor_trans_trace = 3.0 * (0.03 ** 2)
    trans_fraction = floor_trans_trace / (floor_rot_trace + floor_trans_trace)
    trans_trace = trans_fraction * trace
    return trans / math.sqrt(trans_trace / 3.0)


def factor_pull_summary(rows_in: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    by_direction_status: Dict[Tuple[str, str], List[Dict[str, Any]]] = defaultdict(list)
    for row in rows_in:
        by_direction_status[
            (str(row.get("direction", "")), str(row.get("status", "")))
        ].append(row)

    for (direction, status), values in sorted(by_direction_status.items()):
        residuals = [row.get("residual_norm", math.nan) for row in values]
        nus = [row.get("nu", math.nan) for row in values]
        alphas = [row.get("alpha_cons", math.nan) for row in values]
        raw_alphas = [row.get("raw_alpha_cons", math.nan) for row in values]
        trust_weights = [row.get("trust_weight", math.nan) for row in values]
        traces = [row.get("final_trace", math.nan) for row in values]
        wr_iso = [factor_whitened_norm_iso(row) for row in values]
        wr_trans = [factor_whitened_trans_norm_nominal(row) for row in values]
        rows.append(
            {
                "direction": direction,
                "status": status,
                "count": len(values),
                "residual_p50": percentile(residuals, 0.50),
                "residual_p95": percentile(residuals, 0.95),
                "nu_p50": percentile(nus, 0.50),
                "nu_p95": percentile(nus, 0.95),
                "alpha_cons_p95": percentile(alphas, 0.95),
                "raw_alpha_cons_p95": percentile(raw_alphas, 0.95),
                "trust_weight_p50": percentile(trust_weights, 0.50),
                "trust_weight_p95": percentile(trust_weights, 0.95),
                "final_trace_p50": percentile(traces, 0.50),
                "final_trace_p95": percentile(traces, 0.95),
                "factor_wr_iso_p50": percentile(wr_iso, 0.50),
                "factor_wr_iso_p95": percentile(wr_iso, 0.95),
                "factor_wr_trans_nominal_p95": percentile(wr_trans, 0.95),
            }
        )
    return rows


def temporary_postsolve_pull_summary(
    pre_rows: List[Dict[str, Any]],
    post_rows: List[Dict[str, Any]],
) -> List[Dict[str, Any]]:
    pre_by_key: Dict[Tuple[str, str], Dict[str, Any]] = {}
    for row in pre_rows:
        direction = str(row.get("direction", ""))
        belief_key = str(row.get("belief_key", ""))
        if direction and belief_key:
            pre_by_key[(direction, belief_key)] = row

    by_direction_action: Dict[Tuple[str, str], List[Dict[str, Any]]] = defaultdict(list)
    for post in post_rows:
        direction = str(post.get("direction", ""))
        belief_key = str(post.get("belief_key", ""))
        pre = pre_by_key.get((direction, belief_key))
        if not pre:
            continue
        pre_res = pre.get("residual_norm", math.nan)
        post_res = post.get("residual_norm", math.nan)
        if not math.isfinite(pre_res) or not math.isfinite(post_res):
            continue
        action = str(post.get("action", ""))
        row = {
            "direction": direction,
            "action": action,
            "pre_residual": pre_res,
            "post_residual": post_res,
            "residual_delta": pre_res - post_res,
            "residual_ratio": post_res / pre_res if pre_res > 0.0 else math.nan,
        }
        by_direction_action[(direction, action)].append(row)

    rows: List[Dict[str, Any]] = []
    for (direction, action), values in sorted(by_direction_action.items()):
        pre_values = [row["pre_residual"] for row in values]
        post_values = [row["post_residual"] for row in values]
        deltas = [row["residual_delta"] for row in values]
        ratios = [row["residual_ratio"] for row in values]
        rows.append(
            {
                "direction": direction,
                "action": action,
                "count": len(values),
                "pre_residual_p50": percentile(pre_values, 0.50),
                "pre_residual_p95": percentile(pre_values, 0.95),
                "post_residual_p50": percentile(post_values, 0.50),
                "post_residual_p95": percentile(post_values, 0.95),
                "residual_delta_p50": percentile(deltas, 0.50),
                "residual_delta_p95": percentile(deltas, 0.95),
                "residual_ratio_p50": percentile(ratios, 0.50),
                "residual_ratio_p95": percentile(ratios, 0.95),
                "improved_pct": (
                    100.0 * sum(1 for row in values if row["residual_delta"] > 0.0)
                    / len(values)
                    if values
                    else math.nan
                ),
            }
        )
    return rows


def odom_factor_covariance_samples(
    rows_in: List[Dict[str, Any]],
    sample_limit: int = 30,
) -> List[Dict[str, Any]]:
    counts: Counter[str] = Counter()
    samples: List[Dict[str, Any]] = []
    for row in rows_in:
        direction = str(row.get("direction", ""))
        if counts[direction] >= sample_limit:
            continue
        counts[direction] += 1
        sample = dict(row)
        sample["sample"] = counts[direction]
        sample["diag6"] = ",".join(
            format_float(sample.get(f"diag{i}", math.nan), precision=6)
            for i in range(6)
        )
        samples.append(sample)
    return samples


def parse_bpsam_update_diags(message: str) -> Dict[str, Any]:
    values = [to_float(match.group(0)) for match in BPSAM_UPDATE_NUMBER_RE.finditer(message)]
    if len(values) < 36:
        return {}
    groups = [values[0:12], values[12:24], values[24:36]]
    before_diag = groups[0][6:12]
    incoming_diag = groups[1][6:12]
    after_diag = groups[2][6:12]
    return {
        "local_before_diag6": before_diag,
        "incoming_diag6": incoming_diag,
        "local_after_diag6": after_diag,
        "local_before_trace_from_diag": sum(before_diag),
        "incoming_trace_from_diag": sum(incoming_diag),
        "local_after_trace_from_diag": sum(after_diag),
    }


def format_diag6(values: Any) -> str:
    if not isinstance(values, list) or len(values) != 6:
        return "n/a"
    return "[" + ", ".join(format_float(value, precision=3) for value in values) + "]"


def diag_block_traces(values: Any) -> Tuple[float, float]:
    if not isinstance(values, list) or len(values) != 6:
        return math.nan, math.nan
    return sum(values[:3]), sum(values[3:])


def pop_matching_row(
    queues: Dict[Tuple[str, ...], List[Dict[str, Any]]],
    key: Tuple[str, ...],
) -> Optional[Dict[str, Any]]:
    rows = queues.get(key)
    if not rows:
        return None
    return rows.pop(0)


def finite_delta(after: Any, before: Any) -> float:
    try:
        after_value = float(after)
        before_value = float(before)
    except (TypeError, ValueError):
        return math.nan
    if not math.isfinite(after_value) or not math.isfinite(before_value):
        return math.nan
    return after_value - before_value


def injected_belief_covariance_samples(
    transport_rows: List[Dict[str, Any]],
    merge_rows: List[Dict[str, Any]],
    bpsam_rows: List[Dict[str, Any]],
    sample_limit: int = 40,
) -> List[Dict[str, Any]]:
    transport_by_direction_key: Dict[Tuple[str, str], List[Dict[str, Any]]] = defaultdict(list)
    for row in transport_rows:
        transport_by_direction_key[(str(row.get("direction", "")), str(row.get("key", "")))].append(row)

    accepted_merge_by_pair: Dict[Tuple[str, str, str], List[Dict[str, Any]]] = defaultdict(list)
    for row in merge_rows:
        if row.get("status") == "bpsam_added":
            accepted_merge_by_pair[
                (
                    str(row.get("direction", "")),
                    str(row.get("sender_key", "")),
                    str(row.get("receiver_key", "")),
                )
            ].append(row)

    counts: Counter[str] = Counter()
    samples: List[Dict[str, Any]] = []
    for row in bpsam_rows:
        direction = str(row.get("direction", ""))
        if row.get("status") != "accepted" or counts[direction] >= sample_limit:
            continue

        sender_key = str(row.get("sender_key", ""))
        receiver_key = str(row.get("receiver_key", ""))
        diag_info = parse_bpsam_update_diags(str(row.get("message", "")))
        if not diag_info:
            continue
        incoming_rot_trace, incoming_trans_trace = diag_block_traces(
            diag_info.get("incoming_diag6")
        )
        before_rot_trace, before_trans_trace = diag_block_traces(
            diag_info.get("local_before_diag6")
        )
        after_rot_trace, after_trans_trace = diag_block_traces(
            diag_info.get("local_after_diag6")
        )

        transport = pop_matching_row(transport_by_direction_key, (direction, sender_key))
        merge = pop_matching_row(accepted_merge_by_pair, (direction, sender_key, receiver_key))

        sent_trace = (
            merge.get("sent_trace")
            if merge
            else transport.get("sender_cov_trace")
            if transport
            else math.nan
        )
        received_trace = (
            merge.get("received_trace")
            if merge
            else transport.get("received_cov_trace")
            if transport
            else diag_info.get("incoming_trace_from_diag", math.nan)
        )
        local_before_trace = (
            merge.get("receiver_local_before_trace")
            if merge
            else diag_info.get("local_before_trace_from_diag", math.nan)
        )
        local_after_trace = (
            merge.get("receiver_merged_trace")
            if merge
            else diag_info.get("local_after_trace_from_diag", math.nan)
        )

        counts[direction] += 1
        samples.append(
            {
                "direction": direction,
                "sample": counts[direction],
                "sender_key": sender_key,
                "receiver_key": receiver_key,
                "sent_trace": sent_trace,
                "received_trace": received_trace,
                "local_before_trace": local_before_trace,
                "local_after_trace": local_after_trace,
                "local_trace_delta": finite_delta(local_after_trace, local_before_trace),
                "contraction_step_size": row.get("contraction_step_size", math.nan),
                "dxycurr": row.get("dxycurr", math.nan),
                "dxy": row.get("dxy", math.nan),
                "received_rot_trace": incoming_rot_trace,
                "received_trans_trace": incoming_trans_trace,
                "local_before_rot_trace": before_rot_trace,
                "local_before_trans_trace": before_trans_trace,
                "local_after_rot_trace": after_rot_trace,
                "local_after_trans_trace": after_trans_trace,
                "received_diag6": format_diag6(diag_info.get("incoming_diag6")),
                "local_before_diag6": format_diag6(diag_info.get("local_before_diag6")),
                "local_after_diag6": format_diag6(diag_info.get("local_after_diag6")),
                "covariance_source": "merge_row+factor_update"
                if merge
                else "transport_row+factor_update",
            }
        )

    return samples


def format_float(value: Any, precision: int = 4) -> str:
    if isinstance(value, (int, str)):
        return str(value)
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return "n/a"
    if not math.isfinite(numeric):
        return "n/a"
    if abs(numeric) >= 1000 or (0 < abs(numeric) < 0.001):
        return f"{numeric:.3e}"
    return f"{numeric:.{precision}f}"


def markdown_table(headers: Sequence[str], rows: Sequence[Sequence[Any]]) -> str:
    if not rows:
        return "_No rows._\n"
    lines = [
        "| " + " | ".join(headers) + " |",
        "| " + " | ".join("---" for _ in headers) + " |",
    ]
    for row in rows:
        lines.append("| " + " | ".join(format_float(value) for value in row) + " |")
    return "\n".join(lines) + "\n"


def load_manifest(run_dir: Path) -> Dict[str, Any]:
    path = run_dir / "manifest.json"
    if not path.exists():
        return {}
    return json.loads(path.read_text(encoding="utf-8"))


def generate_report(run_dir: Path, gt_path: Optional[Path] = None) -> Dict[str, Any]:
    manifest = load_manifest(run_dir)
    if gt_path is None:
        gt_text = manifest.get("ground_truth")
        gt_path = Path(gt_text) if gt_text else Path()

    parsed = parse_log_artifacts([run_dir / "roslaunch.log"])
    trajectory_rows = compute_trajectory_metrics(run_dir, gt_path) if gt_path else []
    trajectory_timeline_rows = (
        trajectory_error_timeline(run_dir, gt_path) if gt_path else []
    )
    glim_frame_timeline_rows = glim_frame_timeline(parsed, trajectory_timeline_rows)
    glim_frame_timeline_event_rows = glim_frame_timeline_event_summary(
        glim_frame_timeline_rows
    )
    evo_rows = run_evo_metrics(run_dir, gt_path) if gt_path else []
    cov_rows = covariance_summary(parsed["transport"])
    roundtrip_rows = roundtrip_summary(parsed["roundtrip"])
    merge_quality_rows = merge_quality_summary(parsed["merge"])
    outgoing_filter_rows = outgoing_filter_summary(parsed["outgoing_filter"])
    receiver_diagnostic_rows = receiver_diagnostic_summary(parsed["receiver_diagnostic"])
    temporary_linear_accounting_rows = temporary_linear_accounting_summary(
        parsed["temporary_linear_accounting"]
    )
    preinjection_residual_rows = preinjection_residual_summary(
        parsed["preinjection_residual"]
    )
    odom_temporary_postsolve_residual_rows = preinjection_residual_summary(
        parsed["odom_temporary_postsolve_residual"]
    )
    belief_odom_rows = belief_odom_summary(parsed["belief_odom"])
    odom_relative_covariance_rows = odom_relative_covariance_summary(
        parsed["odom_relative_covariance"]
    )
    temporary_linearization_residual_rows = preinjection_residual_summary(
        parsed["temporary_linearization_residual"]
    )
    odom_factor_covariance_rows = odom_factor_covariance_summary(
        parsed["odom_factor_covariance"]
    )
    health_sender_rows = health_sender_summary(parsed["health_sender"])
    health_nis_rows = health_nis_summary(parsed["health_nis"])
    health_flow_rows = health_flow_summary(parsed)
    factor_pull_rows = factor_pull_summary(parsed["health_nis"])
    temporary_postsolve_pull_rows = temporary_postsolve_pull_summary(
        parsed["preinjection_residual"],
        parsed["odom_temporary_postsolve_residual"],
    )
    odom_factor_covariance_sample_rows = odom_factor_covariance_samples(
        parsed["odom_factor_covariance"],
        sample_limit=30,
    )
    marginalization_graph_rows = marginalization_graph_summary(
        parsed["marginalization_graph"]
    )
    timing_summary_rows = timing_summary(parsed["timing"])
    glim_timing_summary_rows = timing_summary(parsed["glim_timing"])
    glim_scan_health_summary_rows = glim_scan_health_summary(
        parsed["glim_scan_health"]
    )
    glim_pose_stage_summary_rows = glim_pose_stage_summary(parsed["glim_pose_stage"])
    glim_active_factor_diagnostic_summary_rows = (
        glim_active_factor_diagnostic_summary(parsed["glim_active_factor_diagnostic"])
    )
    glim_active_factor_detail_summary_rows = glim_active_factor_detail_summary(
        parsed["glim_active_factor_detail"]
    )
    kimera_active_factor_diagnostic_summary_rows = (
        glim_active_factor_diagnostic_summary(parsed["kimera_active_factor_diagnostic"])
    )
    duration_sec = to_float(str(manifest.get("duration_sec", math.nan)))
    odom_outgoing_rate_rows = rate_summary(
        parsed["odom_outgoing"], "direction", duration_sec
    )
    glim_odom_inject_rate_rows = rate_summary(
        parsed["glim_odom_inject"], "direction", duration_sec
    )
    injected_cov_rows = injected_belief_covariance_samples(
        parsed["transport"],
        parsed["merge"],
        parsed["bpsam_add"],
        sample_limit=40,
    )

    artifacts_dir = run_dir / "parsed"
    artifacts_dir.mkdir(exist_ok=True)
    write_dicts_csv(artifacts_dir / "cbs_transport.csv", parsed["transport"])
    write_dicts_csv(artifacts_dir / "cbs_roundtrip.csv", parsed["roundtrip"])
    write_dicts_csv(artifacts_dir / "cbs_merge.csv", parsed["merge"])
    write_dicts_csv(
        artifacts_dir / "cbs_merge_k2l.csv",
        [row for row in parsed["merge"] if row.get("direction") == "K2L"],
    )
    write_dicts_csv(
        artifacts_dir / "cbs_merge_l2k.csv",
        [row for row in parsed["merge"] if row.get("direction") == "L2K"],
    )
    write_dicts_csv(artifacts_dir / "cbs_bpsam_add.csv", parsed["bpsam_add"])
    write_dicts_csv(artifacts_dir / "cbs_outgoing_filter.csv", parsed["outgoing_filter"])
    write_dicts_csv(
        artifacts_dir / "cbs_receiver_diagnostics.csv",
        parsed["receiver_diagnostic"],
    )
    write_dicts_csv(
        artifacts_dir / "cbs_temporary_linear_accounting.csv",
        parsed["temporary_linear_accounting"],
    )
    write_dicts_csv(
        artifacts_dir / "cbs_preinjection_residuals.csv",
        parsed["preinjection_residual"],
    )
    write_dicts_csv(
        artifacts_dir / "cbs_odom_temporary_postsolve_residuals.csv",
        parsed["odom_temporary_postsolve_residual"],
    )
    write_dicts_csv(artifacts_dir / "cbs_belief_odom.csv", parsed["belief_odom"])
    write_dicts_csv(artifacts_dir / "cbs_odom_outgoing.csv", parsed["odom_outgoing"])
    write_dicts_csv(artifacts_dir / "cbs_odom_matches.csv", parsed["odom_match"])
    write_dicts_csv(artifacts_dir / "cbs_odom_retries.csv", parsed["odom_retry"])
    write_dicts_csv(artifacts_dir / "cbs_bpsam_odom_add.csv", parsed["bpsam_odom_add"])
    write_dicts_csv(
        artifacts_dir / "glim_cbs_odom_inject.csv",
        parsed["glim_odom_inject"],
    )
    write_dicts_csv(
        artifacts_dir / "cbs_odom_factor_covariance.csv",
        parsed["odom_factor_covariance"],
    )
    write_dicts_csv(
        artifacts_dir / "cbs_health_sender.csv",
        parsed["health_sender"],
    )
    write_dicts_csv(
        artifacts_dir / "cbs_health_nis.csv",
        parsed["health_nis"],
    )
    write_dicts_csv(
        artifacts_dir / "glim_receiver_covariance.csv",
        parsed["glim_receiver_covariance"],
    )
    write_dicts_csv(
        artifacts_dir / "glim_cbs_active_factor_diagnostics.csv",
        parsed["glim_active_factor_diagnostic"],
    )
    write_dicts_csv(
        artifacts_dir / "glim_cbs_active_factor_details.csv",
        parsed["glim_active_factor_detail"],
    )
    write_dicts_csv(
        artifacts_dir / "kimera_cbs_active_factor_diagnostics.csv",
        parsed["kimera_active_factor_diagnostic"],
    )
    write_dicts_csv(
        artifacts_dir / "glim_pose_stage.csv",
        parsed["glim_pose_stage"],
    )
    write_dicts_csv(
        artifacts_dir / "glim_target_update.csv",
        parsed["glim_target_update"],
    )
    write_dicts_csv(
        artifacts_dir / "cbs_temporary_linearization_residuals.csv",
        parsed["temporary_linearization_residual"],
    )
    write_dicts_csv(artifacts_dir / "cbs_kimera_provenance.csv", parsed["provenance"])
    write_dicts_csv(
        artifacts_dir / "cbs_marginalization_graph.csv",
        parsed["marginalization_graph"],
    )
    write_dicts_csv(artifacts_dir / "kimera_flow.csv", parsed["kimera_flow"])
    write_dicts_csv(artifacts_dir / "kimera_odom_flow.csv", parsed["kimera_odom_flow"])
    write_dicts_csv(artifacts_dir / "liorf_odom_flow.csv", parsed["liorf_odom_flow"])
    write_dicts_csv(artifacts_dir / "glim_odom_flow.csv", parsed["glim_odom_flow"])
    write_dicts_csv(artifacts_dir / "kimera_timing_rows.csv", parsed["timing"])
    write_dicts_csv(artifacts_dir / "glim_timing_rows.csv", parsed["glim_timing"])
    write_dicts_csv(
        artifacts_dir / "glim_scan_health.csv",
        parsed["glim_scan_health"],
    )
    write_dicts_csv(artifacts_dir / "trajectory_metrics.csv", trajectory_rows)
    write_dicts_csv(
        artifacts_dir / "trajectory_error_timeline.csv",
        trajectory_timeline_rows,
    )
    write_dicts_csv(
        artifacts_dir / "glim_frame_timeline.csv",
        glim_frame_timeline_rows,
    )
    write_dicts_csv(
        artifacts_dir / "glim_frame_timeline_event_summary.csv",
        glim_frame_timeline_event_rows,
    )
    write_dicts_csv(artifacts_dir / "evo_metrics.csv", evo_rows)
    write_dicts_csv(artifacts_dir / "covariance_summary.csv", cov_rows)
    write_dicts_csv(artifacts_dir / "roundtrip_summary.csv", roundtrip_rows)
    write_dicts_csv(artifacts_dir / "merge_quality_summary.csv", merge_quality_rows)
    write_dicts_csv(artifacts_dir / "outgoing_filter_summary.csv", outgoing_filter_rows)
    write_dicts_csv(
        artifacts_dir / "receiver_diagnostic_summary.csv",
        receiver_diagnostic_rows,
    )
    write_dicts_csv(
        artifacts_dir / "temporary_linear_accounting_summary.csv",
        temporary_linear_accounting_rows,
    )
    write_dicts_csv(
        artifacts_dir / "preinjection_residual_summary.csv",
        preinjection_residual_rows,
    )
    write_dicts_csv(
        artifacts_dir / "odom_temporary_postsolve_residual_summary.csv",
        odom_temporary_postsolve_residual_rows,
    )
    write_dicts_csv(
        artifacts_dir / "belief_odom_summary.csv",
        belief_odom_rows,
    )
    write_dicts_csv(
        artifacts_dir / "temporary_linearization_residual_summary.csv",
        temporary_linearization_residual_rows,
    )
    write_dicts_csv(
        artifacts_dir / "odom_factor_covariance_summary.csv",
        odom_factor_covariance_rows,
    )
    write_dicts_csv(
        artifacts_dir / "cbs_health_sender_summary.csv",
        health_sender_rows,
    )
    write_dicts_csv(
        artifacts_dir / "cbs_health_nis_summary.csv",
        health_nis_rows,
    )
    write_dicts_csv(
        artifacts_dir / "cbs_health_flow_summary.csv",
        health_flow_rows,
    )
    write_dicts_csv(
        artifacts_dir / "cbs_factor_pull_summary.csv",
        factor_pull_rows,
    )
    write_dicts_csv(
        artifacts_dir / "cbs_temporary_postsolve_pull_summary.csv",
        temporary_postsolve_pull_rows,
    )
    write_dicts_csv(
        artifacts_dir / "odom_factor_covariance_samples.csv",
        odom_factor_covariance_sample_rows,
    )
    write_dicts_csv(
        artifacts_dir / "marginalization_graph_summary.csv",
        marginalization_graph_rows,
    )
    write_dicts_csv(artifacts_dir / "kimera_timing_summary.csv", timing_summary_rows)
    write_dicts_csv(artifacts_dir / "glim_timing_summary.csv", glim_timing_summary_rows)
    write_dicts_csv(
        artifacts_dir / "glim_scan_health_summary.csv",
        glim_scan_health_summary_rows,
    )
    write_dicts_csv(
        artifacts_dir / "glim_pose_stage_summary.csv",
        glim_pose_stage_summary_rows,
    )
    write_dicts_csv(
        artifacts_dir / "glim_cbs_active_factor_diagnostic_summary.csv",
        glim_active_factor_diagnostic_summary_rows,
    )
    write_dicts_csv(
        artifacts_dir / "glim_cbs_active_factor_detail_summary.csv",
        glim_active_factor_detail_summary_rows,
    )
    write_dicts_csv(
        artifacts_dir / "kimera_cbs_active_factor_diagnostic_summary.csv",
        kimera_active_factor_diagnostic_summary_rows,
    )
    write_dicts_csv(
        artifacts_dir / "odom_outgoing_rate_summary.csv",
        odom_outgoing_rate_rows,
    )
    write_dicts_csv(
        artifacts_dir / "glim_odom_inject_rate_summary.csv",
        glim_odom_inject_rate_rows,
    )
    write_dicts_csv(
        artifacts_dir / "injected_belief_covariance_samples.csv",
        injected_cov_rows,
    )

    summary: Dict[str, Any] = {
        "run_id": manifest.get("run_id", run_dir.name),
        "created_at": manifest.get("created_at"),
        "finished_at": manifest.get("finished_at"),
        "roslaunch_returncode": manifest.get("roslaunch_returncode"),
        "transport_counts": direction_counts(parsed["transport"]),
        "roundtrip_counts": direction_counts(parsed["roundtrip"]),
        "outgoing_filter_counts": direction_counts(parsed["outgoing_filter"]),
        "receiver_diagnostic_counts": direction_counts(parsed["receiver_diagnostic"]),
        "temporary_linear_accounting_counts": direction_counts(
            parsed["temporary_linear_accounting"]
        ),
        "preinjection_residual_counts": direction_counts(parsed["preinjection_residual"]),
        "belief_odom_counts": direction_counts(parsed["belief_odom"]),
        "odom_outgoing_counts": direction_counts(parsed["odom_outgoing"]),
        "health_sender_counts": direction_counts(parsed["health_sender"]),
        "health_nis_counts": direction_counts(parsed["health_nis"]),
        "odom_outgoing_rate_summary": odom_outgoing_rate_rows,
        "odom_match_decisions_by_direction": {
            direction: counter_dict(
                [row for row in parsed["odom_match"] if row.get("direction") == direction],
                "decision",
            )
            for direction in sorted(direction_counts(parsed["odom_match"]))
        },
        "odom_retry_decisions_by_direction": {
            direction: counter_dict(
                [row for row in parsed["odom_retry"] if row.get("direction") == direction],
                "decision",
            )
            for direction in sorted(direction_counts(parsed["odom_retry"]))
        },
        "bpsam_odom_add_status_by_direction": {
            direction: counter_dict(
                [row for row in parsed["bpsam_odom_add"] if row.get("direction") == direction],
                "message",
            )
            for direction in sorted(direction_counts(parsed["bpsam_odom_add"]))
        },
        "glim_odom_inject_status_by_direction": {
            direction: counter_dict(
                [
                    row
                    for row in parsed["glim_odom_inject"]
                    if row.get("direction") == direction
                ],
                "status",
            )
            for direction in sorted(direction_counts(parsed["glim_odom_inject"]))
        },
        "glim_odom_inject_rate_summary": glim_odom_inject_rate_rows,
        "temporary_linearization_residual_counts": direction_counts(
            parsed["temporary_linearization_residual"]
        ),
        "merge_status_by_direction": {
            direction: counter_dict(
                [row for row in parsed["merge"] if row.get("direction") == direction],
                "status",
            )
            for direction in sorted(direction_counts(parsed["merge"]))
        },
        "k2l_merge_status": counter_dict(
            [row for row in parsed["merge"] if row.get("direction") == "K2L"],
            "status",
        ),
        "l2k_merge_status": counter_dict(
            [row for row in parsed["merge"] if row.get("direction") == "L2K"],
            "status",
        ),
        "bpsam_add_status_by_direction": {
            direction: counter_dict(
                [row for row in parsed["bpsam_add"] if row.get("direction") == direction],
                "status",
            )
            for direction in sorted(direction_counts(parsed["bpsam_add"]))
        },
        "kimera_flow_totals": sum_kimera_flow(parsed["kimera_flow"]),
        "kimera_odom_flow_totals": sum_kimera_flow(parsed["kimera_odom_flow"]),
        "liorf_odom_flow_totals": sum_kimera_flow(parsed["liorf_odom_flow"]),
        "glim_odom_flow_totals": sum_kimera_flow(parsed["glim_odom_flow"]),
        "alignment": parsed["alignment"],
        "trajectory_metrics": trajectory_rows,
        "trajectory_error_timeline_count": len(trajectory_timeline_rows),
        "glim_frame_timeline_count": len(glim_frame_timeline_rows),
        "glim_frame_timeline_event_summary": glim_frame_timeline_event_rows,
        "evo_metrics": evo_rows,
        "covariance_summary": cov_rows,
        "roundtrip_summary": roundtrip_rows,
        "merge_quality_summary": merge_quality_rows,
        "outgoing_filter_summary": outgoing_filter_rows,
        "receiver_diagnostic_summary": receiver_diagnostic_rows,
        "temporary_linear_accounting_summary": temporary_linear_accounting_rows,
        "preinjection_residual_summary": preinjection_residual_rows,
        "odom_temporary_postsolve_residual_summary": odom_temporary_postsolve_residual_rows,
        "belief_odom_summary": belief_odom_rows,
        "odom_relative_covariance_summary": odom_relative_covariance_rows,
        "temporary_linearization_residual_summary": temporary_linearization_residual_rows,
        "odom_factor_covariance_summary": odom_factor_covariance_rows,
        "health_sender_summary": health_sender_rows,
        "health_nis_summary": health_nis_rows,
        "health_flow_summary": health_flow_rows,
        "factor_pull_summary": factor_pull_rows,
        "temporary_postsolve_pull_summary": temporary_postsolve_pull_rows,
        "odom_factor_covariance_samples": odom_factor_covariance_sample_rows,
        "marginalization_graph_summary": marginalization_graph_rows,
        "kimera_timing_summary": timing_summary_rows,
        "glim_timing_summary": glim_timing_summary_rows,
        "glim_scan_health_summary": glim_scan_health_summary_rows,
        "glim_pose_stage_summary": glim_pose_stage_summary_rows,
        "glim_active_factor_diagnostic_summary": glim_active_factor_diagnostic_summary_rows,
        "glim_active_factor_detail_summary": glim_active_factor_detail_summary_rows,
        "kimera_active_factor_diagnostic_summary": kimera_active_factor_diagnostic_summary_rows,
        "injected_belief_covariance_samples": injected_cov_rows,
        "skipped_log_rows": parsed["skipped_rows"],
    }
    write_json(run_dir / "summary.json", summary)
    (run_dir / "summary.md").write_text(render_markdown_report(run_dir, manifest, summary), encoding="utf-8")
    return summary


def render_markdown_report(run_dir: Path, manifest: Dict[str, Any], summary: Dict[str, Any]) -> str:
    launch_args = manifest.get("launch_args", {})
    git = manifest.get("git", {})
    lines: List[str] = []
    lines.append(f"# CBSMS Run Report: {summary.get('run_id', run_dir.name)}\n")
    lines.append("## Run\n")
    lines.append(f"- Run directory: `{run_dir}`")
    lines.append(f"- Created: `{summary.get('created_at')}`")
    lines.append(f"- Finished: `{summary.get('finished_at')}`")
    lines.append(f"- Roslaunch return code: `{summary.get('roslaunch_returncode')}`")
    lines.append(f"- Experiment profile: `{manifest.get('experiment_profile', 'n/a')}`")
    lines.append(f"- CBS mode preset: `{manifest.get('cbs_mode_preset') or 'custom'}`")
    lines.append(f"- Bag: `{launch_args.get('bag_path', 'n/a')}`")
    lines.append(f"- Duration: `{launch_args.get('bag_duration', 'n/a')} s`")
    lines.append(f"- Ground truth: `{manifest.get('ground_truth', 'n/a')}`")
    lines.append(f"- Rerun host: `{launch_args.get('rerun_host', 'n/a')}`\n")

    lines.append("## Git State\n")
    git_rows = []
    for name in CORE_REPOS:
        info = git.get(name, {})
        git_rows.append(
            [
                name,
                info.get("branch", "n/a"),
                info.get("commit_short", "n/a"),
                "dirty" if info.get("dirty") else "clean",
            ]
        )
    lines.append(markdown_table(["repo", "branch", "commit", "state"], git_rows))

    lines.append("## Trajectories\n")
    traj_rows = [
        [
            row["name"],
            row["samples"],
            row["reference_matches"],
            row["path_length_m"],
            row.get("reference_path_length_m", math.nan),
            row["position_rmse_m"],
            row["position_mean_error_m"],
            row["position_max_error_m"],
            row["alignment_yaw_deg"],
        ]
        for row in summary.get("trajectory_metrics", [])
    ]
    lines.append(
        markdown_table(
            [
                "metric",
                "samples",
                "matches",
                "path m",
                "ref path m",
                "rmse m",
                "mean m",
                "max m",
                "yaw deg",
            ],
            traj_rows,
        )
    )

    frame_events = summary.get("glim_frame_timeline_event_summary", [])
    if frame_events:
        lines.append("## GLIM Frame Timeline Events\n")
        lines.append(
            "First-trigger rows are derived from `parsed/glim_frame_timeline.csv`, "
            "which joins GLIM/Kimera trajectory error, scan health, target-update "
            "source, pose-stage deltas, and K2G activity per GLIM frame. Trajectory "
            "errors in this table use the same SE2 yaw+translation alignment as "
            "`trajectory_metrics.csv`.\n"
        )
        lines.append(
            markdown_table(
                [
                    "condition",
                    "count",
                    "first rel s",
                    "frame",
                    "GLIM err",
                    "Kimera err",
                    "points",
                    "scan/IMU t",
                    "scan/IMU R",
                    "err ratio",
                    "target source",
                    "K2G injected",
                    "K2G near rows",
                ],
                [
                    [
                        row.get("condition", ""),
                        row.get("count", 0),
                        row.get("first_rel_sec", math.nan),
                        row.get("first_frame_id", math.nan),
                        row.get("first_glim_ape_m", math.nan),
                        row.get("first_kimera_nearest_ape_m", math.nan),
                        row.get("first_preprocessed_points", math.nan),
                        row.get("first_scan_imu_translation_m", math.nan),
                        row.get("first_scan_imu_rotation_deg", math.nan),
                        row.get("first_scan_error_ratio", math.nan),
                        row.get("first_target_update_pose_source", ""),
                        row.get("first_k2g_injected_to_frame_count", 0),
                        row.get("first_k2g_active_near_frame_rows", 0),
                    ]
                    for row in frame_events
                ],
            )
        )

    lines.append("## Evo Metrics\n")
    evo_rows = [
        [
            row.get("estimator", ""),
            row.get("metric", ""),
            row.get("status", ""),
            row.get("rmse", math.nan),
            row.get("mean", math.nan),
            row.get("max", math.nan),
            row.get("alignment", ""),
            row.get("delta", ""),
        ]
        for row in summary.get("evo_metrics", [])
    ]
    lines.append(
        markdown_table(
            ["estimator", "metric", "status", "rmse m", "mean m", "max m", "alignment", "delta"],
            evo_rows,
        )
    )
    lines.append(
        "Evo uses TUM files in `trajectories/tum/`; ground truth is interpolated "
        "to each estimator timestamp before running translation APE/RPE.\n"
    )

    lines.append("## CBS Transport\n")
    transport = summary.get("transport_counts", {})
    roundtrip = summary.get("roundtrip_counts", {})
    lines.append(
        markdown_table(
            ["direction", "transport rows", "roundtrip rows"],
            [
                [direction, transport.get(direction, 0), roundtrip.get(direction, 0)]
                for direction in sorted(set(transport) | set(roundtrip))
            ],
        )
    )
    lines.append("- `L2K` means LiORF-side belief received by Kimera.")
    lines.append("- `G2K` means GLIM belief received by Kimera.")
    lines.append("- `K2L` means Kimera belief sent to the LIO-side receiver in legacy Kimera logs.")
    lines.append("- `K2G` means Kimera belief injected into GLIM.\n")
    skipped = summary.get("skipped_log_rows", {})
    if skipped:
        lines.append("Malformed CBS log rows skipped during parsing:\n")
        lines.append(markdown_table(["reason", "count"], sorted(skipped.items())))

    outgoing_filter = summary.get("outgoing_filter_summary", [])
    if outgoing_filter:
        lines.append("## Sender-Side Belief Filter\n")
        lines.append(
            "This compares the current outgoing belief against the last belief "
            "that same sender returned for the same receiver/key. Rows marked "
            "`filtered_similar` were omitted before publishing because their "
            "metric distance was below the sender-side similarity threshold.\n"
        )
        lines.append(
            markdown_table(
                [
                    "direction",
                    "status",
                    "count",
                    "metric n",
                    "metric p50",
                    "metric p95",
                    "metric max",
                    "dmu p95",
                    "dmu max",
                    "trace mean",
                    "threshold",
                ],
                [
                    [
                        row.get("direction", ""),
                        row.get("status", ""),
                        row.get("count", 0),
                        row.get("metric_count", 0),
                        row.get("metric_p50", math.nan),
                        row.get("metric_p95", math.nan),
                        row.get("metric_max", math.nan),
                        row.get("dmu_p95", math.nan),
                        row.get("dmu_max", math.nan),
                        row.get("current_trace_mean", math.nan),
                        row.get("threshold_mean", math.nan),
                    ]
                    for row in outgoing_filter
                ],
            )
        )

    receiver_diag = summary.get("receiver_diagnostic_summary", [])
    if receiver_diag:
        lines.append("## Receiver Belief Diagnostics\n")
        lines.append(
            "`raw` compares the current incoming belief against the previous raw "
            "incoming belief from the same sender/key. `GBP` compares the current "
            "incoming belief against the receiver's existing GBP peer belief for "
            "that sender/key before the update. If raw H is low but GBP H is high, "
            "the sender sequence is smooth while the receiver's GBP copy is stale "
            "or diverged.\n"
        )
        lines.append(
            markdown_table(
                [
                    "direction",
                    "status",
                    "count",
                    "raw gate checked",
                    "raw gate rejected",
                    "raw n",
                    "raw H p50",
                    "raw H p95",
                    "raw H max",
                    "raw dmu p95",
                    "GBP n",
                    "GBP H p50",
                    "GBP H p95",
                    "GBP H max",
                    "GBP dmu p95",
                    "GBP-raw p50",
                    "GBP-raw p95",
                    "trace mean",
                ],
                [
                    [
                        row.get("direction", ""),
                        row.get("status", ""),
                        row.get("count", 0),
                        row.get("raw_gate_checked_count", 0),
                        row.get("raw_gate_rejected_count", 0),
                        row.get("raw_metric_count", 0),
                        row.get("raw_metric_p50", math.nan),
                        row.get("raw_metric_p95", math.nan),
                        row.get("raw_metric_max", math.nan),
                        row.get("raw_dmu_p95", math.nan),
                        row.get("gbp_metric_count", 0),
                        row.get("gbp_metric_p50", math.nan),
                        row.get("gbp_metric_p95", math.nan),
                        row.get("gbp_metric_max", math.nan),
                        row.get("gbp_dmu_p95", math.nan),
                        row.get("gbp_minus_raw_p50", math.nan),
                        row.get("gbp_minus_raw_p95", math.nan),
                        row.get("incoming_trace_mean", math.nan),
                    ]
                    for row in receiver_diag
                ],
            )
        )

    temporary_linear = summary.get("temporary_linear_accounting_summary", [])
    if temporary_linear:
        lines.append("## Temporary Linear Accounting\n")
        lines.append(
            "Temporary-linear CBS odometry factors are non-persistent. This table reports "
            "which accepted beliefs were actually applied as temporary linear "
            "updates and which near-identical already-applied beliefs were "
            "skipped to prevent repeated impulses.\n"
        )
        lines.append(
            markdown_table(
                [
                    "direction",
                    "action",
                    "count",
                    "repeated prevented",
                    "incremental",
                    "has last",
                    "last H p50",
                    "last H p95",
                    "last dmu p95",
                    "last cov rel p95",
                    "trace mean",
                ],
                [
                    [
                        row.get("direction", ""),
                        row.get("action", ""),
                        row.get("count", 0),
                        row.get("repeated_impulse_prevented_count", 0),
                        row.get("applied_incremental_count", 0),
                        row.get("has_last_applied_count", 0),
                        row.get("last_H_p50", math.nan),
                        row.get("last_H_p95", math.nan),
                        row.get("last_dmu_p95", math.nan),
                        row.get("last_cov_rel_p95", math.nan),
                        row.get("incoming_trace_mean", math.nan),
                    ]
                    for row in temporary_linear
                ],
            )
        )

    belief_odom = summary.get("belief_odom_summary", [])
    if belief_odom:
        lines.append("## Belief Odometry Factors\n")
        lines.append(
            "Sender-computed belief odometry edges are matched to receiver key "
            "pairs and inserted as relative `BetweenFactor<Pose3>` constraints.\n"
        )
        lines.append(
            markdown_table(
                ["direction", "status", "count", "odom trace mean", "odom trace p95"],
                [
                    [
                        row.get("direction", ""),
                        row.get("status", ""),
                        row.get("count", 0),
                        row.get("odom_trace_mean", math.nan),
                        row.get("odom_trace_p95", math.nan),
                    ]
                    for row in belief_odom
                ],
            )
        )

    odom_rel_cov = summary.get("odom_relative_covariance_summary", [])
    if odom_rel_cov:
        lines.append("## Relative Odometry Covariance Diagnostics\n")
        lines.append(
            "Sender-side relative covariance diagnostics compare the old "
            "conditional-to-pose covariance against the Schur-relative covariance "
            "used for outgoing odometry beliefs.\n"
        )
        lines.append(
            markdown_table(
                [
                    "direction",
                    "mode",
                    "status",
                    "count",
                    "old trace p50",
                    "new trace p50",
                    "ratio p50",
                    "ratio p95",
                    "old eig min",
                    "new eig min",
                    "H_to rank min",
                    "H_to cond p95",
                    "Schur-direct max",
                    "jitter",
                ],
                [
                    [
                        row.get("direction", ""),
                        row.get("mode", ""),
                        row.get("status", ""),
                        row.get("count", 0),
                        row.get("trace_old_p50", math.nan),
                        row.get("trace_new_p50", math.nan),
                        row.get("ratio_p50", math.nan),
                        row.get("ratio_p95", math.nan),
                        row.get("eig_old_min", math.nan),
                        row.get("eig_new_min", math.nan),
                        row.get("H_to_rank_min", math.nan),
                        row.get("H_to_condition_p95", math.nan),
                        row.get("schur_vs_direct_norm_max", math.nan),
                        row.get("jitter_count", 0),
                    ]
                    for row in odom_rel_cov
                ],
            )
        )

    preinj = summary.get("preinjection_residual_summary", [])
    if preinj:
        lines.append("## Pre-Injection Residuals\n")
        lines.append(
            "These rows compare the receiver's relative motion against the incoming "
            "belief-relative motion before insertion. Translation values are in "
            "meters; yaw is reported as absolute degrees.\n"
        )
        lines.append(
            markdown_table(
                [
                    "direction",
                    "action",
                    "count",
                    "res p50",
                    "res p95",
                    "res max",
                    "rot p95",
                    "trans p50",
                    "trans p95",
                    "trans max",
                    "|yaw| p95 deg",
                    "trace mean",
                ],
                [
                    [
                        row.get("direction", ""),
                        row.get("action", ""),
                        row.get("count", 0),
                        row.get("residual_p50", math.nan),
                        row.get("residual_p95", math.nan),
                        row.get("residual_max", math.nan),
                        row.get("rot_p95", math.nan),
                        row.get("trans_p50", math.nan),
                        row.get("trans_p95", math.nan),
                        row.get("trans_max", math.nan),
                        row.get("abs_yaw_deg_p95", math.nan),
                        row.get("incoming_trace_mean", math.nan),
                    ]
                    for row in preinj
                ],
            )
        )

    postsolve = summary.get("odom_temporary_postsolve_residual_summary", [])
    if postsolve:
        lines.append("## Temporary Odometry Post-Solve Residuals\n")
        lines.append(
            "These rows compare the receiver's relative motion against the incoming "
            "CBS odometry belief after the temporary linear delta solve has been "
            "committed and cleanly relinearized. Low values here mean the temporary "
            "factor was locally satisfied during that update.\n"
        )
        lines.append(
            markdown_table(
                [
                    "direction",
                    "action",
                    "count",
                    "res p50",
                    "res p95",
                    "res max",
                    "rot p95",
                    "trans p50",
                    "trans p95",
                    "trans max",
                    "|yaw| p95 deg",
                    "trace mean",
                ],
                [
                    [
                        row.get("direction", ""),
                        row.get("action", ""),
                        row.get("count", 0),
                        row.get("residual_p50", math.nan),
                        row.get("residual_p95", math.nan),
                        row.get("residual_max", math.nan),
                        row.get("rot_p95", math.nan),
                        row.get("trans_p50", math.nan),
                        row.get("trans_p95", math.nan),
                        row.get("trans_max", math.nan),
                        row.get("abs_yaw_deg_p95", math.nan),
                        row.get("incoming_trace_mean", math.nan),
                    ]
                    for row in postsolve
                ],
            )
        )

    postsolve_pull = summary.get("temporary_postsolve_pull_summary", [])
    if postsolve_pull:
        lines.append("## Temporary CBS Pull Effect\n")
        lines.append(
            "These rows match each temporary odometry belief before insertion and "
            "after the temporary linear delta solve. Positive `delta` means the "
            "CBS residual decreased during that update; `ratio` is post/pre.\n"
        )
        lines.append(
            markdown_table(
                [
                    "direction",
                    "action",
                    "count",
                    "pre p50",
                    "post p50",
                    "delta p50",
                    "delta p95",
                    "ratio p50",
                    "ratio p95",
                    "improved %",
                ],
                [
                    [
                        row.get("direction", ""),
                        row.get("action", ""),
                        row.get("count", 0),
                        row.get("pre_residual_p50", math.nan),
                        row.get("post_residual_p50", math.nan),
                        row.get("residual_delta_p50", math.nan),
                        row.get("residual_delta_p95", math.nan),
                        row.get("residual_ratio_p50", math.nan),
                        row.get("residual_ratio_p95", math.nan),
                        row.get("improved_pct", math.nan),
                    ]
                    for row in postsolve_pull
                ],
            )
        )

    temp_lin_res = summary.get("temporary_linearization_residual_summary", [])
    if temp_lin_res:
        lines.append("## Temporary Linearization Residuals\n")
        lines.append(
            "These rows are logged inside GTSAM at the exact point where "
            "`temporaryFactorsForDelta` are linearized against `theta_`. They "
            "are the residuals seen by the temporary CBS factor in the linear "
            "delta solve, after the regular local iSAM2 update/relinearization "
            "for that cycle.\n"
        )
        lines.append(
            markdown_table(
                [
                    "direction",
                    "action",
                    "count",
                    "res p50",
                    "res p95",
                    "res max",
                    "rot p95",
                    "trans p50",
                    "trans p95",
                    "trans max",
                    "|yaw| p95 deg",
                    "trace mean",
                ],
                [
                    [
                        row.get("direction", ""),
                        row.get("action", ""),
                        row.get("count", 0),
                        row.get("residual_p50", math.nan),
                        row.get("residual_p95", math.nan),
                        row.get("residual_max", math.nan),
                        row.get("rot_p95", math.nan),
                        row.get("trans_p50", math.nan),
                        row.get("trans_p95", math.nan),
                        row.get("trans_max", math.nan),
                        row.get("abs_yaw_deg_p95", math.nan),
                        row.get("incoming_trace_mean", math.nan),
                    ]
                    for row in temp_lin_res
                ],
            )
        )

    lines.append("## Acceptance Status\n")
    merge_status = summary.get("merge_status_by_direction", {})
    direction_titles = {
        "L2K": "Kimera receiving LiORF beliefs (`L2K`)",
        "K2L": "LiORF receiving Kimera beliefs (`K2L`)",
        "G2K": "Kimera receiving GLIM beliefs (`G2K`)",
        "K2G": "GLIM receiving Kimera beliefs (`K2G`)",
    }
    for direction in ("L2K", "K2L", "G2K", "K2G"):
        counts = merge_status.get(direction, {})
        if not counts:
            continue
        lines.append(f"### {direction_titles[direction]}\n")
        lines.append(markdown_table(["status", "count"], sorted(counts.items())))

    lines.append("### BPSAM add rows\n")
    bpsam_rows = []
    for direction, counts in summary.get("bpsam_add_status_by_direction", {}).items():
        for status, count in sorted(counts.items()):
            bpsam_rows.append([direction, status, count])
    lines.append(markdown_table(["direction", "status", "count"], bpsam_rows))

    odom_outgoing = summary.get("odom_outgoing_counts", {})
    if odom_outgoing:
        lines.append("### CBS odometry outgoing rows\n")
        lines.append(markdown_table(["direction", "count"], sorted(odom_outgoing.items())))
        rate_rows = summary.get("odom_outgoing_rate_summary", [])
        if rate_rows:
            lines.append(
                markdown_table(
                    ["direction", "count", "approx Hz"],
                    [
                        [
                            row.get("direction", ""),
                            row.get("count", 0),
                            row.get("approx_hz_over_bag_duration", math.nan),
                        ]
                        for row in rate_rows
                    ],
                )
            )

    odom_match_rows = []
    for direction, counts in summary.get("odom_match_decisions_by_direction", {}).items():
        for decision, count in sorted(counts.items()):
            odom_match_rows.append([direction, decision, count])
    if odom_match_rows:
        lines.append("### CBS odometry match decisions\n")
        lines.append(markdown_table(["direction", "decision", "count"], odom_match_rows))

    odom_retry_rows = []
    for direction, counts in summary.get("odom_retry_decisions_by_direction", {}).items():
        for decision, count in sorted(counts.items()):
            odom_retry_rows.append([direction, decision, count])
    if odom_retry_rows:
        lines.append("### CBS odometry retry decisions\n")
        lines.append(markdown_table(["direction", "decision", "count"], odom_retry_rows))

    bpsam_odom_rows = []
    for direction, counts in summary.get("bpsam_odom_add_status_by_direction", {}).items():
        for message, count in sorted(counts.items()):
            bpsam_odom_rows.append([direction, message, count])
    if bpsam_odom_rows:
        lines.append("### BPSAM odometry add rows\n")
        lines.append(markdown_table(["direction", "message", "count"], bpsam_odom_rows))

    glim_inject_rows = []
    for direction, counts in summary.get("glim_odom_inject_status_by_direction", {}).items():
        for status, count in sorted(counts.items()):
            glim_inject_rows.append([direction, status, count])
    if glim_inject_rows:
        lines.append("### GLIM odometry injections\n")
        lines.append(markdown_table(["direction", "status", "count"], glim_inject_rows))
        rate_rows = summary.get("glim_odom_inject_rate_summary", [])
        if rate_rows:
            lines.append(
                markdown_table(
                    ["direction", "count", "approx Hz"],
                    [
                        [
                            row.get("direction", ""),
                            row.get("count", 0),
                            row.get("approx_hz_over_bag_duration", math.nan),
                        ]
                        for row in rate_rows
                    ],
                )
            )

    odom_factor_cov = summary.get("odom_factor_covariance_summary", [])
    if odom_factor_cov:
        lines.append("### Odometry factor covariance at insertion\n")
        lines.append(
            "These rows are logged immediately before the receiver constructs "
            "the CBS `BetweenFactor<Pose3>` noise model. They include any "
            "receiver-side covariance scale already applied.\n"
        )
        lines.append(
            markdown_table(
                [
                    "direction",
                    "action",
                    "count",
                    "trace p50",
                    "trace p95",
                    "rot tr p50",
                    "trans tr p50",
                    "min eig min",
                    "cond p95",
                    "asym max",
                ],
                [
                    [
                        row.get("direction", ""),
                        row.get("action", ""),
                        row.get("count", 0),
                        row.get("trace_p50", math.nan),
                        row.get("trace_p95", math.nan),
                        row.get("rot_trace_p50", math.nan),
                        row.get("trans_trace_p50", math.nan),
                        row.get("min_eigenvalue_min", math.nan),
                        row.get("condition_p95", math.nan),
                        row.get("asym_relative_max", math.nan),
                    ]
                    for row in odom_factor_cov
                ],
            )
        )

    health_flow = summary.get("health_flow_summary", [])
    if health_flow:
        lines.append("### CBS health belief flow\n")
        lines.append(
            "These rows summarize the health-aware odometry belief pipeline. "
            "`sent` is sender-produced beliefs, `NIS` is receiver-side "
            "consistency checks, and `injected` is accepted CBS odometry "
            "factors.\n"
        )
        lines.append(
            markdown_table(
                [
                    "direction",
                    "sent",
                    "sender ok",
                    "warmup",
                    "NIS",
                    "injected",
                    "duplicate refused",
                    "dropped",
                    "retry",
                    "superseded",
                    "inject %",
                ],
                [
                    [
                        row.get("direction", ""),
                        row.get("sent_or_produced", 0),
                        row.get("sender_ok", 0),
                        row.get("sender_warmup", 0),
                        row.get("receiver_nis_checked", 0),
                        row.get("injected", 0),
                        row.get("duplicate_refused", 0),
                        row.get("dropped", 0),
                        row.get("matcher_retry", 0),
                        row.get("superseded_candidate", 0),
                        row.get("injection_ratio_pct", math.nan),
                    ]
                    for row in health_flow
                ],
            )
        )

    health_sender = summary.get("health_sender_summary", [])
    if health_sender:
        lines.append("### Health-aware sender scaling\n")
        lines.append(
            "These rows summarize sender-side covariance scaling. "
            "`alpha > 1` means the outgoing relative belief was weakened.\n"
        )
        lines.append(
            markdown_table(
                [
                    "direction",
                    "status",
                    "count",
                    "alpha p50",
                    "alpha p95",
                    "alpha max",
                    "raw trace p95",
                    "abs trace p95",
                    "final trace p95",
                    "u p95",
                    "g_det p95",
                ],
                [
                    [
                        row.get("direction", ""),
                        row.get("status", ""),
                        row.get("count", 0),
                        row.get("alpha_p50", math.nan),
                        row.get("alpha_p95", math.nan),
                        row.get("alpha_max", math.nan),
                        row.get("raw_rel_trace_p95", math.nan),
                        row.get("abs_trace_p95", math.nan),
                        row.get("final_trace_p95", math.nan),
                        row.get("u_p95", math.nan),
                        row.get("g_det_p95", math.nan),
                    ]
                    for row in health_sender
                ],
            )
        )

    health_nis = summary.get("health_nis_summary", [])
    if health_nis:
        lines.append("### Health-aware receiver NIS\n")
        lines.append(
            "These rows summarize receiver-side innovation checks. "
            "`alpha_cons > 1` means the incoming factor covariance was inflated "
            "because the relative-pose residual was larger than expected. "
            "`trust w` is the optional relative-trust multiplier applied to "
            "that extra inflation.\n"
        )
        lines.append(
            markdown_table(
                [
                    "direction",
                    "status",
                    "count",
                    "nu p50",
                    "nu p95",
                    "nu max",
                    "alpha p50",
                    "alpha p95",
                    "alpha max",
                    "raw alpha p95",
                    "trust w p50",
                    "trust w p95",
                    "trust bal p50",
                    "sender growth p50",
                    "receiver growth p50",
                    "trans p95",
                    "final trace p95",
                ],
                [
                    [
                        row.get("direction", ""),
                        row.get("status", ""),
                        row.get("count", 0),
                        row.get("nu_p50", math.nan),
                        row.get("nu_p95", math.nan),
                        row.get("nu_max", math.nan),
                        row.get("alpha_cons_p50", math.nan),
                        row.get("alpha_cons_p95", math.nan),
                        row.get("alpha_cons_max", math.nan),
                        row.get("raw_alpha_cons_p95", math.nan),
                        row.get("trust_weight_p50", math.nan),
                        row.get("trust_weight_p95", math.nan),
                        row.get("trust_balance_p50", math.nan),
                        row.get("sender_growth_p50", math.nan),
                        row.get("receiver_growth_p50", math.nan),
                        row.get("trans_p95", math.nan),
                        row.get("final_trace_p95", math.nan),
                    ]
                    for row in health_nis
                ],
            )
        )

    factor_pull = summary.get("factor_pull_summary", [])
    if factor_pull:
        lines.append("### CBS factor pull estimate\n")
        lines.append(
            "`wr iso` estimates the residual norm whitened by the final factor "
            "covariance using `sqrt(trace/6)`. `wr trans` is a translation-only "
            "estimate using the nominal CBS pose-floor rotation/translation "
            "trace split. These are diagnostic approximations; exact whitening "
            "depends on the full 6x6 covariance.\n"
        )
        lines.append(
            markdown_table(
                [
                    "direction",
                    "status",
                    "count",
                    "res p50",
                    "res p95",
                    "nu p50",
                    "nu p95",
                    "alpha p95",
                    "raw alpha p95",
                    "trust w p50",
                    "trust w p95",
                    "wr iso p50",
                    "wr iso p95",
                    "wr trans p95",
                    "trace p50",
                    "trace p95",
                ],
                [
                    [
                        row.get("direction", ""),
                        row.get("status", ""),
                        row.get("count", 0),
                        row.get("residual_p50", math.nan),
                        row.get("residual_p95", math.nan),
                        row.get("nu_p50", math.nan),
                        row.get("nu_p95", math.nan),
                        row.get("alpha_cons_p95", math.nan),
                        row.get("raw_alpha_cons_p95", math.nan),
                        row.get("trust_weight_p50", math.nan),
                        row.get("trust_weight_p95", math.nan),
                        row.get("factor_wr_iso_p50", math.nan),
                        row.get("factor_wr_iso_p95", math.nan),
                        row.get("factor_wr_trans_nominal_p95", math.nan),
                        row.get("final_trace_p50", math.nan),
                        row.get("final_trace_p95", math.nan),
                    ]
                    for row in factor_pull
                ],
            )
        )

    odom_factor_cov_samples = summary.get("odom_factor_covariance_samples", [])
    if odom_factor_cov_samples:
        lines.append("### Odometry factor covariance samples\n")
        lines.append(
            "First 30 inserted CBS odometry factors per direction. Full rows "
            "are saved in `parsed/odom_factor_covariance_samples.csv`.\n"
        )
        for direction in ("L2K", "K2L", "G2K", "K2G"):
            rows = [
                row for row in odom_factor_cov_samples
                if row.get("direction") == direction
            ]
            if not rows:
                continue
            lines.append(f"#### {direction}\n")
            lines.append(
                markdown_table(
                    [
                        "#",
                        "edge",
                        "trace",
                        "rot/trans tr",
                        "diag6",
                        "eig min/max",
                        "asym rel",
                    ],
                    [
                        [
                            row.get("sample", ""),
                            row.get("belief_key", ""),
                            row.get("trace", math.nan),
                            (
                                f"{format_float(row.get('rot_trace', math.nan))} / "
                                f"{format_float(row.get('trans_trace', math.nan))}"
                            ),
                            row.get("diag6", ""),
                            (
                                f"{format_float(row.get('min_eigenvalue', math.nan))} / "
                                f"{format_float(row.get('max_eigenvalue', math.nan))}"
                            ),
                            row.get("asym_relative", math.nan),
                        ]
                        for row in rows
                    ],
                )
            )

    kimera_flow = summary.get("kimera_flow_totals", {})
    if kimera_flow:
        lines.append("### Kimera incoming flow totals\n")
        lines.append(markdown_table(["counter", "sum"], sorted(kimera_flow.items())))

    kimera_odom_flow = summary.get("kimera_odom_flow_totals", {})
    if kimera_odom_flow:
        lines.append("### Kimera incoming odometry flow totals\n")
        lines.append(markdown_table(["counter", "sum"], sorted(kimera_odom_flow.items())))

    liorf_odom_flow = summary.get("liorf_odom_flow_totals", {})
    if liorf_odom_flow:
        lines.append("### LiORF incoming odometry flow totals\n")
        lines.append(markdown_table(["counter", "sum"], sorted(liorf_odom_flow.items())))

    glim_odom_flow = summary.get("glim_odom_flow_totals", {})
    if glim_odom_flow:
        lines.append("### GLIM incoming odometry flow totals\n")
        lines.append(markdown_table(["counter", "sum"], sorted(glim_odom_flow.items())))

    kimera_timing = summary.get("kimera_timing_summary", [])
    if kimera_timing:
        lines.append("## Kimera Timing\n")
        lines.append(
            "Rows are parsed from Kimera timing log markers in `roslaunch.log`; "
            "full raw rows are saved in `parsed/kimera_timing_rows.csv`.\n"
        )
        lines.append(
            markdown_table(
                ["stage", "metric", "count", "mean ms", "p50 ms", "p95 ms", "max ms"],
                [
                    [
                        row.get("stage", ""),
                        row.get("metric", ""),
                        row.get("count", 0),
                        row.get("mean_ms", math.nan),
                        row.get("p50_ms", math.nan),
                        row.get("p95_ms", math.nan),
                        row.get("max_ms", math.nan),
                    ]
                    for row in kimera_timing
                ],
            )
        )

    glim_timing = summary.get("glim_timing_summary", [])
    if glim_timing:
        lines.append("## GLIM Timing\n")
        lines.append(
            "Rows are parsed from GLIM timing log markers in `roslaunch.log`; "
            "full raw rows are saved in `parsed/glim_timing_rows.csv`.\n"
        )
        rows = sorted(
            glim_timing,
            key=lambda row: float(row.get("mean_ms", 0.0) or 0.0),
            reverse=True,
        )
        lines.append(
            markdown_table(
                ["stage", "metric", "count", "mean ms", "p50 ms", "p95 ms", "max ms"],
                [
                    [
                        row.get("stage", ""),
                        row.get("metric", ""),
                        row.get("count", 0),
                        row.get("mean_ms", math.nan),
                        row.get("p50_ms", math.nan),
                        row.get("p95_ms", math.nan),
                        row.get("max_ms", math.nan),
                    ]
                    for row in rows
                ],
            )
        )

    active_factor_details = summary.get("glim_active_factor_detail_summary", [])
    if active_factor_details:
        lines.append("## GLIM Active Factor Details\n")
        lines.append(
            "Rows are parsed from `GLIM_CBS_ACTIVE_FACTOR_DETAIL_ROW`; "
            "raw rows are saved in `parsed/glim_cbs_active_factor_details.csv`.\n"
        )
        rows = sorted(
            active_factor_details,
            key=lambda row: float(row.get("hessian_frobenius_sum", 0.0) or 0.0),
            reverse=True,
        )
        lines.append(
            markdown_table(
                [
                    "direction",
                    "category",
                    "factor type",
                    "count",
                    "keys p50",
                    "err p50",
                    "err p95",
                    "H p50",
                    "H p95",
                    "H sum",
                ],
                [
                    [
                        row.get("direction", ""),
                        row.get("category", ""),
                        row.get("factor_type", ""),
                        row.get("count", 0),
                        row.get("key_count_p50", math.nan),
                        row.get("error_p50", math.nan),
                        row.get("error_p95", math.nan),
                        row.get("hessian_frobenius_p50", math.nan),
                        row.get("hessian_frobenius_p95", math.nan),
                        row.get("hessian_frobenius_sum", math.nan),
                    ]
                    for row in rows
                ],
            )
        )

    pose_stage = summary.get("glim_pose_stage_summary", [])
    if pose_stage:
        lines.append("## GLIM Pose Stage\n")
        lines.append(
            "Rows are parsed from `GLIM_POSE_STAGE_ROW`; full rows are saved in "
            "`parsed/glim_pose_stage.csv`. This compares the IMU-predicted pose, "
            "scan-matched pose, and final fixed-lag smoother pose per LiDAR frame.\n"
        )
        lines.append(
            markdown_table(
                [
                    "count",
                    "scan-IMU t p50",
                    "scan-IMU t p95",
                    "scan-IMU t max",
                    "scan-IMU R p95",
                    "scan-IMU R max",
                    "smooth-scan t p50",
                    "smooth-scan t p95",
                    "smooth-scan t max",
                    "smooth-IMU t p50",
                    "smooth-IMU t p95",
                    "smooth-IMU R p95",
                ],
                [
                    [
                        row.get("count", 0),
                        row.get("scan_minus_imu_translation_p50_m", math.nan),
                        row.get("scan_minus_imu_translation_p95_m", math.nan),
                        row.get("scan_minus_imu_translation_max_m", math.nan),
                        row.get("scan_minus_imu_rotation_p95_deg", math.nan),
                        row.get("scan_minus_imu_rotation_max_deg", math.nan),
                        row.get("smoother_minus_scan_translation_p50_m", math.nan),
                        row.get("smoother_minus_scan_translation_p95_m", math.nan),
                        row.get("smoother_minus_scan_translation_max_m", math.nan),
                        row.get("smoother_minus_imu_translation_p50_m", math.nan),
                        row.get("smoother_minus_imu_translation_p95_m", math.nan),
                        row.get("smoother_minus_imu_rotation_p95_deg", math.nan),
                    ]
                    for row in pose_stage
                ],
            )
        )

    active_factor_diag = summary.get("glim_active_factor_diagnostic_summary", [])
    if active_factor_diag:
        lines.append("## GLIM Active CBS Factor Diagnostics\n")
        lines.append(
            "Rows are parsed from `GLIM_CBS_ACTIVE_FACTOR_DIAGNOSTIC_ROW`; full "
            "rows are saved in `parsed/glim_cbs_active_factor_diagnostics.csv`. "
            "Residuals are evaluated on the post-solve GLIM smoother estimate.\n"
        )
        lines.append(
            markdown_table(
                [
                    "direction",
                    "count",
                    "age p50 s",
                    "trans res p50",
                    "trans res p95",
                    "rot res p95",
                    "white p50",
                    "NIS p50",
                    "CBS H p50",
                    "other local H p50",
                    "other local H p95",
                ],
                [
                    [
                        row.get("direction", ""),
                        row.get("count", 0),
                        row.get("active_age_p50_sec", math.nan),
                        row.get("trans_residual_p50_m", math.nan),
                        row.get("trans_residual_p95_m", math.nan),
                        row.get("rot_residual_p95_rad", math.nan),
                        row.get("whitened_norm_p50", math.nan),
                        row.get("nis_p50", math.nan),
                        row.get("cbs_hessian_p50", math.nan),
                        row.get("other_local_hessian_p50", math.nan),
                        row.get("other_local_hessian_p95", math.nan),
                    ]
                    for row in active_factor_diag
                ],
            )
        )

    kimera_active_factor_diag = summary.get(
        "kimera_active_factor_diagnostic_summary", []
    )
    if kimera_active_factor_diag:
        lines.append("## Kimera Active CBS Factor Diagnostics\n")
        lines.append(
            "Rows are parsed from `KIMERA_CBS_ACTIVE_FACTOR_DIAGNOSTIC_ROW`; full "
            "rows are saved in `parsed/kimera_cbs_active_factor_diagnostics.csv`. "
            "Residuals and Hessian buckets are evaluated on the post-solve Kimera "
            "BPSAM estimate.\n"
        )
        lines.append(
            markdown_table(
                [
                    "direction",
                    "count",
                    "age p50 s",
                    "trans res p50",
                    "trans res p95",
                    "rot res p95",
                    "white p50",
                    "NIS p50",
                    "CBS H p50",
                    "other local H p50",
                    "other local H p95",
                ],
                [
                    [
                        row.get("direction", ""),
                        row.get("count", 0),
                        row.get("active_age_p50_sec", math.nan),
                        row.get("trans_residual_p50_m", math.nan),
                        row.get("trans_residual_p95_m", math.nan),
                        row.get("rot_residual_p95_rad", math.nan),
                        row.get("whitened_norm_p50", math.nan),
                        row.get("nis_p50", math.nan),
                        row.get("cbs_hessian_p50", math.nan),
                        row.get("other_local_hessian_p50", math.nan),
                        row.get("other_local_hessian_p95", math.nan),
                    ]
                    for row in kimera_active_factor_diag
                ],
            )
        )

    glim_scan_health = summary.get("glim_scan_health_summary", [])
    if glim_scan_health:
        lines.append("## GLIM Scan Health\n")
        lines.append(
            "Rows are parsed from `GLIM_SCAN_HEALTH_ROW` in `roslaunch.log`; "
            "full raw rows are saved in `parsed/glim_scan_health.csv`. "
            "Threshold counts are diagnostics only and do not change estimator behavior.\n"
        )
        lines.append(
            markdown_table(
                [
                    "count",
                    "pts p50",
                    "pts min",
                    "pts <3000",
                    "min pts rel s",
                    "scan/IMU trans p95",
                    "scan/IMU trans max",
                    "trans >0.15m",
                    "scan/IMU rot p95",
                    "scan/IMU rot max",
                    "rot >5deg",
                    "err ratio p95",
                    "err ratio >1",
                    "H min eig min",
                    "H cond p95",
                    "H rank min",
                    "health flags E/A/H",
                    "health p50/min",
                    "effective precision p50/min",
                ],
                [
                    [
                        row.get("count", 0),
                        row.get("preprocessed_points_p50", math.nan),
                        row.get("preprocessed_points_min", math.nan),
                        row.get("preprocessed_points_below_3000_count", 0),
                        row.get("min_preprocessed_points_rel_sec", math.nan),
                        row.get("scan_minus_imu_translation_p95_m", math.nan),
                        row.get("scan_minus_imu_translation_max_m", math.nan),
                        row.get("scan_minus_imu_translation_above_0p15m_count", 0),
                        row.get("scan_minus_imu_rotation_p95_deg", math.nan),
                        row.get("scan_minus_imu_rotation_max_deg", math.nan),
                        row.get("scan_minus_imu_rotation_above_5deg_count", 0),
                        row.get("scan_error_ratio_p95", math.nan),
                        row.get("scan_error_ratio_above_1_count", 0),
                        row.get("scan_hessian_min_eigenvalue_min", math.nan),
                        row.get("scan_hessian_condition_p95", math.nan),
                        row.get("scan_hessian_rank_min", math.nan),
                        (
                            f"{row.get('scan_health_enabled_count', 0)} / "
                            f"{row.get('scan_health_apply_local_count', 0)} / "
                            f"{row.get('scan_health_hessian_enabled_count', 0)}"
                        ),
                        (
                            f"{format_float(row.get('scan_health_clamped_p50', math.nan))} / "
                            f"{format_float(row.get('scan_health_clamped_min', math.nan))}"
                        ),
                        (
                            f"{format_float(row.get('effective_scan_precision_p50', math.nan))} / "
                            f"{format_float(row.get('effective_scan_precision_min', math.nan))}"
                        ),
                    ]
                    for row in glim_scan_health
                ],
            )
        )

    merge_quality = summary.get("merge_quality_summary", [])
    if merge_quality:
        lines.append("## Merge Quality\n")
        lines.append(
            "Hellinger, Mahalanobis, and `dmu` compare the receiver's existing "
            "GBP belief for that peer/key against the incoming belief before "
            "the BPSAM update. Empty values mean there was no previous peer "
            "belief for that key or the belief was dropped before the gate.\n"
        )
        lines.append(
            markdown_table(
                [
                    "direction",
                    "status",
                    "count",
                    "H n",
                    "H p50",
                    "H p95",
                    "H max",
                    "dmu p95",
                    "dmu max",
                    "mahal p95",
                    "mahal max",
                    "step mean",
                ],
                [
                    [
                        row.get("direction", ""),
                        row.get("status", ""),
                        row.get("count", 0),
                        row.get("hellinger_count", 0),
                        row.get("hellinger_p50", math.nan),
                        row.get("hellinger_p95", math.nan),
                        row.get("hellinger_max", math.nan),
                        row.get("dmu_p95", math.nan),
                        row.get("dmu_max", math.nan),
                        row.get("mahalanobis_p95", math.nan),
                        row.get("mahalanobis_max", math.nan),
                        row.get("step_mean", math.nan),
                    ]
                    for row in merge_quality
                ],
            )
        )

    marg_graph = summary.get("marginalization_graph_summary", [])
    if marg_graph:
        lines.append("## Outgoing Covariance Graph\n")
        lines.append(
            "These rows are emitted when a sender builds the marginal graph used "
            "for outgoing belief covariance. `LOCAL` with `tmp calls == calls` "
            "means `getBeliefs()` used temporary marginals instead of the full "
            "persistent iSAM2 graph. Removed CBS/pose/anchor columns show "
            "external belief factors excluded from that covariance graph.\n"
        )
        lines.append(
            markdown_table(
                [
                    "direction",
                    "robot",
                    "type",
                    "calls",
                    "active calls",
                    "tmp calls",
                    "active keys mean",
                    "input factors mean",
                    "kept factors mean",
                    "removed outside mean",
                    "removed CBS factor sum",
                    "removed anchor sum",
                ],
                [
                    [
                        row.get("direction", ""),
                        row.get("robot", ""),
                        row.get("type", ""),
                        row.get("calls", 0),
                        row.get("active_filter_calls", 0),
                        row.get("tmp_marginals_calls", 0),
                        row.get("active_keys_mean", math.nan),
                        row.get("input_factors_mean", math.nan),
                        row.get("kept_factors_mean", math.nan),
                        row.get("removed_outside_active_mean", math.nan),
                        row.get("removed_tracked_cbs_factor_sum", math.nan),
                        row.get("removed_anchor_belief_sum", math.nan),
                    ]
                    for row in marg_graph
                ],
            )
        )

    lines.append("## Covariance\n")
    cov_rows = [
        [
            row["direction"],
            row["count"],
            row["sent_trace_mean"],
            row["received_trace_mean"],
            row["received_min_eigen_min"],
            row["cov_error_fro_max"],
            row["cov_symmetry_error_max"],
        ]
        for row in summary.get("covariance_summary", [])
    ]
    lines.append(
        markdown_table(
            [
                "direction",
                "count",
                "sent trace mean",
                "received trace mean",
                "min eig min",
                "cov err max",
                "sym err max",
            ],
            cov_rows,
        )
    )
    injected_cov = summary.get("injected_belief_covariance_samples", [])
    if injected_cov:
        lines.append("## Injected Belief Covariance Samples\n")
        lines.append(
            "First 40 accepted BPSAM updates per direction. Covariances are "
            "reported as 6D Pose3 tangent-space summaries. `trace` is the sum "
            "of the 6x6 covariance diagonal; `rot tr` sums diagonal entries "
            "0-2 and `trans tr` sums entries 3-5. "
            "`sent` is the sender-side belief covariance, `received` is after "
            "receiver-frame conversion, and `local before/after` is the "
            "receiver GBP belief for the peer/key before and after the BPSAM "
            "update. Full diagonal vectors are saved in "
            "`parsed/injected_belief_covariance_samples.csv`.\n"
        )
        for direction in ("L2K", "K2L", "G2K", "K2G"):
            rows = [row for row in injected_cov if row.get("direction") == direction]
            if not rows:
                continue
            lines.append(f"### {direction}\n")
            lines.append(
                markdown_table(
                    [
                        "#",
                        "belief",
                        "sent tr",
                        "recv tr",
                        "before tr",
                        "after tr",
                        "delta tr",
                        "recv rot/trans tr",
                        "before rot/trans tr",
                        "after rot/trans tr",
                        "GBP step",
                    ],
                    [
                        [
                            row.get("sample", ""),
                            f"{row.get('sender_key', '')} -> {row.get('receiver_key', '')}",
                            row.get("sent_trace", math.nan),
                            row.get("received_trace", math.nan),
                            row.get("local_before_trace", math.nan),
                            row.get("local_after_trace", math.nan),
                            row.get("local_trace_delta", math.nan),
                            (
                                f"{format_float(row.get('received_rot_trace', math.nan))} / "
                                f"{format_float(row.get('received_trans_trace', math.nan))}"
                            ),
                            (
                                f"{format_float(row.get('local_before_rot_trace', math.nan))} / "
                                f"{format_float(row.get('local_before_trans_trace', math.nan))}"
                            ),
                            (
                                f"{format_float(row.get('local_after_rot_trace', math.nan))} / "
                                f"{format_float(row.get('local_after_trans_trace', math.nan))}"
                            ),
                            row.get("contraction_step_size", math.nan),
                        ]
                        for row in rows
                    ],
                )
            )

    lines.append("## Roundtrip Checks\n")
    rt_rows = [
        [
            row["direction"],
            row["count"],
            row["mean_pose_roundtrip_error"],
            row["max_pose_roundtrip_error"],
            row["mean_cov_roundtrip_error"],
            row["max_cov_roundtrip_error"],
        ]
        for row in summary.get("roundtrip_summary", [])
    ]
    lines.append(
        markdown_table(
            ["direction", "count", "pose mean", "pose max", "cov mean", "cov max"],
            rt_rows,
        )
    )

    alignment = summary.get("alignment", [])
    if alignment:
        lines.append("## Visualization Alignment\n")
        lines.append(
            markdown_table(
                ["event", "nearest pose dt ms"],
                [[idx, row.get("dt_ms", math.nan)] for idx, row in enumerate(alignment, start=1)],
            )
        )

    lines.append("## Artifacts\n")
    lines.append("- Raw log: `roslaunch.log`")
    lines.append("- Raw odometry CSVs: `trajectories/`")
    lines.append("- TUM trajectories for evo: `trajectories/tum/`")
    lines.append("- Parsed CBS CSVs: `parsed/`")
    lines.append("- Evo outputs: `parsed/evo/`")
    lines.append("- Machine summary: `summary.json`\n")
    return "\n".join(lines)


def cmd_report(args: argparse.Namespace) -> None:
    run_dir = args.run_dir.resolve()
    gt_path = args.gt_path.resolve() if args.gt_path else None
    generate_report(run_dir, gt_path)
    print(run_dir / "summary.md")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    run_parser = subparsers.add_parser("run", help="launch the standard experiment and report it")
    run_parser.add_argument("--workspace", type=Path, default=workspace_root_from_script())
    run_parser.add_argument("--container-workspace", default=DEFAULT_CONTAINER_WORKSPACE)
    run_parser.add_argument("--container", default=DEFAULT_CONTAINER)
    run_parser.add_argument("--output-root", type=Path, default=None)
    run_parser.add_argument("--name", default="")
    run_parser.add_argument("--bag-path", default=DEFAULT_BAG_PATH)
    run_parser.add_argument("--gt-path", type=Path, default=None)
    run_parser.add_argument("--duration", type=float, default=60.0)
    run_parser.add_argument("--timeout-padding", type=float, default=40.0)
    run_parser.add_argument(
        "--experiment-profile",
        choices=sorted(EXPERIMENT_PROFILES),
        default="liorf_kimera",
    )
    run_parser.add_argument("--launch-package", default=None)
    run_parser.add_argument("--launch-file", default=None)
    run_parser.add_argument("--rerun-host", default=DEFAULT_RERUN_HOST)
    run_parser.add_argument(
        "--cbs-mode-preset",
        choices=sorted(CBS_MODE_PRESETS),
        default=None,
    )
    run_parser.add_argument("--extra-arg", action="append", default=[])
    run_parser.add_argument(
        "--trajectory-topic",
        action="append",
        default=[],
        help="Record an additional or replacement odometry topic as label=/topic.",
    )
    run_parser.add_argument("--clean-start", action=argparse.BooleanOptionalAction, default=True)
    run_parser.add_argument("--record-trajectories", action=argparse.BooleanOptionalAction, default=True)
    run_parser.add_argument("--enable-cbs-bridge", action=argparse.BooleanOptionalAction, default=True)
    run_parser.add_argument(
        "--shutdown-on-bag-finish",
        action=argparse.BooleanOptionalAction,
        default=False,
    )
    run_parser.add_argument("--use-kimera-rviz", action=argparse.BooleanOptionalAction, default=True)
    run_parser.add_argument("--use-liorf-rviz", action=argparse.BooleanOptionalAction, default=False)
    run_parser.add_argument("--kimera-visualize", action=argparse.BooleanOptionalAction, default=True)
    run_parser.add_argument("--rerun-visualizer-enable", action=argparse.BooleanOptionalAction, default=True)
    run_parser.add_argument("--rerun-world-alignment-enable", action=argparse.BooleanOptionalAction, default=True)

    report_parser = subparsers.add_parser("report", help="regenerate a report from an existing run dir")
    report_parser.add_argument("run_dir", type=Path)
    report_parser.add_argument("--gt-path", type=Path, default=None)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    try:
        if args.command == "run":
            run_dir = run_experiment(args)
            print(run_dir / "summary.md")
        elif args.command == "report":
            cmd_report(args)
        else:
            parser.error(f"unknown command: {args.command}")
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
