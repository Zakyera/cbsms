#!/usr/bin/env python3
"""Run one passive deterministic GEODE Gamma GLIM/DCReg case in Docker."""

import argparse
import hashlib
import json
import signal
import subprocess
import time
from pathlib import Path


ROOT = Path("/home/yeranis/repos/V4RL/cbs_gtsam4.3")
CONTAINER_ROOT = "/workspace/cbs_gtsam4.3"
CONTAINER = "cbsms_ws"
DEVEL = "devel"


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def docker_process(command, environment=None, **kwargs):
    invocation = ["docker", "exec"]
    for key, value in sorted((environment or {}).items()):
        invocation.extend(["-e", f"{key}={value}"])
    invocation.extend([CONTAINER, "bash", "-lc", command])
    return subprocess.Popen(invocation, **kwargs)


def docker_run(command, **kwargs):
    check = kwargs.pop("check", False)
    return subprocess.run(["docker", "exec", CONTAINER, "bash", "-lc", command],
                          check=check, **kwargs)


def stop_ros():
    for process in ("glim_rosbag", "rosbag", "rosmaster", "roscore", "rosout"):
        docker_run(f"pkill -INT {process}", stdout=subprocess.DEVNULL,
                   stderr=subprocess.DEVNULL)
    time.sleep(0.5)


def wait_master(setup, timeout=30.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if docker_run(setup + "rosparam list", stdout=subprocess.DEVNULL,
                      stderr=subprocess.DEVNULL).returncode == 0:
            return True
        time.sleep(0.1)
    return False


def marker(log_path, prefix):
    for line in reversed(Path(log_path).read_text(errors="replace").splitlines()):
        position = line.find(prefix)
        if position >= 0:
            return line[position:].split("\x1b", 1)[0]
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--name", required=True)
    parser.add_argument("--bag", required=True,
                        help="container path to deterministic PointCloud2+IMU bag")
    parser.add_argument("--config-dir", required=True,
                        help="container path to generated GLIM config")
    parser.add_argument("--duration", type=float, required=True)
    parser.add_argument("--start", type=float, default=0.0)
    parser.add_argument("--run-root", type=Path,
                        default=ROOT / "stage2c_b" / "runs")
    parser.add_argument("--thread-count", type=int, default=1)
    parser.add_argument("--factor-graph-rerun-host")
    parser.add_argument("--factor-graph-recording-id")
    parser.add_argument("--factor-graph-stride", type=int, default=5)
    args = parser.parse_args()
    if bool(args.factor_graph_rerun_host) != bool(args.factor_graph_recording_id):
        parser.error("factor-graph host and recording ID must be provided together")
    if args.factor_graph_stride < 1:
        parser.error("--factor-graph-stride must be positive")
    run_dir = args.run_root / args.name
    if run_dir.exists():
        raise SystemExit(f"refusing to overwrite run: {run_dir}")
    event_dir = run_dir / "events"
    event_dir.mkdir(parents=True)
    container_run = CONTAINER_ROOT + "/" + str(run_dir.relative_to(ROOT))
    setup = ("source /opt/ros/noetic/setup.bash; "
             f"source {CONTAINER_ROOT}/{DEVEL}/setup.bash; ")
    environment = {
        "CBSMS_RUN_DIR": container_run,
        "OMP_NUM_THREADS": str(args.thread_count),
        "OPENBLAS_NUM_THREADS": str(args.thread_count),
        "MKL_NUM_THREADS": str(args.thread_count),
        "OMP_DYNAMIC": "FALSE",
    }
    logs = {name: (run_dir / f"{name}.log").open("w")
            for name in ("roscore", "recorder", "metadata_recorder", "glim")}
    stop_ros()
    roscore = docker_process(setup + "roscore", stdout=logs["roscore"],
                             stderr=subprocess.STDOUT)
    if not wait_master(setup):
        raise SystemExit("ROS master did not become ready")
    docker_run(setup + "rosparam set /use_sim_time true", check=True)
    recorder = docker_process(
        setup + f"python3 {CONTAINER_ROOT}/stage16_diagnosis/tools/"
        f"stage16_event_recorder.py --output-dir {container_run}/events",
        stdout=logs["recorder"], stderr=subprocess.STDOUT)
    capture_bag = f"{container_run}/belief_and_metadata.bag"
    metadata_recorder = docker_process(
        setup + "rosbag record __name:=geode_stage2c_b_recorder "
        f"-O {capture_bag} /kimera/cbs/odom_belief_in "
        "/glim/cbs/dcreg_edge_health_metadata",
        stdout=logs["metadata_recorder"], stderr=subprocess.STDOUT)
    time.sleep(2.0)
    event_csv = f"{container_run}/events/consumed_input_events.csv"
    params = [
        f"_config_path:={args.config_dir}", "_auto_quit:=true",
        "_deterministic_replay_enable:=true",
        "_deterministic_replay_wait_timeout_sec:=180.0",
        f"_deterministic_input_event_log_path:={event_csv}",
        f"_bag_start_offset_sec:={args.start}",
        f"_bag_duration_sec:={args.duration}",
        "_glim_cbs_bridge_enable:=true", "_glim_cbs_mode:=observe_only",
        "_glim_cbs_outgoing_marginal_source:=direct",
        "_glim_cbs_odom_sender_mode:=time_horizon_window",
        "_glim_cbs_odom_covariance_mode:=schur_relative_between",
        "_glim_cbs_odom_horizon_sec:=0.2",
        "_glim_cbs_odom_horizon_tolerance_sec:=0.06",
        "_glim_cbs_odom_max_horizon_pairs_per_update:=6",
        "_glim_cbs_odom_belief_out_topic:=/kimera/cbs/odom_belief_in",
        "_glim_cbs_odom_belief_in_topic:=/geode/stage2c_b/no_k_to_g",
        "_glim_cbs_odometry_topic:=/glim/cbs/odometry",
        "_glim_cbs_belief_receive_start_delay_sec:=999999.0",
        "_glim_cbs_rerun_visualizer_enable:=false",
        "_glim_factor_graph_inspector_enable:=" +
        ("true" if args.factor_graph_rerun_host else "false"),
        "_glim_dcreg_belief_shadow_enable:=false", "_glim_timing_enable:=true",
    ]
    if args.factor_graph_rerun_host:
        params.extend([
            f"_glim_factor_graph_inspector_host:={args.factor_graph_rerun_host}",
            f"_glim_factor_graph_inspector_recording_id:={args.factor_graph_recording_id}",
            f"_glim_factor_graph_inspector_stride:={args.factor_graph_stride}",
            "_glim_factor_graph_inspector_context_windows:=2",
            f"_glim_dcreg_visualization_stride:={args.factor_graph_stride}",
        ])
    command = setup + "rosrun glim_ros glim_rosbag " + args.bag + " " + " ".join(params)
    started = time.monotonic()
    glim = docker_process(command, environment=environment, stdout=logs["glim"],
                          stderr=subprocess.STDOUT)
    timed_out = False
    timeout = max(900.0, args.duration * 8.0)
    try:
        glim_code = glim.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        timed_out = True
        glim.send_signal(signal.SIGINT)
        try:
            glim_code = glim.wait(timeout=30.0)
        except subprocess.TimeoutExpired:
            glim.terminate()
            glim_code = glim.wait(timeout=10.0)
    elapsed = time.monotonic() - started
    for node in ("/stage15_event_recorder", "/geode_stage2c_b_recorder"):
        docker_run(setup + f"rosnode kill {node}", stdout=subprocess.DEVNULL,
                   stderr=subprocess.DEVNULL)
    for process in (recorder, metadata_recorder):
        try:
            process.wait(timeout=30.0)
        except subprocess.TimeoutExpired:
            process.send_signal(signal.SIGINT)
            process.wait(timeout=10.0)
    stop_ros()
    for stream in logs.values():
        stream.close()
    config_host = Path(args.config_dir.replace(CONTAINER_ROOT, str(ROOT)))
    result = {
        "schema_version": 1, "name": args.name, "bag": args.bag,
        "bag_start_sec": args.start, "bag_duration_sec": args.duration,
        "config_dir": args.config_dir,
        "config_generation_manifest_sha256": sha256(config_host / "generation_manifest.json"),
        "glim_returncode": glim_code, "timed_out": timed_out,
        "elapsed_wall_sec": elapsed,
        "incoming_k_to_g_topic": "/geode/stage2c_b/no_k_to_g",
        "cbs_mode": "observe_only", "outgoing_marginal_source": "direct",
        "belief_capture_sha256": sha256(run_dir / "belief_and_metadata.bag")
        if (run_dir / "belief_and_metadata.bag").is_file() else None,
        "health_logger_stats": marker(run_dir / "glim.log", "GLIM_DCREG_LOGGER_STATS_ROW,"),
        "health_worker_stats": marker(run_dir / "glim.log", "GLIM_DCREG_WORKER_STATS_ROW,"),
        "metadata_stats": marker(run_dir / "glim.log", "GLIM_DCREG_EDGE_METADATA_STATS_ROW,"),
        "parameters": params,
        "factors_remaining": ["LiDAR matching", "IMU", "fixed-lag marginal prior"],
        "active_health_or_covariance_policy": False,
        "factor_graph_rerun_host": args.factor_graph_rerun_host,
        "factor_graph_recording_id": args.factor_graph_recording_id,
        "factor_graph_stride": args.factor_graph_stride,
    }
    (run_dir / "run_manifest.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if glim_code == 0 and not timed_out else 1


if __name__ == "__main__":
    raise SystemExit(main())
