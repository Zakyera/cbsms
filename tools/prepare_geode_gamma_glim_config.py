#!/usr/bin/env python3
"""Create a GEODE Gamma runtime config from the frozen Stage 2A template."""

import argparse
import hashlib
import json
import re
import shutil
from pathlib import Path


def replace_once(text, pattern, replacement, label):
    updated, count = re.subn(pattern, replacement, text, count=1, flags=re.S)
    if count != 1:
        raise ValueError(f"expected one {label} field, found {count}")
    return updated


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def prepare(profile_path, output_dir, health_mode):
    profile = json.loads(Path(profile_path).read_text(encoding="utf-8"))
    base = Path(profile["base_config_template"].replace(
        "/workspace/cbs_gtsam4.3", "/home/yeranis/repos/V4RL/cbs_gtsam4.3"))
    output = Path(output_dir)
    if output.exists() and any(output.iterdir()):
        raise ValueError(f"refusing to overwrite non-empty config directory: {output}")
    output.mkdir(parents=True, exist_ok=True)
    for source in sorted(base.glob("*.json")):
        shutil.copy2(source, output / source.name)

    sensors_path = output / "config_sensors.json"
    sensors = sensors_path.read_text(encoding="utf-8")
    values = profile["imu_noise"]
    for key in ("imu_acc_noise", "imu_gyro_noise", "imu_int_noise", "imu_bias_noise"):
        sensors = replace_once(
            sensors,
            rf'("{key}"\s*:\s*)[-+0-9.eE]+',
            rf'\g<1>{values[key]:.17g}',
            key,
        )
    sensors = sensors.replace(
        "// Native MID360 IMU profile. This is not the D435i camera IMU used by Kimera.",
        "// GEODE Gamma Xsens/Livox IMU profile from the hashed official calibration.")
    sensors = sensors.replace(
        "// The local M3DGR bags do not include a MID360-to-camera-IMU extrinsic.\n"
        "    // Keep this profile in the native MID360 lidar/IMU frame.",
        "// T_lidar_imu is inverse(T_IMU_LiDAR) from the hashed Gamma calibration.")
    tum = ",\n      ".join(f"{value:.17g}" for value in profile["T_L_I_glim_tum"])
    sensors = replace_once(
        sensors,
        r'("T_lidar_imu"\s*:\s*)\[[^\]]*\]',
        rf'\g<1>[\n      {tum}\n    ]',
        "T_lidar_imu",
    )
    sensors_path.write_text(sensors, encoding="utf-8")

    ros_path = output / "config_ros.json"
    ros = ros_path.read_text(encoding="utf-8")
    substitutions = {
        "imu_time_offset": 0.0,
        "points_time_offset": 0.0,
        "acc_scale": profile["acc_scale"],
    }
    for key, value in substitutions.items():
        ros = replace_once(ros, rf'("{key}"\s*:\s*)[-+0-9.eE]+',
                           rf'\g<1>{value:.17g}', key)
    ros = ros.replace(
        "// M3DGR MID360 acceleration is stored in g.",
        "// GEODE Gamma Xsens acceleration is stored in m/s^2.")
    strings = {
        "imu_frame_id": profile["imu_frame_id"],
        "lidar_frame_id": profile["lidar_frame_id"],
        "base_frame_id": profile["imu_frame_id"],
        "imu_topic": profile["imu_topic"],
        "points_topic": profile["pointcloud_topic"],
    }
    for key, value in strings.items():
        ros = replace_once(ros, rf'("{key}"\s*:\s*)"[^"]*"',
                           rf'\g<1>"{value}"', key)
    ros = replace_once(ros, r'("extension_modules"\s*:\s*)\[[^\]]*\]',
                       r'\g<1>[]', "extension_modules")
    ros_path.write_text(ros, encoding="utf-8")

    odometry_path = output / "config_odometry_cpu.json"
    odometry = odometry_path.read_text(encoding="utf-8")
    odometry = replace_once(odometry, r'("mode"\s*:\s*)"log_only"',
                            rf'\g<1>"{health_mode}"', "dcreg_health.mode")
    odometry = replace_once(
        odometry,
        r'("sensor_identifier"\s*:\s*)"[^"]*"',
        rf'\g<1>"{profile["sensor_identifier"]}"',
        "sensor_identifier",
    )
    if health_mode == "off":
        marker = '"dcreg_edge_metadata"'
        position = odometry.find(marker)
        if position < 0:
            raise ValueError("missing dcreg_edge_metadata section")
        prefix, suffix = odometry[:position], odometry[position:]
        suffix = replace_once(suffix, r'("enabled"\s*:\s*)true',
                              r'\g<1>false', "edge metadata enabled")
        odometry = prefix + suffix
    odometry_path.write_text(odometry, encoding="utf-8")

    manifest = {
        "schema_version": 1,
        "profile": str(Path(profile_path).resolve()),
        "profile_sha256": sha256(profile_path),
        "base_template": str(base),
        "health_mode": health_mode,
        "generated_files": {
            path.name: sha256(path) for path in sorted(output.glob("*.json"))
        },
    }
    manifest_path = output / "generation_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n",
                             encoding="utf-8")
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--health-mode", choices=("off", "log_only"), required=True)
    args = parser.parse_args()
    print(json.dumps(prepare(args.profile, args.output_dir, args.health_mode),
                     indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
