#!/usr/bin/env python3
"""Send or save the canonical Newer College CBS-off Rerun dashboard."""

import argparse
import sys
from pathlib import Path

import rerun as rr


TOOLS = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS))
from geode_stage3a_rerun import make_blueprint  # noqa: E402


SEQUENCE_LABELS = {
    "quad-easy": "Quad-Easy",
    "quad-medium": "Quad-Medium",
    "quad-hard": "Quad-Hard",
    "stairs": "Stairs",
    "cloister": "Cloister",
    "park": "Park",
    "maths-easy": "Maths-Easy",
    "maths-medium": "Maths-Medium",
    "maths-hard": "Maths-Hard",
}


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Apply the canonical synchronized GLIM + Kimera + GT Newer College "
            "CBS-off Rerun layout to one recording."
        )
    )
    parser.add_argument("--recording-id", required=True)
    parser.add_argument("--sequence", choices=SEQUENCE_LABELS, required=True)
    parser.add_argument("--duration-sec", type=float, required=True)
    parser.add_argument("--application-id", default="cbsms")
    parser.add_argument("--recording-name")
    parser.add_argument(
        "--presentation-root",
        default="",
        help=(
            "Entity root containing synchronized presentation data. Use an "
            "empty value for live/raw entities and 'presentation_v2' for the "
            "canonical post-run overlay."
        ),
    )
    parser.add_argument("--scene-extent", type=float, default=50.0)
    parser.add_argument("--active-tab-index", type=int, default=1)
    destination = parser.add_mutually_exclusive_group(required=True)
    destination.add_argument(
        "--connect",
        help="Existing Rerun gRPC endpoint, for example rerun+http://127.0.0.1:9876/proxy.",
    )
    destination.add_argument("--output", type=Path, help="Write a standalone .rbl file.")
    args = parser.parse_args()

    if args.duration_sec <= 0.0:
        parser.error("--duration-sec must be positive")
    if args.scene_extent <= 0.0:
        parser.error("--scene-extent must be positive")

    sequence_label = SEQUENCE_LABELS[args.sequence]
    duration_label = f"{args.duration_sec:g} s"
    cbs_label = (
        f"CBS off — Newer College {sequence_label} {duration_label} — synchronized"
    )
    recording_name = args.recording_name or cbs_label

    rr.init(args.application_id, recording_id=args.recording_id)
    if args.connect:
        rr.connect_grpc(args.connect)
    else:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        rr.save(str(args.output))

    rr.send_recording_name(recording_name)
    rr.send_blueprint(
        make_blueprint(
            (0.0, 0.0, 0.0),
            args.scene_extent,
            cbs_label=cbs_label,
            include_covariance_audit=True,
            presentation_root=args.presentation_root,
            active_tab_index=args.active_tab_index,
        )
    )
    rr.disconnect()

    destination_label = args.connect or str(args.output)
    print(f"Newer College CBS-off blueprint sent to {destination_label}")
    print(f"recording_id={args.recording_id}")
    print(f"label={cbs_label}")


if __name__ == "__main__":
    main()
