#!/usr/bin/env python3
"""Ingest a GEODE compressed-image stream into an existing Rerun recording."""

import argparse
import json
import struct
import sys

import rerun as rr


MAGIC = b"GEODE_RERUN_IMAGE_STREAM_V1\n"
RECORD = struct.Struct("<BQI")


def read_exact(stream, size):
    value = stream.read(size)
    if len(value) != size:
        raise EOFError(f"expected {size} bytes, received {len(value)}")
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--application-id", default="cbsms")
    parser.add_argument("--recording-id", required=True)
    destination = parser.add_mutually_exclusive_group(required=True)
    destination.add_argument("--connect")
    destination.add_argument("--output")
    args = parser.parse_args()

    source = sys.stdin.buffer
    if read_exact(source, len(MAGIC)) != MAGIC:
        raise ValueError("unsupported compressed-image stream")
    rr.init(args.application_id, recording_id=args.recording_id)
    if args.connect:
        rr.connect_grpc(args.connect)
    else:
        rr.save(args.output)
    counts = [0, 0]
    payload_bytes = [0, 0]
    while True:
        header = source.read(RECORD.size)
        if not header:
            break
        if len(header) != RECORD.size:
            raise EOFError("truncated image record header")
        channel, stamp_ns, payload_size = RECORD.unpack(header)
        if channel not in (0, 1):
            raise ValueError(f"invalid image channel {channel}")
        payload = read_exact(source, payload_size)
        rr.set_time("time", timestamp=stamp_ns / 1.0e9)
        entity = "video/left/image" if channel == 0 else "video/right/image"
        rr.log(entity, rr.EncodedImage(contents=payload, media_type="image/jpeg"))
        counts[channel] += 1
        payload_bytes[channel] += payload_size
    rr.log("video/metadata", rr.TextDocument(
        "Original GEODE compressed-image payloads; left stream sampled at "
        "5 Hz and right stream at 2 Hz for bounded passive visualization."),
        static=True)
    rr.disconnect()
    print(json.dumps({
        "left_count": counts[0],
        "right_count": counts[1],
        "left_payload_bytes": payload_bytes[0],
        "right_payload_bytes": payload_bytes[1],
    }, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
