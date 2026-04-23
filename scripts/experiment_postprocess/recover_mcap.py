#!/usr/bin/env python3
"""Stream a truncated MCAP file into a new, valid MCAP.

Reads as far as the input lets us (NonSeekingReader stops cleanly when it
hits bad bytes) and writes a fresh MCAP file with the same schemas,
channels, and messages up to the truncation point. The output file has a
proper footer + index, so `rosbag2`, `rosbags`, and every other reader
can treat it as normal.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

from mcap.reader import NonSeekingReader
from mcap.writer import Writer


def recover(input_path: Path, output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    schema_map: dict[int, int] = {}   # old_id -> new_id
    channel_map: dict[int, int] = {}  # old_id -> new_id
    n_msgs = 0
    t_start = time.time()
    t_last_report = t_start

    with open(input_path, "rb") as f_in, open(output_path, "wb") as f_out:
        writer = Writer(f_out)
        writer.start(profile="ros2", library="mcap_recovery")
        reader = NonSeekingReader(f_in, validate_crcs=False)
        try:
            for schema, channel, message in reader.iter_messages(log_time_order=False):
                new_schema_id = schema_map.get(schema.id)
                if new_schema_id is None:
                    new_schema_id = writer.register_schema(
                        name=schema.name,
                        encoding=schema.encoding,
                        data=schema.data,
                    )
                    schema_map[schema.id] = new_schema_id

                new_channel_id = channel_map.get(channel.id)
                if new_channel_id is None:
                    new_channel_id = writer.register_channel(
                        topic=channel.topic,
                        message_encoding=channel.message_encoding,
                        schema_id=new_schema_id,
                        metadata=dict(channel.metadata or {}),
                    )
                    channel_map[channel.id] = new_channel_id

                writer.add_message(
                    channel_id=new_channel_id,
                    log_time=message.log_time,
                    publish_time=message.publish_time,
                    data=message.data,
                    sequence=message.sequence,
                )
                n_msgs += 1
                now = time.time()
                if now - t_last_report > 5.0:
                    mb = f_in.tell() / 1e6
                    print(
                        f"  … {n_msgs:>9} messages, {mb:>8.0f} MB read, "
                        f"{(now - t_start) / 60:.1f} min elapsed",
                        file=sys.stderr,
                    )
                    t_last_report = now
        except Exception as e:
            print(f"  recovery stopped at message {n_msgs}: {e.__class__.__name__}: {e}",
                  file=sys.stderr)
        finally:
            writer.finish()

    dur = time.time() - t_start
    in_sz = input_path.stat().st_size
    out_sz = output_path.stat().st_size
    print(
        f"recovered {n_msgs} messages in {dur/60:.1f} min  "
        f"(in {in_sz / 1e9:.1f} GB → out {out_sz / 1e9:.1f} GB)  "
        f"→ {output_path}",
        file=sys.stderr,
    )


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--input", type=Path, required=True)
    ap.add_argument("--output", type=Path, required=True)
    args = ap.parse_args()
    recover(args.input, args.output)
    return 0


if __name__ == "__main__":
    sys.exit(main())
