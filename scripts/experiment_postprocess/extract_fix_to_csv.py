#!/usr/bin/env python3
"""Extract NavSatFix messages from MCAP bags to CSV files.

One CSV per bag per robot, saved to --output-dir with the naming convention:
  <session>__<bag_dir>__fix.csv

Columns:
  timestamp_ns, latitude, longitude, altitude,
  status, cov_xx, cov_yy, cov_zz

Run with ROS 2 Jazzy workspace and spark_env sourced.
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import numpy as np

_THIS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(_THIS_DIR))
import _bag_utils as bu

# Sessions: session_folder -> robot namespace
SESSIONS = {
    "april_15_bravo_euclid":   "euclid",
    "april_15_bravo_hamilton":  "hamilton",
    "april_16_alpha_euclid":   "euclid",
    "april_16_alpha_hamilton":  "hamilton",
    "april_17_alpha_euclid":   "euclid",
    "april_17_alpha_hamilton":  "hamilton",
}

DATA_ROOT = Path("/data/dcist/west_point_2026")


def extract_fix(bag_file: Path, robot: str, out_path: Path) -> int:
    from mcap.reader import make_reader
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    NavSatFix = get_message("sensor_msgs/msg/NavSatFix")
    topic = f"/{robot}/fix"

    rows = []
    with open(bag_file, "rb") as f:
        reader = make_reader(f, validate_crcs=False)
        for _, channel, msg in reader.iter_messages(topics=[topic]):
            m = deserialize_message(msg.data, NavSatFix)
            cov = np.array(m.position_covariance).reshape(3, 3)
            rows.append([
                msg.log_time,
                m.latitude,
                m.longitude,
                m.altitude,
                m.status.status,
                cov[0, 0],
                cov[1, 1],
                cov[2, 2],
            ])

    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["timestamp_ns", "latitude", "longitude", "altitude",
                         "status", "cov_xx", "cov_yy", "cov_zz"])
        writer.writerows(rows)

    return len(rows)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--output-dir", type=Path,
                    default=Path("/home/harel/data/west_point_2026/processed/fix"),
                    help="Directory to write CSVs")
    ap.add_argument("--data-root", type=Path, default=DATA_ROOT)
    args = ap.parse_args()

    args.output_dir.mkdir(parents=True, exist_ok=True)

    for session, robot in SESSIONS.items():
        session_dir = args.data_root / session
        for bag_dir in sorted(session_dir.glob("recorded_data*")):
            if not bag_dir.is_dir():
                continue
            try:
                bag_file = bu.find_mcap(bag_dir)
            except RuntimeError as e:
                print(f"  [skip] {e}", file=sys.stderr)
                continue

            out_path = args.output_dir / f"{session}__{bag_dir.name}__fix.csv"
            if out_path.exists():
                print(f"  skip (exists): {out_path.name}")
                continue

            print(f"  {session} / {bag_dir.name} ...", end=" ", flush=True)
            n = extract_fix(bag_file, robot, out_path)
            print(f"{n} msgs → {out_path.name}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
