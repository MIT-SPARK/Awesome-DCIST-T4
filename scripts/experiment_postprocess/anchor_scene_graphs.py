#!/usr/bin/env python3
"""Preprocess: compute graph -> UTM translation from a known centered/uncentered pair.

The clean scene graphs under
`/data/dcist/west_point_2026/clean_graphs/` were built by Hydra from pre-map
trajectories that were centered via `scripts/generate_ground_truth_pose.py
--center` (mean subtracted from all poses). The graph frame therefore equals
`UTM - mean`, so graph → UTM is a pure 2-D translation where `t == mean`.

We recover `t` without any bag scan by exploiting paired trajectory exports:

    reference_trajectory_centered.csv      -- the centered CSV that was fed to
                                              Hydra (position = UTM - mean)
    gps_trajectory[_max_sigma_*].csv       -- the same robot trajectory written
                                              out without centering (UTM scale)

Taking the mean of `raw[i] - centered[i]` across matching timestamps recovers
`mean` directly (within a small std coming from GPS-vs-fused-odom differences
between the two exports).

Outputs one JSON per graph under
`/home/harel/data/west_point_2026/processed/scene_graph_anchors/<stem>.json`.
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np


UTM_EPSG = "EPSG:32618"   # West Point, NY — zone 18N

# scene-graph filename -> (raw_UTM_csv, centered_csv)
# Both CSVs must be KIMERA-style (#timestamp_kf,x,y,z,qx,qy,qz,qw) for the
# same robot/session; they need not be the exact same fused run — any pair of
# raw and centered that share many timestamps averages out to the correct mean.
GRAPH_SOURCES: dict[str, tuple[str, str]] = {
    "alpha_thurs_clean_2.json": (
        "/data/dcist/west_point_2026/clean_graphs/alpha_trajectories/csv_downloads_2/gps_trajectory.csv",
        "/data/dcist/west_point_2026/clean_graphs/alpha_trajectories/csv_downloads_2/reference_trajectory_centered.csv",
    ),
    "bravo_wed_clean_bbox_corrected.json": (
        "/data/dcist/west_point_2026/clean_graphs/bravo_trajectories/csv_downloads/gps_trajectory_max_sigma_1.0.csv",
        "/data/dcist/west_point_2026/clean_graphs/bravo_trajectories/csv_downloads/reference_trajectory_centered.csv",
    ),
}

DEFAULT_OUT_DIR = Path("/home/harel/data/west_point_2026/processed/scene_graph_anchors")


def _load_xy(path: str) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return (timestamp_ns, x, y) from a KIMERA_MULTI_GT-style CSV."""
    ts, xs, ys = [], [], []
    with open(path) as f:
        reader = csv.reader(f)
        next(reader)   # header
        for row in reader:
            if not row:
                continue
            ts.append(int(row[0]))
            xs.append(float(row[1]))
            ys.append(float(row[2]))
    return np.asarray(ts), np.asarray(xs), np.asarray(ys)


def anchor_from_pair(raw_csv: str, centered_csv: str) -> dict:
    ts_u, x_u, y_u = _load_xy(raw_csv)
    ts_c, x_c, y_c = _load_xy(centered_csv)
    common, idx_u, idx_c = np.intersect1d(ts_u, ts_c, return_indices=True)
    if len(common) == 0:
        raise RuntimeError("no overlapping timestamps between raw and centered CSVs")

    dx = x_u[idx_u] - x_c[idx_c]
    dy = y_u[idx_u] - y_c[idx_c]
    t = np.array([dx.mean(), dy.mean()])
    std = np.array([dx.std(), dy.std()])
    print(f"  matched timestamps: {len(common)}/{len(ts_c)}")
    print(f"  t (UTM) = ({t[0]:.4f}, {t[1]:.4f})  std=({std[0]:.3f}, {std[1]:.3f}) m")

    return {
        "utm_epsg": UTM_EPSG,
        "R": np.eye(2).tolist(),
        "t": t.tolist(),
        "n_matched": int(len(common)),
        "offset_std_m": std.tolist(),
        "raw_csv": raw_csv,
        "centered_csv": centered_csv,
        "method": "mean_offset_between_raw_and_centered_trajectory_pair",
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--output-dir", type=Path, default=DEFAULT_OUT_DIR)
    ap.add_argument("--graph", type=str, default=None,
                    help="scene graph filename (default: all in GRAPH_SOURCES)")
    ap.add_argument("--force", action="store_true", help="overwrite existing JSONs")
    args = ap.parse_args()

    graphs = [args.graph] if args.graph else list(GRAPH_SOURCES)
    args.output_dir.mkdir(parents=True, exist_ok=True)

    for graph_name in graphs:
        if graph_name not in GRAPH_SOURCES:
            print(f"[skip] unknown graph {graph_name!r} (add to GRAPH_SOURCES)",
                  file=sys.stderr)
            continue
        raw_csv, centered_csv = GRAPH_SOURCES[graph_name]
        out_path = args.output_dir / f"{Path(graph_name).stem}.json"
        if out_path.exists() and not args.force:
            print(f"skip (exists): {out_path.name}")
            continue
        print(f"\n=== {graph_name} ===")
        try:
            anchor = anchor_from_pair(raw_csv, centered_csv)
        except Exception as e:
            print(f"  [error] {e}", file=sys.stderr)
            continue
        anchor["graph"] = graph_name
        with open(out_path, "w") as f:
            json.dump(anchor, f, indent=2)
        print(f"  wrote {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
