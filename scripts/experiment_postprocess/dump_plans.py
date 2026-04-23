#!/usr/bin/env python3
"""Dump compiled_plan_out messages for each experiment to a text file.

Reads experiments.csv, finds the plan message nearest to each experiment's
start time, and writes viz_out/<experiment_id>/plan.txt with a human-readable
summary of each robot's action sequence.

Run with the ROS 2 Jazzy workspace sourced and spark_env python on PATH.
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


def _format_path(path_msg) -> str:
    pts = path_msg.poses
    if not pts:
        return "(no waypoints)"
    coords = [(p.pose.position.x, p.pose.position.y) for p in pts]
    summary = ", ".join(f"({x:.1f},{y:.1f})" for x, y in coords)
    return f"{len(coords)} waypoints: {summary}"


def _format_action(i: int, a) -> str:
    t = a.action_type
    if t == "FOLLOW":
        detail = _format_path(a.path)
    elif t == "GAZE":
        gp = a.gaze_point
        rp = a.robot_point
        detail = (f"frame={a.gaze_frame}  "
                  f"robot=({rp.x:.1f},{rp.y:.1f},{rp.z:.1f})  "
                  f"gaze=({gp.x:.1f},{gp.y:.1f},{gp.z:.1f})")
    elif t == "PICK":
        op = a.object_point
        detail = (f"class={a.object_class}  id={a.object_id}  "
                  f"frame={a.pick_frame}  "
                  f"pos=({op.x:.1f},{op.y:.1f},{op.z:.1f})")
    elif t == "PLACE":
        detail = f"frame={a.place_frame}"
    else:
        detail = ""
    return f"  {i+1:>2}. {t:<8}  {detail}"


def dump_plan_for_experiment(row: dict, viz_root: Path) -> None:
    exp_id = row["experiment_id"]
    out_path = viz_root / exp_id / "plan.txt"
    out_path.parent.mkdir(parents=True, exist_ok=True)

    t_start = float(row["start_time_unix"])
    robots = [r for r in row.get("robots", "").split(bu.LIST_SEP) if r]
    suffix = "/omniplanner_node/compiled_plan_out"

    lines = [
        f"Experiment: {exp_id}",
        f"Start:      {row.get('start_time_iso', '')}",
        f"End:        {row.get('end_time_iso', '')}  ({row.get('duration_s', '')}s, {row.get('end_reason', '')})",
        f"Robots:     {', '.join(robots)}",
        "",
    ]

    for rns in robots:
        bag_paths_raw = row.get(f"{rns}_bag_paths", "") or row.get("bag_paths", "")
        bag_files = [Path(p) for p in bag_paths_raw.split(bu.LIST_SEP) if p]
        bag_dirs = list({bf.parent for bf in bag_files})

        # Find the plan message closest to and at/after the experiment start.
        best_msg = None
        best_dt = float("inf")
        for bd in bag_dirs:
            try:
                for topic, t_ns, msg in bu.iter_messages(bd, topic_suffixes=[suffix]):
                    if bu.plan_topic_robot_ns(topic) != rns:
                        continue
                    try:
                        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                    except Exception:
                        t = t_ns * 1e-9
                    dt = abs(t - t_start)
                    if dt < best_dt:
                        best_dt = dt
                        best_msg = msg
            except Exception as e:
                print(f"  [warn] {bd.name} for {rns}: {e}", file=sys.stderr)

        lines.append(f"{'='*60}")
        lines.append(f"Robot: {rns}")
        if best_msg is None:
            lines.append("  (no plan message found)")
            lines.append("")
            continue

        lines.append(f"Plan ID:  {best_msg.plan_id}")
        lines.append(f"Actions:  {len(best_msg.actions)}")
        lines.append("")
        for i, action in enumerate(best_msg.actions):
            lines.append(_format_action(i, action))
        lines.append("")

    text = "\n".join(lines)
    out_path.write_text(text)
    print(f"  wrote {out_path}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--master", type=Path, required=True)
    ap.add_argument("--viz-root", type=Path, default=Path("viz_out"))
    ap.add_argument("--experiment-id", type=str, default=None)
    args = ap.parse_args()

    with open(args.master, newline="") as f:
        rows = list(csv.DictReader(f))

    if args.experiment_id:
        rows = [r for r in rows if r["experiment_id"] == args.experiment_id]
        if not rows:
            ap.error(f"{args.experiment_id!r} not found")

    for row in rows:
        print(f"\n{row['experiment_id']}")
        dump_plan_for_experiment(row, args.viz_root)

    return 0


if __name__ == "__main__":
    sys.exit(main())
