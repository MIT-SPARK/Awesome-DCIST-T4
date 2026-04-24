#!/usr/bin/env python3
"""Build a master CSV of robot experiments from recorded mcap bags.

Walks one or more experiment-set folders under --root, detects every
compiled_plan_out publication, groups same-time multi-robot plans into
experiments, derives start/end times, path length, interventions, and
planned-goal summaries, and writes/merges an experiments.csv.

Run with the ROS 2 Jazzy workspace sourced and `spark_env` python on PATH.
"""

from __future__ import annotations

import argparse
import csv
import sys
import time as time_mod
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np

_THIS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(_THIS_DIR))
import _bag_utils as bu  # noqa: E402


STATIC_COLUMNS = [
    "experiment_id",
    "experiment_set",
    "bag_paths",
    "robots",
    "is_multi_agent",
    "start_time_unix",
    "start_time_iso",
    "end_time_unix",
    "end_time_iso",
    "end_reason",
    "duration_s",
    "candidate_end_times",
    "num_manual_interventions",
    "lease_events_in_window",
    "language_command",
    "notes",
]

PER_ROBOT_SUFFIXES = [
    "plan_id",
    "path_length_m",
    "num_actions",
    "action_type_counts",
    "object_classes",
    "bag_paths",
]


@dataclass
class PlanEvent:
    bag_dir: Path
    topic: str
    robot_ns: str
    unix_time: float
    plan_id: str
    robot_name: str
    actions_summary: dict  # {type_counts, object_classes, num_actions}


@dataclass
class Experiment:
    robots: list[str]
    plans: dict[str, PlanEvent]  # robot_ns -> PlanEvent
    start_time: float
    end_time: float = 0.0
    end_reason: str = ""
    candidate_end_times: list[float] = field(default_factory=list)
    path_length_m: dict[str, float] = field(default_factory=dict)  # per robot
    num_interventions: int = 0
    lease_events: list[tuple[float, str, str]] = field(default_factory=list)
    bag_paths: list[str] = field(default_factory=list)  # union across robots
    per_robot_bag_paths: dict[str, list[str]] = field(default_factory=dict)


# ---- plan detection -----------------------------------------------------


def _summarize_actions(msg) -> dict:
    type_counts: dict[str, int] = {}
    object_classes: list[str] = []
    for a in msg.actions:
        type_counts[a.action_type] = type_counts.get(a.action_type, 0) + 1
        if a.action_type == "PICK" and a.object_class:
            object_classes.append(a.object_class)
    return {
        "num_actions": len(msg.actions),
        "type_counts": type_counts,
        "object_classes": object_classes,
    }


def collect_plan_events(bag_dirs: list[Path]) -> list[PlanEvent]:
    """Read compiled_plan_out messages from each bag.

    A single plan publication is typically echoed into every participating
    robot's bag, so we dedupe by (plan_robot_ns, plan_id) and keep whichever
    copy has the earliest publisher timestamp.
    """
    seen: dict[tuple[str, str], PlanEvent] = {}
    suffix = "/omniplanner_node/compiled_plan_out"
    for bag_dir in bag_dirs:
        for topic, t_ns, msg in bu.iter_messages(bag_dir, topic_suffixes=[suffix]):
            robot_ns = bu.plan_topic_robot_ns(topic)
            if robot_ns is None:
                continue
            try:
                hsec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            except Exception:
                hsec = 0.0
            t_unix = hsec if hsec > 0 else t_ns * 1e-9
            key = (robot_ns, msg.plan_id)
            prior = seen.get(key)
            if prior is not None and prior.unix_time <= t_unix:
                continue
            seen[key] = PlanEvent(
                bag_dir=bag_dir,
                topic=topic,
                robot_ns=robot_ns,
                unix_time=t_unix,
                plan_id=msg.plan_id,
                robot_name=msg.robot_name,
                actions_summary=_summarize_actions(msg),
            )
    events = sorted(seen.values(), key=lambda e: e.unix_time)
    return events


def group_into_experiments(
    events: list[PlanEvent], multiagent_window_s: float
) -> list[Experiment]:
    """Greedy grouping: plans within `multiagent_window_s` of one another AND
    from different robots form a single experiment.
    """
    out: list[Experiment] = []
    cur: Optional[Experiment] = None
    for ev in events:
        if (
            cur is not None
            and (ev.unix_time - cur.start_time) <= multiagent_window_s
            and ev.robot_ns not in cur.plans
        ):
            cur.plans[ev.robot_ns] = ev
            cur.robots.append(ev.robot_ns)
            continue
        if cur is not None:
            out.append(cur)
        cur = Experiment(
            robots=[ev.robot_ns],
            plans={ev.robot_ns: ev},
            start_time=ev.unix_time,
        )
    if cur is not None:
        out.append(cur)
    return out


# ---- fused-trajectory path length helper --------------------------------


def _load_fused_positions(
    bag_dirs: list[Path], fused_dir: Path,
) -> tuple[np.ndarray, np.ndarray]:
    """Load all UTM positions from precomputed fused CSVs for a robot's bag dirs.

    Returns (times_sec, positions_xyz) sorted by time, covering the full session.
    """
    from robotdatapy.data import PoseData
    from robotdatapy.data.pose_data import KIMERA_MULTI_GT_CSV_OPTIONS

    all_t: list[np.ndarray] = []
    all_pos: list[np.ndarray] = []
    for bd in bag_dirs:
        csv_path = fused_dir / f"{bd.parent.name}__{bd.name}.csv"
        if not csv_path.exists():
            continue
        try:
            pd_obj = PoseData.from_csv(str(csv_path), csv_options=KIMERA_MULTI_GT_CSV_OPTIONS, time_tol=30.0)
            poses = pd_obj.all_poses()
            all_t.append(pd_obj.times)
            all_pos.append(poses[:, :3, 3])
        except Exception as e:
            print(f"    [fused] {csv_path.name}: {e}", file=sys.stderr)

    if not all_t:
        return np.array([]), np.zeros((0, 3))
    times = np.concatenate(all_t)
    positions = np.concatenate(all_pos, axis=0)
    order = np.argsort(times)
    return times[order], positions[order]


# ---- odometry-based end detection ---------------------------------------


def _load_odom_windowed(
    bag_dirs: list[Path], robot_ns: str, t_lo: float, t_hi: float
):
    """Load odom (time, position) for robot in [t_lo, t_hi] unix seconds.

    Uses mcap.reader.make_reader() which leverages the chunk index to skip
    chunks outside the requested time window — much faster than a full scan
    when only a short window is needed.
    """
    from mcap.reader import make_reader as _make_reader
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    topic = f"/{robot_ns}/odom"
    start_ns = int(t_lo * 1e9)
    end_ns = int(t_hi * 1e9)

    t_list: list[float] = []
    x_list: list[float] = []
    y_list: list[float] = []
    z_list: list[float] = []
    msg_cls = None

    for bd in bag_dirs:
        try:
            b_lo, b_hi = bu.bag_time_range(bd)
            if b_hi < t_lo or b_lo > t_hi:
                continue
        except Exception:
            pass
        mcap_file = bu.find_mcap(bd)
        try:
            with open(mcap_file, "rb") as fh:
                reader = _make_reader(fh, validate_crcs=False)
                for schema, channel, message in reader.iter_messages(
                    topics=[topic], start_time=start_ns, end_time=end_ns
                ):
                    if msg_cls is None:
                        msg_cls = get_message(schema.name)
                    msg = deserialize_message(message.data, msg_cls)
                    try:
                        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                        if t <= 0:
                            t = message.log_time * 1e-9
                    except Exception:
                        t = message.log_time * 1e-9
                    p = msg.pose.pose.position
                    t_list.append(t)
                    x_list.append(p.x)
                    y_list.append(p.y)
                    z_list.append(p.z)
        except Exception as e:
            print(f"    [odom] {bd.name} for {robot_ns}: {e}", file=sys.stderr)

    if not t_list:
        return np.array([]), np.zeros((0, 3))
    times = np.asarray(t_list, dtype=np.float64)
    positions = np.column_stack([x_list, y_list, z_list]).astype(np.float64)
    order = np.argsort(times)
    return times[order], positions[order]


def detect_stop_windows(
    times: np.ndarray,
    positions: np.ndarray,
    t_lo: float,
    t_hi: float,
    stop_speed: float,
    stop_window: float,
) -> tuple[list[tuple[float, float]], np.ndarray, np.ndarray]:
    """Return list of (stop_start, stop_end) intervals inside [t_lo, t_hi]
    where speed < stop_speed for >= stop_window seconds.

    Also returns the clipped (times, speeds) arrays used for the scan.
    """
    mask = (times >= t_lo) & (times <= t_hi)
    t = times[mask]
    p = positions[mask]
    if len(t) < 2:
        return [], t, np.array([])

    dt = np.diff(t)
    dt[dt <= 0] = np.nan  # guard against zero/negative steps
    steps = np.linalg.norm(np.diff(p, axis=0), axis=1)
    speed = np.concatenate([[0.0], steps / dt])  # align with `t`

    below = speed < stop_speed
    intervals: list[tuple[float, float]] = []
    i = 0
    n = len(t)
    while i < n:
        if not below[i]:
            i += 1
            continue
        j = i
        while j < n and below[j]:
            j += 1
        start, end = t[i], t[j - 1]
        if (end - start) >= stop_window:
            intervals.append((start, end))
        i = j
    return intervals, t, speed


def resolve_end_time(
    plan_start: float,
    next_plan_start: Optional[float],
    bag_end: float,
    involved_robots: list[str],
    odom_by_robot: dict[str, tuple[np.ndarray, np.ndarray]],
    stop_speed: float,
    stop_window: float,
) -> tuple[float, str, list[float]]:
    """Compute end_time, end_reason, and candidate_end_times (unix)."""
    upper = next_plan_start if next_plan_start is not None else bag_end
    # Small epsilon back from the next plan so we don't accidentally slurp it.
    upper = max(plan_start, upper - 0.05)

    per_robot_last: list[float] = []
    all_candidates: list[float] = []
    for rns in involved_robots:
        times, positions = odom_by_robot.get(rns, (np.array([]), np.zeros((0, 3))))
        if len(times) < 2:
            continue
        intervals, _, _ = detect_stop_windows(
            times, positions, plan_start, upper, stop_speed, stop_window
        )
        if not intervals:
            continue
        # Candidate = the timestamp at which the robot first hit 0 for each run.
        for s, _ in intervals:
            all_candidates.append(s)
        # Per-robot "final stop" = start of the last qualifying stop interval.
        per_robot_last.append(intervals[-1][0])

    all_candidates = sorted(set(round(c, 3) for c in all_candidates))

    if per_robot_last:
        # Group end = last robot to stop.
        end = max(per_robot_last)
        return end, "velocity_stop", all_candidates

    # No stop detected — the robot was still moving when the next plan arrived
    # (or when the last bag ended).
    if next_plan_start is not None:
        return next_plan_start, "next_plan", all_candidates
    return bag_end, "bag_end", all_candidates


# ---- experiment assembly ------------------------------------------------


def assemble_session(
    session: bu.Session,
    stop_speed: float,
    stop_window: float,
    multiagent_window: float,
    fused_dir: Optional[Path] = None,
) -> list[Experiment]:
    """Build Experiment records for one session.

    A session may contain one folder per robot; /odom, /fix, images and
    the lease log are local to each robot's folder. Plan events are
    aggregated across folders (and deduped by (robot_ns, plan_id) since
    plans are echoed cross-robot).
    """
    robot_folders = session.robot_folders
    if not robot_folders:
        print(f"  [skip] session {session.name!r} has no recognized robot folders",
              file=sys.stderr)
        return []

    # Per-robot list of bag folders (each may have recorded_data, recorded_data_2, …).
    bag_dirs_by_robot: dict[str, list[Path]] = {}
    for rns, folder in robot_folders.items():
        bag_dirs_by_robot[rns] = bu.list_bag_dirs(folder)
    all_bag_dirs = [bd for bds in bag_dirs_by_robot.values() for bd in bds]
    if not all_bag_dirs:
        print(f"  [skip] no recorded_data* in session {session.name!r}", file=sys.stderr)
        return []

    print(f"  folders: {[f.name for f in robot_folders.values()]}")
    print(f"  bags: {[b.name for b in all_bag_dirs]}")
    print("  scanning plan topics …")
    t0 = time_mod.time()
    events = collect_plan_events(all_bag_dirs)
    print(f"  found {len(events)} plan events in {time_mod.time() - t0:.1f} s")

    if not events:
        return []

    experiments = group_into_experiments(events, multiagent_window)
    print(f"  grouped into {len(experiments)} experiments")

    # Merge lease logs across every robot folder, tagging each event with its source.
    lease_events: list[tuple[str, float, str]] = []  # (robot_ns, t, event)
    for rns, folder in robot_folders.items():
        for ev in bu.load_lease_log(folder / "spot_executor" / "lease_log.txt"):
            lease_events.append((rns, ev.unix_time, ev.event))
    lease_events.sort(key=lambda x: x[1])

    bag_end_overall = max(bu.bag_time_range(bd)[1] for bd in all_bag_dirs)

    # When fused_dir is provided, load corrected positions once per robot for
    # the full session — used for both stop detection and path length, avoiding
    # repeated bag reads.
    pos_by_robot: dict[str, tuple[np.ndarray, np.ndarray]] = {}
    if fused_dir is not None:
        t0_f = time_mod.time()
        for rns, bds in bag_dirs_by_robot.items():
            ft, fp = _load_fused_positions(bds, fused_dir)
            if len(ft) >= 2:
                pos_by_robot[rns] = (ft, fp)
                print(f"  {rns}: loaded {len(ft)} fused poses in {time_mod.time()-t0_f:.1f}s")
            else:
                print(f"  {rns}: no fused CSV found, will fall back to odom", file=sys.stderr)

    starts_sorted = sorted(e.start_time for e in experiments)

    for i, exp in enumerate(experiments):
        provisional_end = None
        for s in starts_sorted:
            if s > exp.start_time:
                provisional_end = s
                break

        odom_hi = (provisional_end if provisional_end is not None else bag_end_overall) + 30.0

        # Build per-experiment position dict: fused if available, else load odom.
        t0_load = time_mod.time()
        pos_for_exp: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        for rns in exp.robots:
            if rns in pos_by_robot:
                pos_for_exp[rns] = pos_by_robot[rns]
            else:
                bds = bag_dirs_by_robot.get(rns, [])
                if not bds:
                    print(f"    {rns}: no bag folder in session", file=sys.stderr)
                    pos_for_exp[rns] = (np.array([]), np.zeros((0, 3)))
                    continue
                pos_for_exp[rns] = _load_odom_windowed(bds, rns, exp.start_time - 5.0, odom_hi)
                n = len(pos_for_exp[rns][0])
                print(f"    exp {i}: {rns}: {n} odom samples in {time_mod.time()-t0_load:.1f}s")

        end_time, end_reason, candidates = resolve_end_time(
            plan_start=exp.start_time,
            next_plan_start=provisional_end,
            bag_end=bag_end_overall,
            involved_robots=exp.robots,
            odom_by_robot=pos_for_exp,
            stop_speed=stop_speed,
            stop_window=stop_window,
        )
        exp.end_time = end_time
        exp.end_reason = end_reason
        exp.candidate_end_times = candidates

        for rns in exp.robots:
            times_pl, positions_pl = pos_for_exp.get(rns, (np.array([]), np.zeros((0, 3))))
            mask_pl = (times_pl >= exp.start_time) & (times_pl <= exp.end_time)
            times_pl = times_pl[mask_pl]
            positions_pl = positions_pl[mask_pl]
            if len(times_pl) < 2:
                exp.path_length_m[rns] = 0.0
                continue
            dts = np.diff(times_pl)
            segs = np.linalg.norm(np.diff(positions_pl, axis=0), axis=1)
            keep = dts <= 1.0
            exp.path_length_m[rns] = float(segs[keep].sum())
            src = "fused" if rns in pos_by_robot else "odom"
            print(f"    exp {i}: {rns}: path_length={exp.path_length_m[rns]:.1f}m ({src})")

        win_events = [
            (rns, t, e) for (rns, t, e) in lease_events if exp.start_time <= t < exp.end_time
        ]
        exp.lease_events = win_events
        exp.num_interventions = sum(1 for _, _, e in win_events if e == "manual_intervention")

        # Per-robot bag paths overlapping the window.
        exp.per_robot_bag_paths = {}
        all_overlapping: list[str] = []
        for rns, bds in bag_dirs_by_robot.items():
            overlapping: list[str] = []
            for bd in bds:
                t_lo, t_hi = bu.bag_time_range(bd)
                if exp.start_time <= t_hi and exp.end_time >= t_lo:
                    overlapping.append(str(bu.find_mcap(bd)))
            exp.per_robot_bag_paths[rns] = overlapping
            all_overlapping.extend(overlapping)
        exp.bag_paths = all_overlapping

    return experiments


# ---- CSV I/O -------------------------------------------------------------


def _flatten_row(
    exp: Experiment, session_name: str, exp_idx: int, per_set_robot_cols: list[str]
) -> dict:
    row = {c: "" for c in STATIC_COLUMNS}
    row["experiment_id"] = f"{session_name}__exp_{exp_idx:04d}"
    row["experiment_set"] = session_name
    row["bag_paths"] = bu.joinish(exp.bag_paths)
    row["robots"] = bu.joinish(sorted(exp.robots))
    row["is_multi_agent"] = "true" if len(exp.robots) > 1 else "false"
    row["start_time_unix"] = f"{exp.start_time:.6f}"
    row["start_time_iso"] = bu.iso_utc(exp.start_time)
    row["end_time_unix"] = f"{exp.end_time:.6f}"
    row["end_time_iso"] = bu.iso_utc(exp.end_time)
    row["end_reason"] = exp.end_reason
    row["duration_s"] = f"{max(0.0, exp.end_time - exp.start_time):.2f}"
    row["candidate_end_times"] = bu.joinish(bu.iso_utc(t) for t in exp.candidate_end_times)
    row["num_manual_interventions"] = str(exp.num_interventions)
    row["lease_events_in_window"] = bu.joinish(
        f"{rns}:{t:.3f}:{ev}" for rns, t, ev in exp.lease_events
    )

    for col in per_set_robot_cols:
        row[col] = ""
    for rns, plan in exp.plans.items():
        counts = ",".join(f"{k}:{v}" for k, v in sorted(plan.actions_summary["type_counts"].items()))
        row[f"{rns}_plan_id"] = plan.plan_id
        row[f"{rns}_path_length_m"] = f"{exp.path_length_m.get(rns, 0.0):.2f}"
        row[f"{rns}_num_actions"] = str(plan.actions_summary["num_actions"])
        row[f"{rns}_action_type_counts"] = counts
        row[f"{rns}_object_classes"] = bu.joinish(plan.actions_summary["object_classes"])
        row[f"{rns}_bag_paths"] = bu.joinish(exp.per_robot_bag_paths.get(rns, []))
    return row


def _per_set_robot_columns(experiments: list[Experiment]) -> list[str]:
    robots = sorted({r for e in experiments for r in e.robots})
    cols: list[str] = []
    for r in robots:
        for sfx in PER_ROBOT_SUFFIXES:
            cols.append(f"{r}_{sfx}")
    return cols


def _stable_key(row: dict) -> tuple:
    # Keyed on the plan publication time + robot set, NOT the folder/session
    # name, so annotations survive folder renames or session-prefix changes.
    return (row.get("start_time_unix", ""), row.get("robots", ""))


def _merge_manual_fields(
    new_rows: list[dict], existing_csv: Optional[Path]
) -> list[dict]:
    if existing_csv is None or not existing_csv.exists():
        return new_rows
    with open(existing_csv, newline="") as f:
        reader = csv.DictReader(f)
        existing = list(reader)
    by_key = {_stable_key(r): r for r in existing}
    carried = 0
    for row in new_rows:
        prev = by_key.get(_stable_key(row))
        if prev is None:
            continue
        for fld in ("language_command", "notes"):
            if prev.get(fld):
                row[fld] = prev[fld]
                carried += 1
    dropped = [r for k, r in by_key.items() if k not in {_stable_key(n) for n in new_rows}]
    if dropped:
        print(
            f"  warning: {len(dropped)} previously-recorded row(s) not present in new scan "
            f"(they will be dropped). Use --force to rebuild without merge.",
            file=sys.stderr,
        )
    if carried:
        print(f"  carried forward {carried} manual field(s) from existing CSV")
    return new_rows


def write_csv(rows: list[dict], all_columns: list[str], output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    with open(output, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=all_columns)
        writer.writeheader()
        for row in rows:
            writer.writerow({c: row.get(c, "") for c in all_columns})


# ---- main ---------------------------------------------------------------


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--root", type=Path, required=True, help="Root data folder")
    ap.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Output CSV (default: <root>/experiments.csv)",
    )
    ap.add_argument("--stop-speed", type=float, default=0.1, help="m/s threshold for 'stopped'")
    ap.add_argument(
        "--stop-window",
        type=float,
        default=120.0,
        help="seconds below stop-speed to count as a stop",
    )
    ap.add_argument(
        "--multiagent-window",
        type=float,
        default=30.0,
        help="seconds between plans to consider them the same experiment",
    )
    ap.add_argument(
        "--force",
        action="store_true",
        help="do not merge with existing CSV (language_command/notes will be lost)",
    )
    ap.add_argument(
        "--fused-dir",
        type=Path,
        default=None,
        help="Directory of precomputed fused/hydra trajectory CSVs; "
             "when present, path lengths come from corrected global positions instead of raw odom.",
    )
    args = ap.parse_args()

    output = args.output or (args.root / "experiments.csv")

    sessions = bu.discover_sessions(args.root)
    if not sessions:
        print(f"No sessions found under {args.root}", file=sys.stderr)
        return 1
    print(f"Found {len(sessions)} session(s): {[s.name for s in sessions]}")

    all_new_rows: list[dict] = []
    per_set_cols_union: list[str] = []
    seen_cols: set[str] = set()

    for session in sessions:
        print(f"\n=== {session.name} "
              f"({'+'.join(sorted(session.robot_folders)) or 'no-robots'}) ===")
        experiments = assemble_session(
            session, args.stop_speed, args.stop_window, args.multiagent_window,
            fused_dir=args.fused_dir,
        )
        per_set_cols = _per_set_robot_columns(experiments)
        for c in per_set_cols:
            if c not in seen_cols:
                per_set_cols_union.append(c)
                seen_cols.add(c)
        for i, exp in enumerate(experiments):
            row = _flatten_row(exp, session.name, i, per_set_cols_union)
            all_new_rows.append(row)

    all_columns = list(STATIC_COLUMNS)
    # Put per-robot columns after duration_s, before the manual fields.
    insert_at = all_columns.index("candidate_end_times") + 1
    for c in per_set_cols_union:
        all_columns.insert(insert_at, c)
        insert_at += 1

    existing = None if args.force else output
    merged = _merge_manual_fields(all_new_rows, existing)
    write_csv(merged, all_columns, output)
    print(f"\nWrote {len(merged)} rows to {output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
