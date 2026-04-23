"""Shared helpers for West Point experiment post-processing."""

from __future__ import annotations

import csv
import os
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Iterator, Optional, Sequence


@dataclass
class LeaseEvent:
    unix_time: float
    event: str


# Robots we recognize when splitting `<session>_<robot>` folder names.
KNOWN_ROBOTS = ("euclid", "hamilton", "gauss", "lewis", "pascal", "newton")


@dataclass
class Session:
    """A session is one logical recording spanning one or more per-robot folders.

    e.g. `bravo_map_1_wed_afternoon_experiment_2_euclid/` and
    `bravo_map_1_wed_afternoon_experiment_2_hamilton/` share session prefix
    `bravo_map_1_wed_afternoon_experiment_2`.
    """

    name: str
    robot_folders: dict[str, Path]  # robot_ns -> folder


def _split_session_and_robot(folder_name: str) -> tuple[str, Optional[str]]:
    for r in KNOWN_ROBOTS:
        suffix = f"_{r}"
        if folder_name.endswith(suffix):
            return folder_name[: -len(suffix)], r
    return folder_name, None


def discover_sessions(root: Path) -> list[Session]:
    """Group sibling per-robot folders under `root` into sessions.

    A candidate folder is one that contains at least one `recorded_data*`
    subdirectory. Its name is split on the trailing `_<known_robot>` suffix
    to derive the session prefix. Folders with no recognizable robot
    suffix become a session of their own with `robot_folders={}`.
    """
    root = Path(root)
    if not root.exists():
        raise FileNotFoundError(f"root does not exist: {root}")

    # Allow running with `--root` pointing at a single experiment folder.
    candidates: list[Path] = []
    if _has_recorded_data(root):
        candidates.append(root)
    for child in sorted(p for p in root.iterdir() if p.is_dir()):
        if _has_recorded_data(child):
            candidates.append(child)

    by_session: dict[str, dict[str, Path]] = {}
    for folder in candidates:
        session_name, robot = _split_session_and_robot(folder.name)
        if robot is None:
            import sys

            print(
                f"  [warn] folder {folder.name!r} has no `_<robot>` suffix; "
                "skipping. Rename or symlink it to e.g. foo_euclid if needed.",
                file=sys.stderr,
            )
            continue
        by_session.setdefault(session_name, {})[robot] = folder

    return [Session(name=n, robot_folders=m) for n, m in sorted(by_session.items())]


def session_bag_dirs(session: Session) -> list[Path]:
    """All recorded_data* dirs across all robot folders in a session."""
    out: list[Path] = []
    for folder in session.robot_folders.values():
        out.extend(list_bag_dirs(folder))
    return out


# Keep the old name for any external callers.
def discover_experiment_sets(root: Path) -> list[Path]:
    """Deprecated: returns the top-level candidate folders; prefer discover_sessions."""
    root = Path(root)
    candidates: list[Path] = []
    if _has_recorded_data(root):
        candidates.append(root)
    for child in sorted(p for p in root.iterdir() if p.is_dir()):
        if _has_recorded_data(child):
            candidates.append(child)
    return candidates


def _has_recorded_data(path: Path) -> bool:
    return any(p.is_dir() and p.name.startswith("recorded_data") for p in path.iterdir())


def list_bag_dirs(experiment_set_dir: Path) -> list[Path]:
    """Return sorted list of `recorded_data*` folders containing a metadata.yaml."""
    experiment_set_dir = Path(experiment_set_dir)
    out = []
    for p in sorted(experiment_set_dir.iterdir()):
        if p.is_dir() and p.name.startswith("recorded_data") and (p / "metadata.yaml").exists():
            out.append(p)
    return out


def find_mcap(bag_dir: Path) -> Path:
    """Return the single .mcap file inside a bag directory.

    Files ending in _original_truncated.mcap are ignored (recovery backups).
    """
    mcaps = [p for p in Path(bag_dir).glob("*.mcap")
             if not p.name.endswith("_original_truncated.mcap")]
    if len(mcaps) != 1:
        raise RuntimeError(f"expected exactly one .mcap in {bag_dir}, got {mcaps}")
    return mcaps[0]


def load_lease_log(path: Path) -> list[LeaseEvent]:
    """Parse spot_executor/lease_log.txt (CSV with columns `time,event`)."""
    out: list[LeaseEvent] = []
    path = Path(path)
    if not path.exists():
        return out
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            t = row.get("time", "").strip()
            ev = row.get("event", "").strip()
            if not t or not ev:
                continue
            try:
                out.append(LeaseEvent(float(t), ev))
            except ValueError:
                continue
    out.sort(key=lambda e: e.unix_time)
    return out


_ROBOT_NS_RE = re.compile(r"^/([^/]+)/omniplanner_node/compiled_plan_out$")


def plan_topic_robot_ns(topic: str) -> Optional[str]:
    """Extract the robot namespace from a compiled_plan_out topic, else None."""
    m = _ROBOT_NS_RE.match(topic)
    return m.group(1) if m else None


# ---- rosbag2 helpers ----------------------------------------------------


def _ros2_imports():
    """Import rosbag2_py lazily so the module can be imported without ROS sourced."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    return rosbag2_py, deserialize_message, get_message


def open_sequential_reader(
    bag_dir: Path,
    topics: Optional[Sequence[str]] = None,
    topic_suffixes: Optional[Sequence[str]] = None,
):
    """Open a SequentialReader on an mcap bag folder.

    Either explicit `topics` or `topic_suffixes` may be supplied. When
    `topic_suffixes` is given, topics whose name ends with any suffix are
    accepted (caller must fetch the full list via get_all_topics_and_types).
    """
    rosbag2_py, _, _ = _ros2_imports()
    storage_opts = rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="mcap")
    converter_opts = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_opts, converter_opts)

    all_topics = reader.get_all_topics_and_types()
    if topic_suffixes is not None:
        matched = [
            t.name for t in all_topics if any(t.name.endswith(sfx) for sfx in topic_suffixes)
        ]
        topics = list(set(list(topics or []) + matched))

    if topics:
        reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))

    type_map = {t.name: t.type for t in all_topics}
    return reader, type_map


def iter_messages(
    bag_dir: Path,
    topics: Optional[Sequence[str]] = None,
    topic_suffixes: Optional[Sequence[str]] = None,
) -> Iterator[tuple[str, int, object]]:
    """Yield `(topic, t_ns, deserialized_msg)` for each matching message in a bag.

    Automatically resolves each topic's type and caches the message class.
    """
    _, deserialize_message, get_message = _ros2_imports()

    reader, type_map = open_sequential_reader(bag_dir, topics=topics, topic_suffixes=topic_suffixes)
    cls_cache: dict[str, type] = {}

    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        cls = cls_cache.get(topic)
        if cls is None:
            type_name = type_map.get(topic)
            if type_name is None:
                continue
            cls = get_message(type_name)
            cls_cache[topic] = cls
        yield topic, t_ns, deserialize_message(raw, cls)


def bag_time_range(bag_dir: Path) -> tuple[float, float]:
    """Return (start, end) unix-seconds by reading bag metadata."""
    rosbag2_py, _, _ = _ros2_imports()
    info = rosbag2_py.Info().read_metadata(str(bag_dir), "mcap")
    start_ns = info.starting_time.nanoseconds
    dur_ns = info.duration.nanoseconds
    return start_ns * 1e-9, (start_ns + dur_ns) * 1e-9


# ---- time formatting ----------------------------------------------------


def iso_utc(ts: float) -> str:
    """Render a unix timestamp as an ISO-8601 UTC string."""
    import datetime as dt

    return dt.datetime.fromtimestamp(ts, tz=dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%fZ")


# Sub-separator for lists packed into a single CSV cell. We deliberately
# avoid `;` because many spreadsheets (especially in European locales) auto-
# detect it as a column delimiter. `|` is safe.
LIST_SEP = "|"


def joinish(items: Iterable) -> str:
    return LIST_SEP.join(str(x) for x in items)
