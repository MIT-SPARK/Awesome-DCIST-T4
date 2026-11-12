"""The mission bag's topic list for one run (see recording/mission_*_topics.txt).

run-adt4 writes the effective list to ``<run>/mission_bag_topics.txt`` at launch, so the
run records (and keeps) exactly the list it used:

* Spot / Husky: a copy of ``recording/mission_{spot,phoenix}_topics.txt``.
* Base station: ``recording/mission_base_topics.txt`` (planner, goal manager, console
  topics) plus every topic the fleetlink base bridge publishes on the base, derived
  from ``fleetlink/relay.yaml`` -- the same file the bridge loads -- so the list cannot
  drift from the relay:

  - ``dir: up`` rows: published on the base under ``base_topic`` if set, else
    ``topic``; ``{robot}`` becomes ``*`` (every robot, whatever the fleet)
  - ``dir: down`` rows: the commands the base sends over the radio (recorded too,
    marked optional: they only appear when a command is sent)
  - TF: the bridge publishes all robots' TF on the base's ``/tf`` / ``/tf_static``,
    remapped to ``/<base>/tf`` / ``/<base>/tf_static`` (in the static base list)
  - ``/fleetlink/link_status`` (the bridge's own status; in the static base list)

  A topic in both keeps the static list's marker (``# optional`` or not).

The DSG, mesh, camera video and image-store frames that fleetlink also carries reach
the console over zenoh, not as ROS topics on the base, so a rosbag cannot record them;
the console keeps them (live store, DSG snapshots).

Standard library + PyYAML only (run-adt4 imports it before the workspace is sourced).
"""

from __future__ import annotations

import os
import pathlib

import yaml

LIST_NAMES = {"base", "spot", "phoenix"}
RUN_TOPIC_FILE = "mission_bag_topics.txt"


def relay_config_path(ws_path) -> pathlib.Path:
    """The relay.yaml the fleetlink bridge loads (installed share copy), else the
    source copy next to run-adt4."""
    for prefix in (os.environ.get("AMENT_PREFIX_PATH") or "").split(os.pathsep):
        if not prefix:
            continue
        p = (
            pathlib.Path(prefix)
            / "share"
            / "dcist_launch_system"
            / "fleetlink"
            / "relay.yaml"
        )
        if p.is_file():
            return p
    return pathlib.Path(ws_path) / "fleetlink" / "relay.yaml"


def relay_base_topics(relay_path) -> list[tuple[str, bool, str]]:
    """``[(topic, optional, note)]`` the fleetlink bridge publishes on the base."""
    with open(relay_path) as f:
        config = yaml.safe_load(f) or {}
    out = []
    seen = set()
    for row in config.get("topics") or []:
        direction = row.get("dir")
        if direction == "up":
            name = row.get("base_topic") or row.get("topic")
            optional = False
        elif direction == "down":
            name = row.get("topic")
            optional = True
        else:
            continue
        if not name:
            continue
        name = name.replace("{robot}", "*")
        if name in seen:
            continue
        seen.add(name)
        platforms = row.get("platform")
        note = f"relay {direction} {row.get('cls', '')} {row.get('type', '')}".strip()
        if platforms:
            note += f" [{', '.join(platforms)}]"
        out.append((name, optional, note))
    return out


def _parse(path) -> list[tuple[str, bool]]:
    from dcist_launch_system.record_topics import parse_topic_file

    return parse_topic_file(path, expand=False)


def compose(list_name: str, recording_dir, relay_path=None) -> str:
    """Text of the run's topic file for ``list_name`` (base | spot | phoenix)."""
    static = pathlib.Path(recording_dir) / f"mission_{list_name}_topics.txt"
    text = static.read_text()
    if list_name != "base" or relay_path is None:
        return text
    known = {t for t, _ in _parse(static)}
    lines = [
        "",
        f"# ---- derived at launch from {relay_path}: every topic the fleetlink base",
        "# bridge publishes on the base ('up' rows) and the commands it carries down",
    ]
    for topic, optional, note in relay_base_topics(relay_path):
        if topic in known:
            continue
        marker = "optional: " if optional else ""
        lines.append(f"{topic}  # {marker}{note}")
    return text.rstrip("\n") + "\n" + "\n".join(lines) + "\n"


def write_run_topics(
    run_dir, list_name, recording_dir, relay_path=None
) -> pathlib.Path:
    """Write ``<run_dir>/mission_bag_topics.txt``; return its path."""
    path = pathlib.Path(run_dir) / RUN_TOPIC_FILE
    path.write_text(compose(list_name, recording_dir, relay_path))
    return path
