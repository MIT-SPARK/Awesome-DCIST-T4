"""Run identity: which run directories are the same run, wherever they were copied.

Every run directory (``~/adt4_output/<experiment>`` on each machine) is identified by a
``run_id``:

* ``<run>/run_info.json`` -- written once by ``run-adt4`` at launch (``write_run_info``)
  and never overwritten, so a restart into the same directory keeps the id::

      {"run_id": uuid4, "experiment", "robot", "platform", "session",
       "started_at": ISO-8601 UTC, "host", "git_commit"}

* ``<copy>/.adt4_origin.json`` -- written on the destination of a transfer
  (``make_origin``)::

      {"run_id", "source_machine", "source_path", "transferred_at", "images",
       "compression", "source_size", "source_mtime", "manifest_digest",
       "store_fingerprint", "experiment", "robot"}

* older runs without either get a stable fallback id from host + realpath + creation
  time (``run_scan.fallback_identity``); nothing is written on the robots.

On the base station a robot's run lands at ``<experiment>__<robot>`` (``copy_name``)
because every robot of a multi-robot launch -- and the base station itself -- uses the
same experiment name.  A name already taken by a different run gets ``__2``, ``__3``...
(``choose_dest_name``); it is never overwritten.

``pair_copies`` matches the copies on the base to the robots' originals (by run_id, then
by origin source machine + path, then -- for copies made before run ids existed -- by
name + identical file manifest) and labels each original ``in_sync`` / ``outdated`` /
``not_copied``.

Standard library only (run-adt4 imports it before the workspace is sourced).
"""

from __future__ import annotations

import datetime
import json
import os
import re
import socket
import subprocess
import uuid

RUN_INFO = "run_info.json"
ORIGIN = ".adt4_origin.json"
COPY_SEP = "__"

IN_SYNC = "in_sync"
OUTDATED = "outdated"
NOT_COPIED = "not_copied"
UNKNOWN = "unknown"  # the copy's source machine is not listed (offline)


def utc_now() -> str:
    return datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="seconds")


def git_commit(path) -> str | None:
    """HEAD of the git checkout containing ``path`` (None if not cheap/possible)."""
    try:
        out = subprocess.run(
            ["git", "-C", str(path), "rev-parse", "HEAD"],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            timeout=2,
            universal_newlines=True,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    sha = out.stdout.strip()
    return sha if out.returncode == 0 and re.fullmatch(r"[0-9a-f]{40}", sha) else None


def read_json(path) -> dict | None:
    try:
        with open(path) as f:
            data = json.load(f)
    except (OSError, ValueError):
        return None
    return data if isinstance(data, dict) else None


def read_run_info(run_dir) -> dict | None:
    info = read_json(os.path.join(str(run_dir), RUN_INFO))
    return info if info and info.get("run_id") else None


def write_json_atomic(path, data: dict) -> None:
    tmp = f"{path}.tmp{os.getpid()}"
    with open(tmp, "w") as f:
        json.dump(data, f, indent=2, sort_keys=True)
        f.write("\n")
    os.replace(tmp, path)


def write_run_info(
    run_dir,
    *,
    robot: str | None = None,
    platform: str | None = None,
    session: str | None = None,
    experiment: str | None = None,
    host: str | None = None,
    commit: str | None = None,
    workspace=None,
) -> dict:
    """Create ``<run_dir>/run_info.json`` unless a valid one exists; return its content.

    Never overwrites: a restart into the same directory keeps the run id.  The file
    is created with O_EXCL so two concurrent launches cannot both write one.
    """
    run_dir = str(run_dir)
    path = os.path.join(run_dir, RUN_INFO)
    existing = read_run_info(run_dir)
    if existing:
        return existing
    os.makedirs(run_dir, exist_ok=True)
    if commit is None and workspace is not None:
        commit = git_commit(workspace)
    info = {
        "run_id": str(uuid.uuid4()),
        "experiment": experiment or os.path.basename(run_dir.rstrip("/")),
        "robot": robot,
        "platform": platform,
        "session": session,
        "started_at": utc_now(),
        "host": host or socket.gethostname(),
        "git_commit": commit,
    }
    if os.path.exists(path):  # present but unreadable / no run_id: keep it aside
        os.replace(path, path + ".invalid")
    try:
        fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
    except FileExistsError:  # lost a race: the other writer's id wins
        return read_run_info(run_dir) or info
    with os.fdopen(fd, "w") as f:
        json.dump(info, f, indent=2, sort_keys=True)
        f.write("\n")
    return info


# ---------------------------------------------------------------------------
# Naming on the base station
# ---------------------------------------------------------------------------


def contains_robot(experiment: str, robot: str) -> bool:
    """``robot`` appears in ``experiment`` as a whole word (``_ - .`` delimited)."""
    if not robot:
        return True
    tokens = re.split(r"[^a-z0-9]+", experiment.lower())
    return robot.lower() in tokens


def copy_name(experiment: str, robot: str | None) -> str:
    """Base-side name of ``robot``'s run ``experiment``.

    ``<experiment>__<robot>``, or ``<experiment>`` unchanged when the experiment name
    already names the robot (``hamilton_floor6``, ``map_0730_hamilton``).
    """
    if not robot or contains_robot(experiment, robot):
        return experiment
    return f"{experiment}{COPY_SEP}{robot}"


def entry_ids(entry: dict) -> set:
    """Every run id a listing entry answers to (its own + the run it copies)."""
    ids = set()
    ident = entry.get("identity") or {}
    if ident.get("run_id"):
        ids.add(ident["run_id"])
    origin = entry.get("origin") or {}
    if origin.get("run_id"):
        ids.add(origin["run_id"])
    return ids


def canonical_id(entry: dict) -> str | None:
    """The id of the run an entry holds: the origin's for a copy, else its own."""
    origin = (entry or {}).get("origin") or {}
    return origin.get("run_id") or ((entry or {}).get("identity") or {}).get("run_id")


def choose_dest_name(base_name: str, existing: dict, run_id: str) -> tuple[str, str]:
    """Pick the destination directory name for run ``run_id``.

    ``existing``: name -> set of run ids of the directories already there (an empty
    set for a directory of unknown identity).  Returns ``(name, status)``: ``"new"``
    (free name), ``"update"`` (that directory already holds this run) or
    ``"collision"`` (``base_name`` holds another run; a ``__N`` suffix was picked).
    """
    n = 1
    while True:
        name = base_name if n == 1 else f"{base_name}{COPY_SEP}{n}"
        if name not in existing:
            return name, ("new" if n == 1 else "collision")
        if run_id in existing[name]:
            return name, ("update" if n == 1 else "collision_update")
        n += 1


def make_origin(
    source_entry: dict,
    source_machine: str,
    images: str,
    compression: str,
    previous: dict | None = None,
) -> dict:
    """``.adt4_origin.json`` content for a copy of ``source_entry`` (a run_scan entry)."""
    ident = source_entry.get("identity") or {}
    man = source_entry.get("manifest") or {}
    origin = {
        "run_id": canonical_id(source_entry),
        "identity_source": ident.get("source"),
        "experiment": ident.get("experiment") or source_entry.get("name"),
        "robot": ident.get("robot") or source_machine,
        "source_machine": source_machine,
        "source_path": source_entry.get("abs_path") or source_entry.get("path"),
        "transferred_at": utc_now(),
        "images": images,
        "compression": compression,
        "source_size": source_entry.get("size_bytes"),
        "source_mtime": max(
            [t for t in (source_entry.get("signature"), man.get("newest")) if t] or [0]
        ),
        "manifest_digest": man.get("digest"),
        "manifest_files": man.get("files"),
        "store_fingerprint": source_entry.get("store_fingerprint"),
    }
    if previous and previous.get("first_transferred_at"):
        origin["first_transferred_at"] = previous["first_transferred_at"]
    elif previous and previous.get("transferred_at"):
        origin["first_transferred_at"] = previous["transferred_at"]
    return origin


def store_needs_send(source_entry: dict, origin: dict | None, images: str, compression: str) -> bool:
    """Does the image store have to be (re)packed for this transfer?"""
    if images == "none" or not source_entry.get("has_image_store", True):
        return False
    if not origin:
        return True
    if origin.get("images") != images or origin.get("compression") != compression:
        return True
    return origin.get("store_fingerprint") != source_entry.get("store_fingerprint")


def copy_state(source_entry: dict, origin: dict | None, copy_entry: dict | None = None) -> str:
    """``in_sync`` when the source still matches what was copied, else ``outdated``.

    With an origin record: the source's file manifest digest and image store
    fingerprint must equal the ones recorded at transfer time.  Without one (a copy
    made before run ids): the copy's own manifest must equal the source's.
    """
    src_digest = (source_entry.get("manifest") or {}).get("digest")
    if not src_digest:
        return OUTDATED
    if origin:
        if origin.get("manifest_digest") != src_digest:
            return OUTDATED
        if origin.get("images", "none") != "none" and source_entry.get("has_image_store"):
            if origin.get("store_fingerprint") != source_entry.get("store_fingerprint"):
                return OUTDATED
        return IN_SYNC
    copy_digest = ((copy_entry or {}).get("manifest") or {}).get("digest")
    return IN_SYNC if copy_digest == src_digest else OUTDATED


def _legacy_candidate(entry: dict, base_by_name: dict) -> dict | None:
    """A same-named base dir with no identity of its own that looks like a copy."""
    cand = base_by_name.get(entry.get("name"))
    if not cand or cand.get("origin"):
        return None
    if (cand.get("identity") or {}).get("source") != "fallback":
        return None
    if (entry.get("identity") or {}).get("source") not in ("fallback", "run_info"):
        return None
    a = (entry.get("manifest") or {}).get("digest")
    b = (cand.get("manifest") or {}).get("digest")
    if a and a == b:
        return cand
    if entry.get("dsg_mtime") and entry.get("dsg_mtime") == cand.get("dsg_mtime"):
        return cand  # same saved DSG, other files differ: an outdated copy
    return None


def find_copy(entry: dict, machine: str, base_entries: list) -> tuple[dict | None, str | None]:
    """The base copy of ``machine``'s run ``entry`` and how it was matched."""
    rid = canonical_id(entry)
    src_path = entry.get("abs_path") or entry.get("path")
    for b in base_entries:
        origin = b.get("origin") or {}
        if rid and origin.get("run_id") == rid:
            return b, "run_id"
    for b in base_entries:
        if rid and (b.get("identity") or {}).get("run_id") == rid and b is not entry:
            return b, "run_id"
    for b in base_entries:
        origin = b.get("origin") or {}
        if origin.get("source_machine") == machine and origin.get("source_path") == src_path:
            return b, "origin"
    cand = _legacy_candidate(entry, {b.get("name"): b for b in base_entries})
    if cand is not None:
        return cand, "contents"
    return None, None


def pair_copies(machines: dict, base: str) -> None:
    """Annotate a ``{machine: [entries] | {error}}`` listing in place.

    Every run on a machine other than ``base`` gets ``copy``: ``{state, machine, name,
    path, matched_by, transferred_at}`` (state ``not_copied`` with no other keys when
    the base has no copy).  Every base copy gets ``copy_of``: ``{machine, name, path,
    state}`` (state ``unknown`` when its source machine is not in the listing).
    """
    base_entries = machines.get(base)
    if not isinstance(base_entries, list):
        base_entries = []
    claimed = set()
    for machine, entries in machines.items():
        if machine == base or not isinstance(entries, list):
            continue
        for e in entries:
            if e.get("error"):
                continue
            copy, how = find_copy(e, machine, base_entries)
            if copy is None:
                e["copy"] = {"state": NOT_COPIED, "machine": base}
                continue
            origin = copy.get("origin") if how != "contents" else None
            state = copy_state(e, origin, copy)
            e["copy"] = {
                "state": state,
                "machine": base,
                "name": copy.get("name"),
                "path": copy.get("path"),
                "matched_by": how,
                "transferred_at": (copy.get("origin") or {}).get("transferred_at"),
            }
            copy["copy_of"] = {
                "machine": machine,
                "name": e.get("name"),
                "path": e.get("abs_path") or e.get("path"),
                "state": state,
                "matched_by": how,
            }
            claimed.add(id(copy))
    for b in base_entries:
        origin = b.get("origin")
        if origin and id(b) not in claimed:
            b["copy_of"] = {
                "machine": origin.get("source_machine"),
                "name": os.path.basename(str(origin.get("source_path") or "").rstrip("/")),
                "path": origin.get("source_path"),
                "state": UNKNOWN,
                "matched_by": "origin",
            }


def remote_write_command(path: str, data: dict) -> str:
    """Shell command writing ``data`` as JSON to ``path`` (``~`` allowed) atomically."""
    import base64
    import shlex

    blob = base64.b64encode((json.dumps(data, indent=2, sort_keys=True) + "\n").encode()).decode()
    p = '"$HOME"' + shlex.quote(path[1:]) if path.startswith("~/") else shlex.quote(path)
    return f'echo {blob} | base64 -d > {p}.tmp && mv -f {p}.tmp {p}'
