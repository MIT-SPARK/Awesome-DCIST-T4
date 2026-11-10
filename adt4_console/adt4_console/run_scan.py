"""Scan the run directories under an output root: what each run holds and how big it is.

Read-only: the image store index is opened with ``mode=ro`` (``immutable=1`` as a
fallback) and nothing is written into the runs.

``du`` is the slow part on big runs, so sizes are optional per call: the caller passes
the sizes it already knows (``known``: path -> [signature, size, store_size]) and a time
budget. A run whose signature (newest mtime of the run dir, its immediate
subdirectories, ``hydra/backend`` and the store index) still matches keeps its cached
size; others are measured until the budget runs out and the rest come back with
``size_pending: true`` for the caller to fill later (``timeout=None`` = no limit).
"""

from __future__ import annotations

import datetime
import json
import os
import re
import sqlite3
import subprocess
import time
from urllib.parse import quote

DSG_FILES = {
    "has_dsg": "hydra/backend/dsg.json",
    "has_dsg_mesh": "hydra/backend/dsg_with_mesh.json",
    "has_mesh": "hydra/backend/mesh.ply",
}
STORE_DIR = "image_store"
STORE_DB = "store.sqlite"
# hydra_python.image_store FrameKind
KIND_NAMES = {0: "keyframe", 2: "object"}
# Hydra/Khronos raw image logs that `hydra image-store index` (or the console, on
# first use) builds image_store/ from. First existing dir wins, as in
# hydra_python.image_store.indexer.
KEYFRAME_DIRS = ("keyframes", "hydra/keyframes")
TRACK_DIRS = ("tracks", "hydra/tracks", "khronos/tracks")
CHECK_SAMPLES = 4  # referenced files checked from each end of the frames table
# A run directory is usable as a prior map when both of these were saved.
PRIOR_SUBDIRS = ("hydra", "roman")


def _mtime(path):
    try:
        return os.stat(path).st_mtime
    except OSError:
        return None


def signature(run, subdirs) -> float:
    """Newest mtime among the places a growing run touches; size cache key."""
    paths = [run, os.path.join(run, "hydra", "backend")]
    paths += [os.path.join(run, s) for s in subdirs]
    db = os.path.join(run, STORE_DIR, STORE_DB)
    paths += [db, db + "-wal"]
    times = [t for t in (_mtime(p) for p in paths) if t is not None]
    return round(max(times), 3) if times else 0.0


def _du_tree(run, timeout):
    """(total, image_store) bytes of ``run`` via ``du -b -d1``; python walk fallback."""
    try:
        out = subprocess.run(
            ["du", "-b", "-d", "1", run],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            timeout=timeout,
            text=True,
        ).stdout
    except subprocess.TimeoutExpired:
        return None
    except OSError:
        out = ""
    total = store = None
    for line in out.splitlines():
        size, _, path = line.partition("\t")
        try:
            size = int(size)
        except ValueError:
            continue
        path = path.rstrip("/")
        if path == run.rstrip("/"):
            total = size
        elif os.path.basename(path) == STORE_DIR:
            store = size
    if total is None:  # du missing / unsupported flags (busybox): walk
        deadline = time.monotonic() + timeout if timeout else None
        total = store = 0
        store_root = os.path.join(run, STORE_DIR)
        for root, _, files in os.walk(run):
            if deadline and time.monotonic() > deadline:
                return None
            for f in files:
                try:
                    n = os.lstat(os.path.join(root, f)).st_size
                except OSError:
                    continue
                total += n
                if root == store_root or root.startswith(store_root + os.sep):
                    store += n
        if not os.path.isdir(store_root):
            store = None
    return total, store


def _connect_ro(db):
    last = None
    for opts in ("mode=ro", "mode=ro&immutable=1"):
        try:
            conn = sqlite3.connect(f"file:{quote(db)}?{opts}", uri=True, timeout=2)
            conn.execute("SELECT 1 FROM sqlite_master LIMIT 1").fetchone()
            return conn
        except sqlite3.Error as e:
            last = e
    raise last


def _first_dir(run, rels):
    for rel in rels:
        if os.path.isdir(os.path.join(run, rel)):
            return os.path.join(run, rel)
    return None


def raw_log_info(run):
    """Counts of a run's raw image logs (``None`` if it has none): keyframe archives
    (``*.zip``) and track dirs with a track.json."""
    kf_dir, tr_dir = _first_dir(run, KEYFRAME_DIRS), _first_dir(run, TRACK_DIRS)
    if kf_dir is None and tr_dir is None:
        return None
    zips = tracks = 0
    try:
        if kf_dir:
            zips = sum(
                1 for e in os.scandir(kf_dir) if e.name.endswith(".zip") and e.is_file()
            )
        if tr_dir:
            tracks = sum(
                1 for e in os.scandir(tr_dir)
                if e.is_dir() and os.path.isfile(os.path.join(e.path, "track.json"))
            )  # fmt: skip
    except OSError:
        pass
    return {"keyframe_archives": zips, "tracks": tracks}


def store_info(store_dir, run=None):
    """Counts + quick structural check of one image store (``None`` if absent).

    ``run``: the run directory; a run with raw image logs but no ``image_store/`` yet
    (never indexed) is described by those counts (``indexed: false``, ``raw``): the
    console indexes it when the map is first opened.
    """
    db = os.path.join(store_dir, STORE_DB)
    raw = raw_log_info(run) if run else None
    if not os.path.isfile(db):
        if raw is None:
            return None
        return {"indexed": False, "raw": raw, "frames": {}, "frames_total": 0, "observations": 0,
                "sensors": 0, "check": {"ok": True, "problems": []}}  # fmt: skip
    info = {
        "frames": {},
        "frames_total": 0,
        "observations": 0,
        "sensors": 0,
        "indexed": True,
    }
    if raw is not None:
        info["raw"] = raw
    problems = []
    try:
        conn = _connect_ro(db)
    except sqlite3.Error as e:
        return dict(info, check={"ok": False, "problems": [f"cannot open index: {e}"]})
    try:
        one = lambda sql: conn.execute(sql).fetchone()[0]  # noqa: E731
        try:
            version = conn.execute(
                "SELECT value FROM meta WHERE key = 'schema_version'"
            ).fetchone()
            info["schema_version"] = version[0] if version else None
            if not version:
                problems.append("no schema_version")
        except sqlite3.Error:
            problems.append("no meta table")
        for kind, n in conn.execute("SELECT kind, COUNT(*) FROM frames GROUP BY kind"):
            info["frames"][KIND_NAMES.get(kind, str(kind))] = n
            if kind not in KIND_NAMES:
                problems.append(f"{n} frames of unknown kind {kind}")
        info["frames_total"] = sum(info["frames"].values())
        info["observations"] = one("SELECT COUNT(*) FROM observations")
        info["sensors"] = one("SELECT COUNT(*) FROM sensors")
        orphans = one(
            "SELECT COUNT(*) FROM frames WHERE sensor_id NOT IN (SELECT sensor_id FROM sensors)"
        )
        if orphans:
            problems.append(f"{orphans} frames without sensor")
        unanchored = one(
            "SELECT COUNT(*) FROM frames WHERE kind = 0 AND anchor_node_id IS NULL"
        )
        info["keyframes_unanchored"] = unanchored
        if unanchored:
            problems.append(f"{unanchored} keyframes unanchored")
        refs = []
        for order in ("ASC", "DESC"):
            refs += conn.execute(
                "SELECT rgb_path, depth_path FROM frames "
                "WHERE rgb_path IS NOT NULL OR depth_path IS NOT NULL "
                f"ORDER BY frame_id {order} LIMIT {CHECK_SAMPLES}"
            ).fetchall()
        paths = {p for row in refs for p in row if p}
        missing = sorted(
            p for p in paths if not os.path.exists(os.path.join(store_dir, p))
        )
        if missing:
            problems.append(f"{len(missing)}/{len(paths)} sampled image files missing")
        info["sampled_files"] = len(paths)
    except sqlite3.Error as e:
        problems.append(f"query failed: {e}")
    finally:
        conn.close()
    info["check"] = {"ok": not problems, "problems": problems}
    return info


def run_info(run) -> dict | None:
    """``<run>/run_info.json`` (written by run-adt4 at launch), if any."""
    try:
        with open(os.path.join(run, "run_info.json")) as f:
            data = json.load(f)
    except (OSError, ValueError):
        return None
    return data if isinstance(data, dict) else None


def scan_run(run, known=None, measure=True, timeout=None) -> dict:
    """One run directory -> listing entry (see module doc). ``known`` is keyed by path."""
    run = str(run)
    name = os.path.basename(run.rstrip("/"))
    try:
        subdirs = sorted(
            e.name for e in os.scandir(run) if e.is_dir(follow_symlinks=False)
        )
    except OSError:
        subdirs = []
    entry = {"name": name, "path": run, "subdirs": subdirs, "mtime": _mtime(run)}
    try:
        entry["ctime"] = os.stat(run).st_ctime
    except OSError:
        entry["ctime"] = None
    for key, rel in DSG_FILES.items():
        entry[key] = os.path.isfile(os.path.join(run, rel))
    if entry["has_dsg"]:
        entry["dsg_mtime"] = _mtime(os.path.join(run, DSG_FILES["has_dsg"]))
    store = store_info(os.path.join(run, STORE_DIR), run)
    entry["has_image_store"] = store is not None
    entry["image_store"] = store
    info = run_info(run)
    if info:
        entry["run_info"] = {
            k: info[k] for k in ("experiment", "robot", "run_id") if k in info
        }
    sig = signature(run, subdirs)
    entry["signature"] = sig
    cached = (known or {}).get(run)
    sizes = None
    if cached and len(cached) >= 3 and cached[0] == sig:
        sizes = (cached[1], cached[2])
    elif measure:
        sizes = _du_tree(run, timeout)
    if sizes is None:
        entry["size_bytes"] = None
        entry["size_pending"] = True
    else:
        entry["size_bytes"] = sizes[0]
        entry["size_pending"] = False
        if store is not None:
            store["size_bytes"] = sizes[1]
    return entry


def human_size(n: int | None) -> str:
    """``du -sh`` style size (1024 based, one decimal under 10)."""
    if n is None:
        return "?"
    v, units = float(n), "BKMGT"
    i = 0
    while v >= 1024 and i < len(units) - 1:
        v /= 1024
        i += 1
    if i == 0:
        return f"{int(v)}B"
    return f"{v:.1f}{units[i]}" if v < 10 else f"{v:.0f}{units[i]}"


_NAME_DATE_PATTERNS = [
    (re.compile(r"(?<!\d)(20\d\d)[-_.](\d\d)[-_.](\d\d)(?!\d)"), "ymd"),
    (re.compile(r"(?<!\d)(20\d\d)(\d\d)(\d\d)(?!\d)"), "ymd"),
    (re.compile(r"(?<!\d)(\d\d)(\d\d)(20\d\d)(?!\d)"), "mdy"),
    (re.compile(r"(?<!\d)(20\d\d)[-_](\d\d)(?![\d])"), "ym"),
]


def name_date(name: str) -> float | None:
    """Epoch seconds (local midnight) of a date embedded in a run name, else None.

    Understands ``2026-10-18`` / ``2026_10_18``, ``20261018`` (YYYYMMDD), ``03192026``
    (MMDDYYYY) and ``2026-07`` (first of the month).
    """
    for pat, kind in _NAME_DATE_PATTERNS:
        for m in pat.finditer(name):
            g = [int(x) for x in m.groups()]
            if kind == "ymd":
                y, mo, d = g
            elif kind == "mdy":
                mo, d, y = g
            else:
                (y, mo), d = g, 1
            try:
                return datetime.datetime(y, mo, d).timestamp()
            except ValueError:
                continue
    return None


def is_prior_map(entry: dict) -> bool:
    """A run usable as a prior map (hydra/ and roman/ were both saved)."""
    subdirs = entry.get("subdirs", [])
    return all(d in subdirs for d in PRIOR_SUBDIRS)


def finish_entry(entry: dict) -> dict:
    """Derived listing fields."""
    entry["size"] = human_size(entry.get("size_bytes"))
    entry["name_date"] = name_date(entry.get("name", ""))
    entry["is_prior"] = is_prior_map(entry)
    return entry
