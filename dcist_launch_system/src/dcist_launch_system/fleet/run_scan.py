"""Scan the run directories under an output root: what each run holds and how big it is.

Standard library only (python >= 3.6): the console runs it in-process for the local
machine and ships this file's source over ssh for remote machines (see
``fleet.maps.remote_scan_command``), so one ssh round trip lists a whole machine and
nothing has to be installed there.  Everything is read-only: the image store index is
opened with ``mode=ro`` (``immutable=1`` as a fallback), no cache file is written.

``du`` is the slow part on big runs, so sizes are optional per call: the caller passes
the sizes it already knows (``known``: name -> [signature, size, store_size]) and a time
``budget``.  A run whose signature (newest mtime of the run dir, its immediate
subdirectories, ``hydra/backend`` and the store index) still matches keeps its cached
size; others are measured until the budget runs out and the rest come back with
``size_pending: true`` for the caller to fill later (``budget=None`` = no limit).

Run identity (see ``fleet.run_identity``): each entry carries ``identity`` -- the
``run_id`` from ``<run>/run_info.json`` (written by run-adt4 at launch), else from
``.adt4_origin.json`` (a transferred copy), else a stable fallback derived from host +
realpath + creation time (older runs; nothing is written) -- plus ``origin`` (the copy
metadata, if any) and ``manifest`` (a digest of the run's files outside the image store
and rosbags, which a copy made with ``rsync -a`` reproduces exactly) used to tell an
in-sync copy from an outdated one.
"""

import base64
import hashlib
import json
import os
import socket
import sqlite3
import subprocess
import sys
import time

DSG_FILES = {
    "has_dsg": "hydra/backend/dsg.json",
    "has_dsg_mesh": "hydra/backend/dsg_with_mesh.json",
    "has_mesh": "hydra/backend/mesh.ply",
}
STORE_DIR = "image_store"
STORE_DB = "store.sqlite"
# kind 1 (sub-keyframes) only exists in stores written before hydra_image_store
# dropped it; such a store needs `hydra-image-store-adt4 convert` before it can be read
KIND_NAMES = {0: "keyframe", 1: "subkeyframe", 2: "object"}
# Hydra/Khronos raw image logs (FORMAT.md): keyframe InputData zips and per-track
# logs, which hydra_image_store indexes into image_store/ (the tailer while Hydra
# runs, else ImageStore.open / pack on first use). First existing dir wins, as in
# hydra_image_store.indexer.
KEYFRAME_DIRS = ("keyframes", "hydra/keyframes")
TRACK_DIRS = ("tracks", "hydra/tracks", "khronos/tracks")
RAW_LOG_DIRS = KEYFRAME_DIRS + TRACK_DIRS
CHECK_SAMPLES = 4  # referenced files checked from each end of the frames table
RUN_INFO = "run_info.json"
ORIGIN = ".adt4_origin.json"
# never part of the manifest digest: the image store is repacked on transfer (its
# index is fingerprinted separately) and the raw image logs it is built from are not
# sent with a packed transfer, bags are not copied by default, and the origin file
# exists only on a copy
MANIFEST_SKIP_DIRS = {STORE_DIR, "recorded_data", "image_store.partial"}
MANIFEST_SKIP_REL = set(RAW_LOG_DIRS)  # relative to the run dir
MANIFEST_SKIP_FILES = {ORIGIN}
MANIFEST_MAX_FILES = 50000
MAP_DIRS = ("hydra/", "roman/")  # the map a relocalize loads (manifest map_digest)


def _mtime(path):
    try:
        return os.stat(path).st_mtime
    except OSError:
        return None


def signature(run, subdirs):
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
            universal_newlines=True,
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
    from urllib.parse import quote

    last = None
    for opts in ("mode=ro", "mode=ro&immutable=1"):
        try:
            conn = sqlite3.connect(
                "file:%s?%s" % (quote(db), opts), uri=True, timeout=2
            )
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
    (``*.zip``; ``.tmp`` files are being written) and track dirs with a track.json."""
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
    events = os.path.isdir(os.path.join(run, "events")) or os.path.isdir(
        os.path.join(run, "hydra", "events")
    )
    return {"keyframe_archives": zips, "tracks": tracks, "events": events}


def _tail_state(conn):
    try:
        row = conn.execute(
            "SELECT value FROM meta WHERE key = 'indexer.tail'"
        ).fetchone()
        return json.loads(row[0]) if row else None
    except (sqlite3.Error, ValueError):
        return None


def store_info(store_dir, run=None):
    """Counts + quick structural check of one image store (``None`` if absent).

    ``run``: the run directory; a run with raw image logs but no ``image_store/`` yet
    (never indexed: no tailer ran, nothing opened it) is described by those counts
    (``indexed: false``, ``raw``), since ``ImageStore.open`` / ``pack`` index it on
    first use. An indexed store also reports ``raw`` and its tailer status (``tail``).
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
        return dict(
            info, check={"ok": False, "problems": ["cannot open index: %s" % e]}
        )
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
        info["frames_total"] = sum(info["frames"].values())
        if info["frames"].get(KIND_NAMES[1]):
            info["needs_convert"] = True
            problems.append(
                "sub-keyframes (frame kind 1): convert the store once with "
                "`hydra-image-store-adt4 convert %s`" % store_dir
            )
        tail = _tail_state(conn)
        if tail:
            info["tail"] = {
                k: tail.get(k) for k in ("live", "final", "heartbeat_wall", "session")
            }
        info["observations"] = one("SELECT COUNT(*) FROM observations")
        info["sensors"] = one("SELECT COUNT(*) FROM sensors")
        orphans = one(
            "SELECT COUNT(*) FROM frames WHERE sensor_id NOT IN (SELECT sensor_id FROM sensors)"
        )
        if orphans:
            problems.append("%d frames without sensor" % orphans)
        unanchored = one(
            "SELECT COUNT(*) FROM frames WHERE kind = 0 AND anchor_node_id IS NULL"
        )
        info["keyframes_unanchored"] = unanchored
        if unanchored:
            problems.append("%d keyframes unanchored" % unanchored)
        refs = []
        for order in ("ASC", "DESC"):
            refs += conn.execute(
                "SELECT rgb_path, depth_path FROM frames "
                "WHERE rgb_path IS NOT NULL OR depth_path IS NOT NULL "
                "ORDER BY frame_id %s LIMIT %d" % (order, CHECK_SAMPLES)
            ).fetchall()
        paths = {p for row in refs for p in row if p}
        missing = sorted(
            p for p in paths if not os.path.exists(os.path.join(store_dir, p))
        )
        if missing:
            problems.append(
                "%d/%d sampled image files missing" % (len(missing), len(paths))
            )
        info["sampled_files"] = len(paths)
    except sqlite3.Error as e:
        problems.append("query failed: %s" % e)
    finally:
        conn.close()
    info["check"] = {"ok": not problems, "problems": problems}
    return info


def _read_json(path):
    try:
        with open(path) as f:
            data = json.load(f)
    except (OSError, ValueError):
        return None
    return data if isinstance(data, dict) else None


def _birth_time(path):
    """Creation time of ``path`` (statx via ``stat -c %W``), else None."""
    bt = getattr(os.stat(path), "st_birthtime", None) if os.path.exists(path) else None
    if bt:
        return float(bt)
    try:
        out = subprocess.run(
            ["stat", "-c", "%W", path],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            timeout=5,
            universal_newlines=True,
        ).stdout.strip()
        v = float(out)
        return v if v > 0 else None
    except (OSError, ValueError, subprocess.TimeoutExpired):
        return None


def fallback_identity(run, host=None):
    """Stable id of a run without run_info.json: host + realpath + creation time.

    The creation time is that of ``hydra/`` (created once at launch; the run dir
    itself is sometimes recreated by copies), falling back to the run dir, then to
    the mtime of ``hydra/hydra_config.yaml``.  Nothing is written.
    """
    host = host or socket.gethostname()
    real = os.path.realpath(run)
    born = None
    for p in (os.path.join(run, "hydra"), run):
        if os.path.isdir(p):
            born = _birth_time(p)
            if born:
                break
    if not born:
        born = (
            _mtime(os.path.join(run, "hydra", "hydra_config.yaml")) or _mtime(run) or 0
        )
    key = "%s|%s|%d" % (host, real, int(born))
    return {
        "run_id": "legacy-" + hashlib.sha1(key.encode()).hexdigest()[:20],
        "source": "fallback",
        "host": host,
        "created": born,
    }


def run_identity(run):
    """``(identity, origin)`` of a run dir (see module doc)."""
    info = _read_json(os.path.join(run, RUN_INFO))
    origin = _read_json(os.path.join(run, ORIGIN))
    if origin is not None and not origin.get("run_id"):
        origin = None
    if info and info.get("run_id"):
        ident = dict(info)
        ident["source"] = "run_info"
    elif origin is not None:
        ident = {"run_id": origin["run_id"], "source": "origin"}
    else:
        ident = fallback_identity(run)
    return ident, origin


def manifest(run):
    """Digest of (path, size, mtime) of every file outside the image store / bags.

    ``rsync -a`` preserves sizes and mtimes, so a complete copy has the same digest
    as its source; any file the robot adds or rewrites afterwards changes it.
    Returns {"digest", "files", "newest"} (digest None when over the file cap).
    """
    rows = []
    newest = 0.0
    run = run.rstrip("/")
    for root, dirs, files in os.walk(run):
        rel_root = os.path.relpath(root, run)
        dirs[:] = sorted(
            d for d in dirs
            if d not in MANIFEST_SKIP_DIRS
            and (d if rel_root == "." else rel_root + "/" + d) not in MANIFEST_SKIP_REL
        )  # fmt: skip
        for f in files:
            if rel_root == "." and f in MANIFEST_SKIP_FILES:
                continue
            try:
                st = os.lstat(os.path.join(root, f))
            except OSError:
                continue
            rel = f if rel_root == "." else rel_root + "/" + f
            rows.append("%s\t%d\t%d" % (rel, st.st_size, int(st.st_mtime)))
            newest = max(newest, st.st_mtime)
        if len(rows) > MANIFEST_MAX_FILES:
            return {
                "digest": None,
                "map_digest": None,
                "files": len(rows),
                "newest": round(newest, 3),
            }
    rows.sort()
    digest = hashlib.sha1("\n".join(rows).encode()).hexdigest()
    # the map alone (what a relocalize loads as the prior), so a copy that gained or
    # lost unrelated files still shows the same map
    map_rows = [r for r in rows if r.startswith(MAP_DIRS)]
    map_digest = (
        hashlib.sha1("\n".join(map_rows).encode()).hexdigest() if map_rows else None
    )
    return {
        "digest": digest,
        "map_digest": map_digest,
        "files": len(rows),
        "newest": round(newest, 3),
    }


def store_fingerprint(store):
    """What changes when the robot adds frames: counts from ``store_info`` (raw log
    counts for a run that was not indexed yet)."""
    if not store:
        return None
    if store.get("indexed") is False:
        raw = store.get("raw") or {}
        return "raw:%s:%s" % (raw.get("keyframe_archives"), raw.get("tracks"))
    frames = store.get("frames") or {}
    return "%s:%s:%s" % (
        store.get("frames_total"),
        store.get("observations"),
        ",".join("%s=%s" % kv for kv in sorted(frames.items())),
    )


def dsg_signature(run):
    """``"<size>:<mtime>"`` of the saved scene graph: two dirs with the same one hold
    the same run's map (rsync -a keeps both), even if other files differ."""
    try:
        st = os.stat(os.path.join(run, DSG_FILES["has_dsg"]))
    except OSError:
        return None
    return "%d:%d" % (st.st_size, int(st.st_mtime))


def scan_identity(run, with_manifest=False):
    """Light listing entry: name, path, subdirs, identity, origin (no sizes/stores).

    ``with_manifest``: also the file ``manifest`` and ``dsg_sig``, to recognise copies
    made before copies recorded their origin.
    """
    name = os.path.basename(run.rstrip("/"))
    try:
        subdirs = sorted(
            e.name for e in os.scandir(run) if e.is_dir(follow_symlinks=False)
        )
    except OSError:
        subdirs = []
    ident, origin = run_identity(run)
    entry = {
        "name": name,
        "path": run,
        "subdirs": subdirs,
        "identity": ident,
        "origin": origin,
    }
    if with_manifest:
        entry["manifest"] = manifest(run)
        entry["dsg_sig"] = dsg_signature(run)
    return entry


def scan_run(run, known=None, measure=True, timeout=None):
    """One run directory -> listing entry (see module doc)."""
    name = os.path.basename(run.rstrip("/"))
    try:
        subdirs = sorted(
            e.name for e in os.scandir(run) if e.is_dir(follow_symlinks=False)
        )
    except OSError:
        subdirs = []
    entry = {
        "name": name,
        "path": run,
        "subdirs": subdirs,
        "mtime": _mtime(run),
    }
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
    entry["identity"], entry["origin"] = run_identity(run)
    entry["manifest"] = manifest(run)
    entry["store_fingerprint"] = store_fingerprint(store)
    sessions = os.path.join(run, "sessions")
    if os.path.isdir(sessions):
        try:
            entry["sessions"] = sum(
                1
                for s in os.listdir(sessions)
                if os.path.isdir(os.path.join(sessions, s, STORE_DIR))
            )
        except OSError:
            entry["sessions"] = 0
    sig = signature(run, subdirs)
    entry["signature"] = sig
    cached = (known or {}).get(name)
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


def scan_root(
    root, known=None, budget=None, only=None, extra=(), light=False, with_manifest=False
):
    """Every run directory under ``root`` (plus ``extra`` absolute run dirs).

    ``light``: identity only (``scan_identity``), for launch-time prior lookups.
    """
    root = os.path.expanduser(root)
    runs = []
    try:
        runs = sorted(
            e.path
            for e in os.scandir(root)
            if e.is_dir() and not e.name.startswith(".")
        )
    except OSError:
        pass
    for x in extra:
        x = os.path.expanduser(x)
        if x not in runs and os.path.isdir(x):
            runs.append(x)
    if only is not None:
        only = set(only)
        runs = [r for r in runs if os.path.basename(r) in only]
    deadline = None if budget is None else time.monotonic() + budget
    out = []
    for run in runs:
        if light:
            try:
                out.append(scan_identity(run, with_manifest))
            except Exception as e:  # noqa: BLE001
                out.append(
                    {
                        "name": os.path.basename(run),
                        "path": run,
                        "subdirs": [],
                        "error": str(e),
                    }
                )
            continue
        left = None if deadline is None else deadline - time.monotonic()
        measure = left is None or left > 0.05
        try:
            out.append(scan_run(run, known, measure=measure, timeout=left))
        except Exception as e:  # noqa: BLE001 - one bad run must not hide the rest
            out.append(
                {
                    "name": os.path.basename(run),
                    "path": run,
                    "subdirs": [],
                    "error": str(e),
                }
            )
    return out


def main(argv):
    """``python run_scan.py <base64 json {root, known, budget, only, extra, light}>`` -> JSON."""
    args = json.loads(base64.b64decode(argv[1]).decode()) if len(argv) > 1 else {}
    runs = scan_root(
        args.get("root", "~/adt4_output"),
        known=args.get("known"),
        budget=args.get("budget"),
        only=args.get("only"),
        extra=args.get("extra") or (),
        light=bool(args.get("light")),
        with_manifest=bool(args.get("manifest")),
    )
    sys.stdout.write(
        json.dumps(
            {
                "root": os.path.expanduser(args.get("root", "~")),
                "host": socket.gethostname(),
                "runs": runs,
            }
        )
    )


if __name__ == "__main__":
    main(sys.argv)
