"""Per-trial rosbag recording on one machine, independent of the ADT4 stack.

Runs ON THE ROBOT (and on the base station), started over ssh by the console
through ``dcist_launch_system/bin/adt4-rec``.  Standard library only, and it
needs no sourced workspace itself: only the recorder it starts does.

A *trial* is one operator-started recording across the fleet.  The console
makes the trial id and gives the same id to every machine; each machine records
into its own bag directory::

    <output_root>/<trial_id>/<machine>/        a normal rosbag2 directory, plus
        trial.json        who / what / when (written here, updated at stop)
        recorder.log      everything the recorder printed
        pane_*.txt        the stack windows listed in recording/capture_panes.txt
                          (none by default), as text, at start and at stop

Design rules (keep them when editing):

* The recorder runs in its own tmux session (``adt4_rec``), never in
  ``adt4_system``; starting or stopping it sends nothing to the stack's panes.
  Reading the stack is limited to ``tmux show-environment`` and
  ``tmux capture-pane``.
* Nothing here talks to another machine.  Once started, a recording keeps
  going with no network at all.
* Nothing is overwritten or deleted, and the recorder is never ``kill -9``'d:
  a bag only gets its ``metadata.yaml`` when the recorder exits by itself.
* A full disk is reported (``free_bytes``), never acted on.
* Every command prints exactly one JSON object on its last line; the exit
  status is 0 only when the command did what was asked.

Commands: ``start``, ``status``, ``stop``, ``check-topics``.
"""

from __future__ import annotations

import argparse
import datetime
import json
import os
import pathlib
import re
import shlex
import signal
import socket
import subprocess
import sys
import time

SCHEMA = 1
SESSION = "adt4_rec"
STACK_SESSION = "adt4_system"
TRIAL_FILE = "trial.json"
LOG_FILE = "recorder.log"
DEFAULT_OUTPUT_ROOT = "~/adt4_output"

# trial ids are directory names made by the console: trial_<UTC stamp>_<slug>
TRIAL_RE = re.compile(r"^trial_[A-Za-z0-9][A-Za-z0-9._+-]{0,110}$")
NAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$")

# the stack's settings a recorder must share to see the same topics
ENV_PREFIXES = (
    "ADT4_",
    "ROS_",
    "RMW_",
    "ZENOH_",
    "CYCLONEDDS_",
    "FASTRTPS_",
    "FASTDDS_",
)

START_WAIT_S = 20.0  # for the recorder process and its bag directory to appear
GROWTH_WAIT_S = 3.0  # then: does the bag grow in this long?
STOP_WAIT_S = 45.0  # for the recorder to close its bag after SIGINT
EXIT_LINGER_S = 30  # the pane stays this long after the recorder exits
PANES_FILE = "capture_panes.txt"  # recording/: which stack windows to save
PANE_MAX_LINES = 5000  # scrollback kept per captured pane, at most
EXIT_MARK = "[adt4-rec] recorder exited"

# exit statuses
OK, FAILED, USAGE, BUSY, STILL_RUNNING = 0, 1, 2, 3, 4


# --------------------------------------------------------------------- pure
def valid_trial(trial) -> bool:
    return bool(trial) and TRIAL_RE.match(trial) is not None and ".." not in trial


def valid_name(name) -> bool:
    return bool(name) and NAME_RE.match(name) is not None and ".." not in name


def utc_iso(t: float) -> str:
    return (
        datetime.datetime.fromtimestamp(t, datetime.timezone.utc)
        .isoformat(timespec="milliseconds")
        .replace("+00:00", "Z")
    )


_VAR_RE = re.compile(r"\$(\w+|\{[^}]*\})", re.ASCII)


def expand_topic(line: str, env: dict) -> str:
    """``$VAR`` / ``${VAR}`` from ``env``; an unknown variable is left as is
    (then ``parse_topics`` reports the line instead of recording a wrong name)."""

    def sub(m):
        name = m.group(1).strip("{}")
        return env.get(name, m.group(0))

    return _VAR_RE.sub(sub, line)


FLEET_VAR = "ADT4_FLEET_ROBOT"


def parse_topics(text: str, env: dict, robots=None) -> tuple[list[str], list[str]]:
    """(topics, problems) of a ``*_topics.txt``: one topic per line, blank
    lines and ``#`` comments skipped, duplicates dropped, order kept.  A line
    with ``${ADT4_FLEET_ROBOT}`` stands for one topic per robot in ``robots``
    (the base station records every robot's plan)."""
    topics, problems = [], []
    for n, raw in enumerate((text or "").splitlines(), 1):
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        if FLEET_VAR in line:
            if not robots:
                problems.append(
                    f"line {n}: {line!r} needs the fleet's robots (--robots)"
                )
                continue
            expanded = [expand_topic(line, {**env, FLEET_VAR: r}) for r in robots]
        else:
            expanded = [expand_topic(line, env)]
        for topic in expanded:
            if "$" in topic:
                problems.append(f"line {n}: unset variable in {line!r}")
            elif not re.fullmatch(r"/[A-Za-z0-9_/~{}]+", topic):
                problems.append(f"line {n}: not a topic name: {topic!r}")
            elif topic not in topics:
                topics.append(topic)
    return topics, problems


def parse_tmux_env(text: str) -> dict:
    """``tmux show-environment`` output -> dict (``-NAME`` lines are unset)."""
    env = {}
    for line in (text or "").splitlines():
        if line.startswith("-") or "=" not in line:
            continue
        k, v = line.split("=", 1)
        env[k] = v
    return env


def shared_env(stack_env: dict) -> dict:
    """The stack session's variables the recorder is started with."""
    return {
        k: v
        for k, v in sorted(stack_env.items())
        if k.startswith(ENV_PREFIXES) and re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", k)
    }


def is_recorder(argv) -> bool:
    """``.../ros2 bag record ...`` (the ros2 CLI records in-process)."""
    for i, a in enumerate(argv or []):
        if os.path.basename(a) == "ros2" and list(argv[i + 1 : i + 3]) == [
            "bag",
            "record",
        ]:
            return True
    return False


def output_arg(argv):
    for i, a in enumerate(argv or []):
        if a in ("-o", "--output") and i + 1 < len(argv):
            return argv[i + 1]
        if a.startswith("--output="):
            return a.split("=", 1)[1]
    return None


def descendants(procs: dict, root: int) -> list[int]:
    """``root`` and every process below it; ``procs``: {pid: {"ppid", ...}}."""
    kids: dict[int, list[int]] = {}
    for pid, p in procs.items():
        kids.setdefault(p.get("ppid"), []).append(pid)
    out, todo = [], [root]
    while todo and len(out) < 4096:
        pid = todo.pop()
        if pid in out:
            continue
        out.append(pid)
        todo.extend(kids.get(pid, []))
    return out


def find_recorders(procs: dict, pane_pids) -> tuple[list[int], list[int]]:
    """(ours, others): recorder pids under one of our session's panes, and
    every other top-level ``ros2 bag record`` on the machine."""
    mine: set[int] = set()
    for pp in pane_pids:
        mine.update(descendants(procs, pp))
    recs = sorted(pid for pid, p in procs.items() if is_recorder(p.get("argv")))
    top = [pid for pid in recs if procs[pid].get("ppid") not in recs]
    return [p for p in top if p in mine], [p for p in top if p not in mine]


STORAGE_CONFIG = "mcap_zstd.yaml"  # recording/: lossless zstd of the mcap chunks


def compression_args(env: dict | None = None) -> list[str]:
    """Record with ``recording/mcap_zstd.yaml`` (lossless zstd chunk compression:
    about a third smaller on camera data for a fraction of one core).
    ``ADT4_REC_COMPRESSION=none`` records raw, and so does a checkout without
    the file: compression must never be why a recording does not start."""
    env = os.environ if env is None else env
    if env.get("ADT4_REC_COMPRESSION", "zstd") in ("", "none", "off", "0"):
        return []
    path = package_root() / "recording" / STORAGE_CONFIG
    return ["--storage-config-file", str(path)] if path.is_file() else []


def record_command(
    bag_dir: str, topics: list[str], extra: list[str] | None = None
) -> list[str]:
    """The same recorder the stack's ``recording`` window runs (``record_topics``),
    with lossless zstd compression of the bag (see ``compression_args``)."""
    return [
        "ros2", "bag", "record", "-o", bag_dir,
        *compression_args(), *(extra or []), "--topics", *topics,
    ]  # fmt: skip


def pane_script(
    env: dict, setup: str, cmd: list[str], linger_s: int = EXIT_LINGER_S
) -> str:
    """What the recorder pane runs.  It stays ``linger_s`` after the recorder
    exits so its last words can still be read from the pane."""
    lines = [f"export {k}={shlex.quote(v)}" for k, v in env.items()]
    if setup:
        lines.append(setup)
    lines.append(" ".join(shlex.quote(c) for c in cmd))
    lines.append(f'echo "{EXIT_MARK} with status $?"')
    lines.append(f"sleep {int(linger_s)}")
    return "\n".join(lines)


def parse_metadata(text: str) -> dict | None:
    """Per-topic message counts, duration and total of a rosbag2
    ``metadata.yaml``.  Uses PyYAML when the machine has it, else a line scan
    (enough for the files rosbag2 writes)."""
    try:
        import yaml  # noqa: PLC0415

        info = yaml.safe_load(text)["rosbag2_bagfile_information"]
        return {
            "message_count": info.get("message_count"),
            "duration_s": (info.get("duration") or {}).get("nanoseconds", 0) / 1e9,
            "topics": {
                t["topic_metadata"]["name"]: t.get("message_count")
                for t in info.get("topics_with_message_count") or []
            },
        }
    except ImportError:
        pass
    except Exception:  # noqa: BLE001
        return None
    topics, name, total, duration = {}, None, None, None
    section = None
    for line in (text or "").splitlines():
        s = line.strip()
        if s.startswith("topics_with_message_count"):
            section = "topics"
        elif s.startswith("duration:"):
            section = "duration"
        elif section == "duration" and s.startswith("nanoseconds:"):
            duration = int(s.split(":", 1)[1]) / 1e9
            section = None
        m = re.match(r"-?\s*name:\s*(\S+)", s)
        if section == "topics" and m:
            name = m.group(1).strip("'\"")
        m = re.match(r"message_count:\s*(\d+)", s)
        if m:
            if section == "topics" and name is not None:
                topics[name], name = int(m.group(1)), None
            elif total is None:
                total = int(m.group(1))
    if total is None and not topics:
        return None
    return {"message_count": total, "duration_s": duration, "topics": topics}


def silent_topics(wanted: list[str], counts: dict | None) -> list[str]:
    """Requested topics the closed bag holds no message of."""
    counts = counts or {}
    return [t for t in wanted if not counts.get(t)]


# ---------------------------------------------------------------- machine
def _run(argv, timeout=10.0, **kw):
    try:
        r = subprocess.run(
            argv,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=timeout,
            **kw,
        )
        return r.returncode, r.stdout, r.stderr
    except (OSError, subprocess.SubprocessError) as e:
        return -1, "", str(e)


def tmux(*args, timeout=10.0):
    return _run(["tmux", *args], timeout=timeout)


def session_exists(name: str) -> bool:
    return tmux("has-session", "-t", f"={name}")[0] == 0


def pane_pids(session: str) -> list[int]:
    rc, out, _ = tmux("list-panes", "-s", "-t", f"={session}", "-F", "#{pane_pid}")
    return [int(x) for x in out.split() if x.isdigit()] if rc == 0 else []


def stack_env() -> dict:
    rc, out, _ = tmux("show-environment", "-t", f"={STACK_SESSION}")
    return parse_tmux_env(out) if rc == 0 else {}


def read_procs() -> dict:
    """{pid: {"ppid", "argv"}} from /proc (Linux)."""
    procs = {}
    try:
        names = os.listdir("/proc")
    except OSError:
        return procs
    for d in names:
        if not d.isdigit():
            continue
        try:
            with open(f"/proc/{d}/stat") as f:
                stat = f.read()
            with open(f"/proc/{d}/cmdline", "rb") as f:
                raw = f.read()
        except OSError:
            continue
        rest = stat[stat.rfind(")") + 2 :].split()  # comm may hold spaces
        procs[int(d)] = {
            "ppid": int(rest[1]),
            "argv": [a.decode("utf-8", "replace") for a in raw.split(b"\0") if a],
        }
    return procs


def proc_cwd(pid: int) -> str | None:
    try:
        return os.readlink(f"/proc/{pid}/cwd")
    except OSError:
        return None


def alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def dir_usage(path) -> dict:
    """Bytes, file count and newest write of a bag directory."""
    total = files = 0
    newest = None
    try:
        for e in os.scandir(path):
            if not e.is_file(follow_symlinks=False):
                continue
            st = e.stat(follow_symlinks=False)
            if e.name.endswith((".mcap", ".db3")):
                newest = max(newest or 0, st.st_mtime)
            total, files = total + st.st_size, files + 1
    except OSError:
        return {"bytes": None, "files": 0, "last_write": None}
    return {"bytes": total, "files": files, "last_write": newest}


def disk(path) -> dict:
    """Free and total bytes of the filesystem holding ``path`` (or its nearest
    existing parent)."""
    p = pathlib.Path(os.path.expanduser(str(path)))
    while not p.exists() and p != p.parent:
        p = p.parent
    try:
        st = os.statvfs(p)
    except OSError:
        return {"free_bytes": None, "total_bytes": None}
    return {
        "free_bytes": st.f_bavail * st.f_frsize,
        "total_bytes": st.f_blocks * st.f_frsize,
    }


# the saved scene graph of a map directory (fleet/run_scan.py DSG_FILES)
MAP_FILES = ("hydra/backend/dsg.json", "hydra/backend/dsg_with_mesh.json")
HASH_MAX_BYTES = 4 * 10**9


def sha256_file(path, max_bytes: int = HASH_MAX_BYTES) -> str | None:
    """sha256 of a file; None when it cannot be read or is over ``max_bytes``."""
    import hashlib  # noqa: PLC0415

    try:
        if os.path.getsize(path) > max_bytes:
            return None
        h = hashlib.sha256()
        with open(path, "rb") as f:
            for chunk in iter(lambda: f.read(1 << 20), b""):
                h.update(chunk)
        return h.hexdigest()
    except OSError:
        return None


def map_identity(path) -> dict | None:
    """Which scene graph a map directory holds: its path, its run id (and the
    run it was copied from) and the sha256 of each saved scene graph file."""
    if not path:
        return None
    path = os.path.expanduser(str(path))
    out = {"path": path, "exists": os.path.isdir(path), "files": {}}
    if not out["exists"]:
        return out
    info = read_json(os.path.join(path, "run_info.json")) or {}
    origin = read_json(os.path.join(path, ".adt4_origin.json")) or {}
    out["run_id"] = info.get("run_id") or origin.get("run_id")
    out["experiment"] = info.get("experiment") or origin.get("experiment")
    out["source_machine"] = origin.get("source_machine")
    out["source_path"] = origin.get("source_path")
    for rel in MAP_FILES:
        f = os.path.join(path, rel)
        try:
            st = os.stat(f)
        except OSError:
            continue
        out["files"][rel] = {
            "sha256": sha256_file(f),
            "size": st.st_size,
            "mtime": round(st.st_mtime, 3),
        }
    return out


def read_json(path) -> dict | None:
    try:
        with open(path) as f:
            data = json.load(f)
    except (OSError, ValueError):
        return None
    return data if isinstance(data, dict) else None


def write_json_atomic(path, data: dict) -> None:
    tmp = f"{path}.tmp.{os.getpid()}"
    with open(tmp, "w") as f:
        json.dump(data, f, indent=2, sort_keys=True)
        f.write("\n")
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)


def git_commit(path) -> str | None:
    rc, out, _ = _run(["git", "-C", str(path), "rev-parse", "HEAD"], timeout=3)
    sha = out.strip()
    return sha if rc == 0 and re.fullmatch(r"[0-9a-f]{40}", sha) else None


def package_root() -> pathlib.Path:
    """``dcist_launch_system/`` (holds ``recording/``), from this file's place."""
    return pathlib.Path(__file__).resolve().parents[2]


def workspace_setup() -> str:
    """The command that sources the workspace, as the stack's panes do.
    ``$ADT4_REC_SETUP`` overrides it ("" = source nothing)."""
    if "ADT4_REC_SETUP" in os.environ:
        return os.environ["ADT4_REC_SETUP"]
    here = pathlib.Path(__file__).resolve()
    candidates = [p / "install" / "setup.zsh" for p in here.parents]
    candidates.append(pathlib.Path("~/dcist_ws/install/setup.zsh").expanduser())
    for c in candidates:
        if c.is_file():
            return f"source {shlex.quote(str(c))}"
    return ""


def pane_shell() -> list[str]:
    """``zsh -ic`` like the stack's panes (``$ADT4_REC_SHELL`` overrides)."""
    return shlex.split(os.environ.get("ADT4_REC_SHELL", "zsh -ic"))


def active() -> dict:
    """What is recording now: our recorder (pid, bag dir) and any other."""
    exists = session_exists(SESSION)
    procs = read_procs()
    ours, others = find_recorders(procs, pane_pids(SESSION) if exists else [])
    out = {"session": exists, "pid": None, "dir": None, "others": []}
    if ours:
        pid = ours[0]
        d = output_arg(procs[pid]["argv"])
        if d and not os.path.isabs(d):
            d = os.path.normpath(os.path.join(proc_cwd(pid) or "/", d))
        out.update(pid=pid, dir=d)
    for pid in others:
        out["others"].append({"pid": pid, "output": output_arg(procs[pid]["argv"])})
    return out


def trial_of(bag_dir) -> tuple[str | None, str | None]:
    """(trial id, machine) of ``<root>/<trial>/<machine>``."""
    if not bag_dir:
        return None, None
    p = pathlib.Path(bag_dir)
    return p.parent.name, p.name


# ----------------------------------------------------------------- status
def status(output_root: str) -> dict:
    now = time.time()
    act = active()
    out = {
        "ok": True,
        "schema": SCHEMA,
        "t": round(now, 3),
        "host": socket.gethostname(),
        "recording": act["pid"] is not None,
        "session": act["session"],
        "pid": act["pid"],
        "dir": act["dir"],
        "trial": None,
        "machine": None,
        "started": None,
        "bytes": None,
        "files": 0,
        "last_write_age_s": None,
        "other_recorders": act["others"],
        "stack_running": session_exists(STACK_SESSION),
        **disk(act["dir"] or output_root),
    }
    if act["dir"]:
        out["trial"], out["machine"] = trial_of(act["dir"])
        use = dir_usage(act["dir"])
        out["bytes"], out["files"] = use["bytes"], use["files"]
        if use["last_write"]:
            out["last_write_age_s"] = round(max(0.0, now - use["last_write"]), 1)
        info = read_json(os.path.join(act["dir"], TRIAL_FILE)) or {}
        out["started"] = info.get("start_time_unix")
        out["trial_file"] = bool(info)
    return out


# ------------------------------------------------------------------ start
def start(args) -> tuple[int, dict]:
    t0 = time.time()
    if not valid_trial(args.trial):
        return USAGE, {"ok": False, "error": f"bad trial id {args.trial!r}"}
    if not valid_name(args.machine):
        return USAGE, {"ok": False, "error": f"bad machine name {args.machine!r}"}

    act = active()
    if act["pid"] is not None:
        trial, _ = trial_of(act["dir"])
        return BUSY, {
            "ok": False,
            "error": f"already recording {trial or act['dir']} (pid {act['pid']})",
            "recording": True,
            "trial": trial,
            "dir": act["dir"],
        }
    if act["session"]:
        # ours, and its recorder is gone (it lingers EXIT_LINGER_S): clear it
        tmux("kill-session", "-t", f"={SESSION}")

    topics_file = pathlib.Path(
        args.topics_file
        or package_root() / "recording" / f"{args.topic_set}_topics.txt"
    ).expanduser()
    try:
        text = topics_file.read_text()
    except OSError as e:
        return FAILED, {
            "ok": False,
            "error": f"cannot read the topic list {topics_file}: {e}",
        }
    senv = stack_env()
    topic_env = {**os.environ, **senv, "ADT4_ROBOT_NAME": args.machine}
    robots = [r for r in (args.robots or "").split(",") if valid_name(r)]
    topics, problems = parse_topics(text, topic_env, robots)
    if problems or not topics:
        return FAILED, {
            "ok": False,
            "error": f"topic list {topics_file}: "
            + ("; ".join(problems) or "no topics"),
        }

    root = pathlib.Path(os.path.expanduser(args.output_root))
    bag_dir = root / args.trial / args.machine
    if bag_dir.exists():
        return FAILED, {
            "ok": False,
            "error": f"{bag_dir} exists already; not recording over it",
        }
    try:
        bag_dir.parent.mkdir(parents=True, exist_ok=True)
    except OSError as e:
        return FAILED, {"ok": False, "error": f"cannot create {bag_dir.parent}: {e}"}

    env = shared_env(senv)
    env["ADT4_ROBOT_NAME"] = args.machine
    cmd = record_command(str(bag_dir), topics, args.record_arg)
    script = pane_script(env, workspace_setup(), cmd)
    rc, _, err = tmux(
        "new-session", "-d", "-s", SESSION, "-n", "record", "-x", "220", "-y", "50",
        *pane_shell(), script,
    )  # fmt: skip
    if rc != 0:
        return FAILED, {
            "ok": False,
            "error": f"tmux could not start the recorder: {err.strip()}",
        }

    # wait for the recorder process and its bag directory
    pid, deadline = None, time.monotonic() + args.start_wait
    while time.monotonic() < deadline:
        act = active()
        pid = act["pid"]
        if pid is not None and bag_dir.is_dir():
            break
        if not act["session"] or (pid is None and EXIT_MARK in pane_text(SESSION)):
            break  # the recorder is gone already: no point waiting
        time.sleep(0.5)
    if pid is None or not bag_dir.is_dir():
        tail = pane_text(SESSION).rstrip()[-1500:]
        return FAILED, {
            "ok": False,
            "error": "the recorder did not start"
            + ("" if pid is None else " writing its bag directory"),
            "pane": tail,
            "dir": str(bag_dir),
        }

    # keep everything the recorder prints, next to the bag
    log_path = bag_dir / LOG_FILE
    try:
        log_path.write_text(pane_text(SESSION))
    except OSError:
        pass
    tmux(
        "pipe-pane", "-t", f"={SESSION}:", "-o", f"cat >> {shlex.quote(str(log_path))}"
    )

    run_dir = senv.get("ADT4_OUTPUT_DIR")
    run_info = read_json(os.path.join(run_dir, "run_info.json")) if run_dir else None
    now = time.time()
    info = {
        "schema": SCHEMA,
        "trial_id": args.trial,
        "machine": args.machine,
        "host": socket.gethostname(),
        "description": args.description or "",
        "started_at": utc_iso(now),
        "start_time_unix": round(now, 3),
        "start_requested_unix": round(t0, 3),
        # the console's clock when it sent the start; the difference to
        # start_requested_unix is this machine's clock offset plus the ssh delay
        "base_time_unix": args.base_time,
        "bag_dir": str(bag_dir),
        "topics_file": str(topics_file),
        "topics": topics,
        "command": cmd,
        "recorder_pid": pid,
        "stack_running": bool(senv) or session_exists(STACK_SESSION),
        "run": {
            "output_dir": run_dir,
            "run_id": (run_info or {}).get("run_id"),
            "experiment": (run_info or {}).get("experiment"),
            "git_commit": (run_info or {}).get("git_commit"),
        },
        "recorder_git_commit": git_commit(package_root()),
        "stopped_at": None,
    }
    # which scene graph the stack was launched on (path and file hashes)
    info["prior_map"] = map_identity(senv.get("ADT4_PRIOR_MAP"))
    info["stack_panes_start"] = capture_stack_panes(bag_dir, "start")
    try:
        write_json_atomic(bag_dir / TRIAL_FILE, info)
    except OSError as e:
        return FAILED, {
            "ok": False,
            "error": f"recording, but cannot write trial.json: {e}",
            "pid": pid,
        }

    # is the bag growing?  (a low-rate topic set may sit in the recorder's
    # write cache for longer than this; the console decides what it means)
    b0 = dir_usage(bag_dir)["bytes"] or 0
    time.sleep(max(0.0, args.growth_wait))
    b1 = dir_usage(bag_dir)["bytes"] or 0
    still = alive(pid)
    return (OK if still else FAILED), {
        "ok": still,
        "error": None if still else "the recorder exited right after starting",
        "recording": still,
        "trial": args.trial,
        "machine": args.machine,
        "dir": str(bag_dir),
        "pid": pid,
        "topics": len(topics),
        "bytes": b1,
        "growing": b1 > b0,
        "stack_running": info["stack_running"],
        "other_recorders": act["others"],
        "t": round(time.time(), 3),
        **disk(bag_dir),
    }


def pane_text(target: str, lines: int | None = None) -> str:
    """Text of a pane (a ``%id``) or of a session's current pane: all its
    scrollback, or with ``lines`` the visible screen plus that many lines."""
    if not target.startswith("%"):
        target = f"={target}:"
    since = "-" if lines is None else str(-int(lines))
    rc, out, _ = tmux("capture-pane", "-p", "-J", "-S", since, "-t", target)
    return out if rc == 0 else ""


# ------------------------------------------------------------------- stop
def parse_panes_spec(text: str) -> dict[str, int]:
    """``capture_panes.txt`` -> {window name: scrollback lines (0 = the
    visible screen only)}."""
    spec = {}
    for raw in (text or "").splitlines():
        parts = raw.split("#", 1)[0].split()
        if not parts:
            continue
        lines = int(parts[1]) if len(parts) > 1 and parts[1].isdigit() else 0
        spec[parts[0]] = min(lines, PANE_MAX_LINES)
    return spec


def panes_spec() -> dict[str, int]:
    """The windows to capture: ``recording/capture_panes.txt``, or the file
    ``$ADT4_REC_PANES_FILE`` names."""
    path = (
        os.environ.get("ADT4_REC_PANES_FILE")
        or package_root() / "recording" / PANES_FILE
    )
    try:
        return parse_panes_spec(pathlib.Path(path).read_text())
    except OSError:
        return {}


def capture_stack_panes(
    bag_dir, tag: str, spec: dict[str, int] | None = None
) -> list[str]:
    """Save the listed ``adt4_system`` windows as text next to the bag
    (read-only for the stack): ``pane_<tag>_<window>_<pane>.txt``.  Only the
    windows in ``recording/capture_panes.txt``; returns the names written."""
    spec = panes_spec() if spec is None else spec
    if not spec:
        return []
    rc, out, _ = tmux(
        "list-panes", "-s", "-t", f"={STACK_SESSION}",
        "-F", "#{pane_id}\t#{window_index}\t#{window_name}\t#{pane_index}",
    )  # fmt: skip
    if rc != 0:
        return []
    written = []
    for line in out.splitlines():
        parts = line.split("\t")
        if len(parts) != 4 or parts[2] not in spec:
            continue
        pane_id, widx, wname, pidx = parts
        text = pane_text(pane_id, spec[wname]).rstrip("\n")
        if not text.strip():
            continue
        safe = re.sub(r"[^A-Za-z0-9._-]", "_", wname)[:60] or "window"
        name = f"pane_{tag}_{safe}_{pidx}.txt"
        try:
            with open(os.path.join(bag_dir, name), "w") as f:
                f.write(text + "\n")
            written.append(name)
        except OSError:
            pass
    return written


def stop(args) -> tuple[int, dict]:
    act = active()
    if act["pid"] is None:
        if act["session"]:
            tmux("kill-session", "-t", f"={SESSION}")
        return OK, {"ok": True, "stopped": False, "was_recording": False,
                    "other_recorders": act["others"]}  # fmt: skip
    pid, bag_dir = act["pid"], act["dir"]
    trial, machine = trial_of(bag_dir)
    if args.trial and trial != args.trial:
        return FAILED, {
            "ok": False,
            "error": f"recording {trial}, not {args.trial}; not stopping it",
            "trial": trial,
            "dir": bag_dir,
        }

    signals = []

    def send(sig, name, wait_s):
        try:
            os.kill(pid, sig)
        except ProcessLookupError:
            return True
        signals.append(name)
        end = time.monotonic() + wait_s
        while time.monotonic() < end:
            if not alive(pid):
                return True
            time.sleep(0.5)
        return not alive(pid)

    gone = send(signal.SIGINT, "SIGINT", args.wait)
    if not gone and args.force:
        gone = send(signal.SIGINT, "SIGINT", 10.0) or send(
            signal.SIGTERM, "SIGTERM", args.wait
        )
    if not gone:
        return STILL_RUNNING, {
            "ok": False,
            "error": f"the recorder (pid {pid}) is still running after {'+'.join(signals)}",
            "recording": True,
            "trial": trial,
            "dir": bag_dir,
            "pid": pid,
            "signals": signals,
        }

    now = time.time()
    panes = (
        capture_stack_panes(bag_dir, "stop") if bag_dir and not args.no_panes else []
    )
    meta = None
    if bag_dir:
        try:
            meta = parse_metadata(pathlib.Path(bag_dir, "metadata.yaml").read_text())
        except OSError:
            meta = None
    use = dir_usage(bag_dir) if bag_dir else {"bytes": None, "files": 0}
    info = read_json(os.path.join(bag_dir, TRIAL_FILE)) if bag_dir else None
    wanted = (info or {}).get("topics") or []
    silent = silent_topics(wanted, (meta or {}).get("topics")) if meta else None
    if info is not None:
        info.update(
            stopped_at=utc_iso(now),
            stop_time_unix=round(now, 3),
            stop_base_time_unix=args.base_time,
            stop_signals=signals,
            metadata=meta is not None,
            size_bytes=use["bytes"],
            message_count=(meta or {}).get("message_count"),
            duration_s=(meta or {}).get("duration_s"),
            topic_counts=(meta or {}).get("topics"),
            silent_topics=silent,
            stack_panes=panes,
        )
        try:
            write_json_atomic(os.path.join(bag_dir, TRIAL_FILE), info)
        except OSError:
            pass
    tmux("kill-session", "-t", f"={SESSION}")  # ours; its recorder has exited
    return OK, {
        "ok": True,
        "stopped": True,
        "was_recording": True,
        "trial": trial,
        "machine": machine,
        "dir": bag_dir,
        "signals": signals,
        "metadata": meta is not None,
        "size_bytes": use["bytes"],
        "message_count": (meta or {}).get("message_count"),
        "duration_s": (meta or {}).get("duration_s"),
        "topics": len(wanted),
        "silent_topics": silent,
        "stack_panes": len(panes),
        "t": round(time.time(), 3),
        **disk(bag_dir),
    }


# ----------------------------------------------------------- check-topics
def check_topics(args) -> tuple[int, dict]:
    """Which of the recording's topics have a publisher right now
    (``ros2 topic list -v`` in the recorder's environment).  The recorder's
    own subscriptions do not count: only the "Published topics" are read."""
    act = active()
    if act["pid"] is None or not act["dir"]:
        return FAILED, {"ok": False, "error": "nothing is recording"}
    info = read_json(os.path.join(act["dir"], TRIAL_FILE)) or {}
    wanted = info.get("topics") or []
    env = shared_env(stack_env())
    lines = [f"export {k}={shlex.quote(v)}" for k, v in env.items()]
    setup = workspace_setup()
    if setup:
        lines.append(setup)
    lines.append("ros2 topic list -v")
    rc, out, err = _run([*pane_shell(), "\n".join(lines)], timeout=args.timeout)
    if rc != 0:
        return FAILED, {
            "ok": False,
            "error": f"ros2 topic list failed: {(err or out).strip()[-300:]}",
        }
    published = published_topics(out)
    missing = [t for t in wanted if t not in published]
    return OK, {
        "ok": True,
        "trial": info.get("trial_id"),
        "topics": len(wanted),
        "published": len(wanted) - len(missing),
        "missing": missing,
        "t": round(time.time(), 3),
    }


def published_topics(text: str) -> set[str]:
    """Topics with at least one publisher, from ``ros2 topic list -v``::

    Published topics:
     * /a/b [std_msgs/msg/String] 1 publisher
    Subscribed topics:
     * ...
    """
    out, section = set(), None
    for line in (text or "").splitlines():
        s = line.strip()
        if s.lower().startswith("published topics"):
            section = "pub"
        elif s.lower().startswith("subscribed topics"):
            section = "sub"
        elif section == "pub" and s.startswith("*"):
            m = re.match(r"\*\s+(\S+)\s+\[.*\]\s+(\d+)\s+publisher", s)
            if m and int(m.group(2)) > 0:
                out.add(m.group(1))
    return out


# -------------------------------------------------------------------- cli
def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="adt4-rec", description=__doc__.split("\n\n")[0])
    p.add_argument(
        "--output-root", default=os.environ.get("ADT4_OUTPUT_ROOT", DEFAULT_OUTPUT_ROOT)
    )
    sub = p.add_subparsers(dest="command", required=True)

    s = sub.add_parser("start", help="start recording a trial")
    s.add_argument("--trial", required=True)
    s.add_argument("--machine", required=True, help="this machine's fleet name")
    s.add_argument("--topic-set", default="spot", help="recording/<set>_topics.txt")
    s.add_argument(
        "--topics-file", default=None, help="a topic list file (overrides --topic-set)"
    )
    s.add_argument("--description", default="")
    s.add_argument(
        "--robots",
        default="",
        help="the fleet's robots, comma separated (for ${ADT4_FLEET_ROBOT} lines)",
    )
    s.add_argument(
        "--base-time", type=float, default=None, help="the console's clock (unix s)"
    )
    s.add_argument(
        "--record-arg",
        action="append",
        default=[],
        help="extra ros2 bag record argument",
    )
    s.add_argument("--start-wait", type=float, default=START_WAIT_S)
    s.add_argument("--growth-wait", type=float, default=GROWTH_WAIT_S)

    sub.add_parser("status", help="what is recording, bag size, free disk")

    s = sub.add_parser("stop", help="stop the recording (SIGINT, so the bag closes)")
    s.add_argument("--trial", default=None, help="only stop this trial")
    s.add_argument("--wait", type=float, default=STOP_WAIT_S)
    s.add_argument(
        "--force", action="store_true", help="second SIGINT then SIGTERM (never -9)"
    )
    s.add_argument(
        "--no-panes", action="store_true", help="do not capture the stack's panes"
    )
    s.add_argument("--base-time", type=float, default=None)

    s = sub.add_parser("check-topics", help="which recorded topics have a publisher")
    s.add_argument("--timeout", type=float, default=20.0)
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    try:
        if args.command == "start":
            rc, out = start(args)
        elif args.command == "stop":
            rc, out = stop(args)
        elif args.command == "check-topics":
            rc, out = check_topics(args)
        else:
            rc, out = OK, status(args.output_root)
    except Exception as e:  # noqa: BLE001  one JSON line, whatever happened
        rc, out = FAILED, {"ok": False, "error": f"{type(e).__name__}: {e}"}
    out.setdefault("command", args.command)
    print(json.dumps(out))
    return rc


if __name__ == "__main__":
    sys.exit(main())
