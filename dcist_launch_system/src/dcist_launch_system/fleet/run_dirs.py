"""Per-run output directories: ``<experiment>/runs/<run_id>/`` plus a ``latest`` symlink.

``run-adt4 -o <experiment>`` never deletes ``<experiment>`` (only ``--wipe`` does).
Every launch gets its own run directory, and ``<experiment>/latest`` points at the
newest one::

    <experiment>/                 -o, ADT4_OUTPUT_DIR (e.g. ~/adt4_output/nov_11_test)
      runs/<run_id>/              ADT4_RUN_DIR: hydra/ roman/ spot_executor/ keyframes/
                                  tracks/ image_store/ recorded_data/ mission_bag/
                                  ros_log/ omniplanner/ run_info.json
      latest -> runs/<run_id>     relative symlink, repointed at every launch

``run_id = <machine>_<YYYYmmdd-HHMMSS>_<4hex>``.

Older experiment directories are *flat* (``hydra/``, ``roman/``, ``image_store/``
directly inside); they stay valid wherever a run directory is expected.  Anything that
takes an experiment path and reads run files must resolve it first:

* ``resolve_run_dir(path)``: ``<path>/latest`` (resolved) when it exists, else ``path``
  (a run dir or a flat legacy dir).  E.g. ``<exp>/hydra/backend/dsg.json`` is
  ``resolve_run_dir(<exp>)/hydra/backend/dsg.json``.
* ``resolve_prior_map(path)``: the same, but it skips a ``latest`` run without a map
  (a launch that crashed before saving) for the newest run that has one.

Standard library only (run-adt4 imports it before the workspace is sourced, and the
console ships ``run_scan`` -- which uses this resolution -- over ssh).
"""

from __future__ import annotations

import datetime
import os
import re
import secrets
import shutil
import socket
import subprocess

RUNS_DIR = "runs"
LATEST = "latest"
# a run directory holds a map usable as a prior when it has one of these
MAP_MARKERS = ("hydra/backend/dsg.json", "roman")
# free space below which run-adt4 warns at launch
LOW_DISK_BYTES = 50 * 1024**3

_RUN_ID_RE = re.compile(
    r"^(?P<machine>.+)_(?P<stamp>\d{8}-\d{6})_(?P<rand>[0-9a-f]{4})$"
)


def machine_name() -> str:
    """Short hostname (the fleet names machines by hostname: willow, hamilton...)."""
    return socket.gethostname().split(".")[0] or "unknown"


def _clean(name: str) -> str:
    return re.sub(r"[^A-Za-z0-9.-]+", "-", name).strip("-") or "unknown"


def make_run_id(
    machine: str | None = None,
    now: datetime.datetime | None = None,
    rand: str | None = None,
) -> str:
    """``<machine>_<YYYYmmdd-HHMMSS>_<4hex>`` (local time)."""
    now = now or datetime.datetime.now()
    rand = rand or secrets.token_hex(2)
    return f"{_clean(machine or machine_name())}_{now.strftime('%Y%m%d-%H%M%S')}_{rand}"


def parse_run_id(run_id: str) -> dict | None:
    m = _RUN_ID_RE.match(run_id or "")
    return m.groupdict() if m else None


def runs_of(experiment) -> list[str]:
    """Run directories under ``<experiment>/runs``, oldest first (by mtime, then name)."""
    root = os.path.join(str(experiment), RUNS_DIR)
    try:
        dirs = [e.path for e in os.scandir(root) if e.is_dir(follow_symlinks=False)]
    except OSError:
        return []

    def key(p):
        try:
            return (os.stat(p).st_mtime, p)
        except OSError:
            return (0.0, p)

    return sorted(dirs, key=key)


def latest_of(experiment) -> str | None:
    """The resolved ``<experiment>/latest`` run dir, or None."""
    link = os.path.join(str(experiment), LATEST)
    if os.path.isdir(link):
        return os.path.realpath(link)
    return None


def is_experiment_dir(path) -> bool:
    """``path`` uses the per-run layout (has ``latest`` or ``runs/``)."""
    path = str(path)
    return os.path.lexists(os.path.join(path, LATEST)) or os.path.isdir(
        os.path.join(path, RUNS_DIR)
    )


def resolve_run_dir(path) -> str:
    """The run directory behind ``path`` (see module doc); ``path`` itself otherwise."""
    path = str(path)
    latest = latest_of(path)
    if latest:
        return latest
    runs = runs_of(path)
    if runs:
        return runs[-1]
    return path


def has_map(run_dir) -> bool:
    return any(os.path.exists(os.path.join(str(run_dir), m)) for m in MAP_MARKERS)


def resolve_prior_map(path) -> tuple[str, str]:
    """``(run_dir, how)`` for a ``-p`` argument.

    ``how``: ``"latest"`` (the experiment's latest run), ``"newest_with_map"`` (latest
    has no map, an older run of the experiment has one), ``"run"`` (``path`` is a run or
    flat legacy directory, used as is) or ``"latest_without_map"`` (nothing in the
    experiment has a map; its latest run is returned anyway).
    """
    path = str(path)
    if not is_experiment_dir(path):
        return path, "run"
    latest = latest_of(path)
    if latest and has_map(latest):
        return latest, "latest"
    for run in reversed(runs_of(path)):
        if has_map(run):
            return run, "newest_with_map"
    if has_map(path):  # a flat legacy dir that a later launch added runs/ to
        return path, "run"
    return latest or resolve_run_dir(path), "latest_without_map"


def point_latest(experiment, run_dir) -> bool:
    """Atomically repoint ``<experiment>/latest`` at ``run_dir`` (relative link).

    Returns False (and leaves it alone) when ``latest`` is a real directory or file.
    """
    experiment = str(experiment)
    link = os.path.join(experiment, LATEST)
    if os.path.lexists(link) and not os.path.islink(link):
        return False
    target = os.path.relpath(str(run_dir), experiment)
    tmp = f"{link}.tmp{os.getpid()}"
    if os.path.lexists(tmp):
        os.unlink(tmp)
    os.symlink(target, tmp)
    os.replace(tmp, link)
    return True


def create_run_dir(
    experiment, machine: str | None = None, run_id: str | None = None
) -> tuple[str, str]:
    """Create ``<experiment>/runs/<run_id>/`` (never reusing one); return ``(run_id, path)``."""
    experiment = str(experiment)
    root = os.path.join(experiment, RUNS_DIR)
    os.makedirs(root, exist_ok=True)
    for _ in range(16):
        rid = run_id or make_run_id(machine)
        path = os.path.join(root, rid)
        try:
            os.mkdir(path)
        except FileExistsError:
            if run_id:
                raise
            continue
        return rid, path
    raise RuntimeError(f"could not create a unique run directory under {root}")


def free_bytes(path) -> int | None:
    """Free bytes on the filesystem holding ``path`` (or its nearest existing parent)."""
    p = os.path.abspath(str(path))
    while p and not os.path.exists(p):
        parent = os.path.dirname(p)
        if parent == p:
            break
        p = parent
    try:
        return shutil.disk_usage(p).free
    except OSError:
        return None


def _git(args, cwd, timeout=3) -> str | None:
    try:
        out = subprocess.run(
            ["git", "-C", str(cwd)] + list(args),
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            timeout=timeout,
            universal_newlines=True,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    return out.stdout if out.returncode == 0 else None


def git_shas(superproject) -> dict:
    """``{"superproject": sha, "superproject_dirty": bool, "submodules": {path: sha}}``.

    Submodule shas are the checked-out commits (``git submodule status``), which can
    differ from the superproject's pins; ``submodules_modified`` lists those.
    """
    out = {
        "superproject": None,
        "superproject_dirty": None,
        "submodules": {},
        "submodules_modified": [],
    }
    head = _git(["rev-parse", "HEAD"], superproject)
    if head and re.fullmatch(r"[0-9a-f]{40}", head.strip()):
        out["superproject"] = head.strip()
    status = _git(
        ["status", "--porcelain", "--ignore-submodules=dirty", "--untracked-files=no"],
        superproject,
    )
    if status is not None:
        out["superproject_dirty"] = bool(status.strip())
    subs = _git(["submodule", "status"], superproject, timeout=10)
    for line in (subs or "").splitlines():
        m = re.match(r"^([ +\-U])([0-9a-f]{40}) (\S+)", line)
        if not m:
            continue
        flag, sha, path = m.groups()
        if flag == "-":  # not initialised
            continue
        out["submodules"][path] = sha
        if flag in "+U":
            out["submodules_modified"].append(path)
    return out
