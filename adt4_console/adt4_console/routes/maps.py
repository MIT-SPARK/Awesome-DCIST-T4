"""Maps: local run listing, file-source registration and the live Neo4j map (API.md §5)."""

from __future__ import annotations

import json
import os
import pathlib
import threading
import time

from fastapi import APIRouter, HTTPException, Request
from pydantic import BaseModel

from ..graph_service import FileSource
from ..run_scan import finish_entry, scan_run
from . import services

router = APIRouter(prefix="/api/maps")

LOCAL_MACHINE = "local"


def _notify_fn():
    """heracles' combined helper (both counters) when this heracles has it."""
    try:
        from heracles.graph_interface import notify_map_change
    except ImportError:
        return None
    return notify_map_change


def bump_map_counters(db, dirty_layers=None, affected_ids=None) -> int:
    """Bump BOTH Neo4j version counters after a write; returns ``_State.version``.

    ``_State{key:'global'}.version`` is what the console (and viz agents) poll;
    ``_MapState.version`` is what the heracles DSG publisher polls to republish
    within 0.5 s (``heracles_ros.map_state.bump_map_version``).  A writer that
    bumps only one leaves the other consumer stale.
    """
    notify = _notify_fn()
    if notify is not None:
        out = notify(db, list(dirty_layers or []), list(affected_ids or []))
        if isinstance(out, (tuple, list)) and out:  # (state_version, map_version)
            return int(out[0])
        if isinstance(out, int):
            return out
        records, _, _ = db.execute(
            "MATCH (s:_State {key: 'global'}) RETURN s.version AS v"
        )
        return int(records[0]["v"]) if records else 0
    # same cypher as heracles graph_interface.record_change + map_state.bump_map_version
    records, _, _ = db.execute(
        "MERGE (s:_State {key: 'global'}) "
        "ON CREATE SET s.version = 1 "
        "ON MATCH SET s.version = s.version + 1 "
        "SET s.dirty_layers = $dl, s.affected_ids = $ai "
        "RETURN s.version AS v",
        dl=list(dirty_layers or []),
        ai=list(affected_ids or []),
    )
    version = records[0]["v"] if records else 0
    db.execute(
        "MERGE (m:_MapState) SET m.version = coalesce(m.version, 0) + 1 "
        "RETURN m.version AS v"
    )
    return int(version)


# ---------------------------------------------------------------------------
# Listing: the run directories under --output-root (plus registered --map runs).
# Directory sizes (du) are the slow part: they are cached per run keyed by its
# mtime signature (memory + a JSON file), measured within a short budget during
# GET /api/maps, and whatever is left is measured by a background thread; the
# response then lists the machine under ``pending`` so the page polls again.
# ---------------------------------------------------------------------------

LOCAL_BUDGET_S = 3.0
_CACHE_FILE = pathlib.Path(os.path.expanduser("~/.cache/adt4_console/map_sizes.json"))
_cache_lock = threading.Lock()
_size_cache: dict[str, list] | None = None  # run path -> [sig, size, store size]
_filling = False


def _cache() -> dict[str, list]:
    global _size_cache
    if _size_cache is None:
        try:
            data = json.loads(_CACHE_FILE.read_text())
            _size_cache = data.get(LOCAL_MACHINE, {}) if isinstance(data, dict) else {}
        except (OSError, ValueError):
            _size_cache = {}
    return _size_cache


def _cache_get() -> dict[str, list]:
    with _cache_lock:
        return dict(_cache())


def _cache_put(items: dict[str, list]) -> None:
    if not items:
        return
    with _cache_lock:
        _cache().update(items)
        try:
            _CACHE_FILE.parent.mkdir(parents=True, exist_ok=True)
            tmp = _CACHE_FILE.with_suffix(".tmp")
            tmp.write_text(json.dumps({LOCAL_MACHINE: _cache()}))
            tmp.replace(_CACHE_FILE)
        except OSError:
            pass


def _sized(entries: list[dict]) -> dict[str, list]:
    """Cache rows for the entries that got a size."""
    return {
        e["path"]: [
            e["signature"],
            e["size_bytes"],
            (e.get("image_store") or {}).get("size_bytes"),
        ]
        for e in entries
        if e.get("size_bytes") is not None and "signature" in e
    }


def _local_dirs(svc) -> list[pathlib.Path]:
    root = svc.settings.output_root_path
    dirs = (
        sorted(p for p in root.iterdir() if p.is_dir() and not p.name.startswith("."))
        if root.is_dir()
        else []
    )
    extra = [s.run_dir for s in svc.graph.sources.values() if isinstance(s, FileSource)]
    return dirs + [d for d in extra if d not in dirs and d.is_dir()]


def _scan(dirs, budget: float | None) -> list[dict]:
    cache = _cache_get()
    deadline = None if budget is None else time.monotonic() + budget
    out = []
    for d in dirs:
        left = None if deadline is None else deadline - time.monotonic()
        try:
            out.append(
                scan_run(
                    str(d), cache, measure=left is None or left > 0.05, timeout=left
                )
            )
        except Exception as e:  # noqa: BLE001 - one bad run must not hide the rest
            out.append({"name": d.name, "path": str(d), "subdirs": [], "error": str(e)})
    _cache_put(_sized(out))
    return out


def _start_fill(todo: list[pathlib.Path]) -> None:
    """Measure the pending sizes in the background (one fill at a time)."""
    global _filling
    with _cache_lock:
        if _filling:
            return
        _filling = True

    def run():
        global _filling
        try:
            _scan(todo, None)
        except Exception:  # noqa: BLE001 - best effort; the next listing retries
            pass
        finally:
            with _cache_lock:
                _filling = False

    threading.Thread(target=run, name="map-sizes", daemon=True).start()


def list_local(svc, pending: set | None = None) -> list[dict]:
    with _cache_lock:
        budget = 0.0 if _filling else LOCAL_BUDGET_S  # no foreground du during a fill
    out = _scan(_local_dirs(svc), budget)
    todo = [pathlib.Path(e["path"]) for e in out if e.get("size_pending")]
    if todo:
        if pending is not None:
            pending.add(LOCAL_MACHINE)
        _start_fill(todo)
    return [finish_entry(e) for e in out]


@router.get("")
def list_maps(request: Request, local: bool = True):
    """The runs under ``--output-root`` (and registered ``--map`` runs), as
    ``{machines: {local: [entry]}, pending: [...], base: "local"}``. ``local`` is
    accepted for compatibility: only this machine is ever listed."""
    svc = services(request)
    pending: set[str] = set()
    try:
        runs = list_local(svc, pending)
    except Exception as e:  # noqa: BLE001
        runs = {"error": str(e)}
    return {
        "machines": {LOCAL_MACHINE: runs},
        "pending": sorted(pending),
        "base": LOCAL_MACHINE,
    }


def load_semantic_features(target, run: pathlib.Path) -> int:
    """Place / room SigLIP2 features (hydra/backend/semantic_features.npz) as the
    ``siglip2_emb`` property with a cosine vector index per label; 0 when absent."""
    import re

    import numpy as np

    path = run / "hydra" / "backend" / "semantic_features.npz"
    if not path.is_file():
        return 0
    F = np.load(path)
    rows = [
        {"s": re.sub(r"\((\d+)\)", r"\1", str(sym)), "e": f.tolist()}
        for sym, f in zip(F["symbols"], F["features"])
    ]
    for i in range(0, len(rows), 500):
        target.query("UNWIND $rows AS r MATCH (n:MeshPlace|Room {nodeSymbol: r.s}) SET n.siglip2_emb = r.e",
                     rows=rows[i:i + 500])  # fmt: skip
    dim = int(F["features"].shape[1])
    for label in ("MeshPlace", "Room"):
        target.query(f"CREATE VECTOR INDEX {label.lower()}_siglip2_emb IF NOT EXISTS FOR (n:{label}) ON (n.siglip2_emb) "
                     "OPTIONS {indexConfig: {`vector.dimensions`: $dim, `vector.similarity_function`: 'cosine'}}", dim=dim)  # fmt: skip
    return len(rows)


class RegisterBody(BaseModel):
    name: str
    path: str


@router.post("/register")
def register(body: RegisterBody, request: Request):
    svc = services(request)
    try:
        src = svc.graph.add_file(body.name, body.path)
    except FileNotFoundError as e:
        raise HTTPException(400, str(e)) from None
    svc.graph.notify_version(src.id, "registered")
    return src.info()


# --- the live map: which run the console's Neo4j holds, and switching it (API.md §5) ----


@router.get("/active")
def active_map(request: Request, target: str | None = None):
    """The live map and its load stages: {target, run, name, ready, stages: [...]}."""
    from ..graph_service import NotFound

    try:
        return services(request).maps.status(target)
    except NotFound as e:
        raise HTTPException(404, str(e)) from None


class ActivateBody(BaseModel):
    # a run dir (with hydra/backend) or a dsg file on this machine
    path: str
    target: str | None = None
    # load it again even when the live Neo4j already holds it
    force: bool = False


@router.post("/activate")
def activate_map(body: ActivateBody, request: Request):
    """Load a run into the live Neo4j as the console's map, in stages: the graph is
    served once the graph stage is done; images, features and keyframes follow (§5)."""
    from ..graph_service import NotFound
    from ..map_loader import Busy

    try:
        return services(request).maps.activate(body.path, body.target, force=body.force)
    except NotFound as e:
        raise HTTPException(404, str(e)) from None
    except FileNotFoundError as e:
        raise HTTPException(400, str(e)) from None
    except Busy as e:
        raise HTTPException(409, {"detail": str(e), "job_id": e.job_id}) from None
