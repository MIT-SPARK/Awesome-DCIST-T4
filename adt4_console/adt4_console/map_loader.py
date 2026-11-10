"""The live map: which scene graph the console's Neo4j holds, and switching it (API.md §5).

Loading a map wipes the live Neo4j and writes it in stages, so planning can
start as soon as the graph is in, while the slower parts follow:

1. ``graph``: objects, places, mesh places, rooms, buildings, agents (poses
   only) and edges, the labelspaces, the mesh path and the source file. The
   graph is served (and planners reading Neo4j work) from here on: the map is
   *ready*.
2. ``images``: with an image store only. The store's location
   (``_GraphMetadata.image_store_paths``, which the viewer reads images from) and,
   when heracles supports it, agent keyframe images, object Observations,
   SubKeyframes, CameraCalibs and observation -> frame links.
3. ``features``: place / room SigLIP2 features
   (``hydra/backend/semantic_features.npz``), when the run has them.
4. ``keyframes``: the place x keyframe visibility index (images only).

A map without an image store (or features) skips those stages. Each stage
reports 0-100 on the job's progress and on the ``maps/active`` channel.

Newer heracles APIs (agents, mesh path, image evidence) are optional
(heracles_compat): without them that part is skipped with a log line.
"""

from __future__ import annotations

import logging
import pathlib
import threading
import time

log = logging.getLogger(__name__)

STAGES = (
    ("graph", "Scene graph"),
    ("images", "Images"),
    ("features", "Place features"),
    ("keyframes", "Place keyframes"),
)

# what other tools (chat, live images, operators) added since a load: a load replaces them
ADDED_OBJECTS = "MATCH (o:Object) WHERE o.source IS NOT NULL OR o.sam3_created IS NOT NULL RETURN count(o) AS c"
MAP_EDITS = "MATCH (g:GraphMutation) RETURN count(g) AS c"


class Busy(RuntimeError):
    """Another map load is running (its job id in ``job_id``)."""

    def __init__(self, job_id: str):
        super().__init__(f"a map is already loading (job {job_id})")
        self.job_id = job_id


def resolve_run(path) -> tuple[pathlib.Path, pathlib.Path]:
    """(dsg file, run dir) for a run directory or a dsg file; FileNotFoundError otherwise."""
    from .graph_service import FileSource

    return FileSource._resolve(pathlib.Path(str(path)).expanduser())


def run_image_store(run: pathlib.Path) -> str | None:
    """The run's image store directory (one with store.sqlite), or None."""
    d = run / "image_store"
    return str(d) if (d / "store.sqlite").is_file() else None


class MapLoader:
    """Loads maps into the live Neo4j target, one at a time, and says where it is."""

    def __init__(self, settings, graph, jobs, hub=None, geo=None):
        self.settings = settings
        self.graph = graph
        self.jobs = jobs
        self.hub = hub
        self.geo = geo
        self._lock = threading.Lock()
        self._job_id: str | None = None
        self._status: dict | None = None

    # -- which target ----------------------------------------------------------
    def target(self, name: str | None = None):
        """The live Neo4j source: ``name``, else ``--live-neo4j``, else the only /
        first ``--neo4j``."""
        name = name or getattr(self.settings, "live_neo4j", None)
        if name:
            return self.graph.neo4j(name)
        for s in self.graph.list():
            if s["kind"] == "neo4j":
                return self.graph.get(s["id"])
        from .graph_service import NotFound

        raise NotFound("no Neo4j target (start the console with --neo4j NAME=URI)")

    # -- status ------------------------------------------------------------------
    def status(self, target_name: str | None = None) -> dict:
        """The live map: {target, run, name, dsg, nodes, has_images, has_features,
        ready (for planning), stages: [{id, label, state, pct, detail}], job_id}."""
        with self._lock:
            if self._status is not None and (
                target_name is None or self._status["target"] == target_name
            ):
                return _copy(self._status)
        t = self.target(target_name)
        out = {"target": t.name, "uri": t.uri, "run": None, "name": None, "dsg": None, "nodes": None,
               "has_images": False, "has_features": False, "ready": False, "job_id": None,
               "stages": [_stage(i, lbl, "pending") for i, lbl in STAGES]}  # fmt: skip
        try:
            rec = t.query(
                "MATCH (m:_GraphMetadata {key: 'source'}) RETURN m.file_path AS p"
            )
            out["nodes"] = t.count_nodes()
            if rec and rec[0]["p"]:
                dsg = pathlib.Path(rec[0]["p"])
                run = dsg.parents[2] if dsg.parent.name == "backend" else dsg.parent
                out.update(dsg=str(dsg), run=str(run), name=run.name, ready=out["nodes"] > 0,
                           has_images=bool(t.image_store_paths()),
                           has_features=bool(t.query("MATCH (n:MeshPlace) WHERE n.siglip2_emb IS NOT NULL "
                                                     "RETURN count(n) > 0 AS f")[0]["f"]))  # fmt: skip
                # what a finished load leaves behind
                for s in out["stages"]:
                    s["state"] = "done" if out["ready"] else "pending"
                    if s["id"] in ("images", "keyframes") and not out["has_images"]:
                        s["state"] = "skipped"
                    if s["id"] == "features" and not out["has_features"]:
                        s["state"] = "skipped"
                    s["pct"] = 100 if s["state"] in ("done", "skipped") else 0
        except Exception as e:  # noqa: BLE001 — Neo4j down: say so
            out["error"] = str(e)
        return out

    def _set(self, **kw) -> None:
        with self._lock:
            if self._status is None:
                return
            self._status.update(kw)
            snap = _copy(self._status)
        self._publish(snap)

    def _stage(
        self,
        job,
        sid: str,
        state: str,
        pct: float | None = None,
        detail: str | None = None,
    ) -> None:
        with self._lock:
            if self._status is None:
                return
            for s in self._status["stages"]:
                if s["id"] == sid:
                    s["state"] = state
                    if pct is not None:
                        s["pct"] = round(float(pct), 1)
                    if detail is not None:
                        s["detail"] = detail
            snap = _copy(self._status)
        if job is not None and pct is not None:
            job.set_progress(sid, pct)
        self._publish(snap)

    def _publish(self, snap: dict) -> None:
        if self.hub is not None:
            self.hub.publish("maps/active", snap)

    # -- what a switch would lose ------------------------------------------------
    def loss(self, t) -> dict:
        """Objects added (chat, live images, operator) and map edits in the live map."""
        q = lambda c: int(t.query(c)[0]["c"])  # noqa: E731
        try:
            return {
                "added_objects": q(ADDED_OBJECTS),
                "map_edits": q(MAP_EDITS),
                "nodes": t.count_nodes(),
            }
        except Exception:  # noqa: BLE001 — an empty / fresh DB
            return {"added_objects": 0, "map_edits": 0, "nodes": 0}

    # -- load ------------------------------------------------------------------
    def activate(self, path, target_name: str | None = None, force: bool = False, image_root: str | None = None,
                 confirm_always: bool = False) -> dict:  # fmt: skip
        """Make the run at ``path`` the live map. Returns {job_id} (or {already: True}
        when the live Neo4j holds it, unless ``force``). Raises Busy, FileNotFoundError."""
        t = self.target(target_name)
        dsg, run = resolve_run(path)
        with self._lock:
            if self._job_id is not None:
                job = self.jobs.get(self._job_id)
                if job is not None and job.state in ("running", "waiting"):
                    raise Busy(self._job_id)
        if not force:
            cur = self.status(t.name)
            if cur.get("dsg") == str(dsg) and cur.get("ready"):
                return {"already": True, **cur}
        image_store = image_root or run_image_store(run)
        feats = run / "hydra" / "backend" / "semantic_features.npz"
        status = {"target": t.name, "uri": t.uri, "run": str(run), "name": run.name, "dsg": str(dsg),
                  "nodes": None, "has_images": bool(image_store), "has_features": feats.is_file(),
                  "ready": False, "job_id": None,
                  "stages": [_stage(i, lbl, "pending" if (i == "graph" or (i in ("images", "keyframes") and image_store)
                                                          or (i == "features" and feats.is_file())) else "skipped")
                             for i, lbl in STAGES]}  # fmt: skip

        def run_job(job):
            return self._load(
                job,
                t,
                dsg,
                run,
                image_store,
                feats if feats.is_file() else None,
                confirm_always,
            )

        with self._lock:
            self._status = status
            job = self.jobs.start(
                "map", f"Load {run.name} into neo4j:{t.name}", run_job
            )
            self._job_id = job.id
            self._status["job_id"] = job.id
            snap = _copy(self._status)
        self._publish(snap)
        return {"job_id": job.id, **snap}

    def _load(self, job, t, dsg, run, image_store, feats, confirm_always=False):
        lost = self.loss(t)
        if confirm_always or lost["added_objects"] or lost["map_edits"]:
            ok = job.confirm(
                f"Loading {run.name} replaces the live map in neo4j:{t.name}. This removes "
                f"{lost['added_objects']} added object(s) (chat, live images, operator) and "
                f"{lost['map_edits']} map edit(s).",
                kind="wipe_neo4j", yes_label="Load map", no_label="Cancel", detail={"target": t.name, **lost},
            )  # fmt: skip
            if not ok:
                job.log("error", "Cancelled: the live map is unchanged.")
                with self._lock:
                    self._status = None  # back to what Neo4j holds
                self._publish(self.status(t.name))
                return False
        t0 = time.time()
        try:
            G, db = self._load_graph(job, t, dsg, run)
            self._set(ready=True, nodes=t.count_nodes())
            job.log("ok", f"Ready for planning: {run.name} in {time.time() - t0:.1f} s")
            if image_store:
                self._load_images(job, t, G, db, image_store)
            if feats is not None:
                self._load_features(job, t, run)
            if image_store:
                self._build_keyframes(job, t)
        except Exception as e:
            for s in (self._status or {}).get("stages", []):
                if s["state"] == "running":
                    self._stage(job, s["id"], "failed", detail=str(e)[:300])
            raise
        finally:
            with self._lock:
                self._job_id = None
        secs = round(time.time() - t0, 1)
        job.log("ok", f"{run.name} fully loaded in {secs} s")
        return {"run": str(run), "nodes": t.count_nodes(), "seconds": secs}

    # -- stages ------------------------------------------------------------------
    def _load_graph(self, job, t, dsg, run):
        """Wipe and write everything planning needs; the image evidence follows."""
        import spark_dsg
        from heracles import graph_interface as gi
        from heracles.utils import extract_labelspaces_from_dsg

        from .heracles_compat import optional

        add_agents = optional(
            "heracles.graph_interface",
            "add_agents_from_dsg",
            "agent poses in loaded maps",
        )
        store_mesh_path = optional(
            "heracles.graph_interface", "store_mesh_path", "the mesh of loaded maps"
        )
        sid = "graph"
        self._stage(job, sid, "running", 2, "reading the scene graph")
        G = spark_dsg.DynamicSceneGraph.load(str(dsg))
        # heracles.utils.load_dsg_to_db's layer names, which db_to_spark_dsg reads back
        G.metadata.add({"LayerIdToHeraclesLayerStr": {2: "Object", 5: "Building", 20: "MeshPlace",
                                                      "3[1]": "MeshPlace", 3: "Place", 4: "Room"}})  # fmt: skip
        job.check_cancelled()
        db = t.db()
        self._stage(job, sid, "running", 10, f"wiping neo4j:{t.name}")
        gi.initialize_db(db)
        object_ls, room_ls = extract_labelspaces_from_dsg(G)
        # heracles.graph_interface.spark_dsg_to_db without an image store, step by step
        steps = (
            [("agents", lambda: add_agents(G, db, None))]
            if add_agents is not None
            else []
        )
        steps += [
            (
                "objects",
                lambda: gi.add_objects_from_dsg(G, db, object_labelspace=object_ls),
            ),
            ("places", lambda: gi.add_places_from_dsg(G, db)),
            (
                "mesh places",
                lambda: gi.add_mesh_places_from_dsg(G, db, object_labelspace=object_ls),
            ),
            ("rooms", lambda: gi.add_rooms_from_dsg(G, db, room_labelspace=room_ls)),
            ("buildings", lambda: gi.add_buildings_from_dsg(G, db)),
            ("edges", lambda: gi.add_edges_from_dsg(G, db)),
        ]
        for i, (what, fn) in enumerate(steps):
            self._stage(job, sid, "running", 15 + 75 * i / len(steps), what)
            fn()
            job.check_cancelled()
        for layer, ls in (("object", object_ls), ("room", room_ls)):
            if ls:
                db.execute(f"MERGE (m:_Labelspace {{layer: '{layer}'}}) SET m.ids = $ids, m.names = $names",
                           ids=[int(k) for k in ls], names=list(ls.values()))  # fmt: skip
        db.execute(
            "MERGE (m:_GraphMetadata {key: 'source'}) SET m.file_path = $p", p=str(dsg)
        )
        mesh = dsg.parent / "mesh.ply"
        if mesh.is_file() and store_mesh_path is not None:
            store_mesh_path(db, str(mesh))
        from .routes.maps import bump_map_counters

        bump_map_counters(db)
        self._refresh(t, "map loaded")
        self._stage(job, sid, "done", 100, f"{t.count_nodes()} nodes")
        return G, db

    def _load_images(self, job, t, G, db, image_store):
        from heracles.utils import extract_labelspaces_from_dsg

        from .heracles_compat import optional

        sid = "images"
        # where the viewer reads images from, whatever heracles writes below
        db.execute("MERGE (m:_GraphMetadata {key: 'image_store'}) SET m.image_store_paths = $paths",
                   paths=[str(pathlib.Path(image_store).resolve())])  # fmt: skip
        mod = "heracles.graph_interface"
        feature = "image evidence (Observations, keyframe images) in loaded maps"
        fns = {n: optional(mod, n, feature) for n in (
            "open_image_store", "add_agents_from_dsg", "add_objects_from_dsg", "add_subkeyframes_from_dsg",
            "attach_camera_calibs", "link_observations_to_frames")}  # fmt: skip
        if any(f is None for f in fns.values()):
            self._refresh(t, "images linked")
            self._stage(
                job,
                sid,
                "done",
                100,
                "image store linked (this heracles writes no observations)",
            )
            return
        self._stage(job, sid, "running", 2, "opening the image store")
        try:
            store = fns["open_image_store"](image_store)
        except ImportError as e:  # heracles' image-store reader is not installed
            job.log(
                "warn",
                f"heracles cannot read the image store ({e}); images come from the store only",
            )
            self._refresh(t, "images linked")
            self._stage(
                job, sid, "done", 100, "image store linked (heracles cannot read it)"
            )
            return
        object_ls, _ = extract_labelspaces_from_dsg(G)
        steps = [
            ("keyframe images", lambda: fns["add_agents_from_dsg"](G, db, store)),
            ("object observations",
             lambda: fns["add_objects_from_dsg"](G, db, store, object_labelspace=object_ls)),
            ("sub-keyframes", lambda: fns["add_subkeyframes_from_dsg"](G, db, store)),
            ("camera calibrations", lambda: fns["attach_camera_calibs"](db, store)),
            ("observation frames", lambda: fns["link_observations_to_frames"](db)),
        ]  # fmt: skip
        for i, (what, fn) in enumerate(steps):
            self._stage(job, sid, "running", 5 + 90 * i / len(steps), what)
            fn()
            job.check_cancelled()
        self._refresh(t, "images loaded")
        n = t.query("MATCH (o:Observation) RETURN count(o) AS c")[0]["c"]
        self._stage(job, sid, "done", 100, f"{n} observations")

    def _load_features(self, job, t, run):
        from .routes.maps import load_semantic_features

        self._stage(job, "features", "running", 10, "writing SigLIP2 features")
        n = load_semantic_features(t, run)
        self._refresh(t, "features loaded")
        self._stage(job, "features", "done", 100, f"{n} places / rooms")

    def _build_keyframes(self, job, t):
        self._stage(job, "keyframes", "running", 10, "place x keyframe visibility")
        try:
            idx = t.place_keyframe_index()
            wait = getattr(idx, "wait", None)
            if callable(wait):
                wait()
            self._stage(job, "keyframes", "done", 100)
        except Exception as e:  # noqa: BLE001 — images still work without the index
            job.log("warn", f"place keyframes not built: {e}")
            self._stage(job, "keyframes", "failed", 100, str(e)[:300])

    def _refresh(self, t, reason: str) -> None:
        """Every cache of the live source follows the new contents."""
        reset = getattr(t, "reset_map", None)
        if callable(reset):
            reset()
        else:
            t.invalidate()
        if self.geo is not None:
            self.geo.forget(t.id)
        self.graph.notify_version(t.id, reason)


def _stage(sid: str, label: str, state: str) -> dict:
    return {
        "id": sid,
        "label": label,
        "state": state,
        "pct": 100 if state == "skipped" else 0,
        "detail": None,
    }


def _copy(st: dict) -> dict:
    return {**st, "stages": [dict(s) for s in st["stages"]]}
