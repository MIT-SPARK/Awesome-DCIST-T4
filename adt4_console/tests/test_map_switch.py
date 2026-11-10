"""Loading a run into the live Neo4j (API.md §5): it loads in stages, the graph is
served once the graph stage is done, and runs with and without an image store both
load. Needs a scratch Neo4j it may wipe (``ADT4_TEST_SCRATCH_NEO4J``, see conftest)."""

import shutil
import time

import pytest

from .conftest import NEO4J_AUTH, make_client, scratch_neo4j
from .synthetic import build_run

pytest.importorskip("spark_dsg")


@pytest.fixture
def live(settings):
    settings.neo4j = {"test": scratch_neo4j()}
    settings.neo4j_user, settings.neo4j_password = NEO4J_AUTH
    client, svc = make_client(settings)
    with client:
        snaps = []
        orig = svc.maps._publish
        svc.maps._publish = lambda snap: (snaps.append(snap), orig(snap))
        yield client, svc, snaps
        svc.graph.get("neo4j:test").query(
            "MATCH (n) DETACH DELETE n"
        )  # leave the scratch DB empty


def _run(tmp_path, images=True):
    """The synthetic run with a (named, empty) mesh places layer, as Hydra writes it:
    heracles' ingest expects one."""
    import spark_dsg as dsg

    run = build_run(tmp_path)
    path = run / "hydra" / "backend" / "dsg.json"
    G = dsg.DynamicSceneGraph.load(str(path))
    G.add_layer(3, 1, dsg.DsgLayers.MESH_PLACES)
    G.save(str(path), False)
    if not images:
        shutil.rmtree(run / "image_store")
    return run


def _run_job(client, job_id, answer=True, timeout=600):
    """Wait for the job, answering a wipe confirmation if one comes."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        job = client.get(f"/api/jobs/{job_id}").json()
        if job["confirm"]:
            client.post(f"/api/jobs/{job_id}/confirm", json={"answer": answer})
        if job["state"] not in ("running", "waiting"):
            return job
        time.sleep(0.1)
    raise AssertionError("map load did not finish")


@pytest.fixture
def plain_run(tmp_path):
    return _run(tmp_path, images=False)


def test_a_run_without_images_loads(live, plain_run):
    client, svc, snaps = live
    run = plain_run
    r = client.post("/api/maps/activate", json={"path": str(run)})
    assert r.status_code == 200, r.text
    job = _run_job(client, r.json()["job_id"])
    assert job["state"] == "succeeded", job["log"][-5:]
    st = client.get("/api/maps/active").json()
    assert st["ready"] and st["name"] == run.name and not st["has_images"]
    states = {s["id"]: s["state"] for s in st["stages"]}
    assert states == {
        "graph": "done",
        "images": "skipped",
        "features": "skipped",
        "keyframes": "skipped",
    }
    # the console's graph is the new map: its objects and places
    snap = client.get("/api/graph/neo4j:test/stats").json()
    layers = {e["key"]: e["nodes"] for e in snap["layers"]}
    assert layers.get("2/0") == 3 and layers.get("3/0") == 2
    # with the robot path and the place the agent is in, as the run directory has them
    # (needs heracles' agent_edges)
    from heracles import graph_interface as gi

    if hasattr(gi, "agent_edges"):
        assert snap["edges"].get("agents-agents") == 2
        assert snap["edges"].get("agents-places") == 1
    # picking it again is a no-op
    again = client.post("/api/maps/activate", json={"path": str(run)}).json()
    assert again.get("already") is True
    # the viewer heard about every stage
    assert any(s["stages"][0]["state"] == "running" for s in snaps)


def test_added_objects_are_confirmed_and_a_second_load_waits(live, plain_run):
    client, svc, _ = live
    run = plain_run
    job = _run_job(
        client,
        client.post(
            "/api/maps/activate", json={"path": str(run), "force": True}
        ).json()["job_id"],
    )
    assert job["state"] == "succeeded"
    src = svc.graph.get("neo4j:test")
    src.query("CREATE (:GraphMutation {kind: 'test'})")
    job_id = client.post(
        "/api/maps/activate", json={"path": str(run), "force": True}
    ).json()["job_id"]
    deadline = time.time() + 30
    while not (job := client.get(f"/api/jobs/{job_id}").json())["confirm"]:
        assert time.time() < deadline
        time.sleep(0.05)
    assert job["confirm"]["detail"]["map_edits"] == 1
    busy = client.post("/api/maps/activate", json={"path": str(run), "force": True})
    assert busy.status_code == 409 and busy.json()["detail"]["job_id"] == job_id
    client.post(f"/api/jobs/{job_id}/confirm", json={"answer": False})
    job = svc.jobs.wait(job_id, timeout=30).to_dict()
    assert job["state"] == "failed"
    # cancelled: the live map is what Neo4j still holds, edit included
    assert src.query("MATCH (g:GraphMutation) RETURN count(g) AS c")[0]["c"] == 1
    assert client.get("/api/maps/active").json()["ready"]


def test_a_run_with_images_is_served_before_the_images_are_in(live, tmp_path):
    client, svc, snaps = live
    run = _run(tmp_path)
    r = client.post("/api/maps/activate", json={"path": str(run), "force": True})
    job = _run_job(client, r.json()["job_id"])
    assert job["state"] == "succeeded", job["log"][-5:]
    st = client.get("/api/maps/active").json()
    assert (
        st["has_images"]
        and {s["id"]: s["state"] for s in st["stages"]}["images"] == "done"
    )
    # ready was announced while the images were still to come
    early = [
        s
        for s in snaps
        if s["ready"]
        and {x["id"]: x["state"] for x in s["stages"]}["images"]
        in ("pending", "running")
    ]
    assert early, "ready only at the end"
    src = svc.graph.get("neo4j:test")
    assert src.image_store_paths() == [str((run / "image_store").resolve())]
    assert job["progress"].get("graph") == 100 and job["progress"].get("images") == 100
    # the object's images reach the viewer (Observation rows, else the linked store)
    assert "O(0)" in client.get("/api/graph/neo4j:test/imaged").json()["syms"]
    assert client.get("/api/graph/neo4j:test/node/O(0)").json()["n_images"] == 2
