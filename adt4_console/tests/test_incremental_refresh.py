"""Neo4jSource.apply_change: a known write (a tool adding an object, a map edit) is
folded into the loaded graph instead of reloading the whole map, and must give exactly
what a full reload gives (records, edges, labelspaces).

The DB tests need a throwaway Neo4j they may wipe: ``ADT4_TEST_SCRATCH_NEO4J`` (see
conftest; skipped when unset or unreachable).  They only wipe a DB that is empty or
holds nothing but their own seed map (``_IncrementalTestMarker``), so pointing the
variable at a real map skips instead of erasing it.
"""

import numpy as np
import pytest

from .conftest import NEO4J_AUTH, scratch_neo4j

MARKER = "_IncrementalTestMarker"


# ------------------------------------------------------------------ no database
class FakeHub:
    def __init__(self):
        self.frames = []

    def publish(self, topic, data):
        self.frames.append((topic, data))
        return 1


def test_version_frame_carries_the_delta_and_its_base(settings):
    from adt4_console.codec import GraphDelta, NodeRecord
    from adt4_console.graph_service import GraphService, Neo4jSource

    hub = FakeHub()
    gs = GraphService(settings, hub=hub)
    src = Neo4jSource("m", "bolt://x:1", "u", "p")
    src._version = 7
    gs.sources[src.id] = src
    gs.notify_version(src.id, "plain")
    d = GraphDelta(
        upserts=[NodeRecord("O(1)", 2, 0, (1.0, 2.0, 3.0))], deletes=["O(2)"]
    )
    gs.notify_version(src.id, "sam3", (d, 9))
    (t0, plain), (t1, rich) = hub.frames
    assert t0 == t1 == f"graph/{src.id}/version"
    assert plain == {"version": 7, "reason": "plain"}
    assert rich["version"] == 9 and rich["base"] == 8 and rich["reason"] == "sam3"
    assert rich["delta"]["del"] == ["O(2)"] and rich["delta"]["up"]["ids"] == ["O(1)"]


# --------------------------------------------------------------------- database
def _seed_graph():
    import spark_dsg as dsg

    G = dsg.DynamicSceneGraph()
    G.add_layer(3, 1, dsg.DsgLayers.MESH_PLACES)  # heracles' ingest expects it
    G.set_labelspace(dsg.Labelspace({0: "unknown", 1: "chair", 2: "table"}), 2, 0)
    G.set_labelspace(dsg.Labelspace({0: "region", 1: "kitchen"}), 4, 0)
    for i, (label, pos) in enumerate(
        [(1, (1.0, 0.0, 0.0)), (1, (9.0, 0.0, 0.0)), (2, (2.0, 2.0, 0.0))]
    ):
        a = dsg.ObjectNodeAttributes()
        a.position = np.array(pos)
        a.semantic_label = label
        a.bounding_box = dsg.BoundingBox(
            np.array([0.5, 0.5, 0.5], np.float32), np.array(pos, np.float32)
        )
        G.add_node(dsg.DsgLayers.OBJECTS, dsg.NodeSymbol("O", i), a)
    for i in range(3):
        p = dsg.PlaceNodeAttributes()
        p.position = np.array([float(3 * i), 1.0, 0.0])
        G.add_node(dsg.DsgLayers.PLACES, dsg.NodeSymbol("P", i), p)
    r = dsg.RoomNodeAttributes()
    r.position = np.array([3.0, 1.0, 0.0])
    r.semantic_label = 1
    G.add_node(dsg.DsgLayers.ROOMS, dsg.NodeSymbol("R", 0), r)
    b = dsg.SemanticNodeAttributes()
    b.position = np.array([3.0, 1.0, 0.0])
    G.add_node(dsg.DsgLayers.BUILDINGS, dsg.NodeSymbol("B", 0), b)
    for a, c in [
        ("P0", "P1"),
        ("P1", "P2"),
        ("P0", "O0"),
        ("P2", "O1"),
        ("R0", "P0"),
        ("R0", "P1"),
        ("B0", "R0"),
    ]:
        G.insert_edge(
            dsg.NodeSymbol(a[0], int(a[1:])), dsg.NodeSymbol(c[0], int(c[1:]))
        )
    # heracles.utils.load_dsg_to_db's layer names, which the ingest reads
    G.metadata.add({"LayerIdToHeraclesLayerStr": {2: "Object", 5: "Building", 20: "MeshPlace",
                                                  "3[1]": "MeshPlace", 3: "Place", 4: "Room"}})  # fmt: skip
    return G


def _bump(db, affected=None) -> int:
    """A map write's version bump (heracles notify_map_change when it has it)."""
    from adt4_console.routes.maps import bump_map_counters

    return bump_map_counters(db, [], affected or [])


@pytest.fixture(scope="module")
def scratch():
    uri = scratch_neo4j()
    gi = pytest.importorskip("heracles.graph_interface")
    if not hasattr(gi, "read_version"):
        pytest.skip("this heracles has no _State version counter (read_version)")
    from heracles.query_interface import Neo4jWrapper

    with Neo4jWrapper(uri, NEO4J_AUTH, atomic_queries=True) as db:
        rows, _, _ = db.execute(
            f"RETURN COUNT {{ MATCH (n) WHERE NOT n:{MARKER} }} AS c, COUNT {{ MATCH (m:{MARKER}) }} AS marked"
        )
        if rows[0]["c"] and not rows[0]["marked"]:
            pytest.skip(f"{uri} holds a map this suite did not write; not wiping it")
    return gi


@pytest.fixture
def seeded(scratch):
    """A fresh seed map in the scratch DB; returns (heracles graph_interface, db)."""
    from heracles.query_interface import Neo4jWrapper

    gi = scratch
    with Neo4jWrapper(scratch_neo4j(), NEO4J_AUTH, atomic_queries=True) as db:
        db.execute("MATCH (n) DETACH DELETE n")
        db.execute(f"CREATE (:{MARKER})")
        gi.spark_dsg_to_db(_seed_graph(), db)
        _bump(db)
        yield gi, db


def _source(name="t"):
    from adt4_console.graph_service import Neo4jSource

    return Neo4jSource(name, scratch_neo4j(), *NEO4J_AUTH)


def _full():
    s = _source("full")
    try:
        return s.loaded()
    finally:
        s.close()


def _assert_matches_full_reload(loaded):
    full = _full()
    assert {k: r.key() for k, r in loaded.records.items()} == {
        k: r.key() for k, r in full.records.items()
    }
    assert loaded.edges == full.edges
    assert loaded.labelspaces == full.labelspaces


def _edit(db, ops):
    ge = pytest.importorskip("heracles.graph_edit")  # map edits: a newer heracles only

    return ge.apply_edit(db, ops, provenance={"source": "test"})


def test_map_edits_match_a_full_reload(seeded):
    _, db = seeded
    src = _source()
    v0 = src.loaded().version
    # a new class grows the object labelspace: applied in place too
    out = _edit(
        db,
        [
            {
                "op": "create_node",
                "layer": "Object",
                "pos": [4.0, 1.0, 0.0],
                "class": "zz widget",
            }
        ],
    )
    sym = out["affected"][0]
    d, v = src.apply_change(out["state_version"], out["affected"])
    assert v == v0 + 1 and src.version() == v and src.loaded().version == v
    assert [r.sym for r in d.upserts] == ["O(3)"] and d.labelspaces
    assert "zz widget" in d.labelspaces["2/0"].values()
    _assert_matches_full_reload(src.loaded())
    for ops in (
        [{"op": "create_edge", "from": "P1", "to": sym}],  # place CONTAINS the object
        [{"op": "update_node", "symbol": sym, "set": {"pos": [5.0, 2.0, 0.0]}}],
        [{"op": "delete_edge", "from": "P0", "to": "P1"}],  # between two existing nodes
        [
            {"op": "retire_node", "symbol": "P2", "reason": "test"}
        ],  # takes its edges along
        [{"op": "retire_node", "symbol": sym, "reason": "test"}],
    ):
        out = _edit(db, ops)
        res = src.apply_change(out["state_version"], out["affected"])
        assert res is not None, ops
        _assert_matches_full_reload(src.loaded())
    assert "P(2)" not in src.loaded().records and not any(
        "P(2)" in e for e in src.loaded().edges
    )
    # the poll has nothing left to see: no second version, no reload
    assert src.poll() is False


def test_object_insert_matches_a_full_reload(seeded):
    """What a tool adding an object does: insert an Object, link it, then
    notify_map_change(db, dirty_layers, [symbol])."""
    _, db = seeded
    src = _source()
    before = src.loaded()
    db.execute(
        "CREATE (o:Object {nodeSymbol: 'O50', class: 'chair', name: 'green chair', "
        "center: point({x: 6.0, y: 1.5, z: 0.2}), attr_type: 'ObjectNodeAttributes'})"
    )
    db.execute(
        "MATCH (p:Place {nodeSymbol: 'P2'}), (o:Object {nodeSymbol: 'O50'}) CREATE (p)-[:CONTAINS]->(o)"
    )
    sv = _bump(db, ["O50"])
    d, v = src.apply_change(sv, ["O50"])
    assert v == before.version + 1
    assert [r.sym for r in d.upserts] == ["O(50)"] and d.edges_add == [
        ("P(2)", "O(50)")
    ]
    assert (
        before.G.has_node(__import__("spark_dsg").NodeSymbol("O", 50)) is False
    )  # old graph untouched
    _assert_matches_full_reload(src.loaded())


def test_an_unknown_write_in_between_falls_back_to_one_reload(seeded):
    _, db = seeded
    src = _source()
    v0 = src.loaded().version
    _bump(db)  # someone else's write the console has not seen
    out = _edit(
        db, [{"op": "update_node", "symbol": "O0", "set": {"pos": [1.5, 0.0, 0.0]}}]
    )
    assert src.apply_change(out["state_version"], out["affected"]) is None
    assert src.version() == v0 + 1  # one bump ...
    assert src.poll() is False  # ... and the poll does not add a second
    _assert_matches_full_reload(src.loaded())


def test_nothing_loaded_or_no_symbols_just_invalidates(seeded):
    _, db = seeded
    src = _source()
    v0 = src.version()  # never loaded
    out = _edit(
        db, [{"op": "update_node", "symbol": "O1", "set": {"pos": [9.5, 0.0, 0.0]}}]
    )
    assert src.apply_change(out["state_version"], out["affected"]) is None
    assert src.version() == v0 + 1 and src.poll() is False
    src.loaded()
    out = _edit(
        db, [{"op": "update_node", "symbol": "O1", "set": {"pos": [9.0, 0.0, 0.0]}}]
    )
    assert src.apply_change(out["state_version"], []) is None
    _assert_matches_full_reload(src.loaded())


def test_invalidate_is_seen_once(seeded):
    """The old double reload: invalidate() after a write, then the poll saw the
    same write as a new DB key and bumped (and reloaded) again."""
    _, db = seeded
    src = _source()
    v0 = src.version()
    _bump(db)
    src.invalidate()
    assert src.version() == v0 + 1
    assert src.poll() is False and src.version() == v0 + 1
