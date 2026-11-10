"""Rooms / open-vocabulary regions (layer 4) reach the console's graph records.

hydra's OpenVocabRegionsUpdateFunctor writes SemanticNodeAttributes nodes
``r(i)`` into ROOMS with ``name`` = best query and an edge to every member
place (MESH_PLACES); hydra's room finder writes RoomNodeAttributes with a
semantic label from the ``_l4p0`` labelspace.  The viewer needs the names,
labels, labelspace and the region -> place parent edges.
"""

import msgpack
import numpy as np
import pytest

from adt4_console import codec
from adt4_console.graph_service import FileSource, graph_stats

dsg = pytest.importorskip("spark_dsg")


def _region_graph(path):
    G = dsg.DynamicSceneGraph()
    G.set_labelspace(dsg.Labelspace({0: "unknown", 3: "kitchen"}), 4, 0)
    places = []
    for i in range(6):
        p = dsg.PlaceNodeAttributes()
        p.position = np.array([float(i), 0.0, 0.0])
        G.add_node(3, dsg.NodeSymbol("P", i), p, 1)  # MESH_PLACES = 3p1
        places.append(dsg.NodeSymbol("P", i))
    r = dsg.SemanticNodeAttributes()
    r.position = np.array([1.0, 0.0, 0.0])
    r.name = "parking lot"
    r.semantic_label = 2
    G.add_node(dsg.DsgLayers.ROOMS, dsg.NodeSymbol("r", 0), r)
    room = dsg.RoomNodeAttributes()
    room.position = np.array([4.0, 0.0, 0.0])
    room.semantic_label = 3
    G.add_node(dsg.DsgLayers.ROOMS, dsg.NodeSymbol("R", 1), room)
    for s in places[:3]:
        G.insert_edge(dsg.NodeSymbol("r", 0), s)
    for s in places[3:]:
        G.insert_edge(dsg.NodeSymbol("R", 1), s)
    G.insert_edge(dsg.NodeSymbol("r", 0), dsg.NodeSymbol("R", 1))
    G.save(str(path), False)


def test_file_source_serves_regions(tmp_path):
    path = tmp_path / "dsg.json"
    _region_graph(path)
    src = FileSource("regions", path)
    enc = msgpack.unpackb(src.snapshot_bytes(), raw=False, strict_map_key=False)
    recs = {r.sym: r for r in codec.decode_records(enc)}
    region = recs["r(0)"]
    assert (region.layer, region.part) == (4, 0)
    assert region.name == "parking lot"
    assert region.label == 2
    room = recs["R(1)"]
    assert (room.layer, room.label, room.name) == (4, 3, "")
    edges = {tuple(e) for e in enc["edges"]}
    # region -> member places are interlayer edges (either orientation)
    for i in range(3):
        assert ("r(0)", f"P({i})") in edges or (f"P({i})", "r(0)") in edges
    for i in range(3, 6):
        assert ("R(1)", f"P({i})") in edges or (f"P({i})", "R(1)") in edges
    assert ("r(0)", "R(1)") in edges or ("R(1)", "r(0)") in edges
    assert enc["labelspaces"]["4/0"]["3"] == "kitchen"
    stats = graph_stats(src.loaded(), {})
    rooms = [layer for layer in stats["layers"] if layer["kind"] == "rooms"]
    assert rooms and rooms[0]["nodes"] == 2


def test_region_records_roundtrip_through_delta():
    """Live links: region names and labels survive the delta codec."""
    rec = codec.NodeRecord(
        sym="r(4)", layer=4, part=0, pos=(1.0, 2.0, 0.0), label=5, name="road"
    )
    state = codec.GraphState()
    delta = state.diff({rec.sym: rec}, {("r(4)", "P(1)")})
    back = codec.GraphDelta.decode(
        msgpack.unpackb(msgpack.packb(delta.encode()), raw=False, strict_map_key=False)
    )
    (up,) = back.upserts
    assert (up.sym, up.layer, up.name, up.label) == ("r(4)", 4, "road", 5)
    assert ("r(4)", "P(1)") in {tuple(e) for e in back.edges_add}
