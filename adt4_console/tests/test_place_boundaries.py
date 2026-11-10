"""TravNodeAttributes ray boundaries (radii/states) reach the console's graph records."""

import msgpack
import numpy as np
import pytest

from adt4_console import codec
from adt4_console.graph_service import FileSource

dsg = pytest.importorskip("spark_dsg")


@pytest.mark.skipif(not hasattr(dsg, "TravNodeAttributes"), reason="old spark_dsg")
def test_file_source_serves_place_boundaries(tmp_path):
    G = dsg.DynamicSceneGraph()
    t = dsg.TravNodeAttributes()
    t.position = np.array([1.0, 2.0, 0.5])
    t.radii = [0.5 + 0.01 * i for i in range(24)]
    t.states = [dsg.TraversabilityState(i % 4) for i in range(24)]
    G.add_node(3, dsg.NodeSymbol("t", 1), t, 1)
    p = dsg.PlaceNodeAttributes()
    p.position = np.array([0.0, 0.0, 0.0])
    G.add_node(3, dsg.NodeSymbol("P", 0), p)
    path = tmp_path / "dsg.json"
    G.save(str(path), False)

    src = FileSource("t", path)
    enc = msgpack.unpackb(src.snapshot_bytes(), raw=False, strict_map_key=False)
    recs = {r.sym: r for r in codec.decode_records(enc)}
    trav = recs["t(1)"]
    assert (trav.layer, trav.part) == (3, 1)
    assert np.allclose(trav.radii, [0.5 + 0.01 * i for i in range(24)], atol=0.006)
    assert trav.states == tuple(i % 4 for i in range(24))
    assert recs["P(0)"].radii is None
