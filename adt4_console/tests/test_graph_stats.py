"""Graph stats (layer / edge / mesh / label counts) of a file source."""

import pytest

from .conftest import make_client


def test_graph_stats_file_source(settings, tmp_path):
    pytest.importorskip("spark_dsg")
    from .synthetic import build_run

    settings.maps = {"syn": str(build_run(tmp_path))}
    client, _ = make_client(settings)
    with client:
        st = client.get("/api/graph/file:syn/stats").json()
        assert st["nodes"] == 8 and st["version"] == 1
        assert {e["key"]: e["nodes"] for e in st["layers"]} == {
            "2/0": 3,
            "2/97": 3,
            "3/0": 2,
        }
        assert st["edges"] == {"objects-places": 1, "places-places": 1}
        assert st["mesh"] == {"vertices": 6, "faces": 2, "tiles": 2}
        assert st["labels"][0] == {
            "label": 1,
            "name": "chair",
            "count": 2,
            "layer": "2/0",
        }
        assert client.get("/api/graph/file:nope/stats").status_code == 404
