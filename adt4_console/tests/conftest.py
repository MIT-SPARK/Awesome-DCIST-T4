import os
import pathlib
import socket
import sys
from urllib.parse import urlparse

import pytest

PKG = pathlib.Path(__file__).resolve().parents[1]
if str(PKG) not in sys.path:
    sys.path.insert(0, str(PKG))

from adt4_console.app import build_services, create_app  # noqa: E402
from adt4_console.settings import Settings  # noqa: E402

# A throwaway Neo4j the Neo4j tests may WIPE, e.g.
#   docker run -d --name console_viewer_neo4j -p 127.0.0.1:7721:7687 \
#       -e NEO4J_AUTH=neo4j/neo4j_pw neo4j:5.25.1
#   ADT4_TEST_SCRATCH_NEO4J=bolt://localhost:7721 pytest
# Never point it at a database holding a map you want to keep.  Unset: those tests skip.
SCRATCH_NEO4J = os.environ.get("ADT4_TEST_SCRATCH_NEO4J")
NEO4J_AUTH = (
    os.environ.get("ADT4_TEST_SCRATCH_NEO4J_USER", "neo4j"),
    os.environ.get("ADT4_TEST_SCRATCH_NEO4J_PASSWORD", "neo4j_pw"),
)


def port_open(host: str, port: int) -> bool:
    with socket.socket() as s:
        s.settimeout(0.5)
        return s.connect_ex((host, port)) == 0


def scratch_neo4j() -> str:
    """The scratch Neo4j URI, or skip the test."""
    if not SCRATCH_NEO4J:
        pytest.skip("set ADT4_TEST_SCRATCH_NEO4J to a Neo4j the tests may wipe")
    u = urlparse(SCRATCH_NEO4J)
    if not port_open(u.hostname or "localhost", u.port or 7687):
        pytest.skip(f"scratch Neo4j {SCRATCH_NEO4J} not reachable")
    pytest.importorskip("heracles")
    return SCRATCH_NEO4J


@pytest.fixture
def settings(tmp_path):
    return Settings(
        output_root=str(tmp_path / "out"),
        static_dir=tmp_path / "static",
        confirm_timeout_s=5.0,
        neo4j_poll_s=3600.0,  # tests poll explicitly
    )


def make_client(settings, **service_kw):
    from fastapi.testclient import TestClient

    services = build_services(settings, **service_kw)
    return TestClient(create_app(settings, services)), services


@pytest.fixture
def client(settings):
    c, services = make_client(settings)
    with c:
        yield c
