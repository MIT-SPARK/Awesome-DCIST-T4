import msgpack
import pytest
from fastapi.testclient import TestClient
from starlette.websockets import WebSocketDisconnect

from adt4_console.app import build_services, create_app
from adt4_console.hub import Client, Hub


def _decode(frame):
    return msgpack.unpackb(frame, raw=False)


def _recv_topic(ws, topic, limit=20):
    for _ in range(limit):
        msg = _decode(ws.receive_bytes())
        if msg["topic"] == topic:
            return msg
    raise AssertionError(f"no {topic} frame")


def test_backpressure_drops_oldest_frame_of_same_topic():
    c = Client(maxlen=3)
    c.push("graph/file:a/delta", b"d1")
    c.push("other/x", b"x1")
    c.push("graph/file:a/delta", b"d2")
    c.push("graph/file:a/delta", b"d3")  # full: drops d1, keeps x1
    assert [f for _, f in c.queue] == [b"x1", b"d2", b"d3"]
    assert c.dropped == 1
    c.push("other/y", b"y1")  # no queued frame of this topic: drop oldest overall
    assert [f for _, f in c.queue] == [b"d2", b"d3", b"y1"]


def test_latest_value_topics_coalesce():
    c = Client(maxlen=10)
    for i in range(5):
        c.push("jobs", str(i).encode())
    assert list(c.queue) == [("jobs", b"4")]


def test_default_subscription_excludes_deltas_and_replays_latest():
    hub = Hub()
    hub.publish("jobs", {"n": 1})
    c = hub.add_client()
    hub.subscribe(c, None)  # replay of cached latest values
    hub.publish("graph/file:euclid/delta", {"up": 1})
    hub.publish("graph/file:euclid/version", {"version": 2})
    topics = [t for t, _ in c.queue]
    assert topics == ["jobs", "graph/file:euclid/version"]
    hub.subscribe(c, ["graph/file:euclid"])
    hub.publish("graph/file:euclid/delta", {"up": 2})
    t, frame = c.queue[-1]
    assert t == "graph/file:euclid/delta" and _decode(frame)["seq"] == 2


def test_websocket_receives_published_frames(client):
    hub = client.app.state.services.hub
    with client.websocket_connect("/api/ws") as ws:
        ws.send_bytes(msgpack.packb({"sub": ["test/"]}))
        # subscription is applied asynchronously; publish until a frame arrives
        hub.publish("test/ping", {"hello": 1})
        msg = _recv_topic(ws, "test/ping")
        assert msg["data"] == {"hello": 1}


@pytest.fixture
def auth_client(settings):
    settings.token = "s3cret"
    services = build_services(settings)
    with TestClient(create_app(settings, services)) as c:
        yield c


def test_token_auth(auth_client):
    c = auth_client
    assert c.get("/api/health").status_code == 401
    assert (
        c.get("/api/health", headers={"Authorization": "Bearer nope"}).status_code
        == 401
    )
    assert (
        c.get("/api/health", headers={"Authorization": "Bearer s3cret"}).status_code
        == 200
    )
    assert c.get("/api/health?token=s3cret").status_code == 200
    assert c.get("/").status_code == 200  # static stays public
    with pytest.raises(WebSocketDisconnect):
        with c.websocket_connect("/api/ws") as ws:
            ws.receive_bytes()
    with c.websocket_connect("/api/ws?token=s3cret") as ws:
        c.app.state.services.hub.publish("jobs/x", 1)
        assert _recv_topic(ws, "jobs/x")["data"] == 1


def test_spa_fallback(settings, tmp_path):
    settings.static_dir.mkdir()
    (settings.static_dir / "index.html").write_text("<html>spa</html>")
    (settings.static_dir / "assets").mkdir()
    (settings.static_dir / "assets" / "app.js").write_text("js")
    (settings.static_dir / "manifest.webmanifest").write_text("{}")
    services = build_services(settings)
    with TestClient(create_app(settings, services)) as c:
        assert c.get("/some/client/route").text == "<html>spa</html>"
        assert c.get("/assets/app.js").text == "js"
        # home-screen install metadata: browsers ignore a manifest without its media type
        assert (
            c.get("/manifest.webmanifest").headers["content-type"]
            == "application/manifest+json"
        )
        assert c.get("/api/does-not-exist").status_code == 404
        assert c.get("/../../etc/passwd").text == "<html>spa</html>"
