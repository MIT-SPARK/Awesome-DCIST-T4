"""Place keyframes: which keyframes observed a traversability place (frustum, range,
depth occlusion, as scripts/gps_fusion/regions/place_features.py) and the endpoint."""

import datetime
import math

import numpy as np
import pytest

from .conftest import make_client
from .synthetic import JPEG

dsg = pytest.importorskip("spark_dsg")
pytest.importorskip("hydra_python.image_store")
PIL = pytest.importorskip("PIL.Image")

from adt4_console import place_keyframes as pk  # noqa: E402

W, H, F = 64, 48, 32.0  # 90 deg horizontal field of view
# camera 1 m above the body, looking along body x (z_cam = x_body, x_cam = -y_body)
BODY_T_SENSOR = np.array(
    [[0, 0, 1, 0], [-1, 0, 0, 0], [0, -1, 0, 1.0], [0, 0, 0, 1]], float
)

# places (t prefix, MESH_PLACES): position, 16 rays of 1 m
PLACES = {
    0: (5.0, 0.0, 0.0),  # ahead of robot a
    1: (-5.0, 0.0, 0.0),  # ahead of robot b (behind a)
    2: (20.0, 0.0, 0.0),  # out of range for everyone
    3: (5.0, 4.5, 0.0),  # at the edge of a's image: partly visible
}
# keyframes: name -> (agent sym, position, yaw deg, t seconds, depth)
# depth: a number = constant depth image (m), None = no depth image
KEYFRAMES = {
    "A0": ("a", 0, (0.0, 0.0, 0.0), 0.0, 100, 30.0),
    "A1": ("a", 1, (0.0, 0.0, 0.0), 0.0, 150, 2.0),  # a wall 2 m ahead hides t0
    "A2": ("a", 2, (2.0, 0.0, 0.0), 0.0, 50, None),  # no depth, t0 within 5 m
    "B0": ("b", 0, (0.0, 0.0, 0.0), 180.0, 200, 30.0),
}
SESSIONS = {
    "sa": {"robot": "alpha", "bag": "/bags/run_one/recorded_data"},
    "sb": {"robot": "beta", "bag": "/bags/run_two"},
}


def _quat(yaw_deg):
    h = math.radians(yaw_deg) / 2
    return dsg.Quaternion(math.cos(h), 0.0, 0.0, math.sin(h))


def _T(xyz, yaw_deg):
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    T = np.eye(4)
    T[:2, :2] = [[c, -s], [s, c]]
    T[:3, 3] = xyz
    return T


def build_place_run(root):
    from hydra_python.image_store import FrameKind, ImageStore, make_node_id

    run = root / "place_run"
    (run / "hydra" / "backend").mkdir(parents=True)
    G = dsg.DynamicSceneGraph()
    G.metadata.set({"sessions": SESSIONS})
    for i, pos in PLACES.items():
        a = dsg.TravNodeAttributes()
        a.position = np.array(pos)
        a.radii = [1.0] * 16
        a.states = [dsg.TraversabilityState.TRAVERSABLE] * 16
        G.add_node(3, dsg.NodeSymbol("t", i), a, 1)
    for name, (c, k, pos, yaw, t, _) in KEYFRAMES.items():
        ag = dsg.AgentNodeAttributes()
        ag.position = np.array(pos)
        ag.world_R_body = _quat(yaw)
        ag.timestamp = datetime.timedelta(seconds=t)
        ag.metadata.set({"session": "sa" if c == "a" else "sb"})
        G.add_node(2, dsg.NodeSymbol(c, k), ag, ord(c))
    a = dsg.PlaceNodeAttributes()  # not a traversability place
    a.position = np.array([1.0, 1.0, 0.0])
    G.add_node(3, dsg.NodeSymbol("P", 0), a)
    G.save(str(run / "hydra" / "backend" / "dsg.json"), False)

    src = run / "src"
    src.mkdir()
    w = ImageStore.open_writer(run / "image_store", robot_id=0)
    sid = w.add_sensor(
        "cam", W, H, F, F, W / 2, H / 2, 0.001, body_T_sensor=BODY_T_SENSOR
    )
    fids = {}
    for name, (c, k, pos, yaw, t, depth) in KEYFRAMES.items():
        rgb = src / f"{name}.jpg"
        rgb.write_bytes(JPEG)
        dpath = None
        if depth is not None:
            dpath = src / f"{name}_depth.png"
            PIL.fromarray(np.full((H, W), int(depth * 1000), np.uint16)).save(dpath)
        fids[name] = w.add_frame_ref(
            sid, int(t * 1e9), kind=FrameKind.KEYFRAME, rgb=rgb, depth=dpath, link="copy",
            anchor_node_id=make_node_id(c, k), anchor_T_body=np.eye(4),
            world_T_body_odom=_T((100.0, 100.0, 0.0), 0.0),  # odom never used: anchors resolve
        )  # fmt: skip
    w.close()
    return run, fids


# --- the visibility test against a plain transcription of place_features.py -----------


def _ref_visible_fraction(pts_w, T_ws, K, w, h, depth):
    """place_features.visible_fraction with its default arguments, point by point."""
    R, t = T_ws[:3, :3], T_ws[:3, 3]
    pc = (pts_w - t) @ R
    z = pc[:, 2]
    ok = z > 0.3
    rng = np.linalg.norm(pc, axis=1)
    ok &= rng < 12.0
    if not ok.any():
        return 0.0
    u = K[0, 0] * pc[:, 0] / np.maximum(z, 1e-6) + K[0, 2]
    v = K[1, 1] * pc[:, 1] / np.maximum(z, 1e-6) + K[1, 2]
    ok &= (u >= 0) & (u < w) & (v >= 0) & (v < h)
    seen = 0
    for i in np.nonzero(ok)[0]:
        if depth is None:
            seen += rng[i] < 5.0
            continue
        dh, dw = depth.shape
        ui, vi = int(u[i] * dw / w), int(v[i] * dh / h)
        win = depth[max(vi - 2, 0) : vi + 3, max(ui - 2, 0) : ui + 3]
        win = win[win > 0]
        if win.size == 0:
            seen += rng[i] < 5.0
        elif np.median(win) >= z[i] - max(0.5, 0.1 * z[i]):
            seen += 1
    return seen / len(pts_w)


def test_visibility_matches_reference_on_random_scenes():
    rng = np.random.default_rng(3)
    cam = pk.Camera(W, H, F, F, W / 2, H / 2, 0.001)
    K = np.array([[F, 0, W / 2], [0, F, H / 2], [0, 0, 1]])
    for trial in range(200):
        T = _T(rng.uniform(-3, 3, 3), rng.uniform(-180, 180)) @ BODY_T_SENSOR
        pts = pk.place_samples(
            rng.uniform(-14, 14, 3) * [1, 1, 0.1], list(rng.uniform(0.2, 3, 16))
        )
        depth = None
        if trial % 4:
            depth = rng.uniform(0.5, 15, (H // 2, W // 2)).astype(
                np.float32
            )  # half resolution
            depth[rng.random(depth.shape) < 0.3] = 0  # invalid pixels
            if trial % 4 == 3:
                depth[:] = 0
        z, r, u, v, ok = pk.project(pts, T, cam)
        got = pk.seen_points(z, r, u, v, ok, cam, depth).mean()
        assert got == pytest.approx(_ref_visible_fraction(pts, T, K, W, H, depth)), (
            trial
        )


def test_place_samples_centre_and_ring():
    pts = pk.place_samples((1.0, 2.0, 0.5), [2.0] * 16)
    assert pts.shape == (9, 3) and np.allclose(pts[0], (1.0, 2.0, 0.5))
    assert np.allclose(
        np.linalg.norm(pts[1:, :2] - (1.0, 2.0), axis=1), 1.2
    )  # 0.6 of the ray
    assert pk.place_samples((0, 0, 0)).shape == (1, 3)  # no boundary: the centre only


# --- sources + endpoint --------------------------------------------------------------


@pytest.fixture
def place_run(tmp_path):
    return build_place_run(tmp_path)


@pytest.fixture
def pclient(settings, place_run, tmp_path):
    settings.maps = {"pl": str(place_run[0])}
    settings.mesh_cache_dir = str(tmp_path / "cache" / "mesh")
    client, services = make_client(settings)
    with client:
        yield client, place_run[1], services


def _get(client, sym, sort="newest"):
    r = client.get(f"/api/graph/file:pl/place/{sym}/keyframes", params={"sort": sort})
    assert r.status_code == 200, r.text
    return r.json()


def test_place_seen_by_frustum_range_and_depth(pclient):
    client, fids, _ = pclient
    d = _get(client, "t0")
    # A0 sees it; A1's wall hides it; A2 has no depth but is within 5 m; B0 looks away
    assert [k["frame_id"] for k in d["keyframes"]] == [
        fids["A0"],
        fids["A2"],
    ]  # newest first
    a0 = d["keyframes"][0]
    assert (
        a0["agent"] == "a(0)" and a0["robot_key"] == "a" and a0["t_ns"] == 100 * 10**9
    )
    assert a0["visible"] == 1.0 and a0["dist_m"] == pytest.approx(
        math.hypot(5, 1), abs=0.01
    )
    assert a0["image"] == f"/api/graph/file:pl/frame/{fids['A0']}"
    assert d["sym"] == "t(0)" and d["n"] == 2 and d["keyframes_total"] == 4
    assert [k["frame_id"] for k in _get(client, "t1")["keyframes"]] == [fids["B0"]]
    assert _get(client, "t2")["keyframes"] == []  # 20 m: out of range
    part = _get(client, "t3")["keyframes"]
    assert [k["frame_id"] for k in part] == [fids["A0"]] and 0 < part[0]["visible"] < 1


def test_sorts_and_sessions(pclient):
    client, fids, _ = pclient
    assert [k["frame_id"] for k in _get(client, "t0", "oldest")["keyframes"]] == [
        fids["A2"],
        fids["A0"],
    ]
    # equal visibility: the nearer keyframe first
    assert [k["frame_id"] for k in _get(client, "t0", "visible")["keyframes"]] == [
        fids["A2"],
        fids["A0"],
    ]
    s = _get(client, "t0")["sessions"]
    assert len(s) == 1
    assert (
        s[0]["key"] == "a"
        and s[0]["session"] == "sa"
        and s[0]["robot"] == "alpha"
        and s[0]["n"] == 2
    )
    assert (
        s[0]["bag_short"] == "run_one"
        and s[0]["first_ns"] == 50 * 10**9
        and s[0]["date"] == "1970-01-01"
    )
    s = _get(client, "t1")["sessions"]
    assert s[0]["robot"] == "beta" and s[0]["bag_short"] == "run_two"


def test_errors(pclient):
    client, _, _ = pclient
    assert (
        client.get(
            "/api/graph/file:pl/place/t0/keyframes", params={"sort": "x"}
        ).status_code
        == 422
    )
    assert (
        client.get("/api/graph/file:pl/place/P0/keyframes").status_code == 404
    )  # not a mesh place
    assert client.get("/api/graph/file:pl/place/t99/keyframes").status_code == 404
    assert client.get("/api/graph/file:pl/frame/123456").status_code == 404


def test_frame_image_served(pclient):
    client, fids, _ = pclient
    r = client.get(f"/api/graph/file:pl/frame/{fids['A0']}")
    assert r.status_code == 200 and r.content == JPEG


def test_full_table_matches_single_place_and_is_cached_on_disk(pclient, tmp_path):
    client, _, services = pclient
    before = {s: _get(client, s)["keyframes"] for s in ("t0", "t1", "t2", "t3")}
    src = services.graph.get("file:pl")
    idx = src.place_keyframe_index()
    assert idx.wait(30)
    after = {s: _get(client, s) for s in before}
    assert all(a["table"] == "ready" for a in after.values())
    assert {s: a["keyframes"] for s, a in after.items()} == before
    files = list((tmp_path / "cache" / "place_keyframes").glob("*.npz"))
    assert len(files) == 1
    # a new index over the same content reads the table back instead of rebuilding
    src._pk = None
    again = src.place_keyframe_index()
    assert again is not idx and again.ready and again.build_seconds == 0.0
    assert _get(client, "t0")["keyframes"] == before["t0"]


def test_index_cached_per_version(pclient):
    _, _, services = pclient
    src = services.graph.get("file:pl")
    assert src.place_keyframe_index() is src.place_keyframe_index()


def test_neo4j_poses_from_agent_rows(place_run):
    """Neo4j sources resolve keyframe anchors with the Agent rows (center + rotation)."""
    from adt4_console import codec
    from adt4_console.graph_service import Loaded, Neo4jSource, open_image_store

    run, fids = place_run
    agents = [
        {
            "s": f"{c}{k}",
            "c": list(pos),
            "w": _quat(yaw).w,
            "x": 0.0,
            "y": 0.0,
            "z": _quat(yaw).z,
        }
        for c, k, pos, yaw, *_ in KEYFRAMES.values()
    ]
    store = open_image_store(run / "image_store")
    records = {
        f"t({i})": codec.NodeRecord(f"t({i})", 3, 1, pos, radii=(1.0,) * 16)
        for i, pos in PLACES.items()
    }

    class Fake(Neo4jSource):
        def query(self, cypher, **params):
            return agents if "MATCH (a:Agent)" in cypher else []

        def version(self):
            return 1

        def loaded(self):
            return Loaded(1, records, set(), {})

        def image_store(self):
            return store

        def trajectory_sources(self):
            return {"a": {"session": "sa"}}

    d = Fake("t", "bolt://x", "u", "p").place_keyframes("t0", "oldest")
    assert [k["frame_id"] for k in d["keyframes"]] == [fids["A2"], fids["A0"]]
    assert d["sessions"][0]["session"] == "sa"


def test_map_without_image_store(settings, tmp_path):
    from .synthetic import build_run

    run = build_run(tmp_path)
    G = dsg.DynamicSceneGraph.load(str(run / "hydra" / "backend" / "dsg.json"))
    a = dsg.TravNodeAttributes()
    a.position = np.array([0.0, 0.0, 0.0])
    G.add_node(3, dsg.NodeSymbol("t", 0), a, 1)  # no radii: the centre only
    G.save(str(run / "hydra" / "backend" / "dsg.json"), False)
    import shutil

    shutil.rmtree(run / "image_store")
    settings.maps = {"bare": str(run)}
    client, _ = make_client(settings)
    with client:
        r = client.get("/api/graph/file:bare/place/t0/keyframes")
        assert (
            r.status_code == 200
            and r.json()["keyframes"] == []
            and r.json()["keyframes_total"] == 0
        )
