"""Agent-node images: the node's own keyframe, else the nearest keyframe anchored to
it, else the nearest keyframe of a neighbouring agent node, with distances."""

import datetime
import math
import shutil

import numpy as np
import pytest

from .conftest import make_client
from .synthetic import JPEG, build_run

dsg = pytest.importorskip("spark_dsg")
pytest.importorskip("hydra_python.image_store")

# agent index -> (position, yaw deg); a3 is far from everything
AGENTS = {
    0: ((0.0, 0.0, 0.0), 0.0),
    1: ((1.0, 0.0, 0.0), 90.0),
    2: ((2.0, 0.0, 0.0), 0.0),
    3: ((20.0, 0.0, 0.0), 0.0),
}


def _T(xyz, yaw_deg):
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    T = np.eye(4)
    T[:2, :2] = [[c, -s], [s, c]]
    T[:3, 3] = xyz
    return T


def _write_graph(run):
    G = dsg.DynamicSceneGraph()
    for k, (pos, yaw) in AGENTS.items():
        ag = dsg.AgentNodeAttributes()
        ag.position = np.array(pos)
        h = math.radians(yaw) / 2
        ag.world_R_body = dsg.Quaternion(math.cos(h), 0.0, 0.0, math.sin(h))
        ag.timestamp = datetime.timedelta(seconds=k)
        G.add_node(2, dsg.NodeSymbol("a", k), ag, 97)
    G.save(str(run / "hydra" / "backend" / "dsg.json"), False)


def _write_store(run):
    """a0: own keyframe (identity offset; a0 has no stamp). a1: no own keyframe (its
    stamp is 1 s, its frames are 2-3 ns), two keyframes 0.5 m / 0.2 m ahead (body x)
    with 30 / 10 deg extra heading. a2, a3: nothing anchored."""
    from hydra_python.image_store import FrameKind, ImageStore, make_node_id

    shutil.rmtree(run / "image_store", ignore_errors=True)
    src = run / "agent_src"
    src.mkdir()
    w = ImageStore.open_writer(run / "image_store", robot_id=0)
    sid = w.add_sensor("cam", 4, 4, 1.0, 1.0, 2.0, 2.0)
    frames = {
        "kf0": (FrameKind.KEYFRAME, 0, np.eye(4)),
        "sk_far": (FrameKind.KEYFRAME, 1, _T((0.5, 0.0, 0.0), 30.0)),
        "sk_near": (FrameKind.KEYFRAME, 1, _T((0.2, 0.0, 0.0), 10.0)),
    }
    fids = {}
    for ts, (name, (kind, anchor, aTb)) in enumerate(frames.items(), start=1):
        f = src / f"{name}.jpg"
        f.write_bytes(JPEG)
        fids[name] = w.add_frame_ref(
            sid, ts, kind=kind, rgb=f, link="copy",
            anchor_node_id=make_node_id("a", anchor), anchor_T_body=aTb,
            world_T_body_odom=_T((100.0, 0.0, 0.0), 0.0),  # odom never used: anchors resolve
        )  # fmt: skip
    w.close()
    return fids


@pytest.fixture
def run_dir(tmp_path):
    run = build_run(tmp_path)
    _write_graph(run)
    fids = _write_store(run)
    return run, fids


@pytest.fixture
def aclient(settings, run_dir):
    settings.maps = {"syn": str(run_dir[0])}
    client, _ = make_client(settings)
    with client:
        yield client, run_dir[1]


def test_own_keyframe_first(aclient):
    client, fids = aclient
    d = client.get("/api/graph/file:syn/node/a(0)").json()
    info = d["image_info"]
    assert info[0]["kind"] == "keyframe" and info[0]["own"] and info[0]["dist_m"] == 0.0
    assert info[0]["frame_id"] == fids["kf0"] and info[0]["anchor"] == "a(0)"
    # the neighbour's keyframes (~1 m away) follow in the strip, nearest first
    assert [i["frame_id"] for i in info[1:]] == [fids["sk_near"], fids["sk_far"]]
    assert len(d["images"]) == 3


def test_agent_without_own_keyframe_uses_nearest_anchored_keyframe(aclient):
    client, fids = aclient
    d = client.get("/api/graph/file:syn/node/a(1)").json()
    info = d["image_info"]
    hero = info[0]
    assert hero["kind"] == "keyframe" and not hero["own"]
    assert hero["frame_id"] == fids["sk_near"] and hero["anchor"] == "a(1)"
    assert hero["dist_m"] == pytest.approx(0.2, abs=1e-3)
    assert hero["yaw_deg"] == pytest.approx(10.0, abs=0.1)
    assert info[1]["frame_id"] == fids["sk_far"]
    assert info[1]["dist_m"] == pytest.approx(0.5, abs=1e-3) and info[1][
        "yaw_deg"
    ] == pytest.approx(30.0, abs=0.1)
    # the neighbour's keyframe is 1 m away: last in the strip
    assert (
        info[2]["frame_id"] == fids["kf0"]
        and info[2]["kind"] == "keyframe"
        and not info[2]["own"]
    )
    assert client.get(d["images"][0]).content == JPEG


def test_agent_with_nothing_anchored_falls_back_to_neighbours(aclient):
    client, fids = aclient
    info = client.get("/api/graph/file:syn/node/a(2)").json()["image_info"]
    # a1 faces +y, so its keyframes sit at (1, 0.2) / (1, 0.5) heading 100 / 120 deg;
    # a2 at (2, 0) heading 0: kf0 (2 m, 0 deg) outranks them (1.02 m + 100 deg ~ 2.13)
    hero = info[0]
    expect = min(
        [
            ("sk_near", math.hypot(1.0, 0.2), 100.0),
            ("sk_far", math.hypot(1.0, 0.5), 120.0),
            ("kf0", 2.0, 0.0),
        ],
        key=lambda t: t[1] + t[2] / 90.0,
    )
    assert expect[0] == "kf0"
    assert hero["frame_id"] == fids[expect[0]] and hero["dist_m"] == pytest.approx(
        expect[1], abs=1e-3
    )
    assert hero["anchor"] != "a(2)" and not hero["own"]
    # the strip is ordered by distance
    rest = [i["dist_m"] for i in info[1:]]
    assert rest == sorted(rest) and len(info) == 3


def test_far_agent_has_no_images(aclient):
    client, _ = aclient
    d = client.get("/api/graph/file:syn/node/a(3)").json()
    assert d["images"] == [] and d["image_info"] == []


def test_index_cached_per_version(aclient):
    client, _ = aclient
    src = client.app.state.services.graph.get("file:syn")
    client.get("/api/graph/file:syn/node/a(1)")
    idx = src._agent_idx[1]
    client.get("/api/graph/file:syn/node/a(2)")
    client.get("/api/graph/file:syn/image/a(2)/0")
    assert src._agent_idx[1] is idx


def test_live_records_without_rotations(tmp_path):
    """Live sources only know node positions: headings come from the frames' odometry."""
    from hydra_python.image_store import FrameKind, ImageStore, make_node_id

    from adt4_console import codec
    from adt4_console.agent_images import AgentFrameIndex

    store_dir = tmp_path / "s"
    img = tmp_path / "x.jpg"
    img.write_bytes(JPEG)
    w = ImageStore.open_writer(store_dir, robot_id=0)
    sid = w.add_sensor("cam", 4, 4, 1.0, 1.0, 2.0, 2.0)
    # anchor a5 odometric heading 90 deg; the frame is 1 m ahead of it in body x
    aTb = _T((1.0, 0.0, 0.0), 0.0)
    odom_anchor = _T((50.0, 50.0, 0.0), 90.0)
    fid = w.add_frame_ref(sid, 1, kind=FrameKind.KEYFRAME, rgb=img, link="copy",
                          anchor_node_id=make_node_id("a", 5), anchor_T_body=aTb,
                          world_T_body_odom=odom_anchor @ aTb)  # fmt: skip
    w.close()
    store = ImageStore.open(store_dir)
    recs = {"a(5)": codec.NodeRecord("a(5)", 2, 97, (3.0, 4.0, 0.0))}
    idx = AgentFrameIndex.from_store(
        store, positions={k: r.pos for k, r in recs.items()}
    )
    out = idx.select("a(5)")
    assert out[0].frame_id == fid
    assert out[0].dist_m == pytest.approx(1.0, abs=1e-6) and out[
        0
    ].yaw_deg == pytest.approx(0.0, abs=1e-6)
    assert np.allclose(
        idx.pos[0], (3.0, 5.0, 0.0)
    )  # optimized position + odometric heading


def test_neo4j_agent_and_subkeyframe_rows(tmp_path):
    from adt4_console import codec
    from adt4_console.graph_service import Loaded, Neo4jSource

    img = tmp_path / "s.jpg"
    img.write_bytes(JPEG)
    agents = [
        {
            "s": "a7",
            "c": [0.0, 0.0, 0.0],
            "w": 1.0,
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
            "p": None,
            "f": None,
        },
    ]
    subs = [
        {
            "c": [0.3, 0.4, 0.0],
            "w": 1.0,
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
            "p": str(img),
            "f": 42,
            "a": "a7",
        },
    ]

    class Fake(Neo4jSource):
        def query(self, cypher, **params):
            if "MATCH (a:Agent)" in cypher:
                return agents
            if "SubKeyframe" in cypher:
                return subs
            return []

        def version(self):
            return 1

    src = Fake("t", "bolt://x", "u", "p")
    loaded = Loaded(
        1, {"a(7)": codec.NodeRecord("a(7)", 2, 97, (0.0, 0.0, 0.0))}, set(), {}
    )
    ims = src.agent_images("a(7)", loaded)
    assert len(ims) == 1
    i = ims[0].info()
    assert i["kind"] == "subkeyframe" and i["frame_id"] == 42 and i["anchor"] == "a(7)"
    assert i["dist_m"] == pytest.approx(0.5) and i["yaw_deg"] == pytest.approx(0.0)


def test_own_keyframes_by_stamp_or_identity_offset():
    from types import SimpleNamespace as NS

    from adt4_console.agent_images import own_keyframes

    def fr(fid, ts, anchor, aTb):
        return NS(
            frame_id=fid, timestamp_ns=ts, anchor_node_id=anchor, anchor_T_body=aTb
        )

    off = _T((0.3, 0.0, 0.0), 0.0)
    frames = [
        fr(1, 100, 7, off),
        fr(2, 200, 7, np.eye(4)),
        fr(3, 300, 8, off),
        fr(4, 400, None, None),
    ]
    # no stamps: the identity-offset frame of each anchor (none for 8)
    assert {a: f.frame_id for a, f in own_keyframes(frames).items()} == {7: 2}
    # stamps (ImageStore.agent_frame): nearest within 50 ms, whatever the offset
    own = own_keyframes(frames, {7: 110, 8: 300 + 60_000_000}.get)
    assert {a: f.frame_id for a, f in own.items()} == {7: 1}
