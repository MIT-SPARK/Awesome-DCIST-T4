"""A tiny Hydra-like run directory built with spark_dsg (objects, places, agents, mesh, images)."""

import datetime
import pathlib

import numpy as np

# sym index -> (label, position); label 1 = chair, 2 = table
OBJECTS = {0: (1, (1.0, 0.0, 0.0)), 1: (1, (9.0, 0.0, 0.0)), 2: (2, (2.0, 2.0, 0.0))}
LABELS = {0: "unknown", 1: "chair", 2: "table"}
JPEG = b"\xff\xd8\xff\xe0" + b"\x00" * 64 + b"\xff\xd9"

PLY = """ply
format ascii 1.0
element vertex 4
property float x
property float y
property float z
property uchar red
property uchar green
property uchar blue
element face 2
property list uchar uint vertex_indices
end_header
0 0 0 255 0 0
1 0 0 0 255 0
0 1 0 0 0 255
30 30 0 10 10 10
3 0 1 2
3 0 1 3
"""


def build_run(root: pathlib.Path) -> pathlib.Path:
    import spark_dsg as dsg

    run = root / "synthetic_run"
    backend = run / "hydra" / "backend"
    backend.mkdir(parents=True)
    G = dsg.DynamicSceneGraph()
    G.set_labelspace(dsg.Labelspace(LABELS), 2, 0)
    for i, (label, pos) in OBJECTS.items():
        a = dsg.KhronosObjectAttributes()
        a.position = np.array(pos)
        a.semantic_label = label
        a.bounding_box = dsg.BoundingBox(
            np.array([0.5, 0.5, 0.5], np.float32), np.array(pos, np.float32)
        )
        G.add_node(dsg.DsgLayers.OBJECTS, dsg.NodeSymbol("O", i), a)
    for i in range(2):
        p = dsg.PlaceNodeAttributes()
        p.position = np.array([float(i), 1.0, 0.0])
        G.add_node(3, dsg.NodeSymbol("P", i), p)
    for k, t in enumerate((30.0, 10.0, 20.0)):  # inserted out of time order
        ag = dsg.AgentNodeAttributes()
        ag.position = np.array([t, 0.0, 0.0])
        # spark_dsg leaves a new agent's orientation uninitialised: a NaN there is
        # saved as null and the file no longer loads (it depended on test order)
        ag.world_R_body = dsg.Quaternion(1.0, 0.0, 0.0, 0.0)
        ag.timestamp = datetime.timedelta(seconds=t)
        G.add_node(2, dsg.NodeSymbol("a", k), ag, 97)
    G.insert_edge(dsg.NodeSymbol("P", 0), dsg.NodeSymbol("P", 1))
    G.insert_edge(dsg.NodeSymbol("P", 0), dsg.NodeSymbol("O", 0))
    # the robot path in time order (a1, a2, a0) and the place a1 is in
    G.insert_edge(dsg.NodeSymbol("a", 1), dsg.NodeSymbol("a", 2))
    G.insert_edge(dsg.NodeSymbol("a", 2), dsg.NodeSymbol("a", 0))
    G.insert_edge(dsg.NodeSymbol("P", 1), dsg.NodeSymbol("a", 1))
    G.save(str(backend / "dsg.json"), False)
    (backend / "mesh.ply").write_text(PLY)
    _build_image_store(run)
    return run


def _build_image_store(run: pathlib.Path) -> None:
    """Images of O(0) (two kept observations) and a(0) (keyframe + sub-keyframe)."""
    from hydra_python.image_store import FrameKind, ImageStore, make_node_id

    src = run / "src_images"
    src.mkdir()
    w = ImageStore.open_writer(run / "image_store", robot_id=0)
    sid = w.add_sensor("cam", 4, 4, 1.0, 1.0, 2.0, 2.0)
    fids = {}
    for ts in (1, 2, 3, 4):
        f = src / f"{ts}.jpg"
        f.write_bytes(JPEG)
        fids[ts] = w.add_frame_ref(
            sid, ts, kind=FrameKind.OBSERVATION if ts < 3 else FrameKind.KEYFRAME,
            rgb=f, link="copy",
        )  # fmt: skip
    o0 = make_node_id("O", 0)
    w.bind_track(5, o0)
    for score, ts in ((0.9, 2), (0.5, 1)):
        w.add_observation(5, fids[ts], timestamp_ns=ts, score=score, kept=True)
    a0 = make_node_id("a", 0)
    w.set_anchor(fids[3], a0)
    w.set_anchor(fids[4], a0)  # a second frame anchored to the same agent
    w.close()
