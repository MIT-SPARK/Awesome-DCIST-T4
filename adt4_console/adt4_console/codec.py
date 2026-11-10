"""Compact scene-graph / mesh wire format served to the browser.

Pure numpy + msgpack: the console serves this over its REST/WebSocket API and
``frontend/src/codec.ts`` mirrors it, so the byte layout must not change.

Graph wire format (``encode_graph`` / ``GraphState.diff``)::

    {
      "ids":    [str, ...]            # node symbols, e.g. "O(12)", "a(3)", "P(850)"
      "layer":  bytes uint8[N]        # spark_dsg layer id
      "part":   bytes uint8[N]        # layer partition (0 for the primary partition)
      "pos":    bytes float32[N*3]
      "label":  bytes int16[N]        # semantic label, -1 if none
      "bbox":   bytes float32[N*6]    # min xyz, max xyz; NaN if none
      "color":  bytes uint8[N*3]
      "names":  [str, ...]            # node name or "" (keyed by index)
      "active": bytes uint8[N]
      # optional, only present when some node has a ray boundary
      # (spark_dsg TravNodeAttributes: traversability / "mesh" places):
      "nrad":   bytes uint16[N]       # boundary rays per node (0 = none)
      "radii":  bytes uint16[sum]     # ray lengths in cm, concatenated
      "states": bytes uint8[sum]      # TraversabilityState per ray
      "edges":  [[src_sym, dst_sym], ...]
      "labelspaces": {"2/0": {"18": "sign", ...}, ...}
    }

Mesh tiles: faces are bucketed by centroid into ``tile_size`` cubes; each
tile carries its own vertex subset quantised to ``QUANT`` metres relative to
the tile origin as int16, uint8 colours and uint32 indices. For the browser
the console also serves them coarser (``decimate_tiles``) and packed for gzip
(``pack_tile``, mirrored by ``frontend/src/codec.ts decodePackedTile``).
"""

from __future__ import annotations

import hashlib
import math
from dataclasses import dataclass, field

import numpy as np

POS_QUANT = 0.01  # metres; positions are compared/hashed at 1 cm
QUANT = 0.01  # mesh vertex quantisation (metres)
DEFAULT_TILE_SIZE = 8.0
RADIUS_QUANT = 0.01  # metres; boundary ray lengths travel as uint16 cm
_RADIUS_MAX_CM = 65535

# ----------------------------------------------------------------------------
# Graph records
# ----------------------------------------------------------------------------


@dataclass
class NodeRecord:
    sym: str
    layer: int
    part: int
    pos: tuple[float, float, float]
    label: int = -1
    bbox: tuple | None = None  # (minx, miny, minz, maxx, maxy, maxz)
    color: tuple[int, int, int] = (0, 0, 0)
    name: str = ""
    active: bool = False
    # TravNodeAttributes ray boundary: radii[i] (metres) at angle i/N*2pi from
    # pos, states[i] a spark_dsg TraversabilityState (0 unknown, 1 traversable,
    # 2 intraversable, 3 traversed).  None for every other attribute type.
    radii: tuple[float, ...] | None = None
    states: tuple[int, ...] | None = None

    def key(self) -> tuple:
        """Quantised identity used for change detection."""
        q = tuple(round(v / POS_QUANT) for v in self.pos)
        b = tuple(round(v / POS_QUANT) for v in self.bbox) if self.bbox else None
        r = _radii_cm(self.radii) if self.radii else None
        s = tuple(self.states) if self.radii else None
        return (self.layer, self.part, q, self.label, b, self.color, self.name, r, s)


def _radii_cm(radii) -> tuple[int, ...]:
    return tuple(
        min(_RADIUS_MAX_CM, max(0, round(float(r) / RADIUS_QUANT)))
        if math.isfinite(float(r))
        else 0
        for r in radii
    )


def _boundary(attrs) -> tuple[tuple | None, tuple | None]:
    """(radii, states) of a TravNodeAttributes-like ``attrs``, radii rounded to
    the wire quantisation and states padded/truncated to len(radii)."""
    radii = getattr(attrs, "radii", None)
    if radii is None:
        return None, None
    try:
        cm = _radii_cm(list(radii))
    except (TypeError, ValueError):
        return None, None
    if not cm:
        return None, None
    states = []
    for s in list(getattr(attrs, "states", None) or [])[: len(cm)]:
        v = int(getattr(s, "value", s))
        states.append(v if 0 <= v <= 255 else 0)
    states += [0] * (len(cm) - len(states))
    return tuple(c * RADIUS_QUANT for c in cm), tuple(states)


def _finite(seq) -> bool:
    return all(math.isfinite(float(v)) for v in seq)


def node_record(node) -> NodeRecord:
    """Project a spark_dsg node to a NodeRecord (duck-typed over attribute types)."""
    attrs = node.attributes
    key = node.layer
    layer, part = (key.layer, key.partition) if hasattr(key, "layer") else (int(key), 0)
    pos = tuple(float(v) for v in attrs.position)
    label = getattr(attrs, "semantic_label", None)
    if label is None or label == getattr(attrs, "NO_SEMANTIC_LABEL", -999):
        label = -1
    bbox = None
    bb = getattr(attrs, "bounding_box", None)
    if bb is not None and getattr(bb, "is_valid", lambda: False)():
        lo, hi = list(bb.min), list(bb.max)
        if _finite(lo + hi):
            bbox = tuple(float(v) for v in lo + hi)
    color = getattr(attrs, "color", None)
    color = tuple(int(c) for c in color[:3]) if color is not None else (0, 0, 0)
    radii, states = _boundary(attrs)
    return NodeRecord(
        sym=str(node.id),
        layer=int(layer),
        part=int(part),
        pos=pos,
        label=int(label) if label is not None and int(label) < 32768 else -1,
        bbox=bbox,
        color=color,
        name=str(getattr(attrs, "name", "") or ""),
        active=bool(getattr(attrs, "is_active", False)),
        radii=radii,
        states=states,
    )


def graph_records(G) -> dict[str, NodeRecord]:
    """All nodes of a spark_dsg graph, keyed by symbol string.

    Walks the primary layers and the partitions; if that finds fewer nodes than
    ``G.num_nodes()`` (bindings whose layer views miss some layer keys), it also
    walks the graph-wide ``G.nodes`` iterator, so no node is silently dropped.
    """
    out = {}
    for layers in (G.layers, G.layer_partitions):
        for L in layers:
            for node in L.nodes:
                rec = node_record(node)
                out[rec.sym] = rec
    try:
        total = int(G.num_nodes())
    except Exception:
        total = None
    if total is not None and len(out) < total and hasattr(G, "nodes"):
        for node in G.nodes:
            rec = node_record(node)
            out.setdefault(rec.sym, rec)
    return out


def graph_edges(G) -> set[tuple[str, str]]:
    import spark_dsg as dsg

    def sym(v):
        return str(dsg.NodeSymbol(v)) if isinstance(v, int) else str(v)

    edges = set()
    for layers in (G.layers, G.layer_partitions):
        for L in layers:
            for e in L.edges:
                edges.add((sym(e.source), sym(e.target)))
    for e in G.interlayer_edges:
        edges.add((sym(e.source), sym(e.target)))
    return edges


def graph_labelspaces(G) -> dict[str, dict[str, str]]:
    """``{"layer/partition": {label_id: name}}`` from the graph metadata."""
    meta = {}
    try:
        meta = G.metadata.get() or {}
    except Exception:
        pass
    out = {}
    for key, pairs in (meta.get("labelspaces") or {}).items():
        if key.startswith("_l") and "p" in key:
            layer, part = key[2:].split("p", 1)
            out[f"{layer}/{part}"] = {str(i): n for i, n in pairs}
        else:
            out[key] = {str(i): n for i, n in pairs}
    return out


def encode_records(records: list[NodeRecord]) -> dict:
    n = len(records)
    bbox = np.full((n, 6), np.nan, dtype=np.float32)
    for i, r in enumerate(records):
        if r.bbox:
            bbox[i] = r.bbox
    enc = {
        "ids": [r.sym for r in records],
        "layer": np.array([r.layer for r in records], np.uint8).tobytes(),
        "part": np.array([r.part for r in records], np.uint8).tobytes(),
        "pos": np.array([r.pos for r in records], np.float32).reshape(-1).tobytes(),
        "label": np.array([r.label for r in records], np.int16).tobytes(),
        "bbox": bbox.reshape(-1).tobytes(),
        "color": np.array([r.color for r in records], np.uint8).reshape(-1).tobytes(),
        "names": [r.name for r in records],
        "active": np.array([r.active for r in records], np.uint8).tobytes(),
    }
    if any(r.radii for r in records):
        # optional keys: peers that predate them ignore them
        nrad, radii, states = [], [], []
        for r in records:
            cm = _radii_cm(r.radii) if r.radii else ()
            nrad.append(len(cm))
            radii.extend(cm)
            st = list(r.states or ())[: len(cm)]
            states.extend(st + [0] * (len(cm) - len(st)))
        enc["nrad"] = np.array(nrad, np.uint16).tobytes()
        enc["radii"] = np.array(radii, np.uint16).tobytes()
        enc["states"] = np.array(states, np.uint8).tobytes()
    return enc


def decode_records(enc: dict) -> list[NodeRecord]:
    n = len(enc["ids"])
    layer = np.frombuffer(enc["layer"], np.uint8)
    part = np.frombuffer(enc["part"], np.uint8)
    pos = np.frombuffer(enc["pos"], np.float32).reshape(n, 3)
    label = np.frombuffer(enc["label"], np.int16)
    bbox = np.frombuffer(enc["bbox"], np.float32).reshape(n, 6)
    color = np.frombuffer(enc["color"], np.uint8).reshape(n, 3)
    active = np.frombuffer(enc["active"], np.uint8)
    nrad = np.frombuffer(enc["nrad"], np.uint16) if enc.get("nrad") else None
    if nrad is not None:
        radii = np.frombuffer(enc["radii"], np.uint16)
        states = np.frombuffer(enc["states"], np.uint8)
        offs = np.concatenate([[0], np.cumsum(nrad, dtype=np.int64)])
    out = []
    for i in range(n):
        b = None if np.isnan(bbox[i, 0]) else tuple(float(v) for v in bbox[i])
        r = st = None
        if nrad is not None and nrad[i]:
            lo, hi = int(offs[i]), int(offs[i + 1])
            r = tuple(int(c) * RADIUS_QUANT for c in radii[lo:hi])
            st = tuple(int(v) for v in states[lo:hi])
        out.append(
            NodeRecord(
                sym=enc["ids"][i],
                layer=int(layer[i]),
                part=int(part[i]),
                pos=tuple(float(v) for v in pos[i]),
                label=int(label[i]),
                bbox=b,
                color=tuple(int(c) for c in color[i]),
                name=enc["names"][i],
                active=bool(active[i]),
                radii=r,
                states=st,
            )
        )
    return out


def encode_graph(records: dict[str, NodeRecord], edges, labelspaces=None) -> dict:
    enc = encode_records(list(records.values()))
    enc["edges"] = sorted([list(e) for e in edges])
    enc["labelspaces"] = labelspaces or {}
    return enc


# ----------------------------------------------------------------------------
# Graph state + delta computation
# ----------------------------------------------------------------------------


@dataclass
class GraphDelta:
    upserts: list[NodeRecord] = field(default_factory=list)
    deletes: list[str] = field(default_factory=list)
    edges_add: list[tuple[str, str]] = field(default_factory=list)
    edges_del: list[tuple[str, str]] = field(default_factory=list)
    labelspaces: dict | None = None

    def empty(self) -> bool:
        return (
            not (self.upserts or self.deletes or self.edges_add or self.edges_del)
            and not self.labelspaces
        )

    def encode(self) -> dict:
        return {
            "up": encode_records(self.upserts),
            "del": list(self.deletes),
            "ea": [list(e) for e in self.edges_add],
            "ed": [list(e) for e in self.edges_del],
            "ls": self.labelspaces,
        }

    @staticmethod
    def decode(enc: dict) -> GraphDelta:
        return GraphDelta(
            upserts=decode_records(enc["up"]),
            deletes=list(enc["del"]),
            edges_add=[tuple(e) for e in enc["ea"]],
            edges_del=[tuple(e) for e in enc["ed"]],
            labelspaces=enc.get("ls"),
        )


class GraphState:
    """A mirror of a scene graph as records + edges.

    The robot keeps one as "what the base station has acknowledged"; the base
    station keeps one per robot as the live mirror.
    """

    def __init__(self):
        self.records: dict[str, NodeRecord] = {}
        self.edges: set[tuple[str, str]] = set()
        self.labelspaces: dict = {}

    def diff(self, records, edges, labelspaces=None) -> GraphDelta:
        """Delta that would turn this state into (records, edges)."""
        ups = [
            r
            for s, r in records.items()
            if s not in self.records or self.records[s].key() != r.key()
        ]
        dels = [s for s in self.records if s not in records]
        ea = sorted(edges - self.edges)
        ed = sorted(self.edges - edges)
        ls = labelspaces if labelspaces and labelspaces != self.labelspaces else None
        return GraphDelta(ups, dels, ea, ed, ls)

    def apply(self, delta: GraphDelta) -> None:
        for r in delta.upserts:
            self.records[r.sym] = r
        for s in delta.deletes:
            self.records.pop(s, None)
        self.edges.difference_update(delta.edges_del)
        self.edges.update(delta.edges_add)
        # edges whose endpoints were deleted go too
        if delta.deletes:
            gone = set(delta.deletes)
            self.edges = {
                e for e in self.edges if e[0] not in gone and e[1] not in gone
            }
        if delta.labelspaces:
            self.labelspaces = delta.labelspaces

    def snapshot(self) -> dict:
        return encode_graph(self.records, self.edges, self.labelspaces)

    def load_snapshot(self, enc: dict) -> None:
        self.records = {r.sym: r for r in decode_records(enc)}
        self.edges = {tuple(e) for e in enc["edges"]}
        self.labelspaces = enc.get("labelspaces") or {}


# ----------------------------------------------------------------------------
# Mesh tiles
# ----------------------------------------------------------------------------


def tile_key(ijk) -> str:
    return f"{int(ijk[0])}_{int(ijk[1])}_{int(ijk[2])}"


def mesh_arrays(mesh):
    """spark_dsg Mesh → (positions float32[N,3], colors uint8[N,3], faces uint32[M,3])."""
    v = np.asarray(mesh.get_vertices())  # (6, N): xyz rgb in [0, 1]
    f = np.asarray(mesh.get_faces())  # (3, M)
    pos = v[:3].T.astype(np.float32)
    col = (np.clip(v[3:6].T, 0, 1) * 255).astype(np.uint8)
    return pos, col, f.T.astype(np.uint32)


# mesh level of detail (tiles a robot sends): "full" = Hydra's mesh as is,
# "decimated" = vertex clustering on a grid of ``cell`` metres (``decimate_mesh``)
MESH_LODS = ("full", "decimated")
DEFAULT_DECIMATE_CELL = 0.4  # divides DEFAULT_TILE_SIZE: clusters never straddle a tile edge; Penn: 8.7x fewer faces


def decimate_mesh(pos, col, faces, cell: float = DEFAULT_DECIMATE_CELL):
    """Vertex clustering: every vertex in a ``cell``-metre grid cell becomes the
    cell's mean vertex (mean colour); faces that collapse or repeat are dropped.

    Numpy only (robots have no mesh library in spark_env), linear in the mesh
    size, and the grid is global and aligned with the tile grid when ``cell``
    divides the tile size, so neighbouring tiles share their border vertices
    exactly (no cracks). Returns (pos float32, col uint8, faces uint32).
    """
    pos = np.asarray(pos, np.float32)
    faces = np.asarray(faces, np.uint32)
    if len(faces) == 0 or len(pos) == 0 or not cell or cell <= 0:
        return pos, np.asarray(col, np.uint8), faces
    q = np.floor(pos / cell).astype(np.int64)
    q -= q.min(axis=0)
    dims = q.max(axis=0) + 1
    lin = (q[:, 0] * dims[1] + q[:, 1]) * dims[2] + q[:, 2]
    keys, inv = np.unique(lin, return_inverse=True)
    n = len(keys)
    cnt = np.bincount(inv, minlength=n).astype(np.float64)
    new_pos = np.empty((n, 3), np.float64)
    new_col = np.empty((n, 3), np.float64)
    colf = np.asarray(col, np.float64)
    for k in range(3):
        new_pos[:, k] = np.bincount(inv, weights=pos[:, k], minlength=n)
        new_col[:, k] = np.bincount(inv, weights=colf[:, k], minlength=n)
    new_pos /= cnt[:, None]
    new_col /= cnt[:, None]
    f = inv[faces.astype(np.int64)]
    f = f[(f[:, 0] != f[:, 1]) & (f[:, 1] != f[:, 2]) & (f[:, 0] != f[:, 2])]
    if len(f) == 0:
        return (
            np.zeros((0, 3), np.float32),
            np.zeros((0, 3), np.uint8),
            np.zeros((0, 3), np.uint32),
        )
    # the same triangle from several original faces (either winding): keep one
    srt = np.sort(f, axis=1)
    _, first = np.unique((srt[:, 0] * n + srt[:, 1]) * n + srt[:, 2], return_index=True)
    f = f[np.sort(first)]
    used, remap = np.unique(f.reshape(-1), return_inverse=True)
    return (
        new_pos[used].astype(np.float32),
        np.clip(np.round(new_col[used]), 0, 255).astype(np.uint8),
        remap.reshape(-1, 3).astype(np.uint32),
    )


def tile_mesh(pos, col, faces, tile_size: float = DEFAULT_TILE_SIZE) -> dict[str, dict]:
    """Split a mesh into spatial tiles; returns ``{tile_key: tile}``.

    Each tile: ``{"key", "origin", "scale", "pos" int16 bytes, "col" uint8 bytes,
    "idx" uint32 bytes, "nv", "nf", "hash"}``.
    """
    if len(faces) == 0:
        return {}
    centroids = pos[faces].mean(axis=1)
    ijk = np.floor(centroids / tile_size).astype(np.int64)
    order = np.lexsort((ijk[:, 2], ijk[:, 1], ijk[:, 0]))
    ijk_sorted = ijk[order]
    breaks = np.flatnonzero(np.any(np.diff(ijk_sorted, axis=0) != 0, axis=1)) + 1
    tiles = {}
    for group in np.split(order, breaks):
        key_ijk = ijk[group[0]]
        tf = faces[group]
        used, inverse = np.unique(tf.reshape(-1), return_inverse=True)
        origin = key_ijk.astype(np.float64) * tile_size
        q = np.round((pos[used] - origin) / QUANT)
        q = np.clip(q, -32768, 32767).astype(np.int16)
        c = col[used]
        idx = inverse.astype(np.uint32)
        h = hashlib.blake2b(digest_size=8)
        for arr in (q, c, idx):
            h.update(arr.tobytes())
        tiles[tile_key(key_ijk)] = {
            "key": tile_key(key_ijk),
            "origin": [float(v) for v in origin],
            "scale": QUANT,
            "pos": q.tobytes(),
            "col": c.tobytes(),
            "idx": idx.tobytes(),
            "nv": int(len(used)),
            "nf": int(len(tf)),
            "hash": h.hexdigest(),
        }
    return tiles


def decode_tile(tile: dict):
    """Tile → (positions float32[N,3], colors uint8[N,3], faces uint32[M,3])."""
    q = np.frombuffer(tile["pos"], np.int16).reshape(-1, 3).astype(np.float32)
    pos = q * tile["scale"] + np.asarray(tile["origin"], np.float32)
    col = np.frombuffer(tile["col"], np.uint8).reshape(-1, 3)
    idx = np.frombuffer(tile["idx"], np.uint32).reshape(-1, 3)
    return pos, col, idx


def decimate_tiles(
    tiles: dict[str, dict], cell: float = DEFAULT_DECIMATE_CELL
) -> dict[str, dict]:
    """``decimate_mesh`` applied to already tiled meshes, keeping every tile's key.

    Vertex clustering runs over *all* tiles at once, in the tiles' integer
    (``scale``) coordinates, so a vertex shared by two tiles lands in the same
    cluster in both and every tile gets the very same cluster vertex: no cracks
    between tiles. Each face stays in the tile it came from (unlike decimating
    the whole mesh and re-tiling), so a coarse tile replaces exactly its full
    tile on the client. Tiles left without faces are omitted.
    Output tiles have the ``tile_mesh`` layout (``hash`` of the decimated data).
    """
    items = [(k, t) for k, t in tiles.items() if t.get("nf")]
    if not items or not cell or cell <= 0:
        return {k: t for k, t in items}
    scale = float(items[0][1]["scale"])
    cq = max(1, int(round(cell / scale)))  # cell in quantisation steps
    qs, cols, offs = [], [], [0]
    for _, t in items:
        oq = np.round(np.asarray(t["origin"], np.float64) / scale).astype(np.int64)
        q = np.frombuffer(t["pos"], np.int16).reshape(-1, 3).astype(np.int64) + oq
        qs.append(q)
        cols.append(np.frombuffer(t["col"], np.uint8).reshape(-1, 3))
        offs.append(offs[-1] + len(q))
    q = np.concatenate(qs)
    col = np.concatenate(cols).astype(np.float64)
    c = np.floor_divide(q, cq)
    c -= c.min(axis=0)
    dims = c.max(axis=0) + 1
    lin = (c[:, 0] * dims[1] + c[:, 1]) * dims[2] + c[:, 2]
    keys, inv = np.unique(lin, return_inverse=True)
    n = len(keys)
    cnt = np.bincount(inv, minlength=n).astype(np.float64)
    mq = np.empty((n, 3), np.int64)  # cluster mean, rounded to the quantisation grid
    mc = np.empty((n, 3), np.uint8)
    for k in range(3):
        mq[:, k] = np.round(np.bincount(inv, weights=q[:, k], minlength=n) / cnt)
        mc[:, k] = np.clip(
            np.round(np.bincount(inv, weights=col[:, k], minlength=n) / cnt), 0, 255
        )
    out = {}
    for i, (key, t) in enumerate(items):
        idx = np.frombuffer(t["idx"], np.uint32).reshape(-1, 3).astype(np.int64)
        f = inv[offs[i] + idx]
        f = f[(f[:, 0] != f[:, 1]) & (f[:, 1] != f[:, 2]) & (f[:, 0] != f[:, 2])]
        if len(f) == 0:
            continue
        srt = np.sort(f, axis=1)
        _, first = np.unique(
            (srt[:, 0] * n + srt[:, 1]) * n + srt[:, 2], return_index=True
        )
        f = f[np.sort(first)]
        used, remap = np.unique(f.reshape(-1), return_inverse=True)
        oq = np.round(np.asarray(t["origin"], np.float64) / scale).astype(np.int64)
        pq = np.clip(mq[used] - oq, -32768, 32767).astype(np.int16)
        pc = mc[used]
        pi = remap.reshape(-1, 3).astype(np.uint32)
        h = hashlib.blake2b(digest_size=8)
        for arr in (pq, pc, pi):
            h.update(arr.tobytes())
        out[key] = {
            "key": key,
            "origin": list(t["origin"]),
            "scale": scale,
            "pos": pq.tobytes(),
            "col": pc.tobytes(),
            "idx": pi.tobytes(),
            "nv": int(len(used)),
            "nf": int(len(pi)),
            "hash": h.hexdigest(),
        }
    return out


# Packed tiles (encoding 2): the tile_mesh content laid out to compress well
# under gzip (~37% smaller than a gzipped plain tile on Hydra meshes). Vertices
# are reordered by first use in the face list, then
#   pos: per axis x, y, z: zigzag(int16 wrapping delta of the quantised
#        coordinate) as uint16, stored as a low-byte plane then a high-byte plane
#   col: per channel r, g, b: uint8 wrapping delta
#   idx: the flat index list (3 per face) as zigzag(wrapping delta) in ``ib``
#        bytes (2 when nv <= 65536, else 4), one byte plane per byte, low first
# frontend/src/codec.ts decodePackedTile is the browser decoder.
PACKED_TILE_ENCODING = 2


def _zigzag_delta(a: np.ndarray, bits: int) -> np.ndarray:
    ut = np.uint16 if bits == 16 else np.uint32
    st = np.int16 if bits == 16 else np.int32
    d = np.diff(a.astype(ut), prepend=ut(0)).view(st).astype(np.int64)
    return ((d << 1) ^ (d >> (bits - 1))).astype(ut)


def _unzigzag_cumsum(z: np.ndarray, bits: int) -> np.ndarray:
    z = z.astype(np.int64)
    d = (z >> 1) ^ -(z & 1)
    return np.cumsum(d) & ((1 << bits) - 1)


def _planes(a: np.ndarray) -> bytes:
    return np.ascontiguousarray(a).view(np.uint8).reshape(-1, a.itemsize).T.tobytes()


def _unplanes(b: bytes, offset: int, n: int, width: int) -> np.ndarray:
    p = np.frombuffer(b, np.uint8, n * width, offset).reshape(width, n).astype(np.int64)
    v = np.zeros(n, np.int64)
    for i in range(width):
        v |= p[i] << (8 * i)
    return v


def pack_tile(tile: dict) -> dict:
    """``tile_mesh`` tile → packed tile (encoding 2; same mesh, vertices reordered)."""
    q = np.frombuffer(tile["pos"], np.int16).reshape(-1, 3)
    c = np.frombuffer(tile["col"], np.uint8).reshape(-1, 3)
    flat = np.frombuffer(tile["idx"], np.uint32).astype(np.int64)
    _, first = np.unique(flat, return_index=True)
    order = flat[np.sort(first)]  # vertices by first use (unused ones dropped)
    remap = np.zeros(max(len(q), 1), np.int64)
    remap[order] = np.arange(len(order))
    q, c, flat = q[order], c[order], remap[flat]
    ib = 2 if len(order) <= 65536 else 4
    return {
        "key": tile["key"],
        "origin": list(tile["origin"]),
        "scale": tile["scale"],
        "nv": int(len(order)),
        "nf": int(len(flat) // 3),
        "hash": tile["hash"],
        "enc": PACKED_TILE_ENCODING,
        "ib": ib,
        "pos": b"".join(_planes(_zigzag_delta(q[:, k], 16)) for k in range(3)),
        "col": b"".join(
            np.diff(c[:, k], prepend=np.uint8(0)).tobytes() for k in range(3)
        ),
        "idx": _planes(_zigzag_delta(flat, 8 * ib)),
    }


def unpack_tile(packed: dict) -> dict:
    """Packed tile → ``tile_mesh`` tile (with the packed vertex order)."""
    nv, nf, ib = int(packed["nv"]), int(packed["nf"]), int(packed["ib"])
    pb, cb = packed["pos"], packed["col"]
    q = np.stack(
        [_unzigzag_cumsum(_unplanes(pb, 2 * nv * k, nv, 2), 16) for k in range(3)], 1
    )
    c = np.stack(
        [
            np.cumsum(np.frombuffer(cb, np.uint8, nv, nv * k).astype(np.int64)) & 255
            for k in range(3)
        ],
        1,
    )
    idx = _unzigzag_cumsum(_unplanes(packed["idx"], 0, 3 * nf, ib), 8 * ib)
    return {
        "key": packed["key"],
        "origin": list(packed["origin"]),
        "scale": packed["scale"],
        "pos": q.astype(np.uint16).view(np.int16).tobytes(),
        "col": c.astype(np.uint8).tobytes(),
        "idx": idx.astype(np.uint32).tobytes(),
        "nv": nv,
        "nf": nf,
        "hash": packed["hash"],
    }


def tile_manifest(tiles: dict[str, dict]) -> dict[str, str]:
    return {k: t["hash"] for k, t in tiles.items()}


def tile_centre(tile: dict, tile_size: float = DEFAULT_TILE_SIZE):
    return np.asarray(tile["origin"]) + tile_size / 2.0
