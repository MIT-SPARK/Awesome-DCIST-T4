"""Fast PLY → (positions, colours, faces) for triangle meshes.

trimesh takes ~9 s on Hydra's 1M-vertex *ASCII* ``mesh.ply``; parsing the
vertex/face blocks with numpy is ~10× faster.  Anything unusual (non-triangle
faces, odd property types) falls back to trimesh.
"""

from __future__ import annotations

import numpy as np

_NP = {
    "char": "i1", "int8": "i1", "uchar": "u1", "uint8": "u1",
    "short": "i2", "int16": "i2", "ushort": "u2", "uint16": "u2",
    "int": "i4", "int32": "i4", "uint": "u4", "uint32": "u4",
    "float": "f4", "float32": "f4", "double": "f8", "float64": "f8",
}  # fmt: skip


def _parse_header(f):
    fmt = None
    elements = []  # [name, count, [(prop, type) | ("list", count_t, item_t, name)]]
    while True:
        line = f.readline()
        if not line:
            raise ValueError("truncated PLY header")
        parts = line.decode("ascii", "replace").split()
        if not parts:
            continue
        if parts[0] == "format":
            fmt = parts[1]
        elif parts[0] == "element":
            elements.append([parts[1], int(parts[2]), []])
        elif parts[0] == "property":
            if parts[1] == "list":
                elements[-1][2].append(("list", parts[2], parts[3], parts[4]))
            else:
                elements[-1][2].append((parts[2], parts[1]))
        elif parts[0] == "end_header":
            return fmt, elements


def _colors(names, data):
    if all(c in names for c in ("red", "green", "blue")):
        return np.stack([data[names.index(c)] for c in ("red", "green", "blue")], 1)
    return None


def load_ply(path):
    """Return (pos float32[N,3], col uint8[N,3], faces uint32[M,3])."""
    try:
        return _load_fast(path)
    except Exception:  # noqa: BLE001
        return _load_trimesh(path)


def _load_fast(path):
    with open(path, "rb") as f:
        fmt, elements = _parse_header(f)
        names = [e[0] for e in elements]
        if names[:2] != ["vertex", "face"]:
            raise ValueError("unsupported element order")
        (_, nv, vprops), (_, nf, fprops) = elements[0], elements[1]
        if any(p[0] == "list" for p in vprops) or len(fprops) != 1:
            raise ValueError("unsupported properties")
        vnames = [p[0] for p in vprops]
        _, count_t, item_t, _ = fprops[0]
        if fmt == "ascii":
            vert = np.fromstring(  # noqa: NPY201 - fastest ascii float parser
                b"".join(f.readline() for _ in range(nv)), sep=" "
            ).reshape(nv, len(vprops))
            cols = [vert[:, i] for i in range(len(vprops))]
            fdata = np.fromstring(f.read(), sep=" ")  # noqa: NPY201
            if fdata.size != nf * 4:
                raise ValueError("non-triangle faces")
            fdata = fdata.reshape(nf, 4)
            if not np.all(fdata[:, 0] == 3):
                raise ValueError("non-triangle faces")
            faces = fdata[:, 1:].astype(np.uint32)
        elif fmt in ("binary_little_endian", "binary_big_endian"):
            end = "<" if fmt == "binary_little_endian" else ">"
            vdt = np.dtype([(n, end + _NP[t]) for n, t in vprops])
            vert = np.frombuffer(f.read(vdt.itemsize * nv), vdt, nv)
            cols = [vert[n] for n in vnames]
            fdt = np.dtype([("n", end + _NP[count_t]), ("i", end + _NP[item_t], 3)])
            fdata = np.frombuffer(f.read(fdt.itemsize * nf), fdt, nf)
            if not np.all(fdata["n"] == 3):
                raise ValueError("non-triangle faces")
            faces = fdata["i"].astype(np.uint32)
        else:
            raise ValueError(f"unknown PLY format {fmt}")
    pos = np.stack([cols[vnames.index(c)] for c in "xyz"], 1).astype(np.float32)
    col = _colors(vnames, cols)
    col = (
        np.clip(col, 0, 255).astype(np.uint8)
        if col is not None
        else np.full((nv, 3), 180, np.uint8)
    )
    return pos, col, faces


def _load_trimesh(path):
    import trimesh

    m = trimesh.load(str(path), process=False, force="mesh")
    pos = np.asarray(m.vertices, np.float32)
    try:
        col = np.asarray(m.visual.vertex_colors)[:, :3].astype(np.uint8)
    except Exception:  # noqa: BLE001
        col = np.full((len(pos), 3), 180, np.uint8)
    return pos, col, np.asarray(m.faces, np.uint32)
