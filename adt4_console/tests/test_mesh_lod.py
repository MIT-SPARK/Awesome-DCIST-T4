"""mesh_lod: levels of detail, blob reuse, disk cache (API.md §7)."""

import gzip

import msgpack
import numpy as np

from adt4_console import codec, mesh_lod


def _mesh(n=40, step=0.1):
    # a flat, gently waved grid at 10 cm spacing over two 8 m tiles
    xs = np.arange(n) * step + 6.0
    X, Y = np.meshgrid(xs, xs)
    Z = 0.05 * np.sin(X)
    pos = np.stack([X.ravel(), Y.ravel(), Z.ravel()], 1).astype(np.float32)
    col = np.full((len(pos), 3), 120, np.uint8)
    faces = []
    for i in range(n - 1):
        for j in range(n - 1):
            a = i * n + j
            faces += [[a, a + 1, a + n], [a + 1, a + n + 1, a + n]]
    return pos, col, np.asarray(faces, np.uint32)


def test_products_levels_sizes_and_reuse():
    tiles = codec.tile_mesh(*_mesh())
    p = mesh_lod.TileProducts.build(tiles, (0, 1, 2), workers=2)
    assert set(k for k, _ in p.blobs) == set(tiles)
    for (k, lv), b in p.blobs.items():
        packed = msgpack.unpackb(gzip.decompress(b.gz), raw=False)
        assert len(gzip.decompress(b.gz)) == b.raw_len and packed["nf"] == b.nf
        assert b.etag.endswith(f"e{codec.PACKED_TILE_ENCODING}")
    nf = [
        sum(b.nf for (_, lv), b in p.blobs.items() if lv == level)
        for level in (0, 1, 2)
    ]
    assert nf[0] > nf[1] > nf[2] > 0
    assert p.total_bytes(0) > p.total_bytes(1) > p.total_bytes(2)
    rows = p.manifest_rows(tiles)
    assert all(
        len(r["lod"]) == 3 and r["lod"][0][0] == len(p.blob(k, 0).gz)
        for k, r in rows.items()
    )
    # unchanged tiles: the same products; one tile changed: other blobs reused
    assert mesh_lod.TileProducts.build(tiles, (0, 1, 2), p) is p
    k0 = sorted(tiles)[0]
    t2 = dict(tiles)
    t2[k0] = dict(tiles[k0], hash="0" * 16)
    p2 = mesh_lod.TileProducts.build(t2, (0,), None)
    p3 = mesh_lod.TileProducts.build(t2, (0,), p2)
    assert p3 is p2
    p4 = mesh_lod.TileProducts.build(tiles, (0,), p2)
    assert all(p4.blob(k, 0) is p2.blob(k, 0) for k in tiles if k != k0)
    assert p4.blob(k0, 0) is not p2.blob(k0, 0)


def test_disk_cache_roundtrip_and_key(tmp_path):
    ply = tmp_path / "mesh.ply"
    ply.write_bytes(b"x")
    key = mesh_lod.MeshDiskCache.file_key(ply)
    assert key and mesh_lod.MeshDiskCache.file_key(tmp_path / "missing") is None
    cache = mesh_lod.MeshDiskCache(tmp_path / "c")
    assert cache.load(key) is None
    tiles = codec.tile_mesh(*_mesh(20))
    p = mesh_lod.TileProducts.build(tiles, (0, 2), workers=1)
    cache.save(key, tiles, p)
    tiles2, p2 = cache.load(key)
    assert tiles2.keys() == tiles.keys() and p2.levels == (0, 2)
    assert p2.signature == p.signature and p2.blobs == p.blobs
    # the key follows the file: a rewrite is a new key
    ply.write_bytes(b"xy")
    assert mesh_lod.MeshDiskCache.file_key(ply) != key
    # a corrupt file is a miss, not an error
    (tmp_path / "c" / f"{key}.msgpack").write_bytes(b"\x00junk")
    assert cache.load(key) is None


def test_disk_cache_prunes_old_entries(tmp_path, monkeypatch):
    monkeypatch.setattr(mesh_lod, "DISK_KEEP", 2)
    cache = mesh_lod.MeshDiskCache(tmp_path)
    tiles = codec.tile_mesh(*_mesh(10))
    p = mesh_lod.TileProducts.build(tiles, (0,), workers=1)
    for i in range(4):
        cache.save(f"k{i}", tiles, p)
    assert len(list(tmp_path.glob("*.msgpack"))) == 2


def test_bundle_is_every_tile_of_a_level():
    tiles = codec.tile_mesh(*_mesh())
    p = mesh_lod.TileProducts.build(tiles, (0, 2), workers=2)
    b = p.bundle(2)
    arr = msgpack.unpackb(gzip.decompress(b.gz), raw=False)
    assert len(gzip.decompress(b.gz)) == b.raw_len
    assert sorted(t["key"] for t in arr) == sorted(k for (k, lv) in p.blobs if lv == 2)
    for t in arr:
        assert t == msgpack.unpackb(gzip.decompress(p.blob(t["key"], 2).gz), raw=False)
    assert p.bundle(2) is b and p.bundle(1) is None
    assert b.etag.startswith(p.signature[:12])
