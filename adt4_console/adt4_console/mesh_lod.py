"""Mesh tiles for the browser: levels of detail, packed and pre-gzipped (API.md §7).

A graph source's mesh is split into ``tile_mesh`` tiles (fleetlink codec). For
the browser every tile is also served at coarser levels of detail (vertex
clustering over all tiles at once, ``codec.decimate_tiles``: crack-free, same
tile keys) and in the packed encoding (``codec.pack_tile``), gzipped once here
so the manifest can tell the client the exact bytes each tile costs on the
wire (its progress bar / ETA) and so repeated requests cost no CPU.

``TileProducts`` holds those blobs for one tiled mesh; ``MeshDiskCache`` keeps
tiles + blobs on disk keyed by the mesh file (path, size, mtime), so a console
restart does not re-parse a 400 MB ASCII PLY (Penn: 25 s → under a second).
"""

from __future__ import annotations

import gzip
import hashlib
import logging
import os
import pathlib
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass

import msgpack

log = logging.getLogger(__name__)

# level → vertex-clustering cell (metres; 0 = Hydra's mesh as is). The cells
# divide the 8 m tile size. Penn (4.7 M faces): 23.7 / 5.8 / 1.7 MB gzipped.
LODS: tuple[dict, ...] = (
    {"level": 0, "cell": 0.0, "name": "full"},
    {"level": 1, "cell": 0.2, "name": "medium"},
    {"level": 2, "cell": 0.4, "name": "coarse"},
)
GZIP_LEVEL = 6  # level 9 is ~2x slower for <1% smaller tiles
DISK_FORMAT = 1  # bump when the cached layout or the packed encoding changes
DISK_KEEP = 6  # cached meshes kept on disk (newest first)


def _codec():
    from . import codec

    return codec


@dataclass(frozen=True)
class Blob:
    """One packed, gzipped tile at one level of detail."""

    gz: bytes
    raw_len: int  # bytes after gunzip (the msgpack packed tile)
    etag: str  # changes whenever the content does (hash of the level's tile)
    nf: int


def gzip_bytes(data: bytes) -> bytes:
    return gzip.compress(data, GZIP_LEVEL, mtime=0)


def etag_of(data: bytes) -> str:
    return hashlib.blake2b(data, digest_size=8).hexdigest()


def _signature(tiles: dict) -> str:
    h = hashlib.blake2b(digest_size=8)
    for k in sorted(tiles):
        h.update(f"{k}:{tiles[k]['hash']};".encode())
    return h.hexdigest()


def _tile_etag(tile: dict) -> str:
    # the tile's content hash + the encoding, so a new encoding never hits an
    # old immutable browser cache entry
    return f"{tile['hash'][:12]}e{_codec().PACKED_TILE_ENCODING}"


def _pack_blob(tile: dict) -> Blob:
    raw = msgpack.packb(_codec().pack_tile(tile), use_bin_type=True)
    return Blob(gzip_bytes(raw), len(raw), _tile_etag(tile), int(tile["nf"]))


class TileProducts:
    """Every (tile key, level) blob of one tiled mesh, plus the manifest rows."""

    def __init__(
        self,
        signature: str,
        levels: tuple[int, ...],
        blobs: dict[tuple[str, int], Blob],
    ):
        self.signature = signature
        self.levels = levels
        self.blobs = blobs
        self._bundles: dict[int, Blob] = {}
        self._bundle_lock = threading.Lock()

    def blob(self, key: str, level: int) -> Blob | None:
        return self.blobs.get((key, level))

    def bundle(self, level: int) -> Blob | None:
        """Every tile of ``level`` in one body (a msgpack array of packed tiles),
        for the coarse preview: hundreds of small tiles cost a round trip each
        on a high-latency link, one bundle costs one. Built on first use."""
        with self._bundle_lock:
            b = self._bundles.get(level)
            if b is not None or level not in self.levels:
                return b
            parts = [
                gzip.decompress(bl.gz)
                for (_, lv), bl in sorted(self.blobs.items())
                if lv == level
            ]
            if not parts:
                return None
            n = len(parts)
            head = (
                bytes([0xDC]) + n.to_bytes(2, "big")
                if n < 65536
                else bytes([0xDD]) + n.to_bytes(4, "big")
            )
            raw = head + b"".join(parts)  # each part is already a msgpack map
            b = Blob(gzip_bytes(raw), len(raw), f"{self.signature[:12]}b{level}e{_codec().PACKED_TILE_ENCODING}",
                     sum(bl.nf for (_, lv), bl in self.blobs.items() if lv == level))  # fmt: skip
            self._bundles[level] = b
            return b

    def manifest_rows(self, tiles: dict) -> dict:
        """``{key: {"hash", "nv", "nf", "origin", "lod": [[bytes, etag] | None, ...]}}``"""
        out = {}
        for k, t in tiles.items():
            lod = []
            for lv in self.levels:
                b = self.blobs.get((k, lv))
                lod.append([len(b.gz), b.etag] if b is not None else None)
            out[k] = {
                "hash": t["hash"],
                "nv": t["nv"],
                "nf": t["nf"],
                "origin": t["origin"],
                "lod": lod,
            }
        return out

    def total_bytes(self, level: int) -> int:
        return sum(len(b.gz) for (_, lv), b in self.blobs.items() if lv == level)

    @staticmethod
    def build(tiles: dict, levels: tuple[int, ...] = (0, 1, 2), prev: "TileProducts | None" = None,
              workers: int = 8) -> "TileProducts":  # fmt: skip
        """Decimate + pack + gzip every tile at every level. Blobs of ``prev``
        whose level tile did not change are reused (live meshes grow tile by tile)."""
        sig = _signature(tiles)
        if prev is not None and prev.signature == sig and prev.levels == levels:
            return prev
        t0 = time.time()
        codec = _codec()
        jobs = []
        for lv in levels:
            cell = LODS[lv]["cell"]
            lt = (
                codec.decimate_tiles(tiles, cell)
                if cell
                else {k: t for k, t in tiles.items() if t.get("nf")}
            )
            for k, t in lt.items():
                old = prev.blobs.get((k, lv)) if prev is not None else None
                jobs.append(
                    (
                        k,
                        lv,
                        t,
                        old if old is not None and old.etag == _tile_etag(t) else None,
                    )
                )
        todo = [(k, lv, t) for k, lv, t, old in jobs if old is None]
        blobs = {(k, lv): old for k, lv, _, old in jobs if old is not None}
        if todo:
            with ThreadPoolExecutor(max(1, min(workers, len(todo)))) as ex:
                for (k, lv, _), b in zip(
                    todo, ex.map(lambda j: _pack_blob(j[2]), todo)
                ):
                    blobs[(k, lv)] = b
        log.info("mesh LODs: %d tiles × %d levels (%d packed) in %.2fs", len(tiles), len(levels),
                 len(todo), time.time() - t0)  # fmt: skip
        return TileProducts(sig, levels, blobs)


class MeshDiskCache:
    """Tiled mesh + its blobs on disk, keyed by the mesh file it came from."""

    def __init__(self, root):
        self.root = pathlib.Path(os.path.expanduser(str(root)))

    @staticmethod
    def file_key(path) -> str | None:
        try:
            st = pathlib.Path(path).stat()
        except OSError:
            return None
        ident = f"{pathlib.Path(path).resolve()}|{st.st_size}|{st.st_mtime_ns}|v{DISK_FORMAT}"
        return hashlib.sha1(ident.encode()).hexdigest()[:20]

    def _path(self, key: str) -> pathlib.Path:
        return self.root / f"{key}.msgpack"

    def load(self, key: str) -> tuple[dict, TileProducts] | None:
        p = self._path(key)
        try:
            with open(p, "rb") as f:
                d = msgpack.unpackb(f.read(), raw=False, strict_map_key=False)
            levels = tuple(d["levels"])
            blobs = {
                (k, int(lv)): Blob(b[0], int(b[1]), str(b[2]), int(b[3]))
                for k, rows in d["blobs"].items()
                for lv, b in rows.items()
            }
            tiles = d["tiles"]
            os.utime(p)  # most recently used
            return tiles, TileProducts(_signature(tiles), levels, blobs)
        except FileNotFoundError:
            return None
        except Exception as e:  # noqa: BLE001  (corrupt / old cache: rebuild)
            log.warning("mesh cache %s unreadable (%s); rebuilding", p, e)
            return None

    def save(self, key: str, tiles: dict, products: TileProducts) -> None:
        rows: dict[str, dict] = {}
        for (k, lv), b in products.blobs.items():
            rows.setdefault(k, {})[lv] = [b.gz, b.raw_len, b.etag, b.nf]
        data = msgpack.packb({"levels": list(products.levels), "tiles": tiles, "blobs": rows},
                             use_bin_type=True)  # fmt: skip
        self.root.mkdir(parents=True, exist_ok=True)
        tmp = self._path(key).with_suffix(f".tmp{os.getpid()}.{threading.get_ident()}")
        tmp.write_bytes(data)
        os.replace(tmp, self._path(key))
        self._prune()

    def save_async(self, key: str, tiles: dict, products: TileProducts) -> None:
        def run():
            try:
                self.save(key, tiles, products)
            except Exception as e:  # noqa: BLE001
                log.warning("mesh cache write failed: %s", e)

        threading.Thread(target=run, name="mesh-cache-write", daemon=True).start()

    def _prune(self) -> None:
        files = sorted(
            self.root.glob("*.msgpack"), key=lambda p: p.stat().st_mtime, reverse=True
        )
        for p in files[DISK_KEEP:]:
            try:
                p.unlink()
            except OSError:
                pass
