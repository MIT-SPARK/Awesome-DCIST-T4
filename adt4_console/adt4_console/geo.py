"""Geo-referencing of graph sources and the satellite tile cache (API.md §6).

A :class:`GeoAnchor` is the 2-D rigid transform map frame → UTM fitted from GPS
(``adt4_console/scripts/geo/anchor_from_gps.py`` writes it as ``geo_anchor.json`` next to a
run's ``hydra/``).  ``utm = R(yaw) @ [x, y] + t``.

Tiles are standard XYZ (slippy-map) images under ``<tiles_dir>/<layer>/{z}/{x}/{y}.jpg``
(``adt4_console/scripts/geo/fetch_tiles.py``).  A miss is fetched from the layer's URL template
only when live fetching is on, and is stored so the field never needs the internet twice.
"""

from __future__ import annotations

import json
import logging
import math
import os
import pathlib
import threading
import urllib.request

log = logging.getLogger(__name__)

ANCHOR_FILE = "geo_anchor.json"
DEFAULT_LAYER = "esri_world_imagery"
LAYER_URLS = {
    "esri_world_imagery": "https://server.arcgisonline.com/ArcGIS/rest/services/"
    "World_Imagery/MapServer/tile/{z}/{y}/{x}",
}
LAYER_ATTRIBUTION = {"esri_world_imagery": "Esri, Maxar, Earthstar Geographics"}
USER_AGENT = "adt4-console/1 (offline tile cache)"


class GeoUnavailable(Exception):
    pass


def _transformer(epsg: int, inverse: bool):
    try:
        from pyproj import Transformer
    except ImportError as e:
        raise GeoUnavailable(f"pyproj unavailable: {e}") from None
    a, b = f"EPSG:{epsg}", "EPSG:4326"
    if inverse:
        a, b = b, a
    return Transformer.from_crs(a, b, always_xy=True)


class GeoAnchor:
    def __init__(self, epsg: int, yaw: float, t, z0: float = 0.0, **extra):
        self.epsg = int(epsg)
        self.yaw = float(yaw)
        self.t = (float(t[0]), float(t[1]))
        self.z0 = float(z0)
        self.extra = extra
        self._c, self._s = math.cos(self.yaw), math.sin(self.yaw)
        self._to_ll = None
        self._from_ll = None

    @classmethod
    def load(cls, path) -> "GeoAnchor":
        d = json.loads(pathlib.Path(os.path.expanduser(str(path))).read_text())
        return cls(**d)

    def to_dict(self) -> dict:
        return {
            **{k: v for k, v in self.extra.items() if k not in ("source",)},
            "epsg": self.epsg,
            "yaw": self.yaw,
            "t": list(self.t),
            "z0": self.z0,
        }

    def local_affine(
        self, cx: float = 0.0, cy: float = 0.0, span: float = 600.0
    ) -> dict:
        """Least-squares affine map↔lon/lat around ``(cx, cy)`` (±``span``/2 m),
        for the browser (tile placement, cursor readout); ``max_err_m`` is the
        worst error on the fitting grid (millimetres at this scale)."""
        import numpy as np

        g = np.linspace(-span / 2, span / 2, 7)
        X, Y = np.meshgrid(g + cx, g + cy)
        xy = np.c_[X.ravel(), Y.ravel()]
        ll = np.array([self.map_to_lonlat(x, y) for x, y in xy])
        A = np.c_[xy, np.ones(len(xy))]
        fwd, *_ = np.linalg.lstsq(A, ll, rcond=None)  # [x y 1] @ fwd = [lon lat]
        B = np.c_[ll, np.ones(len(ll))]
        inv, *_ = np.linalg.lstsq(B, xy, rcond=None)
        back = B @ inv
        return {
            "center": [cx, cy],
            "map_to_lonlat": fwd.T.tolist(),  # 2×3 rows: lon, lat
            "lonlat_to_map": inv.T.tolist(),  # 2×3 rows: x, y
            "max_err_m": float(np.abs(back - xy).max()),
        }

    def map_to_utm(self, x: float, y: float) -> tuple[float, float]:
        return (
            self._c * x - self._s * y + self.t[0],
            self._s * x + self._c * y + self.t[1],
        )

    def utm_to_map(self, e: float, n: float) -> tuple[float, float]:
        dx, dy = e - self.t[0], n - self.t[1]
        return (self._c * dx + self._s * dy, -self._s * dx + self._c * dy)

    def map_to_lonlat(self, x: float, y: float) -> tuple[float, float]:
        if self._to_ll is None:
            self._to_ll = _transformer(self.epsg, inverse=False)
        return self._to_ll.transform(*self.map_to_utm(x, y))

    def lonlat_to_map(self, lon: float, lat: float) -> tuple[float, float]:
        if self._from_ll is None:
            self._from_ll = _transformer(self.epsg, inverse=True)
        return self.utm_to_map(*self._from_ll.transform(lon, lat))


def find_anchor(run_dir) -> pathlib.Path | None:
    """``geo_anchor.json`` in a run dir (or its parent: the anchor may sit
    next to ``output/`` for a bag)."""
    if run_dir is None:
        return None
    run = pathlib.Path(run_dir)
    for p in (run / ANCHOR_FILE, run.parent / ANCHOR_FILE):
        if p.is_file():
            return p
    return None


# ------------------------------------------------------------------- tiles
def lonlat_to_tile(lon: float, lat: float, z: int) -> tuple[float, float]:
    """Fractional XYZ tile coordinates (web mercator)."""
    n = 2.0**z
    lat_r = math.radians(lat)
    x = (lon + 180.0) / 360.0 * n
    y = (1.0 - math.asinh(math.tan(lat_r)) / math.pi) / 2.0 * n
    return x, y


def tile_to_lonlat(x: float, y: float, z: int) -> tuple[float, float]:
    """Lon/lat of a (fractional) tile corner."""
    n = 2.0**z
    lon = x / n * 360.0 - 180.0
    lat = math.degrees(math.atan(math.sinh(math.pi * (1 - 2 * y / n))))
    return lon, lat


class TileCache:
    def __init__(self, root, live: bool = False, urls: dict | None = None, fetch=None):
        self.root = pathlib.Path(os.path.expanduser(str(root)))
        self.live = live
        self.urls = dict(LAYER_URLS, **(urls or {}))
        # a layer's manifest may name its own source (``url``), e.g. a city's aerial tile service
        for name, m in self._manifests().items():
            if m.get("url"):
                self.urls.setdefault(name, m["url"])
        self._fetch = fetch or _http_get
        self._lock = threading.Lock()
        self._failed: dict[
            str, float
        ] = {}  # url -> time, so a dead link is not hammered

    def _manifests(self) -> dict[str, dict]:
        """``tiles.json`` of every layer dir under the root ({} for a dir without one)."""
        out = {}
        if self.root.is_dir():
            for d in self.root.iterdir():
                if d.is_dir():
                    try:
                        out[d.name] = json.loads((d / "tiles.json").read_text())
                    except (OSError, ValueError):
                        out[d.name] = {}
        return out

    def layers(self) -> list[dict]:
        out = []
        manifests = self._manifests()
        names = set(manifests) | (set(self.urls) if self.live else set())
        for name in sorted(names):
            manifest = manifests.get(name, {})
            out.append(
                {
                    "id": name,
                    "attribution": manifest.get("attribution")
                    or LAYER_ATTRIBUTION.get(name, ""),
                    "zmin": manifest.get("zmin"),
                    "zmax": manifest.get("zmax"),
                    "bbox_lonlat": manifest.get("bbox_lonlat"),
                    "cached": manifest.get("count"),
                    "live": self.live and name in self.urls,
                    "default": bool(manifest.get("default")),
                }
            )
        return out

    def path(self, layer: str, z: int, x: int, y: int) -> pathlib.Path:
        if not layer.replace("_", "").replace("-", "").isalnum():
            raise ValueError(f"bad layer name {layer!r}")
        return self.root / layer / str(int(z)) / str(int(x)) / f"{int(y)}.jpg"

    def get(self, layer: str, z: int, x: int, y: int) -> bytes | None:
        """Cached tile bytes; on a miss fetch it (live only) and keep it."""
        p = self.path(layer, z, x, y)
        try:
            return p.read_bytes()
        except OSError:
            pass
        for ext in (".png", ".jpeg"):
            try:
                return p.with_suffix(ext).read_bytes()
            except OSError:
                pass
        if not self.live or layer not in self.urls:
            return None
        url = self.urls[layer].format(z=int(z), x=int(x), y=int(y))
        import time

        if time.time() - self._failed.get(url, 0.0) < 60.0:
            return None
        try:
            data = self._fetch(url)
        except Exception as e:  # noqa: BLE001
            log.info("tile fetch failed %s: %s", url, e)
            self._failed[url] = time.time()
            return None
        if data:
            try:
                p.parent.mkdir(parents=True, exist_ok=True)
                tmp = p.with_suffix(".part")
                tmp.write_bytes(data)
                tmp.replace(p)
            except OSError as e:
                log.warning("could not cache tile %s: %s", p, e)
        return data


def _http_get(url: str) -> bytes:
    req = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})
    with urllib.request.urlopen(req, timeout=8) as r:
        return r.read()


class GeoService:
    """Anchors per graph source (explicit ``--geo-anchor`` wins over the run
    dir's ``geo_anchor.json``) + the tile cache."""

    def __init__(self, settings, graph=None, fetch=None):
        self.settings = settings
        self.graph = graph
        self.tiles = TileCache(
            getattr(settings, "tiles_dir", "~/.adt4_console/tiles"),
            live=getattr(settings, "tiles_live", False),
            fetch=fetch,
        )
        self._explicit = dict(getattr(settings, "geo_anchors", {}) or {})
        self._cache: dict[str, GeoAnchor | None] = {}

    def forget(self, src_id: str) -> None:
        """The source now holds another map: look its anchor up again."""
        self._cache.pop(src_id, None)

    def anchor(self, src_id: str | None) -> GeoAnchor | None:
        if not src_id:
            return self.default_anchor()
        if src_id in self._cache:
            return self._cache[src_id]
        a = None
        name = src_id.split(":", 1)[-1]
        path = self._explicit.get(src_id) or self._explicit.get(name)
        if path is None and self.graph is not None:
            try:
                src = self.graph.get(src_id)
                run = getattr(src, "run_dir", None)
                run = run() if callable(run) else run
                path = find_anchor(run)
            except Exception:  # noqa: BLE001
                path = None
        if path is not None:
            try:
                a = GeoAnchor.load(path)
            except (OSError, ValueError, TypeError) as e:
                log.warning("bad geo anchor %s: %s", path, e)
        if a is None and len(self._explicit) == 1 and "*" in self._explicit:
            a = GeoAnchor.load(self._explicit["*"])
        self._cache[src_id] = a
        return a

    def default_anchor(self) -> GeoAnchor | None:
        """The first source with an anchor (all sources share the map frame
        of one experiment)."""
        if self.graph is None:
            return None
        for s in self.graph.list():
            a = self.anchor(s["id"])
            if a is not None:
                return a
        return None
