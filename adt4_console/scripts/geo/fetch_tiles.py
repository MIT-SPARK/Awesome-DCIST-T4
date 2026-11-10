#!/usr/bin/env python3
"""Fetch (or render from a GeoTIFF) XYZ map tiles for offline use.

Layout: <out-dir>/<layer>/{z}/{x}/{y}.jpg plus <layer>/tiles.json.
Bounds come from --bbox-lonlat, or --anchor + --trajectory (map extent
transformed to UTM, padded by --margin-m).
"""

import argparse
import json
import math
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from geo_common import tile_bounds_lonlat, tiles_for_bbox, utm_to_lonlat  # noqa: E402

ESRI = "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}"
UA = "adt4-console-tile-fetcher/1.0"


def bbox_from_anchor(anchor_path, traj_path, margin_m):
    a = json.loads(Path(anchor_path).read_text())
    xy = np.loadtxt(traj_path, delimiter=",", skiprows=1)[:, 1:3]
    c, s = math.cos(a["yaw"]), math.sin(a["yaw"])
    e = c * xy[:, 0] - s * xy[:, 1] + a["t"][0]
    n = s * xy[:, 0] + c * xy[:, 1] + a["t"][1]
    lon0, lat0 = utm_to_lonlat(e.min() - margin_m, n.min() - margin_m, a["epsg"])
    lon1, lat1 = utm_to_lonlat(e.max() + margin_m, n.max() + margin_m, a["epsg"])
    return [lon0, lat0, lon1, lat1]


def fetch_one(args):
    url, dest, retries = args
    if dest.exists() and dest.stat().st_size > 0:
        return "skip"
    import urllib.request

    for k in range(retries):
        try:
            req = urllib.request.Request(url, headers={"User-Agent": UA})
            with urllib.request.urlopen(req, timeout=20) as r:
                data = r.read()
            dest.parent.mkdir(parents=True, exist_ok=True)
            tmp = dest.with_suffix(".part")
            tmp.write_bytes(data)
            tmp.rename(dest)
            return "ok"
        except Exception:
            time.sleep(1.5 * (k + 1))
    return "fail"


def render_geotiff(tif, tiles, z, layer_dir):
    """Render tiles of zoom z from a GeoTIFF into layer_dir as JPEG."""
    import rasterio
    from PIL import Image
    from pyproj import Transformer
    from rasterio.transform import from_bounds
    from rasterio.warp import Resampling, reproject

    to_merc = Transformer.from_crs(4326, 3857, always_xy=True)
    done = 0
    with rasterio.open(tif) as src:
        nb = min(src.count, 3)
        for x, y in tiles:
            w, s, e, n = tile_bounds_lonlat(x, y, z)
            x0, y0 = to_merc.transform(w, n)
            x1, y1 = to_merc.transform(e, s)
            dst_t = from_bounds(x0, y1, x1, y0, 256, 256)
            out = np.zeros((3, 256, 256), np.uint8)
            mask = np.zeros((256, 256), np.uint8)
            for b in range(nb):
                reproject(
                    rasterio.band(src, b + 1),
                    out[b],
                    dst_transform=dst_t,
                    dst_crs="EPSG:3857",
                    resampling=Resampling.bilinear,
                )
            if nb == 1:
                out[1] = out[2] = out[0]
            reproject(
                np.ones((src.height, src.width), np.uint8),
                mask,
                src_transform=src.transform,
                src_crs=src.crs,
                dst_transform=dst_t,
                dst_crs="EPSG:3857",
                dst_nodata=0,
            )
            if not mask.any():
                continue
            dest = layer_dir / str(z) / str(x) / f"{y}.jpg"
            dest.parent.mkdir(parents=True, exist_ok=True)
            Image.fromarray(np.moveaxis(out, 0, -1)).save(dest, quality=88)
            done += 1
    return done


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--anchor")
    ap.add_argument("--trajectory", help="trajectory.csv used for the map extent")
    ap.add_argument("--map-bbox", dest="trajectory_alias", help=argparse.SUPPRESS)
    ap.add_argument(
        "--bbox-lonlat",
        type=float,
        nargs=4,
        metavar=("MINLON", "MINLAT", "MAXLON", "MAXLAT"),
    )
    ap.add_argument("--margin-m", type=float, default=80)
    ap.add_argument("--zmin", type=int, default=15)
    ap.add_argument("--zmax", type=int, default=20)
    ap.add_argument(
        "--max-tiles", type=int, default=6000, help="drop top zooms until total <= this"
    )
    ap.add_argument("--layer", default="esri_world_imagery")
    ap.add_argument("--url-template", default=ESRI)
    ap.add_argument("--attribution", default=None)
    ap.add_argument("--out-dir", default="~/.adt4_console/tiles")
    ap.add_argument("--geotiff")
    ap.add_argument("--concurrency", type=int, default=4)
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()

    traj = a.trajectory or a.trajectory_alias
    if a.bbox_lonlat:
        bbox = a.bbox_lonlat
    elif a.anchor and traj:
        bbox = bbox_from_anchor(a.anchor, traj, a.margin_m)
    elif a.geotiff:
        import rasterio
        from rasterio.warp import transform_bounds

        with rasterio.open(a.geotiff) as s:
            bbox = list(transform_bounds(s.crs, "EPSG:4326", *s.bounds))
    else:
        ap.error("need --bbox-lonlat, or --anchor with --trajectory")
    print("bbox lonlat:", bbox)

    zmax = a.zmax
    while zmax >= a.zmin:
        total = sum(len(tiles_for_bbox(*bbox, z)) for z in range(a.zmin, zmax + 1))
        if total <= a.max_tiles:
            break
        zmax -= 1
    if zmax != a.zmax:
        print(f"capping zmax {a.zmax} -> {zmax} ({total} tiles <= {a.max_tiles})")
    print(f"z{a.zmin}-{zmax}: {total} tiles")
    if a.dry_run:
        return

    layer_dir = Path(a.out_dir).expanduser() / a.layer
    layer_dir.mkdir(parents=True, exist_ok=True)
    stats = {"ok": 0, "skip": 0, "fail": 0}
    for z in range(a.zmin, zmax + 1):
        tiles = tiles_for_bbox(*bbox, z)
        if a.geotiff:
            stats["ok"] += render_geotiff(a.geotiff, tiles, z, layer_dir)
        else:
            jobs = [
                (
                    a.url_template.format(z=z, x=x, y=y),
                    layer_dir / str(z) / str(x) / f"{y}.jpg",
                    3,
                )
                for x, y in tiles
            ]
            with ThreadPoolExecutor(a.concurrency) as ex:
                for r in ex.map(fetch_one, jobs):
                    stats[r] += 1
        print(f"z{z}: {len(tiles)} tiles, cumulative {stats}", flush=True)

    count = sum(1 for _ in layer_dir.glob("*/*/*.jpg"))
    attribution = a.attribution or (
        "Esri, Maxar, Earthstar Geographics"
        if a.url_template == ESRI and not a.geotiff
        else ""
    )
    (layer_dir / "tiles.json").write_text(
        json.dumps(
            {
                "layer": a.layer,
                "attribution": attribution,
                "zmin": a.zmin,
                "zmax": zmax,
                "bbox_lonlat": bbox,
                "count": count,
            },
            indent=2,
        )
    )
    print("manifest written; tiles on disk:", count, "failures:", stats["fail"])


if __name__ == "__main__":
    main()
