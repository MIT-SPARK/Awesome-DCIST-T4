"""Shared geo helpers: UTM <-> lon/lat and slippy-map (XYZ) tile math."""

import math

from pyproj import Transformer

EPSG_UTM18N = 32618


def lonlat_to_utm(lon, lat, epsg=EPSG_UTM18N):
    """Return (easting, northing); accepts scalars or numpy arrays."""
    return Transformer.from_crs(4326, epsg, always_xy=True).transform(lon, lat)


def utm_to_lonlat(e, n, epsg=EPSG_UTM18N):
    """Return (lon, lat); accepts scalars or numpy arrays."""
    return Transformer.from_crs(epsg, 4326, always_xy=True).transform(e, n)


def lonlat_to_tile(lon, lat, z):
    """Slippy-map tile (x, y) containing lon/lat at zoom z (y grows southward)."""
    n = 2**z
    lat = max(min(lat, 85.0511287798), -85.0511287798)
    x = int((lon + 180.0) / 360.0 * n)
    y = int((1.0 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2.0 * n)
    return min(max(x, 0), n - 1), min(max(y, 0), n - 1)


def tile_to_lonlat(x, y, z):
    """Lon/lat of the north-west corner of tile (x, y) at zoom z."""
    n = 2**z
    lon = x / n * 360.0 - 180.0
    lat = math.degrees(math.atan(math.sinh(math.pi * (1 - 2 * y / n))))
    return lon, lat


def tile_bounds_lonlat(x, y, z):
    """(minlon, minlat, maxlon, maxlat) of a tile."""
    w, n = tile_to_lonlat(x, y, z)
    e, s = tile_to_lonlat(x + 1, y + 1, z)
    return w, s, e, n


def tiles_for_bbox(minlon, minlat, maxlon, maxlat, z):
    """All (x, y) tiles at zoom z covering the lon/lat bbox."""
    x0, y1 = lonlat_to_tile(minlon, minlat, z)
    x1, y0 = lonlat_to_tile(maxlon, maxlat, z)
    return [(x, y) for x in range(x0, x1 + 1) for y in range(y0, y1 + 1)]
