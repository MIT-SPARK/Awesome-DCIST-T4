// Map frame <-> lon/lat (the server's local affine fit of the GPS anchor,
// GET /api/geo/anchor/<src>) and XYZ satellite tile math (API.md §14).

export interface GeoAnchor {
  epsg: number;
  yaw: number;
  t: [number, number];
  z0: number;
  rms?: number;
}

export interface GeoAffine {
  center: [number, number];
  /** 2×3 rows: lon, lat = row · [x, y, 1] */
  map_to_lonlat: number[][];
  /** 2×3 rows: x, y = row · [lon, lat, 1] */
  lonlat_to_map: number[][];
  max_err_m: number;
}

export interface GeoInfo {
  anchor: GeoAnchor;
  affine: GeoAffine;
}

export interface TileLayer {
  id: string;
  attribution: string;
  zmin: number | null;
  zmax: number | null;
  bbox_lonlat: number[] | null;
  cached: number | null;
  live: boolean;
  default: boolean; // the manifest's pick for the map's ground
}

const apply = (m: number[][], a: number, b: number): [number, number] => [m[0][0] * a + m[0][1] * b + m[0][2], m[1][0] * a + m[1][1] * b + m[1][2]];

export const mapToLonLat = (g: GeoInfo, x: number, y: number) => apply(g.affine.map_to_lonlat, x, y);
export const lonLatToMap = (g: GeoInfo, lon: number, lat: number) => apply(g.affine.lonlat_to_map, lon, lat);

/** map → UTM (exact: the anchor is rigid). */
export function mapToUtm(a: GeoAnchor, x: number, y: number): [number, number] {
  const c = Math.cos(a.yaw);
  const s = Math.sin(a.yaw);
  return [c * x - s * y + a.t[0], s * x + c * y + a.t[1]];
}

export function utmZone(epsg: number): string {
  const north = epsg >= 32601 && epsg <= 32660;
  const south = epsg >= 32701 && epsg <= 32760;
  if (!north && !south) return `EPSG:${epsg}`;
  return `${epsg % 100}${north ? "N" : "S"}`;
}

/** Fractional XYZ tile coordinates (web mercator). */
export function lonLatToTile(lon: number, lat: number, z: number): [number, number] {
  const n = 2 ** z;
  const r = (lat * Math.PI) / 180;
  return [((lon + 180) / 360) * n, ((1 - Math.asinh(Math.tan(r)) / Math.PI) / 2) * n];
}

export function tileToLonLat(x: number, y: number, z: number): [number, number] {
  const n = 2 ** z;
  const lon = (x / n) * 360 - 180;
  const lat = (Math.atan(Math.sinh(Math.PI * (1 - (2 * y) / n))) * 180) / Math.PI;
  return [lon, lat];
}

export interface TileQuad {
  key: string;
  z: number;
  x: number;
  y: number;
  /** map-frame corners: NW, NE, SE, SW */
  corners: [number, number][];
}

/** Tiles at zoom ``z`` covering a map-frame box, each with its corners in
 *  the map frame (``max`` caps the count: the caller lowers the zoom). */
export function tilesForBox(g: GeoInfo, box: {min: number[]; max: number[]}, z: number, max = 400): TileQuad[] | null {
  const pts = [
    [box.min[0], box.min[1]],
    [box.max[0], box.min[1]],
    [box.max[0], box.max[1]],
    [box.min[0], box.max[1]],
  ].map(([x, y]) => {
    const [lon, lat] = mapToLonLat(g, x, y);
    return lonLatToTile(lon, lat, z);
  });
  const x0 = Math.floor(Math.min(...pts.map(p => p[0])));
  const x1 = Math.floor(Math.max(...pts.map(p => p[0])));
  const y0 = Math.floor(Math.min(...pts.map(p => p[1])));
  const y1 = Math.floor(Math.max(...pts.map(p => p[1])));
  if ((x1 - x0 + 1) * (y1 - y0 + 1) > max) return null;
  const corner = (tx: number, ty: number): [number, number] => {
    const [lon, lat] = tileToLonLat(tx, ty, z);
    return lonLatToMap(g, lon, lat);
  };
  const out: TileQuad[] = [];
  for (let x = x0; x <= x1; x++)
    for (let y = y0; y <= y1; y++)
      out.push({key: `${z}/${x}/${y}`, z, x, y, corners: [corner(x, y), corner(x + 1, y), corner(x + 1, y + 1), corner(x, y + 1)]});
  return out;
}

/** The highest zoom (≤ ``zmax``) whose tiles over ``box`` stay within ``max``. */
export function pickZoom(g: GeoInfo, box: {min: number[]; max: number[]}, zmax: number, zmin = 12, max = 160): number {
  for (let z = zmax; z > zmin; z--) {
    const t = tilesForBox(g, box, z, max);
    if (t) return z;
  }
  return zmin;
}

export function fmtLonLat(lon: number, lat: number): string {
  return `${Math.abs(lat).toFixed(6)}°${lat >= 0 ? "N" : "S"} ${Math.abs(lon).toFixed(6)}°${lon >= 0 ? "E" : "W"}`;
}
