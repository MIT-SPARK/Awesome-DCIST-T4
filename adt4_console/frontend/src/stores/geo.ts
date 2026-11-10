// Satellite underlay state (API.md §14): the tile layers the console serves, the
// shown map's GPS anchor, and the viewer's on/off + opacity. On/off and opacity
// are remembered per browser; the underlay is off until turned on, and nothing
// geo is fetched before that.
import {create} from "zustand";
import {errorText, getJSON} from "../lib/api";
import type {GeoInfo, TileLayer} from "../lib/geo";

export const SATELLITE_KEY = "adt4.viewer.satellite.v1";

export interface SatellitePrefs {
  on: boolean;
  opacity: number;
}

export const DEFAULT_SATELLITE: SatellitePrefs = {on: false, opacity: 1};

/** Stored prefs, sanitised; anything missing or malformed falls back to the default (off). */
export function loadSatellite(storage: Pick<Storage, "getItem"> | null | undefined = safeStorage()): SatellitePrefs {
  try {
    const v = JSON.parse(storage?.getItem(SATELLITE_KEY) ?? "null") as Partial<SatellitePrefs> | null;
    if (!v || typeof v !== "object") return {...DEFAULT_SATELLITE};
    const op = typeof v.opacity === "number" && Number.isFinite(v.opacity) ? Math.min(1, Math.max(0.1, v.opacity)) : DEFAULT_SATELLITE.opacity;
    return {on: v.on === true, opacity: op};
  } catch {
    return {...DEFAULT_SATELLITE};
  }
}

export function saveSatellite(p: SatellitePrefs, storage: Pick<Storage, "setItem"> | null | undefined = safeStorage()) {
  try {
    storage?.setItem(SATELLITE_KEY, JSON.stringify(p));
  } catch {
    /* private window / quota: the choice holds for this page */
  }
}

function safeStorage(): Storage | null {
  try {
    return typeof window !== "undefined" ? window.localStorage : null;
  } catch {
    return null;
  }
}

/** Map extent in the map frame: [xmin, ymin, xmax, ymax]. */
export type Bounds2 = [number, number, number, number];

interface GeoStore {
  /** satellite underlay shown (per browser, default off) */
  satellite: boolean;
  /** underlay opacity 0.1..1 (per browser) */
  satOpacity: number;
  layers: TileLayer[];
  /** the anchor of `geoSrc`, null when that source has no GPS alignment */
  geo: GeoInfo | null;
  geoSrc: string | null;
  /** the map extent the underlay covers (detail tiles) */
  bounds: Bounds2 | null;
  /** the last load finished (with or without an anchor) */
  loaded: boolean;
  error: string | null;
  setSatellite: (on: boolean) => void;
  setOpacity: (opacity: number) => void;
}

const initial = loadSatellite();

export const useGeo = create<GeoStore>((set, get) => ({
  satellite: initial.on,
  satOpacity: initial.opacity,
  layers: [],
  geo: null,
  geoSrc: null,
  bounds: null,
  loaded: false,
  error: null,
  setSatellite: on => {
    set({satellite: on});
    saveSatellite({on, opacity: get().satOpacity});
  },
  setOpacity: opacity => {
    const op = Math.min(1, Math.max(0.1, opacity));
    set({satOpacity: op});
    saveSatellite({on: get().satellite, opacity: op});
  },
}));

const enc = encodeURIComponent;

/** Tile layers + the GPS anchor of source ``src`` (its local affine fit taken at the
 *  centre of ``bounds``). A source without an anchor leaves ``geo`` null. */
export async function loadGeo(src: string | null, bounds: Bounds2 | null): Promise<void> {
  const st = useGeo.getState;
  useGeo.setState({error: null, bounds});
  try {
    if (!st().layers.length) {
      const r = await getJSON<{layers: TileLayer[]}>("api/geo/layers");
      useGeo.setState({layers: Array.isArray(r.layers) ? r.layers : []});
    }
    if (!src) {
      useGeo.setState({geo: null, geoSrc: null, loaded: true});
      return;
    }
    const cx = bounds ? (bounds[0] + bounds[2]) / 2 : 0;
    const cy = bounds ? (bounds[1] + bounds[3]) / 2 : 0;
    const r = await getJSON<{anchor: GeoInfo["anchor"] | null; affine?: GeoInfo["affine"]}>(`api/geo/anchor/${enc(src)}?cx=${cx}&cy=${cy}`);
    useGeo.setState({geo: r.anchor && r.affine ? {anchor: r.anchor, affine: r.affine} : null, geoSrc: src, loaded: true});
  } catch (e) {
    useGeo.setState({geo: null, geoSrc: src, loaded: true, error: errorText(e)});
  }
}

/** Why the underlay is not showing, for the toggle's hint (null: it is, or it is off). */
export function satelliteNote(s: Pick<GeoStore, "satellite" | "layers" | "geo" | "loaded" | "error">): string | null {
  if (!s.satellite || !s.loaded) return null;
  if (s.error) return `Satellite unavailable: ${s.error}`;
  if (!s.layers.length) return "No satellite tile layers configured on the console (--tiles-dir).";
  if (!s.geo) return "This map has no GPS anchor: nothing to place the imagery by.";
  return null;
}
