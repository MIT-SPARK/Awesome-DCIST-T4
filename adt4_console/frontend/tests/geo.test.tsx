// Satellite underlay: off by default, on/off + opacity remembered per browser,
// geo (layers + anchor) fetched for the shown source.
import {afterEach, beforeEach, describe, expect, it, vi} from "vitest";
import {act} from "react";
import {createRoot, type Root} from "react-dom/client";
import {DEFAULT_SATELLITE, SATELLITE_KEY, loadGeo, loadSatellite, saveSatellite, satelliteNote, useGeo} from "../src/stores/geo";
import {SatelliteControl} from "../src/viewer/StylePanels";

(globalThis as unknown as {IS_REACT_ACT_ENVIRONMENT: boolean}).IS_REACT_ACT_ENVIRONMENT = true;

const mem = () => {
  const m = new Map<string, string>();
  return {getItem: (k: string) => m.get(k) ?? null, setItem: (k: string, v: string) => void m.set(k, v), m};
};
const throwing = {
  getItem: () => {
    throw new Error("blocked");
  },
  setItem: () => {
    throw new Error("blocked");
  },
};
const json = (body: unknown) => new Response(JSON.stringify(body), {status: 200, headers: {"Content-Type": "application/json"}});
const LAYER = {id: "esri_world_imagery", attribution: "", zmin: 12, zmax: 19, bbox_lonlat: null, cached: 10, live: false, default: true};
const ANCHOR = {epsg: 32618, yaw: 0, t: [480000, 4420000], z0: 0};
const AFFINE = {center: [-75.2, 39.9], map_to_lonlat: [[1, 0, 0], [0, 1, 0]], lonlat_to_map: [[1, 0, 0], [0, 1, 0]], max_err_m: 0.01};

function reset() {
  useGeo.setState({satellite: false, satOpacity: 1, layers: [], geo: null, geoSrc: null, bounds: null, loaded: false, error: null});
}

describe("satellite prefs", () => {
  it("default to off", () => {
    expect(DEFAULT_SATELLITE.on).toBe(false);
    expect(loadSatellite(mem())).toEqual({on: false, opacity: 1});
    expect(loadSatellite(null)).toEqual({on: false, opacity: 1});
  });

  it("round-trip, clamp the opacity and survive junk or blocked storage", () => {
    const s = mem();
    saveSatellite({on: true, opacity: 0.4}, s);
    expect(loadSatellite(s)).toEqual({on: true, opacity: 0.4});
    s.setItem(SATELLITE_KEY, JSON.stringify({on: "yes", opacity: 7}));
    expect(loadSatellite(s)).toEqual({on: false, opacity: 1});
    s.setItem(SATELLITE_KEY, JSON.stringify({on: true, opacity: 0}));
    expect(loadSatellite(s)).toEqual({on: true, opacity: 0.1});
    s.setItem(SATELLITE_KEY, "{not json");
    expect(loadSatellite(s)).toEqual({on: false, opacity: 1});
    expect(loadSatellite(throwing)).toEqual({on: false, opacity: 1});
    expect(() => saveSatellite({on: true, opacity: 1}, throwing)).not.toThrow();
  });

  it("the store's setters persist to localStorage", () => {
    reset();
    useGeo.getState().setSatellite(true);
    useGeo.getState().setOpacity(0.55);
    expect(JSON.parse(window.localStorage.getItem(SATELLITE_KEY)!)).toEqual({on: true, opacity: 0.55});
    useGeo.getState().setSatellite(false);
    expect(loadSatellite()).toEqual({on: false, opacity: 0.55});
    window.localStorage.removeItem(SATELLITE_KEY);
  });
});

describe("loadGeo", () => {
  beforeEach(reset);
  afterEach(() => vi.restoreAllMocks());

  it("fetches the layers once and the source's anchor at the map centre", async () => {
    const urls: string[] = [];
    vi.spyOn(globalThis, "fetch").mockImplementation(async input => {
      const u = String(input);
      urls.push(u);
      return json(u.includes("api/geo/layers") ? {layers: [LAYER]} : {src: "neo4j:live", anchor: ANCHOR, affine: AFFINE});
    });
    await loadGeo("neo4j:live", [0, 0, 100, 40]);
    expect(urls.some(u => u.includes("api/geo/layers"))).toBe(true);
    const anchorUrl = urls.find(u => u.includes("api/geo/anchor/"))!;
    expect(anchorUrl).toContain("api/geo/anchor/neo4j%3Alive?cx=50&cy=20");
    const st = useGeo.getState();
    expect(st.layers).toHaveLength(1);
    expect(st.geo).toEqual({anchor: ANCHOR, affine: AFFINE});
    expect(st.geoSrc).toBe("neo4j:live");
    expect(st.bounds).toEqual([0, 0, 100, 40]);
    // layers are cached: a second load asks only for the anchor
    urls.length = 0;
    await loadGeo("file:run", [0, 0, 10, 10]);
    expect(urls.filter(u => u.includes("api/geo/layers"))).toHaveLength(0);
  });

  it("a source without a GPS anchor leaves geo empty", async () => {
    vi.spyOn(globalThis, "fetch").mockImplementation(async input => json(String(input).includes("layers") ? {layers: [LAYER]} : {src: "file:x", anchor: null}));
    await loadGeo("file:x", null);
    expect(useGeo.getState().geo).toBeNull();
    expect(useGeo.getState().loaded).toBe(true);
    expect(satelliteNote({...useGeo.getState(), satellite: true})).toMatch(/no GPS anchor/);
  });

  it("explains why nothing shows only while the underlay is on", () => {
    const base = {satellite: true, layers: [LAYER], geo: {anchor: ANCHOR, affine: AFFINE}, loaded: true, error: null} as Parameters<typeof satelliteNote>[0];
    expect(satelliteNote(base)).toBeNull();
    expect(satelliteNote({...base, satellite: false, geo: null})).toBeNull();
    expect(satelliteNote({...base, layers: []})).toMatch(/No satellite tile layers/);
    expect(satelliteNote({...base, error: "boom"})).toMatch(/unavailable: boom/);
  });
});

describe("SatelliteControl", () => {
  let host: HTMLDivElement;
  let root: Root;
  beforeEach(() => {
    reset();
    window.localStorage.removeItem(SATELLITE_KEY);
    host = document.createElement("div");
    document.body.appendChild(host);
    root = createRoot(host);
  });
  afterEach(() => {
    act(() => root.unmount());
    host.remove();
    window.localStorage.removeItem(SATELLITE_KEY);
  });

  it("starts off; turning it on shows the opacity slider and is remembered", () => {
    act(() => root.render(<SatelliteControl />));
    const box = host.querySelector<HTMLInputElement>('[data-testid="satellite-control"] input[type="checkbox"]')!;
    expect(box.checked).toBe(false);
    expect(host.querySelector('[aria-label="Satellite opacity"]')).toBeNull();
    act(() => box.click());
    expect(useGeo.getState().satellite).toBe(true);
    expect(host.querySelector('[aria-label="Satellite opacity"]')).not.toBeNull();
    expect(loadSatellite().on).toBe(true);
    act(() => box.click());
    expect(useGeo.getState().satellite).toBe(false);
    expect(loadSatellite().on).toBe(false);
  });
});
