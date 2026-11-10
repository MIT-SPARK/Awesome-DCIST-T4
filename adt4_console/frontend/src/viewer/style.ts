// Map-viewer styling: background, per-layer colour overrides, per-layer heights,
// region display and screenshot options. Pure helpers (tested) + the persisted
// settings shape; the zustand store lives in stores/viewStyle.ts.
import type {LayerInfo} from "../lib/graphModel";

export type RGB = [number, number, number];

// ---------------------------------------------------------------- colours

export function hexToRgb(hex: string): RGB | null {
  const s = hex.trim().replace(/^#/, "");
  const full = /^[0-9a-f]{3}$/i.test(s) ? s.replace(/./g, c => c + c) : s;
  if (!/^[0-9a-f]{6}$/i.test(full)) return null;
  const v = parseInt(full, 16);
  return [(v >> 16) & 255, (v >> 8) & 255, v & 255];
}

const h2 = (n: number) => Math.max(0, Math.min(255, Math.round(n))).toString(16).padStart(2, "0");
export const rgbToHex = (c: RGB) => `#${h2(c[0])}${h2(c[1])}${h2(c[2])}`;

/** WCAG relative luminance of an sRGB hex colour (0 black .. 1 white). */
export function luminance(hex: string): number {
  const c = hexToRgb(hex) ?? [0, 0, 0];
  const ch = (v: number) => {
    const s = v / 255;
    return s <= 0.03928 ? s / 12.92 : ((s + 0.055) / 1.055) ** 2.4;
  };
  return 0.2126 * ch(c[0]) + 0.7152 * ch(c[1]) + 0.0722 * ch(c[2]);
}

/** WCAG contrast ratio between two colours (1 .. 21). */
export function contrastRatio(a: string, b: string): number {
  const la = luminance(a);
  const lb = luminance(b);
  return (Math.max(la, lb) + 0.05) / (Math.min(la, lb) + 0.05);
}

/** Linear mix in sRGB space, t = 0 -> a, 1 -> b. */
export function mixHex(a: string, b: string, t: number): string {
  const ca = hexToRgb(a) ?? [0, 0, 0];
  const cb = hexToRgb(b) ?? [0, 0, 0];
  return rgbToHex([0, 1, 2].map(i => ca[i] + (cb[i] - ca[i]) * t) as RGB);
}

/** `hex` darkened just enough to reach `min` contrast against `bg` (unchanged
 *  if it already does). Keeps pale robot colours (yellow, cream, mint) visible
 *  on a light 3D background. */
export function readableOn(hex: string, bg: string, min = 3): string {
  if (!hexToRgb(hex)) return hex;
  let c = hex.toLowerCase();
  for (let t = 0.1; contrastRatio(c, bg) < min && t <= 0.8 + 1e-9; t += 0.1) c = mixHex(hex, "#000000", t);
  return c;
}

/** Background quick picks (Display panel). "RViz gray" is the hydra style default. */
export const BG_PRESETS: {name: string; color: string}[] = [
  {name: "White", color: "#ffffff"},
  {name: "Light gray", color: "#d9d9d9"},
  {name: "RViz gray", color: "#303030"},
  {name: "Dark", color: "#0b0f14"},
  {name: "Black", color: "#000000"},
];

/** Style defaults: RViz look for hydra, near-black for classic. */
export const STYLE_BACKGROUND = {hydra: "#303030", classic: "#0b0f14"} as const;
/** Light-theme defaults: a soft grey (not pure white) so white highlights and
 *  pale mesh colours still separate from the background. */
export const LIGHT_BACKGROUND = {hydra: "#e4e4e4", classic: "#f1f3f6"} as const;

/** Default 3D background for a viz style under the console theme. */
export function defaultBackground(style: "hydra" | "classic", theme: "dark" | "light" = "dark"): string {
  return (theme === "light" ? LIGHT_BACKGROUND : STYLE_BACKGROUND)[style];
}
const STYLE_GRID = {hydra: ["#7a7a80", "#55555a"], classic: ["#2a3440", "#1a222b"]} as const;

export interface Contrast {
  /** a light background (dark ink on it) */
  light: boolean;
  grid: [string, string];
  gridOpacity: number;
  /** text label colour + halo */
  text: string;
  halo: string;
  /** edges drawn black by hydra_visualizer: kept black unless the background is darker than RViz gray */
  edge: string;
  /** legend / scale bar ink */
  ink: string;
}

/** Grid / label / edge colours that stay readable on `bg`. The two style
 *  defaults keep their historical grid colours exactly. */
export function contrastFor(bg: string, style: "hydra" | "classic" = "hydra"): Contrast {
  const lum = luminance(bg);
  const light = lum > 0.18;
  const ink = light ? "#000000" : "#ffffff";
  const norm = bg.toLowerCase();
  let grid: [string, string];
  let gridOpacity = 0.5;
  if (norm === STYLE_BACKGROUND.hydra) grid = [...STYLE_GRID.hydra];
  else if (norm === STYLE_BACKGROUND.classic) {
    grid = [...STYLE_GRID.classic];
    gridOpacity = 0.6;
  } else grid = [mixHex(bg, ink, light ? 0.45 : 0.4), mixHex(bg, ink, light ? 0.22 : 0.2)];
  void style;
  return {
    light,
    grid,
    gridOpacity,
    text: light ? "#111111" : "#ffffff",
    halo: light ? "#ffffff" : "#000000",
    edge: lum < luminance(STYLE_BACKGROUND.hydra) - 1e-4 ? "#a8a8a8" : "#000000",
    ink,
  };
}

// ---------------------------------------------------------------- colour overrides

export type ColorKey = "objects" | "agents" | "places" | "rooms" | "edges" | "trajectory" | "bboxes";
export type ColorMode = "default" | "uniform" | "label";
export interface ColorRule {
  mode: ColorMode;
  color: string;
}
export const COLOR_KEYS: {key: ColorKey; name: string; label?: boolean}[] = [
  {key: "objects", name: "Objects", label: true},
  {key: "bboxes", name: "Bounding boxes"},
  {key: "agents", name: "Agents"},
  {key: "places", name: "Places"},
  {key: "rooms", name: "Rooms / regions"},
  {key: "edges", name: "Edges"},
  {key: "trajectory", name: "Trajectory"},
];
export const DEFAULT_RULE_COLOR: Record<ColorKey, string> = {
  objects: "#ff7f00",
  bboxes: "#ffd400",
  agents: "#1f78b4",
  places: "#4fc3f7",
  rooms: "#e05ad0",
  edges: "#808080",
  trajectory: "#ff7a1a",
};

export type MeshColorMode = "rgb" | "uniform" | "height";

/** Which colour key a layer kind's nodes follow. */
export function colorKeyOf(kind: LayerInfo["kind"]): ColorKey | null {
  switch (kind) {
    case "objects":
    case "segments":
      return "objects";
    case "agents":
    case "subkeyframes":
      return "agents";
    case "places":
      return "places";
    case "rooms":
    case "buildings":
      return "rooms";
    default:
      return null;
  }
}

/** Resolved colour overrides handed to layoutGraph (null = draw the style default). */
export interface ColorOpts {
  rules: Partial<Record<ColorKey, ColorRule>>;
  /** label name -> hex, used in "by label" mode */
  labelColors: Record<string, string>;
  /** places with a parent room / region take that region's colour (hydra ParentColorAdapter) */
  placesByRegion: boolean;
}

/** Deterministic default colour for a label in "by label" mode. */
export function defaultLabelColor(label: number): RGB {
  return LABEL_PALETTE[(Math.imul(label + 1, 2654435761) >>> 0) % LABEL_PALETTE.length];
}
const LABEL_PALETTE: RGB[] = [
  [230, 25, 75], [60, 180, 75], [255, 225, 25], [67, 99, 216], [245, 130, 49],
  [145, 30, 180], [66, 212, 244], [240, 50, 230], [191, 239, 69], [250, 190, 212],
  [70, 153, 144], [220, 190, 255], [154, 99, 36], [255, 250, 200], [128, 0, 0],
  [170, 255, 195], [128, 128, 0], [255, 216, 177], [0, 117, 255], [169, 169, 169],
];

/** Override colour for a node, or null for the style default. */
export function overrideColor(
  key: ColorKey | null,
  opts: ColorOpts | undefined,
  label: {id: number; name: string} | null,
): RGB | null {
  if (!key || !opts) return null;
  const rule = opts.rules[key];
  if (!rule || rule.mode === "default") return null;
  if (rule.mode === "uniform") return hexToRgb(rule.color);
  if (rule.mode === "label" && label) {
    const custom = label.name ? opts.labelColors[label.name] : undefined;
    return (custom && hexToRgb(custom)) || defaultLabelColor(label.id);
  }
  return null;
}

/** Uniform override for a non-node element (edges, trajectory, bboxes), or null. */
export function uniformOf(key: ColorKey, opts: ColorOpts | undefined): string | null {
  const r = opts?.rules[key];
  return r && r.mode === "uniform" ? r.color : null;
}

// ---------------------------------------------------------------- heights

export type HeightKey = "agents" | "objects" | "bboxes" | "places" | "rooms";
export const HEIGHT_KEYS: {key: HeightKey; name: string}[] = [
  {key: "places", name: "Places"},
  {key: "objects", name: "Object nodes"},
  {key: "bboxes", name: "Bounding boxes"},
  {key: "rooms", name: "Rooms / regions"},
  {key: "agents", name: "Agents"},
];
/** Per-layer height in units of the layer spacing (hydra z_offset_scale).
 *  Missing keys use the style default; bboxes default to the object height. */
export type Heights = Partial<Record<HeightKey, number>>;

/** Default z_offset_scale per style (hydra_visualizer.yaml / classic stacking order). */
export const DEFAULT_HEIGHTS: Record<"hydra" | "classic", Record<Exclude<HeightKey, "bboxes">, number>> = {
  hydra: {agents: 0, objects: 1, places: 2, rooms: 3},
  classic: {agents: 0, objects: 0, places: 1, rooms: 2},
};

export function heightKeyOf(kind: LayerInfo["kind"]): Exclude<HeightKey, "bboxes"> | null {
  switch (kind) {
    case "agents":
    case "subkeyframes":
      return "agents";
    case "objects":
    case "segments":
      return "objects";
    case "places":
      return "places";
    case "rooms":
      return "rooms";
    default:
      return null;
  }
}

/** z_offset_scale of a height key after overrides (bboxes follow objects unless set). */
export function heightScale(key: HeightKey, style: "hydra" | "classic", heights: Heights | undefined): number {
  const h = heights?.[key];
  if (h !== undefined && Number.isFinite(h)) return h;
  if (key === "bboxes") return heightScale("objects", style, heights);
  return DEFAULT_HEIGHTS[style][key];
}

// ---------------------------------------------------------------- screenshot

export type ShotFormat = "png" | "jpeg";
export interface ShotOpts {
  scale: 1 | 2;
  format: ShotFormat;
  /** JPEG quality 0..1 */
  quality: number;
  /** PNG only: clear to transparent instead of the background colour */
  transparent: boolean;
  /** legend: source name + timestamp + scale bar */
  legend: boolean;
  labels: boolean;
}
export const DEFAULT_SHOT: ShotOpts = {scale: 1, format: "png", quality: 0.92, transparent: false, legend: true, labels: true};

/** Effective options: JPEG has no alpha, so transparency is ignored. */
export function effectiveShot(o: ShotOpts): ShotOpts & {mime: string; ext: string} {
  const jpeg = o.format === "jpeg";
  return {
    ...o,
    transparent: jpeg ? false : o.transparent,
    quality: Math.min(1, Math.max(0.1, o.quality)),
    mime: jpeg ? "image/jpeg" : "image/png",
    ext: jpeg ? "jpg" : "png",
  };
}

const pad = (n: number) => String(n).padStart(2, "0");
export function stamp(d: Date): string {
  return `${d.getFullYear()}${pad(d.getMonth() + 1)}${pad(d.getDate())}-${pad(d.getHours())}${pad(d.getMinutes())}${pad(d.getSeconds())}`;
}

/** adt4_<source>_<YYYYMMDD-HHMMSS>.<ext>; source ids lose their kind prefix. */
export function shotFilename(sourceIds: string[], d: Date, ext: string): string {
  const names = sourceIds.map(id => id.replace(/^[a-z]+:/, ""));
  const src = (names.length ? names.join("+") : "view").replace(/[^A-Za-z0-9+._-]+/g, "-").replace(/^-+|-+$/g, "").slice(0, 60) || "view";
  return `adt4_${src}_${stamp(d)}.${ext}`;
}

/** A "nice" scale-bar length (1/2/5 x 10^k metres) close to `target` metres. */
export function niceLength(target: number): number {
  if (!(target > 0) || !Number.isFinite(target)) return 1;
  const p = 10 ** Math.floor(Math.log10(target));
  const f = target / p;
  return (f >= 5 ? 5 : f >= 2 ? 2 : 1) * p;
}

// ---------------------------------------------------------------- regions

/** Footprint points of a member place: its ray boundary, else bbox corners,
 *  else a small square around its position (pad metres). */
export function placeFootprint(
  n: {pos: [number, number, number]; bbox?: number[] | null; radii?: ArrayLike<number> | null},
  pad = 0.35,
): [number, number][] {
  const out: [number, number][] = [];
  if (n.radii && n.radii.length) {
    const N = n.radii.length;
    for (let i = 0; i < N; i++) {
      const a = (i / N) * 2 * Math.PI;
      out.push([n.pos[0] + n.radii[i] * Math.cos(a), n.pos[1] + n.radii[i] * Math.sin(a)]);
    }
  } else if (n.bbox && n.bbox.length >= 6 && n.bbox[3] - n.bbox[0] > 1e-3) {
    const b = n.bbox;
    out.push([b[0], b[1]], [b[3], b[1]], [b[3], b[4]], [b[0], b[4]]);
  } else {
    const [x, y] = n.pos;
    out.push([x - pad, y - pad], [x + pad, y - pad], [x + pad, y + pad], [x - pad, y + pad]);
  }
  return out;
}

/** The area covered by a region's member footprints (their union, gaps up to one
 *  cell closed), traced by marching squares on a ``cell``-metre grid: fill
 *  triangles (x, y per vertex) and outline segments (x, y per end). Unlike the
 *  convex hull it keeps a road's shape, holes (a loop around a building) and
 *  separate parts. */
export function regionFootprint(rings: [number, number][][], cell = 0.5): {tris: number[]; segs: number[]} {
  let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity;
  for (const ring of rings)
    for (const [x, y] of ring) {
      if (!Number.isFinite(x) || !Number.isFinite(y)) continue;
      x0 = Math.min(x0, x);
      y0 = Math.min(y0, y);
      x1 = Math.max(x1, x);
      y1 = Math.max(y1, y);
    }
  if (!(x1 > x0 || y1 > y0)) return {tris: [], segs: []};
  const pad = 3; // cells of empty margin, so every contour closes inside the grid
  const ox = x0 - pad * cell;
  const oy = y0 - pad * cell;
  const W = Math.ceil((x1 - x0) / cell) + 2 * pad + 1;
  const H = Math.ceil((y1 - y0) / cell) + 2 * pad + 1;
  if (W * H > 16e6) return {tris: [], segs: []};
  let grid = new Uint8Array(W * H); // inside samples at the grid vertices
  for (const ring of rings) {
    if (ring.length < 3) continue;
    let rx0 = Infinity, ry0 = Infinity, rx1 = -Infinity, ry1 = -Infinity;
    for (const [x, y] of ring) {
      rx0 = Math.min(rx0, x);
      ry0 = Math.min(ry0, y);
      rx1 = Math.max(rx1, x);
      ry1 = Math.max(ry1, y);
    }
    if (!Number.isFinite(rx0 + ry0 + rx1 + ry1)) continue;
    for (let j = Math.ceil((ry0 - oy) / cell); j <= Math.floor((ry1 - oy) / cell); j++) {
      const py = oy + j * cell;
      for (let i = Math.ceil((rx0 - ox) / cell); i <= Math.floor((rx1 - ox) / cell); i++) {
        if (grid[j * W + i]) continue;
        const px = ox + i * cell;
        let inside = false;
        for (let a = 0, b = ring.length - 1; a < ring.length; b = a++) {
          const [ax, ay] = ring[a];
          const [bx, by] = ring[b];
          if (ay > py !== by > py && px < ((bx - ax) * (py - ay)) / (by - ay) + ax) inside = !inside;
        }
        if (inside) grid[j * W + i] = 1;
      }
    }
  }
  // closing (dilate then erode, 8-neighbourhood): fuses footprints a cell apart
  const morph = (src: Uint8Array, dilate: boolean) => {
    const out = new Uint8Array(W * H);
    for (let j = 1; j < H - 1; j++)
      for (let i = 1; i < W - 1; i++) {
        let v = dilate ? 0 : 1;
        for (let dj = -1; dj <= 1; dj++)
          for (let di = -1; di <= 1; di++) {
            const s = src[(j + dj) * W + i + di];
            v = dilate ? v | s : v & s;
          }
        out[j * W + i] = v;
      }
    return out;
  };
  grid = morph(morph(grid, true), false);
  const tris: number[] = [];
  const segs: number[] = [];
  const poly: [number, number][] = [];
  const crossed: number[] = [];
  // per cell: corners c0..c3 = (i,j) (i+1,j) (i+1,j+1) (i,j+1); edge e runs from corner e to e+1
  for (let j = 0; j < H - 1; j++)
    for (let i = 0; i < W - 1; i++) {
      const c = [grid[j * W + i], grid[j * W + i + 1], grid[(j + 1) * W + i + 1], grid[(j + 1) * W + i]];
      const k = c[0] | (c[1] << 1) | (c[2] << 2) | (c[3] << 3);
      if (k === 0) continue;
      const X = ox + i * cell;
      const Y = oy + j * cell;
      if (k === 15) {
        tris.push(X, Y, X + cell, Y, X + cell, Y + cell, X, Y, X + cell, Y + cell, X, Y + cell);
        continue;
      }
      const h = cell / 2;
      const corner: [number, number][] = [[X, Y], [X + cell, Y], [X + cell, Y + cell], [X, Y + cell]];
      const mid: [number, number][] = [[X + h, Y], [X + cell, Y + h], [X + h, Y + cell], [X, Y + h]];
      // the cell's inside part: inside corners plus the midpoints of crossed edges, in
      // perimeter order (a saddle is joined across the diagonal; the polygon stays convex)
      poly.length = 0;
      crossed.length = 0;
      for (let e = 0; e < 4; e++) {
        if (c[e]) poly.push(corner[e]);
        if (c[e] !== c[(e + 1) % 4]) {
          poly.push(mid[e]);
          crossed.push(e);
        }
      }
      for (let t = 1; t + 1 < poly.length; t++) tris.push(poly[0][0], poly[0][1], poly[t][0], poly[t][1], poly[t + 1][0], poly[t + 1][1]);
      // boundary: an edge leaving the inside joins the next crossed edge, cutting off the outside corners between
      for (let a = 0; a < crossed.length; a++) {
        const e = crossed[a];
        if (c[(e + 1) % 4]) continue;
        const f = crossed[(a + 1) % crossed.length];
        segs.push(mid[e][0], mid[e][1], mid[f][0], mid[f][1]);
      }
    }
  return {tris, segs};
}

export const REGION_FILL_ALPHA = 0.3;
