// Persisted (per browser) map-viewer styling: background, colour overrides,
// per-layer heights, region display and screenshot options.
import {create} from "zustand";
import {
  DEFAULT_SHOT,
  type ColorKey,
  type ColorRule,
  type Heights,
  type MeshColorMode,
  type ShotOpts,
} from "../viewer/style";

export interface ViewStyleState {
  /** null = the viz style's default background */
  background: string | null;
  colors: Partial<Record<ColorKey, ColorRule>>;
  labelColors: Record<string, string>;
  meshColor: {mode: MeshColorMode; color: string};
  heights: Heights;
  /** Layers › "Colour members by area": places with a parent area (Room) take its
   *  colour */
  regionColorPlaces: boolean;
  /** Layers › "Area boundaries": fill + outline of the ground each Room's places cover, or
   *  its drawn polygon */
  regionHulls: boolean;
  /** the ground grid under the map */
  grid: boolean;
  /** opacity of the map's robot trajectories (0.1..1) */
  trajectoryOpacity: number;
  shot: ShotOpts;
}

export const STYLE_STORAGE_KEY = "adt4.viewer.style.v1";

export const DEFAULT_VIEW_STYLE: ViewStyleState = {
  background: null,
  colors: {},
  labelColors: {},
  meshColor: {mode: "rgb", color: "#b0b0b0"},
  heights: {},
  regionColorPlaces: true,
  regionHulls: true,
  grid: true,
  trajectoryOpacity: 1,
  shot: DEFAULT_SHOT,
};

const PERSISTED = Object.keys(DEFAULT_VIEW_STYLE) as (keyof ViewStyleState)[];

const isObj = (v: unknown): v is Record<string, unknown> => !!v && typeof v === "object" && !Array.isArray(v);

/** Parse stored settings, keeping only well-typed fields over the defaults. */
export function parseStoredStyle(raw: string | null): ViewStyleState {
  const out: ViewStyleState = structuredClone(DEFAULT_VIEW_STYLE);
  if (!raw) return out;
  let v: unknown;
  try {
    v = JSON.parse(raw);
  } catch {
    return out;
  }
  if (!isObj(v)) return out;
  if (typeof v.background === "string" || v.background === null) out.background = v.background as string | null;
  if (isObj(v.colors)) {
    for (const [k, r] of Object.entries(v.colors))
      if (isObj(r) && typeof r.mode === "string" && typeof r.color === "string") out.colors[k as ColorKey] = {mode: r.mode as ColorRule["mode"], color: r.color};
  }
  if (isObj(v.labelColors)) for (const [k, c] of Object.entries(v.labelColors)) if (typeof c === "string") out.labelColors[k] = c;
  if (isObj(v.meshColor) && typeof v.meshColor.mode === "string" && typeof v.meshColor.color === "string")
    out.meshColor = {mode: v.meshColor.mode as MeshColorMode, color: v.meshColor.color};
  if (isObj(v.heights)) for (const [k, h] of Object.entries(v.heights)) if (typeof h === "number" && Number.isFinite(h)) out.heights[k as keyof Heights] = h;
  if (typeof v.regionColorPlaces === "boolean") out.regionColorPlaces = v.regionColorPlaces;
  if (typeof v.regionHulls === "boolean") out.regionHulls = v.regionHulls;
  if (typeof v.grid === "boolean") out.grid = v.grid;
  const num = (x: unknown, lo: number, hi: number) => (typeof x === "number" && Number.isFinite(x) ? Math.min(hi, Math.max(lo, x)) : null);
  out.trajectoryOpacity = num(v.trajectoryOpacity, 0.1, 1) ?? out.trajectoryOpacity;
  if (isObj(v.shot)) {
    const s = v.shot;
    out.shot = {
      scale: s.scale === 2 ? 2 : 1,
      format: s.format === "jpeg" ? "jpeg" : "png",
      quality: typeof s.quality === "number" && s.quality > 0 && s.quality <= 1 ? s.quality : DEFAULT_SHOT.quality,
      transparent: s.transparent === true,
      legend: s.legend !== false,
      labels: s.labels !== false,
    };
  }
  return out;
}

function load(): ViewStyleState {
  try {
    return parseStoredStyle(window.localStorage.getItem(STYLE_STORAGE_KEY));
  } catch {
    return structuredClone(DEFAULT_VIEW_STYLE);
  }
}

export function saveStyle(s: ViewStyleState): void {
  try {
    const out: Record<string, unknown> = {};
    for (const k of PERSISTED) out[k] = s[k];
    window.localStorage.setItem(STYLE_STORAGE_KEY, JSON.stringify(out));
  } catch {
    /* private window / blocked storage: settings last for this page only */
  }
}

interface ViewStyleStore extends ViewStyleState {
  patch: (p: Partial<ViewStyleState>) => void;
  setRule: (k: ColorKey, r: Partial<ColorRule> | null) => void;
  setHeight: (k: keyof Heights, h: number | null) => void;
  setLabelColor: (name: string, hex: string | null) => void;
  resetColors: () => void;
  resetHeights: () => void;
}

export const useViewStyle = create<ViewStyleStore>(set => ({
  ...load(),
  patch: p => set(p),
  setRule: (k, r) =>
    set(s => {
      const colors = {...s.colors};
      if (r === null) delete colors[k];
      else colors[k] = {mode: "default", color: "#808080", ...s.colors[k], ...r};
      return {colors};
    }),
  setHeight: (k, h) =>
    set(s => {
      const heights = {...s.heights};
      if (h === null) delete heights[k];
      else heights[k] = h;
      return {heights};
    }),
  setLabelColor: (name, hex) =>
    set(s => {
      const labelColors = {...s.labelColors};
      if (hex === null) delete labelColors[name];
      else labelColors[name] = hex;
      return {labelColors};
    }),
  resetColors: () => set({colors: {}, labelColors: {}, meshColor: {...DEFAULT_VIEW_STYLE.meshColor}}),
  resetHeights: () => set({heights: {}}),
}));

let saveTimer: number | null = null;
useViewStyle.subscribe(s => {
  if (typeof window === "undefined") return;
  if (saveTimer !== null) window.clearTimeout(saveTimer);
  saveTimer = window.setTimeout(() => {
    saveTimer = null;
    saveStyle(s);
  }, 150);
});
