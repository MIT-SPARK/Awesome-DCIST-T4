// Viewer pane layout: splitter widths and floating (popped-out) side-panel
// tabs. Pure state + geometry helpers, persisted per browser in localStorage.
// Every storage access is wrapped: the layout falls back to defaults when
// storage is blocked, empty or holds junk.

export type PaneTab = "inspect" | "images" | "map";
/** "map": the active map (LiveMapPanel) */
export const PANE_TABS: PaneTab[] = ["inspect", "images", "map"];

export interface WinRect {
  x: number;
  y: number;
  w: number;
  h: number;
}

export interface FloatWin extends WinRect {
  /** collapsed to its title bar */
  min: boolean;
}

export interface PaneLayout {
  leftW: number;
  rightW: number;
  /** popped-out tabs; absent = docked in the right panel */
  windows: Partial<Record<PaneTab, FloatWin>>;
  /** stacking order of floating windows, back to front */
  order: PaneTab[];
}

export const LAYOUT_KEY = "adt4.viewer.layout.v1";
export const LEFT = {def: 280, min: 200, max: 560};
export const RIGHT = {def: 380, min: 280, max: 960};
/** the canvas keeps at least this much width between the docked panels */
export const MIN_CANVAS = 320;
export const WIN_MIN = {w: 260, h: 160};
/** collapsed (minimized) window height: the title bar */
export const TITLE_H = 32;
export const SNAP_PX = 14;
/** floating windows only above this viewport width (below: docked / bottom sheet) */
export const FLOAT_MQ = "(min-width: 901px)";

/** Default floating size per tab. */
const DEFAULT_SIZE: Record<PaneTab, {w: number; h: number}> = {
  inspect: {w: 380, h: 520},
  images: {w: 560, h: 640},
  map: {w: 380, h: 420},
};

export const defaultLayout = (): PaneLayout => ({leftW: LEFT.def, rightW: RIGHT.def, windows: {}, order: []});

const clamp = (v: number, lo: number, hi: number) => Math.min(Math.max(v, lo), Math.max(lo, hi));
const num = (v: unknown): v is number => typeof v === "number" && Number.isFinite(v);

/** Validate untrusted (stored) data into a layout; anything malformed falls back to defaults. */
export function sanitizeLayout(raw: unknown): PaneLayout {
  const out = defaultLayout();
  if (!raw || typeof raw !== "object") return out;
  const r = raw as Record<string, unknown>;
  if (num(r.leftW)) out.leftW = clamp(Math.round(r.leftW), LEFT.min, LEFT.max);
  if (num(r.rightW)) out.rightW = clamp(Math.round(r.rightW), RIGHT.min, RIGHT.max);
  const wins = r.windows && typeof r.windows === "object" ? (r.windows as Record<string, unknown>) : {};
  for (const t of PANE_TABS) {
    const w = wins[t] as Record<string, unknown> | undefined;
    if (!w || typeof w !== "object" || ![w.x, w.y, w.w, w.h].every(num)) continue;
    out.windows[t] = {
      x: Math.round(w.x as number),
      y: Math.round(w.y as number),
      w: Math.max(WIN_MIN.w, Math.round(w.w as number)),
      h: Math.max(WIN_MIN.h, Math.round(w.h as number)),
      min: w.min === true,
    };
  }
  const order = Array.isArray(r.order) ? (r.order as unknown[]).filter((t): t is PaneTab => PANE_TABS.includes(t as PaneTab)) : [];
  out.order = [...new Set(order)].filter(t => out.windows[t]);
  for (const t of PANE_TABS) if (out.windows[t] && !out.order.includes(t)) out.order.push(t);
  return out;
}

export function loadLayout(storage: Pick<Storage, "getItem"> | null | undefined = safeStorage()): PaneLayout {
  try {
    const s = storage?.getItem(LAYOUT_KEY);
    return s ? sanitizeLayout(JSON.parse(s)) : defaultLayout();
  } catch {
    return defaultLayout();
  }
}

export function saveLayout(l: PaneLayout, storage: Pick<Storage, "setItem"> | null | undefined = safeStorage()): boolean {
  try {
    storage?.setItem(LAYOUT_KEY, JSON.stringify(l));
    return !!storage;
  } catch {
    return false;
  }
}

export function clearLayout(storage: Pick<Storage, "removeItem"> | null | undefined = safeStorage()) {
  try {
    storage?.removeItem(LAYOUT_KEY);
  } catch {
    /* storage unavailable */
  }
}

/** window.localStorage, or null where even touching it throws (sandboxed / privacy modes). */
export function safeStorage(): Storage | null {
  try {
    return typeof window !== "undefined" ? window.localStorage : null;
  } catch {
    return null;
  }
}

/** Docked panel widths for a viewer `total` px wide: each within its own
 *  bounds, and together leaving the canvas at least MIN_CANVAS. The panel
 *  being dragged (`prefer`) keeps its width; the other one gives way. */
export function clampWidths(leftW: number, rightW: number, total: number, leftOpen: boolean, rightOpen: boolean, prefer: "left" | "right" = "right"): {leftW: number; rightW: number} {
  let l = clamp(leftW, LEFT.min, LEFT.max);
  let r = clamp(rightW, RIGHT.min, RIGHT.max);
  const room = total - MIN_CANVAS;
  if (leftOpen && rightOpen) {
    if (prefer === "right") {
      r = clamp(r, RIGHT.min, room - LEFT.min);
      l = clamp(l, LEFT.min, room - r);
    } else {
      l = clamp(l, LEFT.min, room - RIGHT.min);
      r = clamp(r, RIGHT.min, room - l);
    }
  } else if (leftOpen) l = clamp(l, LEFT.min, room);
  else if (rightOpen) r = clamp(r, RIGHT.min, room);
  return {leftW: l, rightW: r};
}

/** Keep a window fully inside a `bw`×`bh` area: shrink it if it is larger than
 *  the area, then move it in. A minimized window only needs its title bar in. */
export function clampRect(r: WinRect, bw: number, bh: number, minimized = false): WinRect {
  const w = clamp(r.w, Math.min(WIN_MIN.w, bw), bw);
  const h = clamp(r.h, Math.min(WIN_MIN.h, bh), bh);
  const vh = minimized ? TITLE_H : h;
  return {w, h, x: clamp(r.x, 0, bw - w), y: clamp(r.y, 0, bh - vh)};
}

/** Snap a window's edges to the area edges when within SNAP_PX. */
export function snapRect(r: WinRect, bw: number, bh: number, px = SNAP_PX): WinRect {
  let {x, y} = r;
  if (Math.abs(x) <= px) x = 0;
  else if (Math.abs(bw - (x + r.w)) <= px) x = bw - r.w;
  if (Math.abs(y) <= px) y = 0;
  else if (Math.abs(bh - (y + r.h)) <= px) y = bh - r.h;
  return {...r, x, y};
}

export type Edge = "n" | "s" | "e" | "w" | "ne" | "nw" | "se" | "sw";
export const EDGES: Edge[] = ["n", "s", "e", "w", "ne", "nw", "se", "sw"];

/** Resize `start` by a pointer delta from the given edge/corner, honouring the
 *  minimum size (the opposite edge stays put) and the area bounds. */
export function resizeRect(start: WinRect, edge: Edge, dx: number, dy: number, bw: number, bh: number): WinRect {
  let {x, y, w, h} = start;
  if (edge.includes("e")) w = clamp(start.w + dx, WIN_MIN.w, bw - start.x);
  if (edge.includes("s")) h = clamp(start.h + dy, WIN_MIN.h, bh - start.y);
  if (edge.includes("w")) {
    const right = start.x + start.w;
    x = clamp(start.x + dx, 0, right - WIN_MIN.w);
    w = right - x;
  }
  if (edge.includes("n")) {
    const bottom = start.y + start.h;
    y = clamp(start.y + dy, 0, bottom - WIN_MIN.h);
    h = bottom - y;
  }
  return {x, y, w, h};
}

/** Pop a tab out into a floating window: reuse its last rectangle, else a
 *  default size cascaded from the top-right of the area (or centred on `at`). */
export function popOut(l: PaneLayout, tab: PaneTab, bw: number, bh: number, at?: {x: number; y: number}): PaneLayout {
  const prev = l.windows[tab];
  const size = DEFAULT_SIZE[tab];
  const n = l.order.length;
  let rect: WinRect = prev ?? {w: size.w, h: size.h, x: bw - size.w - 24 - n * 28, y: 56 + n * 28};
  if (at) rect = {...rect, x: at.x - rect.w / 2, y: at.y - TITLE_H / 2};
  const c = clampRect(rect, bw, bh);
  const win: FloatWin = {x: Math.round(c.x), y: Math.round(c.y), w: Math.round(c.w), h: Math.round(c.h), min: false};
  return {...l, windows: {...l.windows, [tab]: win}, order: [...l.order.filter(t => t !== tab), tab]};
}

export function dock(l: PaneLayout, tab: PaneTab): PaneLayout {
  if (!l.windows[tab]) return l;
  const windows = {...l.windows};
  delete windows[tab];
  return {...l, windows, order: l.order.filter(t => t !== tab)};
}

export function raise(l: PaneLayout, tab: PaneTab): PaneLayout {
  if (!l.windows[tab] || l.order[l.order.length - 1] === tab) return l;
  return {...l, order: [...l.order.filter(t => t !== tab), tab]};
}

export function updateWin(l: PaneLayout, tab: PaneTab, patch: Partial<FloatWin>): PaneLayout {
  const w = l.windows[tab];
  if (!w) return l;
  return {...l, windows: {...l.windows, [tab]: {...w, ...patch}}};
}
