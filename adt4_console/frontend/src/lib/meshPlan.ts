// Which mesh tiles to fetch, at which level of detail, in which order
// (viewer/MeshTiles.tsx). The manifest (API.md §7) lists every tile at each
// level with its gzipped size and etag; the viewer shows a coarse level of
// every tile first (the whole map in a few hundred kB), then refines the tiles
// nearest the camera target to the wanted detail.
import type {TileManifest} from "../types";

export type MeshDetail = "auto" | "full" | "medium" | "coarse";
export const DETAIL_LEVEL: Record<Exclude<MeshDetail, "auto">, number> = {full: 0, medium: 1, coarse: 2};

/** The mesh detail "auto" means: a phone gets the medium level (a fifth of the
 *  full mesh's bytes and triangles); a desktop viewer the full mesh. */
export function autoDetail(phone: boolean): Exclude<MeshDetail, "auto"> {
  return phone ? "medium" : "full";
}

const DETAIL_KEY = "adt4.meshDetail";

/** The mesh detail picked on this device (localStorage; "auto" when unset or unavailable). */
export function loadDetail(): MeshDetail {
  try {
    const v = localStorage.getItem(DETAIL_KEY);
    return v === "full" || v === "medium" || v === "coarse" ? v : "auto";
  } catch {
    return "auto";
  }
}

export function saveDetail(d: MeshDetail) {
  try {
    if (d === "auto") localStorage.removeItem(DETAIL_KEY);
    else localStorage.setItem(DETAIL_KEY, d);
  } catch {
    /* private mode: this session only */
  }
}

/** Levels the manifest offers (ascending; [0] for a manifest without levels). */
export function manifestLevels(m: TileManifest | null): number[] {
  const lv = (m?.lods ?? []).map(l => l.level).sort((a, b) => a - b);
  return lv.length ? lv : [0];
}

/** The level to show for a wanted one: the coarsest offered level that is at
 *  least as fine as wanted, else the finest offered. */
export function resolveLevel(levels: number[], want: number): number {
  const ok = levels.filter(l => l <= want);
  return ok.length ? Math.max(...ok) : Math.min(...levels);
}

export interface TileStep {
  key: string;
  level: number;
  /** gzipped bytes on the wire (0 when the manifest does not say) */
  bytes: number;
  etag: string;
  /** a coarse stand-in shown until the target level arrives */
  preview: boolean;
}

export interface Shown {
  level: number;
  etag: string;
}

/** A tile's row for `level`: [bytes, etag], null = empty at that level,
 *  undefined = the manifest has no levels (legacy: full tile, hash as etag). */
export function tileRow(m: TileManifest, key: string, level: number): [number, string] | null | undefined {
  const t = m.tiles[key];
  if (!t) return null;
  if (!t.lod || !m.lods?.length) return level === 0 ? [0, t.hash] : null;
  const i = m.lods.findIndex(l => l.level === level);
  if (i < 0) return null;
  return (t.lod[i] as [number, string] | null) ?? null;
}

/**
 * The fetches tile `key` still needs to show `target` (in order), and whether
 * the tile must be cleared (it is empty at the target level).
 */
export function planTile(m: TileManifest, key: string, target: number, shown: Shown | undefined): {steps: TileStep[]; clear: boolean} {
  const levels = manifestLevels(m);
  const row = tileRow(m, key, target);
  if (!row) return {steps: [], clear: !!shown};
  const [bytes, etag] = row;
  if (shown && shown.level === target && shown.etag === etag) return {steps: [], clear: false};
  const steps: TileStep[] = [];
  const coarsest = Math.max(...levels);
  if (!shown && coarsest > target) {
    const p = tileRow(m, key, coarsest);
    if (p) steps.push({key, level: coarsest, bytes: p[0], etag: p[1], preview: true});
  }
  steps.push({key, level: target, bytes, etag, preview: false});
  return {steps, clear: false};
}

/** Whether an arrived tile should be shown: the planned target, or a preview
 *  while the tile shows nothing yet (never over a finer, even stale, tile). */
export function wanted(m: TileManifest, step: TileStep, target: number, shown: Shown | undefined): boolean {
  const row = tileRow(m, step.key, step.level);
  if (!row || row[1] !== step.etag) return false;
  if (step.level === target) return true;
  return step.preview && !shown;
}

/** Gzipped bytes of all tiles at the level shown for `want` (0 if unknown). */
export function levelBytes(m: TileManifest | null, want: number): number {
  if (!m || m.enc !== 2) return 0;
  const level = resolveLevel(manifestLevels(m), want);
  let n = 0;
  for (const k of Object.keys(m.tiles)) n += tileRow(m, k, level)?.[0] ?? 0;
  return n;
}

/** Next step to fetch: previews before refinements, then nearest the camera
 *  target (squared distance from `dist2`). Returns the index into `heads`. */
export function nextStep(heads: TileStep[], dist2: (key: string) => number): number {
  let best = -1;
  let bestP = Infinity;
  let bestD = Infinity;
  heads.forEach((s, i) => {
    const p = s.preview ? 0 : 1;
    const d = dist2(s.key);
    if (p < bestP || (p === bestP && d < bestD)) {
      best = i;
      bestP = p;
      bestD = d;
    }
  });
  return best;
}
