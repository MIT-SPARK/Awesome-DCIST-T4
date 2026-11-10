/** Maps page: search + sort over the runs of every machine (pure, unit tested). */
import type {MapEntry} from "../types";
import {fuzzyMatch, type FuzzyMatch} from "./fuzzy";

export interface MapRow {
  machine: string;
  map: MapEntry;
}

export type MatchField = "name" | "machine" | "path";

export interface SearchHit extends MapRow {
  score: number;
  /** highlight positions per field (union over query tokens) */
  hits: Partial<Record<MatchField, number[]>>;
}

// a match in the run name counts more than the same match in the machine or path
const FIELD_WEIGHT: Record<MatchField, number> = {name: 1, machine: 0.8, path: 0.6};

/**
 * Fuzzy search: every whitespace-separated token must match the run name, machine or
 * path (best field wins); score = sum of token scores. Empty query keeps every row
 * with score 0.
 */
export function searchMaps(rows: MapRow[], query: string): SearchHit[] {
  const tokens = query.trim().split(/\s+/).filter(Boolean);
  const out: SearchHit[] = [];
  for (const row of rows) {
    if (!tokens.length) {
      out.push({...row, score: 0, hits: {}});
      continue;
    }
    const fields: Record<MatchField, string> = {name: row.map.name, machine: row.machine, path: row.map.path ?? ""};
    let total = 0;
    const hits: SearchHit["hits"] = {};
    let ok = true;
    for (const tok of tokens) {
      let best: {field: MatchField; m: FuzzyMatch; s: number} | null = null;
      for (const f of ["name", "machine", "path"] as MatchField[]) {
        if (!fields[f]) continue;
        const m = fuzzyMatch(tok, fields[f]);
        if (!m) continue;
        const s = m.score * FIELD_WEIGHT[f];
        if (!best || s > best.s) best = {field: f, m, s};
      }
      if (!best) {
        ok = false;
        break;
      }
      total += best.s;
      hits[best.field] = [...(hits[best.field] ?? []), ...best.m.indices];
    }
    if (ok) out.push({...row, score: total, hits});
  }
  return out;
}

export type SortKey = "relevance" | "newest" | "oldest" | "size" | "name" | "machine" | "keyframes";

export const SORT_LABELS: Record<SortKey, string> = {
  relevance: "Best match",
  newest: "Newest first",
  oldest: "Oldest first",
  size: "Largest first",
  name: "Name A-Z",
  machine: "Machine",
  keyframes: "Most keyframes",
};

/** Run time for sorting: directory mtime, else a date parsed from the name. */
export function mapTime(m: MapEntry): number | null {
  return m.mtime ?? m.name_date ?? null;
}

export function keyframes(m: MapEntry): number {
  return m.image_store?.frames?.keyframe ?? 0;
}

const byName = (a: SearchHit, b: SearchHit) => a.map.name.localeCompare(b.map.name, undefined, {numeric: true});
const byNewest = (a: SearchHit, b: SearchHit) => (mapTime(b.map) ?? -Infinity) - (mapTime(a.map) ?? -Infinity);

/** Sorted copy. Unknown values (no size, no date) sink to the bottom. */
export function sortMaps(rows: SearchHit[], key: SortKey, query = ""): SearchHit[] {
  const r = [...rows];
  const tie = (a: SearchHit, b: SearchHit) => byNewest(a, b) || byName(a, b) || a.machine.localeCompare(b.machine);
  const cmp: Record<SortKey, (a: SearchHit, b: SearchHit) => number> = {
    relevance: (a, b) => (query.trim() ? b.score - a.score : 0) || tie(a, b),
    newest: (a, b) => byNewest(a, b) || byName(a, b),
    oldest: (a, b) => {
      const ta = mapTime(a.map);
      const tb = mapTime(b.map);
      if (ta === null || tb === null) return ta === tb ? byName(a, b) : ta === null ? 1 : -1;
      return ta - tb || byName(a, b);
    },
    size: (a, b) => (b.map.size_bytes ?? -1) - (a.map.size_bytes ?? -1) || tie(a, b),
    name: (a, b) => byName(a, b) || a.machine.localeCompare(b.machine),
    machine: (a, b) => a.machine.localeCompare(b.machine) || tie(a, b),
    keyframes: (a, b) => keyframes(b.map) - keyframes(a.map) || tie(a, b),
  };
  return r.sort(cmp[key] ?? cmp.newest);
}

/** A run missing a hydra backend output (dsg.json, dsg_with_mesh.json or
 *  mesh.ply): crashed, still running, or stopped before hydra saved. Unknown
 *  (an older backend that does not report the files) counts as complete. */
export function isIncomplete(map: MapEntry): boolean {
  if (map.has_dsg === undefined) return false;
  return !map.has_dsg || !map.has_dsg_mesh || !map.has_mesh;
}

// ---- remembered view settings (localStorage may be unavailable) ----

export interface MapsPrefs {
  sort: SortKey;
  grouped: boolean;
  hideIncomplete: boolean;
}

const PREFS_KEY = "adt4.maps.prefs";
export const DEFAULT_PREFS: MapsPrefs = {sort: "newest", grouped: true, hideIncomplete: false};

export function loadPrefs(): MapsPrefs {
  try {
    const raw = localStorage.getItem(PREFS_KEY);
    if (!raw) return DEFAULT_PREFS;
    const p = JSON.parse(raw) as Partial<MapsPrefs>;
    return {
      sort: p.sort && p.sort in SORT_LABELS ? p.sort : DEFAULT_PREFS.sort,
      grouped: typeof p.grouped === "boolean" ? p.grouped : DEFAULT_PREFS.grouped,
      hideIncomplete: typeof p.hideIncomplete === "boolean" ? p.hideIncomplete : DEFAULT_PREFS.hideIncomplete,
    };
  } catch {
    return DEFAULT_PREFS;
  }
}

export function savePrefs(p: MapsPrefs): void {
  try {
    localStorage.setItem(PREFS_KEY, JSON.stringify(p));
  } catch {
    /* private window / blocked storage: keep the in-memory choice only */
  }
}
