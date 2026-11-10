// Keyframes that observed a traversability place (GET /api/graph/{src}/place/{sym}/keyframes,
// API.md §7): sorting and grouping by session for the Inspect tab (viewer/PlaceKeyframes.tsx).

export type PlaceKfSort = "newest" | "oldest" | "visible";

export interface PlaceKeyframe {
  frame_id: number;
  /** agent (robot pose) node the keyframe is anchored to */
  agent: string | null;
  /** trajectory robot prefix (groups by session) */
  robot_key: string;
  t_ns: number;
  /** fraction of the place's 9 sample points the keyframe saw (its weight) */
  visible: number;
  /** camera to place centre (m) */
  dist_m: number;
  image: string;
}

export interface PlaceKfSession {
  key: string;
  session?: string | null;
  robot?: string | null;
  bag?: string | null;
  bag_short?: string | null;
  n: number;
  first_ns: number;
  last_ns: number;
  date?: string | null;
}

export interface PlaceKeyframes {
  src: string;
  sym: string;
  version: number;
  sort: PlaceKfSort;
  n: number;
  keyframes: PlaceKeyframe[];
  sessions: PlaceKfSession[];
  keyframes_total: number;
  table: "ready" | "building";
  seconds: number;
}

/** A copy of ``kfs`` in ``sort`` order (the server's order, so a sort switch needs no request). */
export function sortKeyframes(kfs: PlaceKeyframe[], sort: PlaceKfSort): PlaceKeyframe[] {
  const out = kfs.slice();
  if (sort === "visible") out.sort((a, b) => b.visible - a.visible || a.dist_m - b.dist_m || b.t_ns - a.t_ns);
  else if (sort === "oldest") out.sort((a, b) => a.t_ns - b.t_ns || a.frame_id - b.frame_id);
  else out.sort((a, b) => b.t_ns - a.t_ns || b.frame_id - a.frame_id);
  return out;
}

export interface KfGroup {
  session: PlaceKfSession;
  items: PlaceKeyframe[];
}

/** Sessions with their keyframes, each group in ``sort`` order. Groups follow the
 *  session start (newest session first for "newest", oldest first for "oldest", the
 *  best-seen session first for "visible"). Robots the server names no session for get
 *  a bare one. */
export function groupBySession(kfs: PlaceKeyframe[], sessions: PlaceKfSession[], sort: PlaceKfSort): KfGroup[] {
  const byKey = new Map<string, KfGroup>();
  for (const k of sortKeyframes(kfs, sort)) {
    let g = byKey.get(k.robot_key);
    if (!g) {
      const s = sessions.find(x => x.key === k.robot_key);
      g = {session: {...(s ?? {key: k.robot_key}), n: 0, first_ns: k.t_ns, last_ns: k.t_ns}, items: []};
      byKey.set(k.robot_key, g);
    }
    g.items.push(k);
    g.session.n = g.items.length;
    g.session.first_ns = Math.min(g.session.first_ns, k.t_ns);
    g.session.last_ns = Math.max(g.session.last_ns, k.t_ns);
  }
  const groups = [...byKey.values()];
  if (sort === "visible") groups.sort((a, b) => b.items[0].visible - a.items[0].visible || a.session.first_ns - b.session.first_ns);
  else if (sort === "oldest") groups.sort((a, b) => a.session.first_ns - b.session.first_ns);
  else groups.sort((a, b) => b.session.first_ns - a.session.first_ns);
  return groups;
}

/** Keyframe times are unix ns on recorded maps; small values are not dates. */
const isClock = (t_ns: number) => t_ns > 1e17;

function pad(n: number) {
  return String(n).padStart(2, "0");
}

/** "2026-10-22" (local date) or "" when the time is no date. */
export function kfDate(t_ns: number): string {
  if (!isClock(t_ns)) return "";
  const d = new Date(t_ns / 1e6);
  return `${d.getFullYear()}-${pad(d.getMonth() + 1)}-${pad(d.getDate())}`;
}

/** "14:03:12" (local time) or "#<seconds>" when the time is no date. */
export function kfClock(t_ns: number): string {
  if (!isClock(t_ns)) return `#${Math.round(t_ns / 1e9)}`;
  const d = new Date(t_ns / 1e6);
  return `${pad(d.getHours())}:${pad(d.getMinutes())}:${pad(d.getSeconds())}`;
}

/** "hamilton · hard_robot_2_hamilton": robot and bag short name (else the session, unless it
 *  is the robot's name), or "robot b" when the server names neither. */
export function sessionName(s: PlaceKfSession): string {
  const parts = [s.robot ?? "", s.bag_short ?? (s.session !== s.robot ? s.session ?? "" : "")].filter(Boolean);
  return parts.length ? parts.join(" · ") : `robot ${s.key}`;
}

/** Session header: "2026-10-22 · hamilton · hard_robot_2_hamilton" (the date of its first keyframe here). */
export function sessionTitle(s: PlaceKfSession): string {
  return [kfDate(s.first_ns), sessionName(s)].filter(Boolean).join(" · ");
}

/** Thumbnail badge: "78% · 6.2 m". */
export function kfBadge(k: PlaceKeyframe): string {
  return `${Math.round(k.visible * 100)}% · ${k.dist_m.toFixed(1)} m`;
}
