// Map-viewer state: overlaid graph sources (heavy data kept in mutable GraphModel
// objects, React re-renders keyed on `rev`) plus all viewer UI toggles.
import {PALETTE_HEX} from "../lib/palette";
import {create} from "zustand";
import {decodeDelta, decodeGraph, unpack} from "../codec";
import {GraphModel} from "../lib/graphModel";
import {errorText, fetchWithProgress, getJSON} from "../lib/api";
import {loadDetail, type MeshDetail} from "../lib/meshPlan";
import {beginTask, dropTask, endTask, progressTask} from "./loading";
import type {GraphSource, TileManifest} from "../types";
import type {VizStyle} from "../viewer/layout";
import {ws} from "../ws";

export interface SourceView {
  id: string;
  kind: GraphSource["kind"];
  label: string;
  enabled: boolean;
  status: "idle" | "loading" | "ready" | "error";
  error: string | null;
  model: GraphModel | null;
  rev: number;
  version: number | null;
  manifest: TileManifest | null;
  manifestRev: number;
  /** robot paths: polylines per robot, split where its keyframes have a gap */
  trajectory: TrajPath[] | null;
  /** the robots of `trajectory`, with their session / robot / source bag when known */
  trajectoryRobots: TrajRobot[];
  /** each robot's keyframes in time order (agent symbol, time, position), for browsing images */
  keyframes: KeyframeTrack[];
  showMesh: boolean;
  showGraph: boolean;
}

export interface Selection {
  src: string;
  sym: string;
}

interface ViewerStore {
  sources: Record<string, SourceView>;
  order: string[];
  catalog: GraphSource[] | null;
  catalogError: string | null;
  // display
  hiddenLayers: Record<string, boolean>; // "layer/part" -> hidden
  edgeKinds: Record<string, boolean>; // kind -> visible
  flat: boolean;
  stackHeight: number;
  showBBoxes: boolean;
  showTrajectory: boolean;
  /** robot path colour: one colour, per robot, or per bag (session) */
  pathColor: PathColorMode;
  /** label each robot's path with its session, robot and source bag */
  pathLabels: boolean;
  topDown: boolean;
  meshOpacity: number;
  /** mesh level of detail; "auto" = `meshAutoDetail` */
  meshDetail: MeshDetail;
  /** what "auto" means on this page (ViewerPage: lib/meshPlan autoDetail) */
  meshAutoDetail: Exclude<MeshDetail, "auto">;
  /** "hydra" mimics the ADT4 hydra_visualizer config; "classic" is the old look */
  vizStyle: VizStyle;
  /** node / object text labels (hydra text adapters) */
  showText: boolean;
  /** cull mesh back faces like RViz MeshDisplay (hides ceilings seen from above) */
  meshCull: boolean;
  /** clip the mesh above `meshClipHeight` metres over the floor */
  meshClipOn: boolean;
  meshClipHeight: number;
  // interaction
  selected: Selection | null;
  query: string;
  highlight: Selection[];
  flyTo: {target: [number, number, number]; nonce: number} | null;
  // actions
  loadCatalog: () => Promise<void>;
  enableSource: (id: string, on: boolean) => void;
  reloadSource: (id: string) => Promise<void>;
  patchSource: (id: string, patch: Partial<SourceView>) => void;
  set: (patch: Partial<ViewerStore>) => void;
  select: (sel: Selection | null) => void;
  fly: (target: [number, number, number]) => void;
}

const subs = new Map<string, () => void>();

function blankSource(s: GraphSource): SourceView {
  return {
    id: s.id,
    kind: s.kind,
    label: s.label || s.id,
    enabled: false,
    status: "idle",
    error: null,
    model: null,
    rev: 0,
    version: s.version ?? null,
    manifest: null,
    manifestRev: 0,
    trajectory: null,
    trajectoryRobots: [],
    keyframes: [],
    showMesh: true,
    showGraph: true,
  };
}

export const srcPath = (id: string) => `api/graph/${encodeURIComponent(id)}`;

export const useViewer = create<ViewerStore>((set, get) => ({
  sources: {},
  order: [],
  catalog: null,
  catalogError: null,
  // sub-keyframes are dense camera anchors; off by default
  hiddenLayers: {"2/115": true},
  edgeKinds: {},
  flat: false,
  stackHeight: 2, // hydra_visualizer layer_z_step
  showBBoxes: true,
  showTrajectory: true,
  pathColor: "bag",
  pathLabels: false,
  topDown: false,
  meshOpacity: 1,
  meshDetail: loadDetail(),
  meshAutoDetail: "full",
  vizStyle: "hydra",
  showText: true,
  meshCull: true,
  meshClipOn: false,
  meshClipHeight: 2.0,
  selected: null,
  query: "",
  highlight: [],
  flyTo: null,

  loadCatalog: async () => {
    try {
      const raw = await getJSON<GraphSource[] | {sources: GraphSource[]}>("api/graph/sources");
      const list = Array.isArray(raw) ? raw : raw.sources ?? [];
      set(s => {
        const sources = {...s.sources};
        for (const g of list) {
          sources[g.id] = sources[g.id]
            ? {...sources[g.id], label: g.label || g.id, kind: g.kind}
            : blankSource(g);
        }
        const order = [...new Set([...s.order.filter(id => sources[id]), ...list.map(g => g.id)])];
        return {catalog: list, catalogError: null, sources, order};
      });
    } catch (e) {
      set({catalogError: errorText(e)});
    }
  },

  patchSource: (id, patch) =>
    set(s => (s.sources[id] ? {sources: {...s.sources, [id]: {...s.sources[id], ...patch}}} : {})),

  enableSource: (id, on) => {
    const src = get().sources[id];
    if (!src) return;
    get().patchSource(id, {enabled: on});
    if (on) {
      if (!subs.has(id)) subs.set(id, subscribeSource(id));
      if (src.status === "idle" || src.status === "error") void get().reloadSource(id);
    } else {
      subs.get(id)?.();
      subs.delete(id);
    }
  },

  reloadSource: async id => {
    const {patchSource} = get();
    patchSource(id, {status: "loading", error: null});
    // the graph shows as soon as its snapshot is in: the mesh manifest (which
    // a cold server may take a while to build) and the trajectory arrive on
    // their own, each with byte progress on the load bar (stores/loading)
    // (the manifest is requested at once, but its tiles and the trajectory only
    // start once the snapshot is in: the graph gets the whole link first)
    let snapDone = () => {};
    const afterSnap = new Promise<void>(r => (snapDone = r));
    const mesh = loadManifest(id, afterSnap);
    try {
      const snap = unpack(await fetchTracked(`${srcPath(id)}/snapshot`, `graph:${id}`, "Graph", "application/x-msgpack"));
      const g = decodeGraph(snap);
      const cur = get().sources[id];
      const model = cur?.model ?? new GraphModel();
      model.loadSnapshot(g);
      model.lastSeq = g.seq ?? null;
      patchSource(id, {status: "ready", model, rev: model.rev, version: g.version ?? cur?.version ?? null});
    } catch (e) {
      patchSource(id, {status: "error", error: errorText(e)});
    }
    snapDone();
    await Promise.all([mesh, loadTrajectory(id)]);
  },

  set: patch => set(patch as Partial<ViewerStore>),
  select: sel => set({selected: sel}),
  fly: target => set(s => ({flyTo: {target, nonce: (s.flyTo?.nonce ?? 0) + 1}})),
}));

/** GET with byte progress as load-bar task `task` ("" label = bytes only). */
async function fetchTracked(path: string, task: string, label: string, accept?: string): Promise<Uint8Array> {
  beginTask(task, label);
  try {
    const body = await fetchWithProgress(path, {
      accept,
      onTotal: t => progressTask(task, 0, t),
      onBytes: b => progressTask(task, b),
    });
    endTask(task);
    return body;
  } catch (e) {
    endTask(task, false);
    throw e;
  }
}

async function loadManifest(id: string, after: Promise<void>) {
  const task = `mesh:${id}`;
  // until the tile loader takes over: the server may still be building the mesh
  beginTask(task, "Mesh", "bytes", null, "preparing");
  let manifest: TileManifest | null = null;
  try {
    manifest = await getJSON<TileManifest>(`${srcPath(id)}/mesh/manifest`);
  } catch {
    /* keep what we show */
  }
  await after;
  const cur = useViewer.getState().sources[id];
  // the tile loader (viewer/MeshTiles) reports the tiles; nothing to load ends it here
  if (!manifest || !Object.keys(manifest.tiles ?? {}).length || !cur?.showMesh || !cur.enabled) dropTask(task);
  if (!cur) return;
  useViewer.getState().patchSource(id, {
    // a failed manifest fetch keeps the tiles we already show
    manifest: manifest ?? cur.manifest ?? {tile_size: 8, tiles: {}},
    manifestRev: (cur.manifestRev ?? 0) + 1,
  });
}

type V3 = [number, number, number];

export type PathColorMode = "single" | "robot" | "bag";

/** A robot of a trajectory: its prefix key and, when the map knows them, the session,
 *  robot name and source bag (GET .../trajectory ``robots``). */
export interface TrajRobot {
  key: string;
  session?: string;
  robot?: string;
  bag?: string;
}

export interface TrajPath {
  robot: string; // TrajRobot.key
  points: V3[];
}

/** Server points cut at ``breaks`` into paths, each tagged with the robot whose
 *  ``start`` index it falls after. */
export function trajectoryPaths(points: V3[], breaks: number[], robots: {key: string; start: number}[]): TrajPath[] {
  const out: TrajPath[] = [];
  let start = 0;
  for (const b of [...breaks, points.length]) {
    if (b - start > 1) {
      const r = robots.filter(x => x.start <= start).at(-1);
      out.push({robot: r?.key ?? "", points: points.slice(start, b)});
    }
    start = b;
  }
  return out;
}

/** One robot's keyframes, time-ordered: ``t`` seconds (unix time for timestamped
 *  maps), the agent node ``syms`` (images) and positions. */
export interface KeyframeTrack {
  robot: TrajRobot;
  t: number[];
  syms: string[];
  pos: V3[];
}

/** Server trajectory points [t, x, y, z] + per-point agent symbols -> one track per robot
 *  (points without a symbol are skipped: they have no images to browse). */
export function keyframeTracks(points: number[][], syms: (string | null)[], robots: (TrajRobot & {start: number})[]): KeyframeTrack[] {
  return robots
    .map((r, k) => {
      const end = robots[k + 1]?.start ?? points.length;
      const track: KeyframeTrack = {robot: (({start: _, ...rest}) => rest)(r), t: [], syms: [], pos: []};
      for (let i = r.start; i < end; i++) {
        const s = syms[i];
        if (!s) continue;
        track.t.push(points[i][0]);
        track.syms.push(s);
        track.pos.push([points[i][1], points[i][2], points[i][3]]);
      }
      return track;
    })
    .filter(tr => tr.syms.length > 0);
}

/** Path colour per robot key (null: one colour). "bag" gives each session its own
 *  colour; "robot" uses the robot's colour, so sessions of one robot match. */
export function pathColors(robots: TrajRobot[], mode: PathColorMode, robotColor: (name: string) => string): Record<string, string> | null {
  if (mode === "single" || !robots.length) return null;
  const out: Record<string, string> = {};
  robots.forEach((r, i) => (out[r.key] = mode === "robot" ? robotColor(r.robot ?? r.key) : PALETTE_HEX[i % PALETTE_HEX.length]));
  return out;
}

/** ``points`` cut into polylines at ``breaks`` (start indices); single points dropped. */
export function splitPath<T>(points: T[], breaks: number[]): T[][] {
  const out: T[][] = [];
  let start = 0;
  for (const b of [...breaks, points.length]) {
    if (b - start > 1) out.push(points.slice(start, b));
    start = b;
  }
  return out;
}

async function loadTrajectory(id: string) {
  try {
    const body = await fetchTracked(`${srcPath(id)}/trajectory`, `traj:${id}`, "", "application/json");
    const traj = JSON.parse(new TextDecoder().decode(body)) as {points?: number[][]; breaks?: number[]; robots?: (TrajRobot & {start: number})[]; syms?: (string | null)[]};
    const robots = traj?.robots ?? [];
    const paths = traj?.points?.length ? trajectoryPaths(traj.points.map(p => [p[1], p[2], p[3]] as V3), traj.breaks ?? [], robots) : null;
    const keyframes = keyframeTracks(traj?.points ?? [], traj?.syms ?? [], robots);
    if (useViewer.getState().sources[id]) useViewer.getState().patchSource(id, {trajectory: paths, trajectoryRobots: robots.map(({start: _, ...r}) => r), keyframes});
  } catch {
    /* no trajectory */
  }
}

async function refetchManifest(id: string) {
  try {
    const manifest = await getJSON<TileManifest>(`${srcPath(id)}/mesh/manifest`);
    const cur = useViewer.getState().sources[id];
    useViewer.getState().patchSource(id, {manifest, manifestRev: (cur?.manifestRev ?? 0) + 1});
  } catch {
    /* keep old */
  }
}

/** WS subscription for one source: version bumps, deltas, tile changes. */
function subscribeSource(id: string): () => void {
  const prefix = `graph/${id}/`;
  return ws.on(prefix, (data, frame) => {
    const kind = frame.topic.slice(prefix.length);
    const st = useViewer.getState();
    const src = st.sources[id];
    if (!src?.enabled) return;
    if (kind === "version") {
      const msg = (data ?? {}) as {version?: number; base?: number; delta?: unknown};
      const v = msg.version ?? null;
      if (v === src.version) return;
      // a known write comes with its delta: applied in
      // place when we hold exactly the version it was made against; otherwise
      // (missed a frame, still loading) reload the snapshot
      if (msg.delta && msg.base != null && msg.base === src.version && src.model && src.status === "ready") {
        src.model.applyDelta(decodeDelta(msg.delta));
        st.patchSource(id, {version: v, rev: src.model.rev});
        return;
      }
      st.patchSource(id, {version: v});
      void st.reloadSource(id);
    } else if (kind === "delta") {
      if (!src.model || src.status !== "ready") return;
      const d = decodeDelta(data);
      const last = src.model.lastSeq;
      // already folded into the snapshot we hold
      if (d.seq !== undefined && last !== null && d.seq <= last && d.seq !== 1) return;
      if (d.seq !== undefined && last !== null && d.seq > last + 1) {
        // missed frames (queue dropped them): resync from a fresh snapshot
        void st.reloadSource(id);
        return;
      }
      src.model.applyDelta(d);
      st.patchSource(id, {rev: src.model.rev});
    } else if (kind === "tiles") {
      void refetchManifest(id);
    }
  });
}

/** All enabled + ready models (for picking / nearest-place). */
export function readyModels(): {id: string; model: GraphModel}[] {
  const s = useViewer.getState();
  return s.order
    .map(id => s.sources[id])
    .filter(v => v?.enabled && v.model)
    .map(v => ({id: v.id, model: v.model!}));
}
