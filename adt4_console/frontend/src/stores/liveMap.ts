// The active map (API.md §6.7): which run the console's Neo4j holds, picking another
// one, and where its staged load is. The "graph" stage comes first; images, features
// and keyframes follow and report their progress.
import {create} from "zustand";
import {errorText, getJSON, postJSON} from "../lib/api";
import {ws} from "../ws";
import {trackJob} from "./jobs";
import {useViewer} from "./viewer";
import type {MapEntry} from "../types";

export type StageState = "pending" | "running" | "done" | "skipped" | "failed";
export interface MapStage {
  id: "graph" | "images" | "features" | "keyframes";
  label: string;
  state: StageState;
  pct: number;
  detail?: string | null;
}
export interface LiveMap {
  target: string;
  uri?: string;
  run: string | null;
  name: string | null;
  dsg?: string | null;
  nodes?: number | null;
  has_images: boolean;
  has_features: boolean;
  /** the graph stage is done */
  ready: boolean;
  job_id: string | null;
  stages: MapStage[];
  error?: string;
}
/** A run on this machine the picker offers. */
export interface MapChoice {
  name: string;
  path: string;
  hasImages: boolean;
}

interface LiveMapStore {
  map: LiveMap | null;
  choices: MapChoice[];
  error: string | null;
  set: (p: Partial<LiveMapStore>) => void;
}

export const useLiveMap = create<LiveMapStore>(set => ({
  map: null,
  choices: [],
  error: null,
  set: p => set(p),
}));

/** Every stage done or skipped. */
export function mapLoaded(m: LiveMap | null): boolean {
  return !!m && m.stages.every(s => s.state === "done" || s.state === "skipped");
}

/** Overall progress of the stages that run (skipped ones do not count), 0-100. */
export function mapProgress(m: LiveMap | null): number {
  const run = (m?.stages ?? []).filter(s => s.state !== "skipped");
  if (!run.length) return 100;
  return Math.round(run.reduce((a, s) => a + (s.state === "done" ? 100 : s.pct), 0) / run.length);
}

// the last map seen ready, and whether a status arrived yet (the map the page opened
// on is not a switch: the viewer already shows it)
let lastReady: string | null = null;
let seenStatus = false;

/** A new status: when the graph stage of a new map completes, the viewer switches to it. */
export function applyLiveMap(m: LiveMap) {
  useLiveMap.getState().set({map: m});
  if (m.ready) {
    const key = `${m.target}|${m.dsg}`;
    if (key !== lastReady) {
      if (seenStatus) onNewMap(m);
      lastReady = key;
    }
  }
  seenStatus = true;
}

/** Tests: forget what was seen. */
export function _resetLiveMap() {
  lastReady = null;
  seenStatus = false;
}

/** The live Neo4j holds another map: show it, drop what belonged to the old one. */
function onNewMap(m: LiveMap) {
  const v = useViewer.getState();
  const id = `neo4j:${m.target}`;
  void v.loadCatalog().then(() => {
    const st = useViewer.getState();
    // one map: file sources of other runs go off, the live one on
    for (const sid of st.order) if (st.sources[sid]?.kind === "file" && st.sources[sid]?.enabled) st.enableSource(sid, false);
    if (st.sources[id]) {
      if (st.sources[id].enabled) void st.reloadSource(id);
      else st.enableSource(id, true);
    }
  });
}

export async function refreshLiveMap(): Promise<void> {
  try {
    applyLiveMap(await getJSON<LiveMap>("api/maps/active"));
    useLiveMap.getState().set({error: null});
  } catch (e) {
    useLiveMap.getState().set({error: errorText(e)});
  }
}

/** This machine's runs with a scene graph (Maps page's list, local machine only). */
export async function refreshMapChoices(): Promise<void> {
  try {
    const r = await getJSON<{machines: Record<string, MapEntry[] | {error: string}>; base: string}>("api/maps?local=true");
    const local = r.machines[r.base];
    const list = Array.isArray(local) ? local : [];
    useLiveMap.getState().set({
      choices: list
        .filter(m => (m.has_dsg || m.has_dsg_mesh) && (m.abs_path || m.path))
        .map(m => ({name: m.name, path: (m.abs_path || m.path)!, hasImages: !!m.has_image_store}))
        .sort((a, b) => a.name.localeCompare(b.name)),
    });
  } catch (e) {
    useLiveMap.getState().set({error: errorText(e)});
  }
}

/** Make ``path`` the live map. Its load is tracked as a job (it asks before wiping edits). */
export async function activateMap(path: string, name: string): Promise<void> {
  useLiveMap.getState().set({error: null});
  try {
    const r = await postJSON<LiveMap & {already?: boolean; job_id?: string}>("api/maps/activate", {path});
    if (r.already) return;
    applyLiveMap(r);
    if (r.job_id) trackJob(r.job_id, `Load ${name}`);
  } catch (e) {
    useLiveMap.getState().set({error: errorText(e)});
  }
}

let bound = false;
/** Follow the live map over the websocket (once per page). */
export function bindLiveMap() {
  if (bound) return;
  bound = true;
  ws.on("maps/active", d => applyLiveMap(d as LiveMap));
  void refreshLiveMap();
}
