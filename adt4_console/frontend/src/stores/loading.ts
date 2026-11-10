// Startup-load progress shared by the loaders (viewer store: snapshot +
// trajectory + manifest; MeshTiles: tiles; SatelliteGround: imagery) and the
// viewer's LoadBar. The maths is lib/loadProgress.ts; this keeps the tasks,
// one throughput meter for all transfers, and a throttled snapshot for React.
import {create} from "zustand";
import {RateMeter, summarize, type LoadSummary, type LoadTask, type LoadUnit} from "../lib/loadProgress";

const now = () => performance.now() / 1000;
const NOTIFY_MS = 250;

const tasks = new Map<string, LoadTask>();
let meter = new RateMeter();
let inflight = 0;
let timer: ReturnType<typeof setTimeout> | null = null;
let doneAt: number | null = null;

interface LoadingStore {
  summary: LoadSummary;
  /** when the last batch finished (performance.now() seconds), null while loading */
  finishedAt: number | null;
  /** bumps on every published change */
  rev: number;
}

export const useLoading = create<LoadingStore>(() => ({summary: summarize([], null), finishedAt: null, rev: 0}));

function publish() {
  timer = null;
  const s = summarize([...tasks.values()], meter);
  useLoading.setState(st => ({summary: s, finishedAt: doneAt, rev: st.rev + 1}));
}

function schedule(immediate = false) {
  if (immediate) {
    if (timer !== null) clearTimeout(timer);
    publish();
  } else if (timer === null) timer = setTimeout(publish, NOTIFY_MS);
}

/** A finished batch stays on the bar this long (and new tasks join it). */
export const LINGER_S = 2.5;

/** Start (or restart) a task. A batch finished more than LINGER_S ago is
 *  forgotten when a new task starts (a new load: fresh totals and meter). */
export function beginTask(id: string, label: string, unit: LoadUnit = "bytes", total: number | null = null, note?: string) {
  // (the throughput meter carries over: same link, an ETA from the start)
  if (doneAt !== null && now() - doneAt > LINGER_S && ![...tasks.values()].some(t => t.state === "active")) tasks.clear();
  doneAt = null;
  tasks.set(id, {label, unit, done: 0, total, state: "active", note});
  schedule(true);
}

export function progressTask(id: string, done: number, total?: number | null) {
  const t = tasks.get(id);
  if (!t) return;
  t.done = done;
  if (total !== undefined) t.total = total;
  if (t.state !== "active") t.state = "active";
  schedule();
}

export function endTask(id: string, ok = true) {
  const t = tasks.get(id);
  if (!t) return;
  t.state = ok ? "done" : "error";
  if (ok && t.total !== null) t.done = t.total;
  if (![...tasks.values()].some(x => x.state === "active")) doneAt = now();
  schedule(true);
}

/** Forget a task (its source was hidden or closed). */
export function dropTask(id: string) {
  if (!tasks.delete(id)) return;
  if (tasks.size && ![...tasks.values()].some(x => x.state === "active")) doneAt = now();
  schedule(true);
}

/** Byte-level hooks for lib/api fetchWithProgress: drive the throughput meter. */
export const transfer = {
  start() {
    if (inflight++ === 0) meter.busy(now());
  },
  bytes(n: number) {
    if (n > 0) meter.add(n, now());
  },
  end() {
    if (--inflight <= 0) {
      inflight = 0;
      meter.idle(now());
    }
  },
};

export function loadTaskIds(): string[] {
  return [...tasks.keys()];
}

/** Test hook: forget everything. */
export function resetLoading() {
  tasks.clear();
  meter = new RateMeter();
  inflight = 0;
  doneAt = null;
  if (timer !== null) clearTimeout(timer);
  timer = null;
  publish();
}
