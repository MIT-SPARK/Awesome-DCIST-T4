import {create} from "zustand";
import type {Job} from "../types";
import {ws} from "../ws";
import {getJSON} from "../lib/api";

interface JobsStore {
  jobs: Record<string, Job>;
  order: string[]; // newest first
  drawerOpen: boolean;
  focusId: string | null;
  upsert: (j: Job) => void;
  setSummaries: (js: Job[]) => void;
  open: (id?: string | null) => void;
  close: () => void;
  load: () => Promise<void>;
}

const ACTIVE = new Set(["running", "waiting"]);

export const isActive = (j: Job) => ACTIVE.has(j.state);

function sortIds(jobs: Record<string, Job>): string[] {
  return Object.values(jobs)
    .sort((a, b) => (b.created ?? 0) - (a.created ?? 0))
    .map(j => j.id);
}

export const useJobs = create<JobsStore>((set, get) => ({
  jobs: {},
  order: [],
  drawerOpen: false,
  focusId: null,
  upsert: j =>
    set(s => {
      const prev = s.jobs[j.id];
      // summaries carry no log: keep the one we already have
      const merged: Job = {...prev, ...j, log: j.log ?? prev?.log};
      const jobs = {...s.jobs, [j.id]: merged};
      return {jobs, order: sortIds(jobs)};
    }),
  setSummaries: js =>
    set(s => {
      const jobs: Record<string, Job> = {...s.jobs};
      for (const j of js) jobs[j.id] = {...jobs[j.id], ...j, log: j.log ?? jobs[j.id]?.log};
      return {jobs, order: sortIds(jobs)};
    }),
  open: id => set({drawerOpen: true, focusId: id ?? get().focusId ?? get().order[0] ?? null}),
  close: () => set({drawerOpen: false}),
  load: async () => {
    try {
      const js = await getJSON<Job[] | {jobs: Job[]}>("api/jobs");
      get().setSummaries(Array.isArray(js) ? js : js.jobs ?? []);
    } catch {
      /* backend banner handles it */
    }
  },
}));

/** Fetch the full job (with log) once; WS keeps it current afterwards. */
export async function fetchJob(id: string) {
  try {
    useJobs.getState().upsert(await getJSON<Job>(`api/jobs/${encodeURIComponent(id)}`));
  } catch {
    /* ignore */
  }
}

/** Start-of-job helper: record the id and pop the drawer open on it. */
export function trackJob(jobId: string, title = "Job") {
  const s = useJobs.getState();
  if (!s.jobs[jobId]) s.upsert({id: jobId, kind: "", title, state: "running", created: Date.now() / 1000});
  s.open(jobId);
  void fetchJob(jobId);
}

export function bindJobs() {
  ws.on("jobs", d => {
    const list = Array.isArray(d) ? (d as Job[]) : ((d as {jobs?: Job[]})?.jobs ?? []);
    useJobs.getState().setSummaries(list);
  });
  ws.on("job/", (d, f) => {
    const j = d as Job;
    if (!j || typeof j !== "object") return;
    useJobs.getState().upsert({...j, id: j.id ?? f.topic.slice(4)});
  });
  ws.onReconnect(() => void useJobs.getState().load());
}
