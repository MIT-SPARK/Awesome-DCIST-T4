import {create} from "zustand";
import type {Health} from "../types";
import {ws} from "../ws";
import {getJSON} from "../lib/api";

/** `api/health`: server version and per-source Neo4j reachability, for the top bar. */
interface HealthStore {
  health: Health | null;
  load: () => Promise<void>;
}

export const useHealthStore = create<HealthStore>(set => ({
  health: null,
  load: async () => {
    try {
      set({health: await getJSON<Health>("api/health")});
    } catch {
      /* keep the last value; the server banner covers an unreachable backend */
    }
  },
}));

export function bindHealth() {
  ws.onReconnect(() => void useHealthStore.getState().load());
}
