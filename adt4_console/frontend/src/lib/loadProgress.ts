// Startup-load progress: byte counts per task (graph snapshot, mesh tiles,
// trajectory, satellite imagery), a measured-throughput meter and the ETA the
// viewer's load bar shows ("Graph ✓ · Mesh 12.4/38 MB · ~25 s left").
// Pure (clock passed in) so the maths is unit-tested; stores/loading.ts wires it up.

export type LoadUnit = "bytes" | "count";

export interface LoadTask {
  /** shown in the bar; tasks with the same label are summed; "" = counted in
   *  the bytes / ETA but not named (e.g. the trajectory) */
  label: string;
  unit: LoadUnit;
  done: number;
  /** null until known (e.g. before the manifest / response headers arrive) */
  total: number | null;
  state: "active" | "done" | "error";
  /** shown instead of numbers while total is unknown, e.g. "preparing" */
  note?: string;
}

/**
 * Throughput estimate: an exponentially weighted moving average of bytes/s,
 * sampled only while transfers are in flight (idle gaps, e.g. a server that
 * is still building the mesh, would otherwise drag it to zero).
 */
export class RateMeter {
  /** bytes/s, null until enough has been seen */
  rate: number | null = null;
  /** seconds of transfer time sampled so far */
  sampled = 0;
  private since: number | null = null;
  private pending = 0;

  /** @param tau smoothing time constant (s) @param minDt shortest sample (s) */
  constructor(
    private tau = 3,
    private minDt = 0.25,
  ) {}

  /** Transfers in flight from `now` on (a no-op while already busy). */
  busy(now: number) {
    if (this.since === null) {
      this.since = now;
      this.pending = 0;
    }
  }

  /** `bytes` arrived at `now`. */
  add(bytes: number, now: number) {
    if (this.since === null) this.busy(now);
    this.pending += bytes;
    this.sample(now);
  }

  /** Nothing in flight any more: close the current sample. */
  idle(now: number) {
    this.sample(now, true);
    this.since = null;
    this.pending = 0;
  }

  private sample(now: number, final = false) {
    if (this.since === null) return;
    const dt = now - this.since;
    if (dt < this.minDt && !(final && this.pending > 0 && dt > 0)) return;
    if (dt <= 0) return;
    const inst = this.pending / dt;
    const a = 1 - Math.exp(-dt / this.tau);
    this.rate = this.rate === null ? inst : this.rate + a * (inst - this.rate);
    this.sampled += dt;
    this.since = now;
    this.pending = 0;
  }

  /** Seconds to move `remaining` bytes at the measured rate; null while unknown. */
  eta(remaining: number): number | null {
    if (remaining <= 0) return 0;
    // the first round trips say more about latency than throughput
    if (this.rate === null || this.rate <= 0 || this.sampled < 2) return null;
    return remaining / this.rate;
  }
}

/** "~25 s left", "~3 min left"; null → "" (still estimating). */
export function fmtEta(s: number | null): string {
  if (s === null || !Number.isFinite(s)) return "";
  if (s < 1) return "<1 s left";
  if (s < 10) return `~${Math.ceil(s)} s left`;
  if (s < 90) return `~${Math.round(s / 5) * 5} s left`;
  if (s < 3600) return `~${Math.round(s / 60)} min left`;
  const h = Math.floor(s / 3600);
  return `~${h} h ${Math.round((s - h * 3600) / 60)} min left`;
}

/** "12.4/38 MB", "74/120 kB" (both in the unit of the total). */
export function fmtBytesPair(done: number, total: number): string {
  const mb = total >= 1e6;
  const div = mb ? 1e6 : 1e3;
  const f = (v: number) => {
    const x = v / div;
    return mb ? (x >= 10 ? x.toFixed(1) : x.toFixed(2)) : x.toFixed(0);
  };
  return `${f(Math.min(done, total))}/${f(total)} ${mb ? "MB" : "kB"}`;
}

export interface LoadSummary {
  /** "Graph ✓", "Mesh 12.4/38 MB", "Imagery 30/42", "Mesh preparing" */
  parts: {label: string; text: string; done: boolean}[];
  /** 0..1 over the byte tasks (count tasks when there are none); null = unknown */
  fraction: number | null;
  bytesDone: number;
  bytesTotal: number;
  eta: number | null;
  active: boolean;
  error: boolean;
}

/** Labels in the order the bar names them (others after, as they start). */
export const PART_ORDER = ["Graph", "Mesh", "Imagery"];

/** Fold the tasks into what the load bar shows. */
export function summarize(tasks: LoadTask[], meter: RateMeter | null): LoadSummary {
  const groups = new Map<string, LoadTask[]>();
  for (const t of tasks) {
    if (!groups.has(t.label)) groups.set(t.label, []);
    groups.get(t.label)!.push(t);
  }
  const parts: LoadSummary["parts"] = [];
  const order = (l: string) => {
    const i = PART_ORDER.indexOf(l);
    return i < 0 ? PART_ORDER.length : i;
  };
  for (const [label, ts] of [...groups].sort((a, b) => order(a[0]) - order(b[0]))) {
    if (!label) continue;
    const done = ts.every(t => t.state !== "active");
    const unit = ts[0].unit;
    const known = ts.every(t => t.total !== null);
    const d = ts.reduce((n, t) => n + t.done, 0);
    const tot = ts.reduce((n, t) => n + (t.total ?? 0), 0);
    let text: string;
    if (done) text = ts.some(t => t.state === "error") ? "failed" : "✓";
    else if (!known && !d) text = ts.find(t => t.note)?.note ?? "…";
    else if (unit === "count") text = `${d}/${known ? tot : "?"}`;
    else text = known && tot > 0 ? fmtBytesPair(d, tot) : d >= 1e6 ? `${(d / 1e6).toFixed(1)} MB` : `${Math.round(d / 1e3)} kB`;
    parts.push({label, text, done});
  }
  const live = tasks.filter(t => t.state === "active" || t.state === "done");
  const bytes = live.filter(t => t.unit === "bytes");
  const bytesDone = bytes.reduce((n, t) => n + t.done, 0);
  const bytesTotal = bytes.reduce((n, t) => n + (t.total ?? t.done), 0);
  const unknown = tasks.some(t => t.state === "active" && t.total === null);
  let fraction: number | null = null;
  if (bytesTotal > 0 && !unknown) fraction = bytesDone / bytesTotal;
  else if (!bytes.length) {
    const c = live.filter(t => t.unit === "count" && t.total);
    const tot = c.reduce((n, t) => n + (t.total ?? 0), 0);
    if (tot) fraction = c.reduce((n, t) => n + t.done, 0) / tot;
  }
  const active = tasks.some(t => t.state === "active");
  return {
    parts,
    fraction: fraction === null ? null : Math.max(0, Math.min(1, fraction)),
    bytesDone,
    bytesTotal,
    eta: active && !unknown && meter ? meter.eta(bytesTotal - bytesDone) : null,
    active,
    error: tasks.some(t => t.state === "error"),
  };
}

/** The bar's one-line text: "Graph ✓ · Mesh 12.4/38 MB · ~25 s left". */
export function summaryText(s: LoadSummary): string {
  const bits = s.parts.map(p => `${p.label} ${p.text}`);
  const eta = s.active ? fmtEta(s.eta) : "";
  if (eta) bits.push(eta);
  return bits.join(" · ");
}

/**
 * Wire bytes a response has delivered so far, from what a fetch body reader
 * sees (bytes after HTTP decompression): scaled by Content-Length /
 * X-Uncompressed-Length when the body is compressed, as is when it is not.
 */
export function wireProgress(read: number, contentLength: number | null, uncompressed: number | null, encoded: boolean): number {
  if (!encoded) return read;
  if (contentLength && uncompressed) return Math.min(contentLength, (read / uncompressed) * contentLength);
  return read; // unknown ratio: decoded bytes are the best we have
}
