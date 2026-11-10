// REST client: bearer auth, typed errors, msgpack support.
import {useCallback, useEffect, useRef, useState} from "react";
import {getToken} from "./auth";
import {unpack} from "../codec";
import {wireProgress} from "./loadProgress";
import {transfer} from "../stores/loading";

export class ApiError extends Error {
  status: number; // 0 = network error / backend down
  detail: string;
  /** the parsed JSON error body, for routes that say more than `detail` */
  body: unknown;
  constructor(status: number, detail: string, body: unknown = null) {
    super(detail || `HTTP ${status}`);
    this.status = status;
    this.detail = detail;
    this.body = body;
  }
  get kind(): "down" | "unavailable" | "auth" | "notfound" | "other" {
    if (this.status === 0 || this.status === 502 || this.status === 504) return "down";
    // 503: a service the route needs (Neo4j, the geo conversion) is down
    if (this.status === 503) return "unavailable";
    if (this.status === 401 || this.status === 403) return "auth";
    if (this.status === 404) return "notfound";
    return "other";
  }
}

type Listener = (ok: boolean, err?: ApiError) => void;
const listeners = new Set<Listener>();
export function onBackendState(l: Listener) {
  listeners.add(l);
  return () => void listeners.delete(l);
}
function notify(ok: boolean, err?: ApiError) {
  for (const l of listeners) l(ok, err);
}

function headers(extra?: HeadersInit): Headers {
  const h = new Headers(extra);
  const t = getToken();
  if (t) h.set("Authorization", `Bearer ${t}`);
  return h;
}

async function errorFrom(res: Response): Promise<ApiError> {
  let detail = "";
  let body: unknown = null;
  try {
    const text = await res.text();
    try {
      const j = JSON.parse(text);
      body = j;
      detail = typeof j.detail === "string" ? j.detail : JSON.stringify(j.detail ?? j);
    } catch {
      detail = text.slice(0, 300);
    }
  } catch {
    /* ignore */
  }
  return new ApiError(res.status, detail || res.statusText, body);
}

export async function request(path: string, init: RequestInit = {}): Promise<Response> {
  let res: Response;
  try {
    res = await fetch(path, {...init, headers: headers(init.headers)});
  } catch (e) {
    // a request its caller cancelled (selection changed, panel closed) says
    // nothing about the server: rethrow it without marking the backend down
    if (init.signal?.aborted || (e as {name?: string} | null)?.name === "AbortError") throw e;
    const err = new ApiError(0, "Backend unreachable");
    notify(false, err);
    throw err;
  }
  if (!res.ok) {
    const err = await errorFrom(res);
    if (err.kind === "down") notify(false, err);
    else notify(true);
    throw err;
  }
  notify(true);
  return res;
}

export async function getJSON<T>(path: string, signal?: AbortSignal): Promise<T> {
  return (await (await request(path, {signal})).json()) as T;
}

export async function sendJSON<T>(method: string, path: string, body?: unknown): Promise<T> {
  const res = await request(path, {
    method,
    headers: body === undefined ? undefined : {"Content-Type": "application/json"},
    body: body === undefined ? undefined : JSON.stringify(body),
  });
  const text = await res.text();
  return (text ? JSON.parse(text) : {}) as T;
}

export const postJSON = <T,>(path: string, body?: unknown) => sendJSON<T>("POST", path, body ?? {});
export const putJSON = <T,>(path: string, body?: unknown) => sendJSON<T>("PUT", path, body ?? {});
export const deleteJSON = <T,>(path: string) => sendJSON<T>("DELETE", path);

export async function getMsgpack(path: string, signal?: AbortSignal): Promise<unknown> {
  const res = await request(path, {signal, headers: {Accept: "application/x-msgpack"}});
  return unpack(await res.arrayBuffer());
}

/** Wire-byte progress of one GET (see lib/loadProgress wireProgress). */
export interface FetchProgress {
  /** called once the headers are in: the wire size, when the server says */
  onTotal?: (wireTotal: number | null) => void;
  /** wire bytes delivered so far (monotonic) */
  onBytes?: (wireDone: number) => void;
  signal?: AbortSignal;
  accept?: string;
}

/** GET a body while reporting wire-byte progress (load bar) and feeding the
 *  shared throughput meter (stores/loading `transfer`). */
export async function fetchWithProgress(path: string, p: FetchProgress = {}): Promise<Uint8Array> {
  transfer.start();
  try {
    const res = await request(path, {signal: p.signal, headers: p.accept ? {Accept: p.accept} : undefined});
    const num = (h: string) => {
      const v = res.headers.get(h);
      return v !== null && v !== "" && Number.isFinite(Number(v)) ? Number(v) : null;
    };
    const encoded = !!res.headers.get("content-encoding");
    const cl = num("content-length");
    const raw = num("x-uncompressed-length");
    const wireTotal = encoded ? (cl && raw ? cl : null) : (cl ?? raw);
    p.onTotal?.(wireTotal);
    if (!res.body) {
      const buf = new Uint8Array(await res.arrayBuffer());
      const w = wireProgress(buf.length, cl, raw, encoded);
      transfer.bytes(w);
      p.onBytes?.(w);
      return buf;
    }
    const reader = res.body.getReader();
    const chunks: Uint8Array[] = [];
    let read = 0;
    let wire = 0;
    for (;;) {
      const {done, value} = await reader.read();
      if (done) break;
      chunks.push(value);
      read += value.length;
      const w = wireProgress(read, cl, raw, encoded);
      transfer.bytes(w - wire);
      wire = w;
      p.onBytes?.(wire);
    }
    if (chunks.length === 1) return chunks[0];
    const out = new Uint8Array(read);
    let o = 0;
    for (const c of chunks) {
      out.set(c, o);
      o += c.length;
    }
    return out;
  } finally {
    transfer.end();
  }
}

/** msgpack GET with ``If-None-Match``: ``changed`` false on a 304 (nothing new
 *  since ``etag``), else the decoded body and the new ETag. */
export async function getMsgpackIfChanged(
  path: string,
  etag: string | null,
  signal?: AbortSignal,
): Promise<{changed: false} | {changed: true; data: unknown; etag: string | null}> {
  let res: Response;
  try {
    const h: Record<string, string> = {Accept: "application/x-msgpack"};
    if (etag) h["If-None-Match"] = etag;
    res = await fetch(path, {signal, headers: headers(h), cache: "no-store"});
  } catch (e) {
    if (signal?.aborted || (e as {name?: string} | null)?.name === "AbortError") throw e;
    const err = new ApiError(0, "Backend unreachable");
    notify(false, err);
    throw err;
  }
  if (res.status === 304) return {changed: false};
  if (!res.ok) throw await errorFrom(res);
  return {changed: true, data: unpack(await res.arrayBuffer()), etag: res.headers.get("ETag")};
}

export function errorText(e: unknown): string {
  if (e instanceof ApiError) {
    if (e.kind === "down") return "Backend unreachable";
    if (e.kind === "unavailable") return e.detail ? `Unavailable: ${e.detail}` : "Service unavailable";
    if (e.kind === "auth") return "Not authorised: open the console with ?token=...";
    return e.detail || `HTTP ${e.status}`;
  }
  return e instanceof Error ? e.message : String(e);
}

export interface ApiState<T> {
  data: T | undefined;
  error: ApiError | null;
  loading: boolean;
  reload: () => void;
  setData: (d: T) => void;
}

/** Fetch JSON on mount (and every `pollMs`), keeping the last good value on error. */
export function useApi<T>(path: string | null, pollMs = 0): ApiState<T> {
  const [data, setData] = useState<T | undefined>(undefined);
  const [error, setError] = useState<ApiError | null>(null);
  const [loading, setLoading] = useState<boolean>(Boolean(path));
  const [tick, setTick] = useState(0);
  const alive = useRef(true);
  useEffect(() => {
    alive.current = true;
    return () => {
      alive.current = false;
    };
  }, []);
  useEffect(() => {
    if (!path) {
      setLoading(false);
      return;
    }
    const ctl = new AbortController();
    let retry: number | undefined;
    setLoading(true);
    getJSON<T>(path, ctl.signal)
      .then(d => {
        if (!alive.current) return;
        setData(d);
        setError(null);
      })
      .catch(e => {
        if (!alive.current || ctl.signal.aborted) return;
        const err = e instanceof ApiError ? e : new ApiError(0, String(e));
        setError(err);
        // backend down: retry on our own so pages recover when it comes back
        if (err.kind === "down" && !pollMs) retry = window.setTimeout(() => alive.current && setTick(t => t + 1), 5000);
      })
      .finally(() => {
        if (alive.current && !ctl.signal.aborted) setLoading(false);
      });
    return () => {
      ctl.abort();
      window.clearTimeout(retry);
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [path, tick]);
  useEffect(() => {
    if (!pollMs || !path) return;
    const id = window.setInterval(() => setTick(t => t + 1), pollMs);
    return () => window.clearInterval(id);
  }, [pollMs, path]);
  const reload = useCallback(() => setTick(t => t + 1), []);
  return {data, error, loading, reload, setData};
}
