// One multiplexed WebSocket: msgpack frames {topic, seq, data}, topic-prefix
// subscriptions, auto-reconnect with exponential backoff. Coming back to the
// page (phone unlocked, tab shown, network back) reconnects at once instead of
// waiting out the backoff.
import {create} from "zustand";
import {decode, encode} from "@msgpack/msgpack";
import {withToken} from "./lib/auth";

export interface Frame {
  topic: string;
  seq: number;
  data: unknown;
}
type Handler = (data: unknown, frame: Frame) => void;

// Always subscribed; graph/<source>/ and maps/active subscribe on demand.
export const BASE_TOPICS = ["jobs", "job/"];

interface WsStatus {
  connected: boolean;
  authError: boolean;
  attempts: number;
  lastFrame: number | null;
  nextRetryAt: number | null;
}

export const useWsStatus = create<WsStatus>(() => ({
  connected: false,
  authError: false,
  attempts: 0,
  lastFrame: null,
  nextRetryAt: null,
}));

class WsClient {
  private ws: WebSocket | null = null;
  private handlers = new Map<string, Set<Handler>>();
  private extraSubs = new Map<string, number>(); // prefix -> refcount
  private attempts = 0;
  private timer: number | null = null;
  private stopped = true;
  private reconnectHooks = new Set<() => void>();
  private wakeBound = false;

  start() {
    if (!this.stopped) return;
    this.stopped = false;
    if (!this.wakeBound) {
      this.wakeBound = true;
      const wake = () => this.wake();
      document.addEventListener("visibilitychange", () => document.visibilityState === "visible" && wake());
      window.addEventListener("pageshow", wake);
      window.addEventListener("online", wake);
    }
    this.connect();
  }

  /** Reconnect now if the socket is down (skips the pending backoff). */
  wake() {
    if (this.stopped || this.ws) return;
    if (this.timer) window.clearTimeout(this.timer);
    this.timer = null;
    this.attempts = 0;
    useWsStatus.setState({nextRetryAt: null});
    this.connect();
  }

  stop() {
    this.stopped = true;
    if (this.timer) window.clearTimeout(this.timer);
    this.ws?.close();
    this.ws = null;
  }

  /** Called after every (re)connect, so stores can refetch snapshots they may have missed. */
  onReconnect(fn: () => void) {
    this.reconnectHooks.add(fn);
    return () => void this.reconnectHooks.delete(fn);
  }

  private url(): string {
    const proto = window.location.protocol === "https:" ? "wss:" : "ws:";
    const base = new URL("api/ws", new URL(".", window.location.href)).pathname;
    return withToken(`${proto}//${window.location.host}${base}`);
  }

  private connect() {
    if (this.stopped) return;
    let ws: WebSocket;
    try {
      ws = new WebSocket(this.url());
    } catch {
      this.schedule();
      return;
    }
    ws.binaryType = "arraybuffer";
    this.ws = ws;
    ws.onopen = () => {
      this.attempts = 0;
      useWsStatus.setState({connected: true, authError: false, attempts: 0, nextRetryAt: null});
      this.sendSubs();
      for (const h of this.reconnectHooks) h();
    };
    ws.onmessage = ev => {
      let frame: Frame;
      try {
        const raw = typeof ev.data === "string" ? JSON.parse(ev.data) : decode(new Uint8Array(ev.data as ArrayBuffer));
        frame = raw as Frame;
      } catch {
        return;
      }
      if (!frame || typeof frame.topic !== "string") return;
      useWsStatus.setState({lastFrame: Date.now()});
      this.dispatch(frame);
    };
    ws.onclose = ev => {
      if (this.ws === ws) this.ws = null;
      // 4401 = bad/missing token (server closes before accept)
      useWsStatus.setState({connected: false, authError: ev.code === 4401});
      this.schedule();
    };
    ws.onerror = () => {
      /* onclose follows */
    };
  }

  private schedule() {
    if (this.stopped) return;
    this.attempts++;
    const delay = Math.min(10_000, 500 * 2 ** Math.min(this.attempts - 1, 5)) * (0.8 + Math.random() * 0.4);
    useWsStatus.setState({attempts: this.attempts, nextRetryAt: Date.now() + delay});
    this.timer = window.setTimeout(() => this.connect(), delay);
  }

  private dispatch(frame: Frame) {
    for (const [prefix, set] of this.handlers) {
      if (frame.topic === prefix || frame.topic.startsWith(prefix)) {
        for (const h of set) {
          try {
            h(frame.data, frame);
          } catch (e) {
            // a broken handler must not kill the socket loop
            console.warn("ws handler failed for", frame.topic, e);
          }
        }
      }
    }
  }

  private subsList(): string[] {
    return [...new Set([...BASE_TOPICS, ...this.extraSubs.keys()])];
  }

  private sendSubs() {
    if (this.ws?.readyState === WebSocket.OPEN) {
      this.ws.send(encode({sub: this.subsList()}));
    }
  }

  /** Listen for frames whose topic starts with `prefix`. Also subscribes server-side. */
  on(prefix: string, handler: Handler): () => void {
    let set = this.handlers.get(prefix);
    if (!set) this.handlers.set(prefix, (set = new Set()));
    set.add(handler);
    const needsSub = !BASE_TOPICS.some(b => prefix.startsWith(b));
    if (needsSub) {
      this.extraSubs.set(prefix, (this.extraSubs.get(prefix) ?? 0) + 1);
      if (this.extraSubs.get(prefix) === 1) this.sendSubs();
    }
    return () => {
      set!.delete(handler);
      if (!set!.size) this.handlers.delete(prefix);
      if (needsSub) {
        const n = (this.extraSubs.get(prefix) ?? 1) - 1;
        if (n <= 0) {
          this.extraSubs.delete(prefix);
          this.sendSubs();
        } else this.extraSubs.set(prefix, n);
      }
    };
  }

  /** For tests / mock: inject a frame as if received. */
  inject(frame: Frame) {
    this.dispatch(frame);
  }
}

export const ws = new WsClient();
