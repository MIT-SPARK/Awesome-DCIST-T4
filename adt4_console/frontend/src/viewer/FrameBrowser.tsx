// Images tab › Browse: walk a robot's keyframes in time order — pick the map's day,
// then a session (robot / bag), then scrub, step or play through its frames. The
// current keyframe is selected on the map (its pose is marked) and its images come
// from the node detail (own keyframe first, then nearby sub-keyframes).
import {useEffect, useMemo, useState} from "react";
import {Empty, Segmented, Spinner} from "../components/ui";
import {errorText, getJSON} from "../lib/api";
import {useRobotColorOf} from "../lib/palette";
import {pathColors, srcPath, useViewer, type KeyframeTrack} from "../stores/viewer";
import {bagName} from "./GraphLayers";
import {ObjectImages, type ImageInfo} from "./objectImages";
import type {NodeDetail} from "../types";

/** Keyframe times are unix seconds on timestamped maps; small values are indices. */
const isClock = (t: number) => t > 1e8;

export function dayOf(t: number): string {
  if (!isClock(t)) return "undated";
  const d = new Date(t * 1000);
  return `${d.getFullYear()}-${String(d.getMonth() + 1).padStart(2, "0")}-${String(d.getDate()).padStart(2, "0")}`;
}

export function clockOf(t: number): string {
  return isClock(t) ? new Date(t * 1000).toLocaleTimeString([], {hour: "2-digit", minute: "2-digit", second: "2-digit"}) : `#${t}`;
}

/** Tracks grouped by the day each starts on, days newest first. */
export function tracksByDay(tracks: {src: string; track: KeyframeTrack}[]): [string, {src: string; track: KeyframeTrack}[]][] {
  const days = new Map<string, {src: string; track: KeyframeTrack}[]>();
  for (const x of tracks) {
    const d = dayOf(x.track.t[0]);
    days.set(d, [...(days.get(d) ?? []), x]);
  }
  return [...days.entries()].sort((a, b) => b[0].localeCompare(a[0]));
}

/** Index of the keyframe nearest in time to ``t`` (``ts`` sorted). */
export function nearestIndex(ts: number[], t: number): number {
  let lo = 0;
  let hi = ts.length - 1;
  while (lo < hi) {
    const mid = (lo + hi) >> 1;
    if (ts[mid] < t) lo = mid + 1;
    else hi = mid;
  }
  return lo > 0 && Math.abs(ts[lo - 1] - t) <= Math.abs(ts[lo] - t) ? lo - 1 : lo;
}

const PLAY_MS = 700;

export function FrameBrowser() {
  const sources = useViewer(s => s.sources);
  const order = useViewer(s => s.order);
  const select = useViewer(s => s.select);
  const fly = useViewer(s => s.fly);
  const pathColor = useViewer(s => s.pathColor);
  const robotColorOf = useRobotColorOf();
  const tracks = useMemo(() => order.flatMap(src => (sources[src]?.enabled ? (sources[src].keyframes ?? []).map(track => ({src, track})) : [])), [order, sources]);
  const days = useMemo(() => tracksByDay(tracks), [tracks]);
  const [day, setDay] = useState<string | null>(null);
  const [pick, setPick] = useState<string | null>(null); // `${src}|${robot key}`
  const [i, setI] = useState(0);
  const [playing, setPlaying] = useState(false);
  const [detail, setDetail] = useState<(NodeDetail & {image_info?: ImageInfo[]}) | null>(null);
  const [err, setErr] = useState<string | null>(null);
  const [loading, setLoading] = useState(false);

  const dayTracks = days.find(([d]) => d === day)?.[1] ?? days[0]?.[1] ?? [];
  const cur = dayTracks.find(x => `${x.src}|${x.track.robot.key}` === pick) ?? dayTracks[0] ?? null;
  const n = cur?.track.syms.length ?? 0;
  const idx = Math.min(i, Math.max(0, n - 1));
  const sym = cur ? cur.track.syms[idx] : null;
  const colors = useMemo(() => {
    const out: Record<string, string> = {};
    for (const src of order) Object.assign(out, pathColors(sources[src]?.trajectoryRobots ?? [], pathColor === "single" ? "bag" : pathColor, robotColorOf) ?? {});
    return out;
  }, [order, sources, pathColor, robotColorOf]);

  // the current keyframe: marked on the map, images from its node detail (debounced while scrubbing)
  useEffect(() => {
    if (!cur || !sym) return;
    select({src: cur.src, sym});
    const ctl = new AbortController();
    const timer = window.setTimeout(() => {
      setLoading(true);
      setErr(null);
      getJSON<NodeDetail & {image_info?: ImageInfo[]}>(`${srcPath(cur.src)}/node/${encodeURIComponent(sym)}`, ctl.signal)
        .then(setDetail)
        .catch(e => !ctl.signal.aborted && setErr(errorText(e)))
        .finally(() => !ctl.signal.aborted && setLoading(false));
    }, 150);
    return () => {
      window.clearTimeout(timer);
      ctl.abort();
    };
  }, [cur?.src, sym, select]); // eslint-disable-line react-hooks/exhaustive-deps

  useEffect(() => {
    if (!playing) return;
    const id = window.setInterval(() => setI(k => (k + 1 < n ? k + 1 : (setPlaying(false), k))), PLAY_MS);
    return () => window.clearInterval(id);
  }, [playing, n]);

  if (!tracks.length)
    return <Empty title="No keyframes to browse">This map has no robot keyframes with images (agent nodes), or its trajectory has not loaded yet.</Empty>;

  const t0 = cur?.track.t[0] ?? 0;
  const t = cur?.track.t[idx] ?? 0;
  const step = (d: number) => {
    setPlaying(false);
    setI(Math.max(0, Math.min(n - 1, idx + d)));
  };
  const choose = (key: string) => {
    const next = dayTracks.find(x => `${x.src}|${x.track.robot.key}` === key);
    // keep the clock time when switching between sessions of the same day
    setI(next && cur ? nearestIndex(next.track.t, t) : 0);
    setPick(key);
    setPlaying(false);
  };
  return (
    <div className="frame-browser">
      <label className="slider-row">
        <span>Day</span>
        <select
          className="input input-sm"
          value={day ?? days[0]?.[0] ?? ""}
          onChange={e => {
            setDay(e.target.value);
            setPick(null);
            setI(0);
            setPlaying(false);
          }}
          aria-label="Map day"
        >
          {days.map(([d, xs]) => (
            <option key={d} value={d}>
              {d} · {xs.length} session{xs.length === 1 ? "" : "s"}
            </option>
          ))}
        </select>
      </label>
      <ul className="frame-sessions" role="listbox" aria-label="Sessions">
        {dayTracks.map(({src, track}) => {
          const key = `${src}|${track.robot.key}`;
          const on = cur && key === `${cur.src}|${cur.track.robot.key}`;
          const r = track.robot;
          return (
            <li key={key}>
              <button className={`frame-session ${on ? "on" : ""}`} role="option" aria-selected={!!on} onClick={() => choose(key)} title={r.bag}>
                <span className="path-swatch" style={{background: colors[r.key] ?? "var(--text-2, #888)"}} />
                <span className="frame-session-text">
                  <b>{r.session ?? r.key}</b>
                  {r.robot && r.robot !== r.session ? ` · ${r.robot}` : ""}
                  <span className="small muted">
                    {" "}
                    {clockOf(track.t[0])}–{clockOf(track.t[track.t.length - 1])} · {track.syms.length} frames
                  </span>
                  {r.bag && <span className="mono small muted frame-bag">{bagName(r.bag)}</span>}
                </span>
              </button>
            </li>
          );
        })}
      </ul>
      {cur && (
        <>
          <div className="frame-scrub">
            <button className="btn btn-xs" onClick={() => step(-1)} disabled={idx === 0} aria-label="Previous keyframe">
              ◀
            </button>
            <button className="btn btn-xs" onClick={() => setPlaying(p => !p)} aria-label={playing ? "Pause" : "Play"}>
              {playing ? "❚❚" : "▶"}
            </button>
            <button className="btn btn-xs" onClick={() => step(1)} disabled={idx >= n - 1} aria-label="Next keyframe">
              ▶|
            </button>
            <input
              type="range"
              min={0}
              max={Math.max(0, n - 1)}
              value={idx}
              onChange={e => {
                setPlaying(false);
                setI(Number(e.target.value));
              }}
              aria-label="Keyframe"
            />
          </div>
          <div className="row gap between small">
            <span className="mono">
              {clockOf(t)}
              {isClock(t) ? ` · +${Math.floor((t - t0) / 60)}:${String(Math.round(t - t0) % 60).padStart(2, "0")}` : ""}
            </span>
            <span className="muted">
              {idx + 1}/{n} · {sym}
              <button className="btn btn-xs link-btn" onClick={() => fly(cur.track.pos[idx])} title="Centre the map on this pose">
                show on map
              </button>
            </span>
          </div>
          {err && <div className="small err">{err}</div>}
          {loading && !detail && <Spinner />}
          {detail && (detail.images?.length ? <ObjectImages images={detail.images} info={detail.image_info} title={`${cur.track.robot.session ?? cur.track.robot.key} ${clockOf(t)}`} sym={sym ?? undefined} isObject={false} /> : <div className="small muted">No images for this keyframe.</div>)}
        </>
      )}
    </div>
  );
}

/** The Images tab: browse keyframes by day / session. */
export function ImagesTab() {
  return (
    <div className="images-tab">
      <FrameBrowser />
    </div>
  );
}
