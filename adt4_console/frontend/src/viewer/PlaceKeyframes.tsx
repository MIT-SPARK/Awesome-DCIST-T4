// Inspect › a traversability place: every keyframe that observed it (place_keyframes.py
// visibility test), sorted by date or visibility, optionally grouped by session. A
// thumbnail opens the lightbox, which can select the keyframe's robot pose on the map
// (the Inspector then offers the way back to the place).
import {useEffect, useMemo, useState} from "react";
import {create} from "zustand";
import {Modal, Segmented, Spinner, Toggle} from "../components/ui";
import {ApiError, errorText, getJSON} from "../lib/api";
import {withToken} from "../lib/auth";
import {groupBySession, kfBadge, kfClock, kfDate, sessionName, sessionTitle, sortKeyframes, type PlaceKeyframe, type PlaceKeyframes as PlaceKfResp, type PlaceKfSort} from "../lib/placeKeyframes";
import {srcPath, type Selection} from "../stores/viewer";
import {imageUrl} from "./objectImages";
import "./objectImages.css";

const SORT_KEY = "adt4.placeKf.sort";
const GROUP_KEY = "adt4.placeKf.group";
/** thumbnails rendered before "Show all" (they load lazily either way) */
const PAGE = 60;

function loadPref<T extends string>(key: string, ok: readonly T[], dflt: T): T {
  try {
    const v = localStorage.getItem(key) as T | null;
    return v && ok.includes(v) ? v : dflt;
  } catch {
    return dflt;
  }
}

function savePref(key: string, v: string) {
  try {
    localStorage.setItem(key, v);
  } catch {
    /* private mode */
  }
}

/** The place a keyframe's pose was selected from, and that pose (Inspector's "Back to place"). */
export const usePlaceReturn = create<{from: Selection | null; pose: string | null; set: (from: Selection | null, pose?: string | null) => void}>(set => ({
  from: null,
  pose: null,
  set: (from, pose = null) => set({from, pose}),
}));

const SORTS: PlaceKfSort[] = ["newest", "oldest", "visible"];
const kfUrl = (src: string, k: PlaceKeyframe, w?: number) => withToken(imageUrl(`${srcPath(src)}/frame/${k.frame_id}`, {w}));

export function PlaceKeyframes({src, sym, onShowPose, hasPose}: {src: string; sym: string; onShowPose: (agent: string) => void; hasPose: (agent: string) => boolean}) {
  const [data, setData] = useState<PlaceKfResp | null>(null);
  const [err, setErr] = useState<string | null>(null);
  const [sort, setSortRaw] = useState<PlaceKfSort>(() => loadPref(SORT_KEY, SORTS, "newest"));
  const [grouped, setGroupedRaw] = useState(() => loadPref(GROUP_KEY, ["1", "0"] as const, "0") === "1");
  const [all, setAll] = useState(false);
  const [lightbox, setLightbox] = useState<number | null>(null);

  useEffect(() => {
    setData(null);
    setErr(null);
    setAll(false);
    setLightbox(null);
    const ctl = new AbortController();
    getJSON<PlaceKfResp>(`${srcPath(src)}/place/${encodeURIComponent(sym)}/keyframes?sort=${sort}`, ctl.signal)
      .then(setData)
      // a console server older than this route answers FastAPI's bare "Not Found": no section
      .catch(e => !ctl.signal.aborted && setErr(e instanceof ApiError && e.status === 404 && e.detail === "Not Found" ? "" : errorText(e)));
    return () => ctl.abort();
    // a sort switch re-orders what is loaded; only a new place fetches
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [src, sym]);

  const setSort = (v: PlaceKfSort) => {
    setSortRaw(v);
    savePref(SORT_KEY, v);
  };
  const setGrouped = (v: boolean) => {
    setGroupedRaw(v);
    savePref(GROUP_KEY, v ? "1" : "0");
  };

  const groups = useMemo(() => (data ? (grouped ? groupBySession(data.keyframes, data.sessions, sort) : [{session: null, items: sortKeyframes(data.keyframes, sort)}]) : []), [data, grouped, sort]);
  // the lightbox walks the keyframes in display order
  const flat = useMemo(() => groups.flatMap(g => g.items), [groups]);

  if (err !== null) return err ? <div className="inline-err small">Keyframes: {err}</div> : null;
  if (!data)
    return (
      <div className="place-kf">
        <h4 className="sub-title">Keyframes</h4>
        <Spinner label="Finding the keyframes that saw this place" />
      </div>
    );
  let shown = 0;
  const cur = lightbox !== null ? flat[lightbox] : null;
  return (
    <div className="place-kf" data-testid="place-keyframes">
      <div className="row gap between wrap place-kf-head">
        <h4 className="sub-title" title={`${data.keyframes_total} keyframes in the map; a keyframe counts when the place centre or its boundary is in view, within 12 m and not occluded in its depth image`}>
          Keyframes ({data.n})
        </h4>
        {data.n > 1 && <Toggle checked={grouped} onChange={setGrouped} label="By session" />}
      </div>
      {data.n > 1 && <Segmented value={sort} onChange={setSort} ariaLabel="Sort keyframes" options={[{value: "newest", label: "Newest"}, {value: "oldest", label: "Oldest"}, {value: "visible", label: "Most visible"}]} />}
      {data.n === 0 && <div className="muted small">{data.keyframes_total ? "No keyframe saw this place." : "This map has no keyframes with images."}</div>}
      {groups.map((g, gi) => {
        const items = all ? g.items : g.items.slice(0, Math.max(0, PAGE - shown));
        const offset = groups.slice(0, gi).reduce((n, x) => n + x.items.length, 0);
        shown += items.length;
        if (!items.length) return null;
        return (
          <section key={g.session?.key ?? "all"} className="place-kf-group">
            {g.session && (
              <div className="place-kf-session small" title={g.session.bag ?? g.session.session ?? undefined}>
                <span className="place-kf-session-name">{sessionTitle(g.session)}</span>
                <span className="muted mono">{g.items.length}</span>
              </div>
            )}
            <div className="image-strip place-kf-grid">
              {items.map((k, i) => (
                <button key={k.frame_id} className="thumb thumb-badged" onClick={() => setLightbox(offset + i)} aria-label={`Keyframe ${kfDate(k.t_ns)} ${kfClock(k.t_ns)}, ${kfBadge(k)}`} title={`${kfDate(k.t_ns)} ${kfClock(k.t_ns)} · ${kfBadge(k)}${k.agent ? ` · ${k.agent}` : ""}`}>
                  <img src={kfUrl(src, k, 240)} loading="lazy" decoding="async" alt="" />
                  <span className="thumb-tag mono">
                    <span className="nowrap">{grouped ? kfClock(k.t_ns) : `${kfDate(k.t_ns).slice(5)} ${kfClock(k.t_ns).slice(0, 5)}`.trim()}</span> · <span className="nowrap">{Math.round(k.visible * 100)}%</span>
                  </span>
                </button>
              ))}
            </div>
          </section>
        );
      })}
      {!all && data.n > PAGE && (
        <button className="btn btn-sm" onClick={() => setAll(true)}>
          Show all {data.n}
        </button>
      )}
      {cur && lightbox !== null && (
        <Modal title={`${sym}: keyframe ${lightbox + 1}/${flat.length}`} onClose={() => setLightbox(null)} wide>
          <img className="lightbox-img" src={kfUrl(src, cur)} alt="" />
          <div className="small muted place-kf-meta">
            {kfDate(cur.t_ns)} {kfClock(cur.t_ns)} · {kfBadge(cur)} visible{cur.agent ? ` · pose ${cur.agent}` : ""}
            {(() => {
              const s = data.sessions.find(x => x.key === cur.robot_key);
              return s ? ` · ${sessionName(s)}` : "";
            })()}
          </div>
          <div className="row gap center wrap">
            <button className="btn btn-sm" disabled={lightbox === 0} onClick={() => setLightbox(lightbox - 1)}>
              Previous
            </button>
            {cur.agent && (
              <button
                className="btn btn-sm btn-primary"
                disabled={!hasPose(cur.agent)}
                title={hasPose(cur.agent) ? "Select the robot pose this keyframe was taken from" : "The pose is not in the loaded graph"}
                onClick={() => {
                  usePlaceReturn.getState().set({src, sym}, cur.agent);
                  setLightbox(null);
                  onShowPose(cur.agent!);
                }}
              >
                Show pose on map
              </button>
            )}
            <button className="btn btn-sm" disabled={lightbox >= flat.length - 1} onClick={() => setLightbox(lightbox + 1)}>
              Next
            </button>
          </div>
        </Modal>
      )}
    </div>
  );
}
