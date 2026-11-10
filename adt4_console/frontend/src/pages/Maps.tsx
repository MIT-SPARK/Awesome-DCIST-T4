// Maps page: the runs under the console's output root and the graph sources it
// serves. Open either in the viewer, or load a run into Neo4j as the active map
// (a tracked job; its stages show here and in the viewer's Map tab).
import {useEffect, useMemo, useRef, useState, type ReactNode} from "react";
import {useNavigate} from "react-router";
import {errorText, getJSON, postJSON, useApi} from "../lib/api";
import {activateMap, bindLiveMap, mapLoading, mapProgress, useLiveMap} from "../stores/liveMap";
import type {GraphSource, MapEntry, MapsResponse} from "../types";
import {Badge, Card, Empty, ErrorBox, Loading, PageHeader, Spinner, Toggle} from "../components/ui";
import {MapLoadBars} from "../viewer/LiveMapPanel";
import {fmtBytes, matchRanges} from "../lib/fuzzy";
import {isIncomplete, loadPrefs, mapTime, savePrefs, searchMaps, sortMaps, SORT_LABELS, type MapRow, type MapsPrefs, type SearchHit, type SortKey} from "../lib/mapList";

const fmtDate = (epochS: number | null | undefined, withTime = true): string => {
  if (!epochS) return "-";
  const d = new Date(epochS * 1000);
  const pad = (n: number) => String(n).padStart(2, "0");
  const day = `${d.getFullYear()}-${pad(d.getMonth() + 1)}-${pad(d.getDate())}`;
  return withTime ? `${day} ${pad(d.getHours())}:${pad(d.getMinutes())}` : day;
};

function sizeText(m: MapEntry): string {
  if (typeof m.size_bytes === "number") return fmtBytes(m.size_bytes);
  if (typeof m.size === "number") return fmtBytes(m.size);
  return m.size && m.size !== "?" ? m.size : "-";
}

/** `text` with the fuzzy-matched characters marked. */
export function Highlight({text, indices}: {text: string; indices?: number[]}) {
  if (!indices?.length) return <>{text}</>;
  const parts: ReactNode[] = [];
  let at = 0;
  for (const [a, b] of matchRanges(indices.filter(i => i < text.length))) {
    if (a > at) parts.push(text.slice(at, a));
    parts.push(
      <mark key={a} className="fz-hit">
        {text.slice(a, b)}
      </mark>,
    );
    at = b;
  }
  if (at < text.length) parts.push(text.slice(at));
  return <>{parts}</>;
}

/** "images: 559 kf · 117 MB" (+ check result) or "no images". */
export function ImageStoreBadge({map}: {map: MapEntry}) {
  const st = map.image_store;
  if (!map.has_image_store || !st)
    return (
      <span className="img-badge">
        <Badge tone="off">no images</Badge>
      </span>
    );
  const kf = st.frames?.keyframe ?? 0;
  const size = st.size_bytes != null ? ` · ${fmtBytes(st.size_bytes)}` : map.size_pending ? " · …" : "";
  const kinds = Object.entries(st.frames ?? {})
    .map(([k, n]) => `${n} ${k}`)
    .join(", ");
  const title = [
    `image store: ${kinds || "no frames"}`,
    `${st.observations} object observations, ${st.sensors} sensor(s)`,
    map.sessions ? `${map.sessions} earlier session store(s) under sessions/` : "",
    st.check.ok ? "quick check: ok" : `quick check FAILED: ${st.check.problems.join("; ")}`,
  ]
    .filter(Boolean)
    .join("\n");
  return (
    <span className="img-badge">
      <Badge tone={st.check.ok ? "ok" : "warn"} title={title}>
        images: {kf} kf{size}
        {!st.check.ok && " · check failed"}
      </Badge>
    </span>
  );
}

function FileChips({map}: {map: MapEntry}) {
  if (map.has_dsg === undefined) return null; // older backend
  const files: [string, boolean | undefined][] = [
    ["dsg.json", map.has_dsg],
    ["dsg_with_mesh", map.has_dsg_mesh],
    ["mesh.ply", map.has_mesh],
  ];
  return (
    <>
      {files.map(([label, ok]) => (
        <span key={label} className={`chip mono ${ok ? "chip-ok" : "chip-missing"}`} title={`hydra/backend/${label}${label.endsWith("mesh") ? ".json" : ""} ${ok ? "present" : "missing"}`}>
          {label}
        </span>
      ))}
    </>
  );
}

const OTHER_SUBDIR_SKIP = new Set(["image_store"]);
const MAX_SUBDIR_CHIPS = 6;

function SubdirChips({subdirs}: {subdirs: string[]}) {
  const all = subdirs.filter(s => !OTHER_SUBDIR_SKIP.has(s));
  const rest = all.slice(MAX_SUBDIR_CHIPS);
  return (
    <>
      {all.slice(0, MAX_SUBDIR_CHIPS).map(s => (
        <span key={s} className="chip mono chip-dim">
          {s}/
        </span>
      ))}
      {rest.length > 0 && (
        <span className="chip chip-dim" title={rest.map(s => `${s}/`).join("\n")}>
          +{rest.length} more
        </span>
      )}
    </>
  );
}

/** The active map: which run the console's Neo4j holds and how far its load is. */
function ActiveMapCard() {
  const map = useLiveMap(s => s.map);
  const error = useLiveMap(s => s.error);
  useEffect(() => bindLiveMap(), []);
  if (!map && !error) return null;
  const loading = mapLoading(map);
  return (
    <Card
      title={
        <span className="row gap">
          Active map
          {map?.ready && !loading && <Badge tone="ok">loaded</Badge>}
          {loading && <Badge tone="warn">loading · {mapProgress(map)}%</Badge>}
        </span>
      }
    >
      {map && (
        <div className="small" data-testid="active-map">
          <span className="mono strong">{map.name ?? "none"}</span>{" "}
          <span className="muted">
            in neo4j:{map.target} · {map.nodes ?? "?"} nodes · {map.has_images ? "with images" : "no images"}
          </span>
          {map.run && <div className="muted small mono ellipsis">{map.run}</div>}
        </div>
      )}
      {map && (loading || map.stages.some(s => s.state === "failed")) && <MapLoadBars map={map} />}
      {error && <div className="inline-err small">{error}</div>}
    </Card>
  );
}

/** Graph sources the console serves (Neo4j databases, registered run dirs). */
function SourcesCard({sources, onOpen}: {sources: GraphSource[]; onOpen: (id: string) => void}) {
  if (!sources.length) return null;
  return (
    <Card title="Graph sources" actions={<span className="muted small">{sources.length} sources</span>}>
      <div className="table-wrap">
        <table className="table stack maps-table">
          <thead>
            <tr>
              <th>source</th>
              <th className="num">nodes</th>
              <th className="actions-col">actions</th>
            </tr>
          </thead>
          <tbody>
            {sources.map(s => (
              <tr key={s.id} data-testid="graph-source">
                <td className="cell-title">
                  <div className="row gap wrap">
                    <span className="mono strong">{s.label || s.id}</span>
                    <Badge tone={s.kind === "neo4j" ? "info" : "off"}>{s.kind}</Badge>
                  </div>
                  <div className="muted small mono">{s.id}</div>
                </td>
                <td className="num mono" data-label="nodes">
                  {s.nodes?.toLocaleString() ?? "-"}
                </td>
                <td className="actions-col cell-actions">
                  <div className="row gap-sm end wrap">
                    <button className="btn btn-sm btn-primary" onClick={() => onOpen(s.id)}>
                      Open in viewer
                    </button>
                  </div>
                </td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>
    </Card>
  );
}

export function MapsPage() {
  const maps = useApi<MapsResponse>("api/maps?local=true");
  const sources = useApi<GraphSource[]>("api/graph/sources");
  const [query, setQuery] = useState("");
  const [prefs, setPrefsState] = useState<MapsPrefs>(loadPrefs);
  const [err, setErr] = useState<string | null>(null);
  const [opening, setOpening] = useState<string | null>(null);
  const navigate = useNavigate();
  const setPrefs = (p: Partial<MapsPrefs>) =>
    setPrefsState(old => {
      const next = {...old, ...p};
      savePrefs(next);
      return next;
    });

  // "/" focuses the search box (fzf habit), unless already typing somewhere
  const searchRef = useRef<HTMLInputElement>(null);
  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      const t = e.target as HTMLElement | null;
      if (e.key !== "/" || e.ctrlKey || e.metaKey || e.altKey) return;
      if (t && (t.tagName === "INPUT" || t.tagName === "TEXTAREA" || t.tagName === "SELECT" || t.isContentEditable)) return;
      e.preventDefault();
      searchRef.current?.focus();
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, []);

  // sizes still being measured in the background: list again shortly (bounded)
  const polls = useRef(0);
  const pending = maps.data?.pending ?? [];
  useEffect(() => {
    if (!pending.length || maps.loading || polls.current >= 40) return;
    const id = window.setTimeout(() => {
      polls.current += 1;
      maps.reload();
    }, 4000);
    return () => window.clearTimeout(id);
  }, [pending.length, maps.loading, maps.reload]);

  // local=true lists this host's runs only, under `base`
  const local = maps.data?.base ?? "local";
  const listed = maps.data?.machines?.[local];
  const listError = listed && !Array.isArray(listed) ? listed.error : null;
  const rows = useMemo<MapRow[]>(() => (Array.isArray(listed) ? listed.map(map => ({machine: local, map})) : []), [listed, local]);
  const incomplete = useMemo(() => rows.filter(r => isIncomplete(r.map)).length, [rows]);
  const shown = useMemo(() => {
    const kept = prefs.hideIncomplete ? rows.filter(r => !isIncomplete(r.map)) : rows;
    return sortMaps(searchMaps(kept, query), prefs.sort, query);
  }, [rows, query, prefs.sort, prefs.hideIncomplete]);
  const searching = query.trim().length > 0;
  const total = rows.length;
  const withImages = rows.filter(r => r.map.has_image_store).length;
  const sourceList = Array.isArray(sources.data) ? sources.data : [];

  const openInViewer = async (m: MapEntry) => {
    setErr(null);
    setOpening(m.name);
    try {
      // reuse an already registered file source pointing at this run dir, else register one
      const list = await getJSON<GraphSource[]>("api/graph/sources").catch(() => [] as GraphSource[]);
      let id = (Array.isArray(list) ? list : []).find(s => s.id === `file:${m.name}`)?.id;
      if (!id) {
        await postJSON("api/maps/register", {name: m.name, path: m.abs_path || m.path});
        id = `file:${m.name}`;
      }
      navigate(`/viewer?src=${encodeURIComponent(id)}`);
    } catch (e) {
      setErr(errorText(e));
    } finally {
      setOpening(null);
    }
  };

  return (
    <div className="page">
      <PageHeader
        title="Maps"
        sub={
          maps.data ? (
            <span className="row gap wrap">
              {searching || (prefs.hideIncomplete && incomplete) ? `${shown.length} of ${total} runs` : `${total} runs`} · {withImages} with an image store
              {prefs.hideIncomplete && incomplete > 0 && ` · ${incomplete} incomplete hidden`}
              {pending.length > 0 && (
                <span className="muted small row gap-sm">
                  <Spinner /> measuring sizes
                </span>
              )}
            </span>
          ) : (
            "Runs on this machine and the graph sources the console serves"
          )
        }
        actions={
          <>
            <input
              className="input maps-search"
              type="search"
              placeholder="Search maps (fuzzy)  /"
              value={query}
              onChange={e => setQuery(e.target.value)}
              onKeyDown={e => e.key === "Escape" && setQuery("")}
              aria-label="Search maps"
              ref={searchRef}
            />
            <select className="input maps-sort" value={prefs.sort} onChange={e => setPrefs({sort: e.target.value as SortKey})} aria-label="Sort maps">
              {(Object.keys(SORT_LABELS) as SortKey[])
                .filter(k => k !== "machine")
                .map(k => (
                  <option key={k} value={k}>
                    {SORT_LABELS[k]}
                  </option>
                ))}
            </select>
            <Toggle checked={prefs.hideIncomplete} onChange={v => setPrefs({hideIncomplete: v})} label="Hide incomplete" />
            <button
              className="btn"
              onClick={() => {
                maps.reload();
                sources.reload();
              }}
              disabled={maps.loading}
            >
              {maps.loading && <Spinner />} Rescan
            </button>
          </>
        }
      />
      {err && <ErrorBox error={err} compact />}
      <ActiveMapCard />
      <SourcesCard sources={sourceList} onOpen={id => navigate(`/viewer?src=${encodeURIComponent(id)}`)} />
      {maps.loading && !maps.data && <Loading label="Listing runs" />}
      {maps.error && !maps.data && <ErrorBox error={maps.error} onRetry={maps.reload} />}
      {listError && <div className="inline-err mono">{listError}</div>}
      {maps.data && (
        <Card className="map-machine" title="Runs" actions={<span className="muted small">{shown.length} runs</span>}>
          {total === 0 && !listError ? (
            <Empty title="No runs found">The console lists run directories under its --output-root.</Empty>
          ) : (
            <MapTable rows={shown} opening={opening} onOpen={r => openInViewer(r.map)} emptyText={searching ? `No map matches "${query}".` : "No runs."} />
          )}
        </Card>
      )}
    </div>
  );
}

function MapTable({rows, opening, emptyText, onOpen}: {rows: SearchHit[]; opening: string | null; emptyText: string; onOpen: (r: SearchHit) => void}) {
  if (!rows.length) return <div className="muted pad">{emptyText}</div>;
  return (
    <div className="table-wrap">
      <table className="table stack maps-table">
        <thead>
          <tr>
            <th>experiment</th>
            <th>date</th>
            <th className="num">size</th>
            <th>contents</th>
            <th className="actions-col">actions</th>
          </tr>
        </thead>
        <tbody>
          {rows.map(r => {
            const m = r.map;
            const t = mapTime(m);
            const path = m.abs_path || m.path || "";
            return (
              <tr key={m.path ?? m.name} data-testid="map-row">
                <td className="cell-title">
                  <div className="row gap wrap">
                    <span className="mono strong">
                      <Highlight text={m.name} indices={r.hits.name} />
                    </span>
                    {m.is_prior && <Badge tone="info">prior</Badge>}
                  </div>
                  {m.path && (
                    <div className="muted small mono ellipsis" title={m.abs_path ?? m.path}>
                      <Highlight text={m.path} indices={r.hits.path} />
                    </div>
                  )}
                  {m.error && <div className="inline-err mono small">{m.error}</div>}
                </td>
                <td className="mono small nowrap" data-label="date" title={m.mtime ? `modified ${fmtDate(m.mtime)}` : m.name_date ? "date parsed from the run name" : "unknown"}>
                  {m.mtime ? fmtDate(t) : m.name_date ? `${fmtDate(m.name_date, false)}*` : "-"}
                </td>
                <td className="num mono nowrap" data-label="size">
                  {m.size_pending ? (
                    <span className="muted" title="measuring (du) in the background">
                      <Spinner />
                    </span>
                  ) : (
                    sizeText(m)
                  )}
                </td>
                <td data-label="contents">
                  <div className="chips">
                    <ImageStoreBadge map={m} />
                    <FileChips map={m} />
                    <SubdirChips subdirs={m.subdirs ?? []} />
                  </div>
                </td>
                <td className="actions-col cell-actions">
                  <div className="row gap-sm end wrap">
                    <button
                      className="btn btn-sm"
                      onClick={() => void activateMap(path, m.name)}
                      disabled={!path || !(m.has_dsg || m.has_dsg_mesh)}
                      title="Load it into Neo4j as the console's active map (a job: the scene graph first, then images and features)"
                    >
                      Load into Neo4j
                    </button>
                    <button className="btn btn-sm btn-primary" onClick={() => onOpen(r)} disabled={opening === m.name || !path}>
                      {opening === m.name && <Spinner />} Open in viewer
                    </button>
                  </div>
                </td>
              </tr>
            );
          })}
        </tbody>
      </table>
    </div>
  );
}
