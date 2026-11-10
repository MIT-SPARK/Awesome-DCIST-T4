// The active map (API.md §6.7): pick which run the console's Neo4j holds and watch
// it load. The scene graph comes first; images, place features and keyframes fill
// in after, each with its own bar. Runs without an image store skip those stages.
import {useEffect, useState} from "react";
import {Badge} from "../components/ui";
import {activateMap, bindLiveMap, mapLoading, mapProgress, refreshMapChoices, useLiveMap, type LiveMap, type MapStage} from "../stores/liveMap";

const STATE_TEXT: Record<MapStage["state"], string> = {
  pending: "waiting",
  running: "loading",
  done: "done",
  skipped: "not in this map",
  failed: "failed",
};

export function MapLoadBars({map, compact = false}: {map: LiveMap; compact?: boolean}) {
  const stages = compact ? map.stages.filter(s => s.state !== "skipped") : map.stages;
  return (
    <div className={`map-load${compact ? " compact" : ""}`} data-testid="map-load">
      {stages.map(s => (
        <div key={s.id} className={`map-stage map-stage-${s.state}`} data-testid={`map-stage-${s.id}`} title={s.detail ?? undefined}>
          <span className="map-stage-label">{s.label}</span>
          <div className="map-stage-bar" role="progressbar" aria-label={s.label} aria-valuemin={0} aria-valuemax={100} aria-valuenow={s.state === "done" ? 100 : s.pct}>
            <div className="map-stage-fill" style={{width: `${s.state === "done" ? 100 : s.state === "skipped" ? 0 : s.pct}%`}} />
          </div>
          <span className="map-stage-state muted">{s.state === "running" && s.detail ? s.detail : STATE_TEXT[s.state]}</span>
        </div>
      ))}
    </div>
  );
}

export function LiveMapPanel() {
  const {map, choices, error} = useLiveMap();
  const [pick, setPick] = useState("");
  useEffect(() => {
    bindLiveMap();
    void refreshMapChoices();
  }, []);
  const loading = mapLoading(map);
  const current = choices.find(c => map?.run && (c.path === map.run || c.name === map.name));
  return (
    <section className="vsec" data-testid="live-map">
      <header className="vsec-head">
        <h4>Map</h4>
        <span className="grow" />
        {map?.ready && <Badge tone="ok" title="The scene graph is in Neo4j">Graph loaded</Badge>}
        {map && !map.ready && map.job_id && <Badge tone="warn">Loading…</Badge>}
      </header>
      <div className="row gap">
        <select
          className="input input-xs grow"
          aria-label="Live map"
          value={pick || current?.path || ""}
          disabled={loading}
          onChange={e => {
            const c = choices.find(x => x.path === e.target.value);
            setPick(e.target.value);
            if (c) void activateMap(c.path, c.name).finally(() => setPick(""));
          }}
        >
          {!current && <option value="">{map?.name ?? "— pick a map —"}</option>}
          {choices.map(c => (
            <option key={c.path} value={c.path}>
              {c.name}
              {c.hasImages ? "" : " (no images)"}
            </option>
          ))}
        </select>
        <button className="btn btn-xs" onClick={() => void refreshMapChoices()} title="Rescan this machine's runs">
          ↻
        </button>
      </div>
      {map && (
        <div className="small muted" title={map.run ?? undefined}>
          neo4j:{map.target} · {map.nodes ?? "?"} nodes · {map.has_images ? "with images" : "no images"}
          {loading && ` · ${mapProgress(map)}%`}
        </div>
      )}
      {map && (loading || map.stages.some(s => s.state === "failed")) && <MapLoadBars map={map} />}
      {error && <div className="inline-err small">{error}</div>}
    </section>
  );
}

/** A small floating progress while a map loads (the viewer, whichever panel is open). */
export function MapLoadPill() {
  const map = useLiveMap(s => s.map);
  useEffect(() => bindLiveMap(), []);
  if (!map || !mapLoading(map)) return null;
  return (
    <div className="map-load-pill" data-testid="map-load-pill">
      <div className="row gap small">
        <b>{map.name}</b>
        <span className="muted">{map.ready ? "graph loaded · loading the rest" : "loading the scene graph"}</span>
        <span className="grow" />
        <span className="mono">{mapProgress(map)}%</span>
      </div>
      <MapLoadBars map={map} compact />
    </div>
  );
}
