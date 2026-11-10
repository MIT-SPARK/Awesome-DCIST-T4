import {useEffect, useMemo, useState} from "react";
import {useViewer, readyModels, srcPath} from "../stores/viewer";
import {layerHidden, layerInfo, searchNodes} from "../lib/graphModel";
import {nodeColor, LAYER_COLORS, rgbCss} from "../lib/palette";
import {errorText, getJSON} from "../lib/api";
import {Badge, Dot, Empty, Spinner, Toggle} from "../components/ui";
import type {NodeDetail} from "../types";
import {displayPos, defaultEdgeOn, EDGE_COLORS, HYDRA_EDGES} from "./layout";
import type {TileStats} from "./MeshTiles";
import {fmtBytes} from "../lib/fuzzy";
import {DETAIL_LEVEL, levelBytes, saveDetail, type MeshDetail} from "../lib/meshPlan";
import {StatsPanel} from "./StatsPanel";
import {ObjectImages, SearchThumb, useImaged, type ImageInfo} from "./objectImages";
import {PlaceKeyframes, usePlaceReturn} from "./PlaceKeyframes";
import {useViewStyle} from "../stores/viewStyle";
import {BackgroundControl, HeightsControl, PathControls, RegionControls, SatelliteControl, ThemeControl} from "./StylePanels";

const KIND_LABEL: Record<string, string> = {neo4j: "Neo4j", file: "file"};

export function SourcesPanel({stats}: {stats: Record<string, TileStats>}) {
  const {sources, order, catalogError, loadCatalog, enableSource, patchSource, reloadSource} = useViewer();
  return (
    <section className="vsec">
      <header className="vsec-head">
        <h4>Sources</h4>
        <button className="btn btn-xs" onClick={() => void loadCatalog()}>
          Refresh
        </button>
      </header>
      {catalogError && <div className="inline-err small">{catalogError}</div>}
      {!catalogError && order.length === 0 && <div className="muted small">No graph sources. Load a map from the Maps page.</div>}
      <ul className="src-list">
        {order.map(id => {
          const s = sources[id];
          if (!s) return null;
          const st = stats[id];
          return (
            <li key={id} className={s.enabled ? "on" : ""}>
              <div className="row gap between">
                <Toggle checked={s.enabled} onChange={v => enableSource(id, v)} label={<span className="src-label">{s.label}</span>} />
                <Badge tone={s.kind === "neo4j" ? "info" : "off"}>{KIND_LABEL[s.kind] ?? s.kind}</Badge>
              </div>
              <div className="src-meta mono small">{id}</div>
              {s.enabled && (
                <div className="src-sub">
                  {s.status === "loading" && <Spinner label="loading" />}
                  {s.status === "error" && (
                    <div className="inline-err small">
                      {s.error}{" "}
                      <button className="btn btn-xs" onClick={() => void reloadSource(id)}>
                        Retry
                      </button>
                    </div>
                  )}
                  {s.status === "ready" && (
                    <>
                      <div className="row gap small">
                        <label className="mini-check">
                          <input type="checkbox" checked={s.showMesh} onChange={e => patchSource(id, {showMesh: e.target.checked})} /> mesh
                        </label>
                        <label className="mini-check">
                          <input type="checkbox" checked={s.showGraph} onChange={e => patchSource(id, {showGraph: e.target.checked})} /> graph
                        </label>
                        <span className="muted">{s.model?.nodes.size ?? 0} nodes</span>
                      </div>
                      {st && st.total > 0 && (
                        <div className="tile-bar" title={`${st.loaded}/${st.total} tiles`}>
                          <div className="tile-fill" style={{width: `${(100 * st.loaded) / st.total}%`}} />
                          <span className="tile-text mono">
                            {st.loaded}/{st.total} tiles · {fmtBytes(st.triangles).replace("B", "").trim()} tris
                          </span>
                        </div>
                      )}
                    </>
                  )}
                </div>
              )}
            </li>
          );
        })}
      </ul>
    </section>
  );
}

/** Layer swatches in hydra style (places black, rooms/agents colorbrewer). */
const HYDRA_SWATCH: Record<string, string> = {places: "#000000", rooms: "#a6cee3", agents: "#1f78b4", subkeyframes: "#ff7f00"};

export function LayersPanel() {
  const {sources, order, hiddenLayers, edgeKinds, set, vizStyle} = useViewer();
  const hydra = vizStyle === "hydra";
  const revs = order.map(id => sources[id]?.rev).join(",");
  const {layers, edgeCounts} = useMemo(() => {
    const layers = new Map<string, {name: string; kind: string; count: number; order: number}>();
    const edgeCounts: Record<string, number> = {};
    for (const {model} of readyModels()) {
      for (const [k, v] of model.layers()) {
        const cur = layers.get(k);
        if (cur) cur.count += v.count;
        else layers.set(k, {name: v.info.name, kind: v.info.kind, count: v.count, order: v.info.order});
      }
      for (const [a, b] of model.edges.values()) {
        const na = model.nodes.get(a);
        const nb = model.nodes.get(b);
        if (!na || !nb) continue;
        const ka = layerInfo(na.layer, na.part).kind;
        const kb = layerInfo(nb.layer, nb.part).kind;
        const k = ka < kb ? `${ka}-${kb}` : `${kb}-${ka}`;
        edgeCounts[k] = (edgeCounts[k] ?? 0) + 1;
      }
    }
    return {layers: [...layers.entries()].sort((a, b) => a[1].order - b[1].order || a[0].localeCompare(b[0])), edgeCounts};
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [revs, order, sources]);
  if (!layers.length) return null;
  return (
    <section className="vsec">
      <header className="vsec-head">
        <h4>Layers</h4>
      </header>
      {layers.some(([, l]) => l.kind === "rooms") && <RegionControls />}
      {(layers.some(([, l]) => l.kind === "agents") || order.some(id => sources[id]?.trajectory?.length)) && <PathControls />}
      <ul className="layer-list">
        {layers.map(([k, l]) => (
          <li key={k}>
            <label className="layer-row">
              <input type="checkbox" checked={!layerHidden(hiddenLayers, ...(k.split("/").map(Number) as [number, number]))} onChange={e => set({hiddenLayers: {...hiddenLayers, [k]: !e.target.checked}})} />
              <span
                className="swatch"
                style={{
                  background:
                    hydra && HYDRA_SWATCH[l.kind] ? HYDRA_SWATCH[l.kind] : rgbCss((LAYER_COLORS as Record<string, [number, number, number]>)[l.kind] ?? [230, 25, 75]),
                }}
              />
              <span className="layer-name" title={l.kind === "rooms" ? "Room nodes (layer 4)" : undefined}>
                {l.kind === "rooms" ? "areas" : l.name}
              </span>
              <span className="mono small muted count-r">{l.count.toLocaleString()}</span>
            </label>
          </li>
        ))}
      </ul>
      <header className="vsec-head">
        <h4>Edges</h4>
      </header>
      <ul className="layer-list">
        {Object.entries(edgeCounts)
          .sort()
          .map(([k, n]) => (
            <li key={k}>
              <label className="layer-row">
                <input type="checkbox" checked={edgeKinds[k] ?? defaultEdgeOn(k, vizStyle)} onChange={e => set({edgeKinds: {...edgeKinds, [k]: e.target.checked}})} />
                <span className="swatch line" style={{background: (hydra ? HYDRA_EDGES[k]?.color : EDGE_COLORS[k]) ?? "#7f8b96"}} />
                <span className="layer-name">{k === "agents-agents" ? "trajectory" : k.replace("-", " / ")}</span>
                <span className="mono small muted count-r">{n.toLocaleString()}</span>
              </label>
            </li>
          ))}
      </ul>
    </section>
  );
}

/** Mesh level of detail, with each level's download size for the shown sources. */
function MeshDetailControl() {
  const detail = useViewer(s => s.meshDetail);
  const auto = useViewer(s => s.meshAutoDetail);
  const sources = useViewer(s => s.sources);
  const set = useViewer(s => s.set);
  const size = (d: Exclude<MeshDetail, "auto">) => {
    let n = 0;
    for (const src of Object.values(sources)) if (src.enabled && src.showMesh) n += levelBytes(src.manifest, DETAIL_LEVEL[d]);
    return n ? ` · ${fmtBytes(n)}` : "";
  };
  const opts: Exclude<MeshDetail, "auto">[] = ["full", "medium", "coarse"];
  return (
    <label className="slider-row">
      <span>Mesh detail</span>
      <select
        className="input input-sm"
        value={detail}
        onChange={e => {
          const d = e.target.value as MeshDetail;
          saveDetail(d); // per device: a phone on a slow link keeps "coarse"
          set({meshDetail: d});
        }}
        aria-label="Mesh detail"
        title="Full: Hydra's mesh. Medium / coarse: simplified on a 0.2 / 0.4 m grid; far less to download over a slow link. Auto: medium on a phone, else full."
      >
        <option value="auto">Auto ({auto})</option>
        {opts.map(d => (
          <option key={d} value={d}>
            {d[0].toUpperCase() + d.slice(1)}
            {size(d)}
          </option>
        ))}
      </select>
    </label>
  );
}

export function DisplayPanel() {
  const {showBBoxes, meshOpacity, vizStyle, showText, meshCull, meshClipOn, meshClipHeight, set} = useViewer();
  return (
    <section className="vsec">
      <header className="vsec-head">
        <h4>Display</h4>
      </header>
      <label className="slider-row">
        <span>Style</span>
        <select
          className="input input-sm"
          value={vizStyle}
          onChange={e => set({vizStyle: e.target.value as typeof vizStyle})}
          aria-label="Graph style"
          title="hydra_visualizer: the ADT4 RViz look (base_params/hydra_visualizer.yaml)"
        >
          <option value="hydra">hydra_visualizer</option>
          <option value="classic">Classic console</option>
        </select>
      </label>
      <ThemeControl />
      <BackgroundControl />
      <SatelliteControl />
      <HeightsControl />
      <MeshDetailControl />
      <label className="slider-row">
        <span>Mesh opacity</span>
        <input type="range" min={0} max={1} step={0.05} value={meshOpacity} onChange={e => set({meshOpacity: Number(e.target.value)})} />
        <span className="mono small">{Math.round(meshOpacity * 100)}%</span>
      </label>
      <Toggle checked={meshCull} onChange={v => set({meshCull: v})} label="Cull mesh back faces (hide ceilings)" />
      <Toggle checked={meshClipOn} onChange={v => set({meshClipOn: v})} label="Clip mesh above a height" />
      {meshClipOn && (
        <label className="slider-row">
          <span>Max height</span>
          <input type="range" min={0.2} max={6} step={0.1} value={meshClipHeight} onChange={e => set({meshClipHeight: Number(e.target.value)})} aria-label="Mesh max height above floor" />
          <span className="mono small">{meshClipHeight.toFixed(1)} m</span>
        </label>
      )}
      <Toggle checked={showBBoxes} onChange={v => set({showBBoxes: v})} label="Object bounding boxes" />
      <Toggle checked={showText} onChange={v => set({showText: v})} label="Text labels" />
    </section>
  );
}

export function SearchBox({autoFocus, onPicked}: {autoFocus?: boolean; onPicked?: (src: string, sym: string) => void} = {}) {
  const {query, set, select, fly, sources, flat, stackHeight, vizStyle} = useViewer();
  const heights = useViewStyle(s => s.heights);
  const [open, setOpen] = useState(false);
  const revs = Object.values(sources).map(s => s.rev).join(",");
  const imaged = useImaged(readyModels().map(({id}) => ({id, rev: sources[id]?.rev ?? 0})));
  const results = useMemo(() => {
    const out: {src: string; sym: string; text: string; kind: string; label: number; color: [number, number, number]}[] = [];
    for (const {id, model} of readyModels()) {
      for (const n of searchNodes(model, query, 200)) {
        out.push({src: id, sym: n.sym, text: model.displayName(n), kind: layerInfo(n.layer, n.part).kind, label: n.label, color: n.color});
      }
    }
    return out;
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [query, revs]);
  useEffect(() => {
    set({highlight: query.trim() ? results.map(r => ({src: r.src, sym: r.sym})) : []});
  }, [results, query, set]);
  const go = (r: (typeof results)[number]) => {
    const n = sources[r.src]?.model?.nodes.get(r.sym);
    if (!n) return;
    select({src: r.src, sym: r.sym});
    fly(displayPos(n, flat, stackHeight, vizStyle, heights));
    setOpen(false);
    onPicked?.(r.src, r.sym);
  };
  return (
    <div className="searchbox">
      <input
        className="input"
        placeholder="Search objects, labels, symbols (e.g. sign, O(12))"
        value={query}
        autoFocus={autoFocus}
        enterKeyHint="search"
        aria-label="Search graph"
        onChange={e => {
          set({query: e.target.value});
          setOpen(true);
        }}
        onFocus={() => setOpen(true)}
        onBlur={() => setTimeout(() => setOpen(false), 150)}
        onKeyDown={e => {
          if (e.key === "Enter" && results[0]) go(results[0]);
          if (e.key === "Escape") {
            set({query: ""});
            setOpen(false);
          }
        }}
      />
      {query && <span className="search-count mono">{results.length >= 200 ? "200+" : results.length} hits</span>}
      {open && query.trim() && results.length > 0 && (
        <ul className="search-results">
          {results.slice(0, 40).map(r => {
            const color = rgbCss(r.kind === "objects" ? nodeColor(r.label, r.color) : (LAYER_COLORS as Record<string, [number, number, number]>)[r.kind] ?? [200, 200, 200]);
            if (r.kind !== "objects")
              return (
                <li key={r.src + r.sym} onMouseDown={e => e.preventDefault()} onClick={() => go(r)}>
                  <span className="swatch" style={{background: color}} />
                  <span className="sr-text">{r.text}</span>
                  <span className="mono small muted">{r.sym}</span>
                </li>
              );
            const set_ = imaged[r.src];
            const has = set_ ? set_.has(r.sym) : null;
            return (
              <li key={r.src + r.sym} className="has-thumb" onMouseDown={e => e.preventDefault()} onClick={() => go(r)}>
                <SearchThumb src={r.src} sym={r.sym} has={has} color={color} />
                <span className="sr-main">
                  <span className="sr-text">{r.text}</span>
                  <span className="sr-sub small">
                    <span className="mono muted">{r.sym}</span>
                    {has === false && <span className="muted">no image</span>}
                  </span>
                </span>
              </li>
            );
          })}
        </ul>
      )}
    </div>
  );
}

export function Inspector() {
  const {selected, sources, fly, flat, stackHeight, select, vizStyle} = useViewer();
  const heights = useViewStyle(s => s.heights);
  const placeReturn = usePlaceReturn(s => s.from);
  const placeReturnPose = usePlaceReturn(s => s.pose);
  const [detail, setDetail] = useState<(NodeDetail & {masks?: boolean[]; image_info?: ImageInfo[]}) | null>(null);
  const [err, setErr] = useState<string | null>(null);
  const [loading, setLoading] = useState(false);
  const node = selected ? sources[selected.src]?.model?.nodes.get(selected.sym) : null;
  const model = selected ? sources[selected.src]?.model : null;
  useEffect(() => {
    setDetail(null);
    setErr(null);
    if (!selected) return;
    const ctl = new AbortController();
    setLoading(true);
    getJSON<NodeDetail & {masks?: boolean[]; image_info?: ImageInfo[]}>(`${srcPath(selected.src)}/node/${encodeURIComponent(selected.sym)}`, ctl.signal)
      .then(setDetail)
      .catch(e => !ctl.signal.aborted && setErr(errorText(e)))
      .finally(() => !ctl.signal.aborted && setLoading(false));
    return () => ctl.abort();
  }, [selected]);
  if (!selected || !node || !model)
    return (
      <>
        <Empty title="Nothing selected">Click a node in the 3D view, or search above.</Empty>
        <StatsPanel />
      </>
    );
  const info = layerInfo(node.layer, node.part);
  const images = detail?.images ?? [];
  const isObject = info.kind === "objects";
  const isAgent = info.kind === "agents";
  // traversability ("mesh") places: the keyframes that observed them
  const isMeshPlace = node.layer === 3 && node.part === 1;
  const flyTo = (src: string, sym: string) => {
    const n = sources[src]?.model?.nodes.get(sym);
    if (n) fly(displayPos(n, flat, stackHeight, vizStyle, heights));
  };
  const backToPlace = placeReturn && isAgent && placeReturn.src === selected.src && placeReturnPose === selected.sym && sources[placeReturn.src]?.model?.nodes.has(placeReturn.sym) ? placeReturn : null;
  const imagesEl = (
    <ObjectImages images={images} masks={detail?.masks} info={detail?.image_info} title={model.displayName(node)} sym={node.sym} isObject={isObject} />
  );
  const color = info.kind === "objects" ? nodeColor(node.label, node.color) : (LAYER_COLORS as Record<string, [number, number, number]>)[info.kind] ?? [200, 200, 200];
  return (
    <div className="inspector">
      <div className="insp-head">
        <span className="swatch big" style={{background: rgbCss(color)}} />
        <div>
          <div className="insp-title">{model.displayName(node)}</div>
          <div className="mono small muted">
            {node.sym} · {info.name} · {sources[selected.src]?.label}
          </div>
        </div>
        <button className="icon-btn" aria-label="Clear selection" onClick={() => select(null)}>
          ×
        </button>
      </div>
      <div className="row gap-sm wrap">
        <button className="btn btn-sm" onClick={() => fly(displayPos(node, flat, stackHeight, vizStyle, heights))}>
          Fly to
        </button>
      </div>
      {backToPlace && (
        <div className="row gap-sm">
          <button className="btn btn-sm" onClick={() => select(backToPlace)} title="Back to the place whose keyframe this pose took">
            ← Back to place {backToPlace.sym}
          </button>
        </div>
      )}
      {isMeshPlace && (
        <PlaceKeyframes
          src={selected.src}
          sym={selected.sym}
          hasPose={agent => Boolean(model.nodes.get(agent))}
          onShowPose={agent => {
            select({src: selected.src, sym: agent});
            flyTo(selected.src, agent);
          }}
        />
      )}
      {(isObject || isAgent) && imagesEl}
      {(isObject || isAgent) && loading && !detail && <div className="obj-hero-wait muted small">Loading images…</div>}
      <table className="table compact kv-table">
        <tbody>
          <tr>
            <th>position</th>
            <td className="mono">{node.pos.map(v => v.toFixed(2)).join(", ")}</td>
          </tr>
          {node.label >= 0 && (
            <tr>
              <th>label</th>
              <td>
                {detail?.label_name ?? model.labelName(node)} <span className="muted mono">({node.label})</span>
              </td>
            </tr>
          )}
          {node.bbox && (
            <tr>
              <th>bbox size</th>
              <td className="mono">
                {(node.bbox[3] - node.bbox[0]).toFixed(2)} x {(node.bbox[4] - node.bbox[1]).toFixed(2)} x {(node.bbox[5] - node.bbox[2]).toFixed(2)} m
              </td>
            </tr>
          )}
          <tr>
            <th>active</th>
            <td>{node.active ? "yes" : "no"}</td>
          </tr>
          {Object.entries(detail?.attributes ?? {}).map(([k, v]) => (
            <tr key={k}>
              <th>{k}</th>
              <td className="mono small attr-v">{typeof v === "object" ? JSON.stringify(v) : String(v)}</td>
            </tr>
          ))}
        </tbody>
      </table>
      {loading && <Spinner label="Loading attributes" />}
      {err && <div className="inline-err small">{err}</div>}
      {!isObject && !isAgent && imagesEl}
    </div>
  );
}

export function SourceDot({status}: {status: string}) {
  return <Dot tone={status === "ready" ? "ok" : status === "error" ? "err" : status === "loading" ? "warn" : "off"} />;
}
