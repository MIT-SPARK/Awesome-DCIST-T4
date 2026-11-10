// Map-viewer styling controls: background, per-layer heights, colours, areas
// (left panel) and the screenshot toolbar button.
import {useCallback, useEffect, useMemo, useRef, useState} from "react";
import {pathColors, useViewer, readyModels, type PathColorMode} from "../stores/viewer";
import {useRobotColorOf} from "../lib/palette";
import {useViewStyle} from "../stores/viewStyle";
import {layerInfo} from "../lib/graphModel";
import {Toggle} from "../components/ui";
import {labelSpaceName} from "./layout";
import {
  BG_PRESETS,
  COLOR_KEYS,
  DEFAULT_RULE_COLOR,
  HEIGHT_KEYS,
  defaultLabelColor,
  effectiveShot,
  heightScale,
  rgbToHex,
  type ColorMode,
  type MeshColorMode,
} from "./style";
import {captureView, downloadBlob} from "./screenshot";
import {useBackground} from "./Scene";
import {useTheme, type Theme} from "../stores/theme";
import {satelliteNote, useGeo} from "../stores/geo";

/** Console theme (same switch as the top bar): the whole app, incl. this viewer. */
export function ThemeControl() {
  const theme = useTheme(s => s.theme);
  const setTheme = useTheme(s => s.setTheme);
  return (
    <div className="bg-row">
      <span>Theme</span>
      <span>
        <span className="segmented seg-sm" role="group" aria-label="Console theme">
          {(["dark", "light"] as Theme[]).map(t => (
            <button key={t} type="button" className={theme === t ? "on" : ""} aria-pressed={theme === t} onClick={() => setTheme(t)}>
              {t === "dark" ? "Dark" : "Light"}
            </button>
          ))}
        </span>
      </span>
    </div>
  );
}

export function BackgroundControl() {
  const bg = useViewStyle(s => s.background);
  const grid = useViewStyle(s => s.grid);
  const patch = useViewStyle(s => s.patch);
  const cur = useBackground().toLowerCase();
  return (
    <div className="bg-control">
      <div className="bg-row">
        <span>Background</span>
        <span className="bg-swatches" role="group" aria-label="Background presets">
          {BG_PRESETS.map(p => (
            <button
              key={p.name}
              type="button"
              className={`bg-swatch ${cur === p.color ? "on" : ""}`}
              style={{background: p.color}}
              title={p.name}
              aria-label={`Background ${p.name}`}
              aria-pressed={cur === p.color}
              onClick={() => patch({background: p.color})}
            />
          ))}
          <input type="color" className="color-in" value={cur} onChange={e => patch({background: e.target.value})} aria-label="Background colour" title="Custom background colour" />
        </span>
      </div>
      {bg !== null && (
        <button className="btn btn-xs link-btn" onClick={() => patch({background: null})} title="Back to the style's default background">
          Use style default
        </button>
      )}
      <Toggle checked={grid} onChange={v => patch({grid: v})} label="Grid" title="The ground grid under the map" />
    </div>
  );
}

/** Satellite imagery under the map: off by default, on/off and opacity remembered
 *  per browser (stores/geo.ts). Needs the map's GPS anchor and a tile layer. */
export function SatelliteControl() {
  const on = useGeo(s => s.satellite);
  const opacity = useGeo(s => s.satOpacity);
  const setSatellite = useGeo(s => s.setSatellite);
  const setOpacity = useGeo(s => s.setOpacity);
  const note = useGeo(satelliteNote);
  return (
    <div className="sat-control" data-testid="satellite-control">
      <Toggle checked={on} onChange={setSatellite} label="Satellite imagery" title="Satellite tiles under the map, placed by the map's GPS anchor" />
      {on && (
        <label className="slider-row">
          <span>Satellite opacity</span>
          <input type="range" min={0.1} max={1} step={0.05} value={opacity} onChange={e => setOpacity(Number(e.target.value))} aria-label="Satellite opacity" />
          <span className="mono small">{Math.round(opacity * 100)}%</span>
        </label>
      )}
      {note && <div className="muted small">{note}</div>}
    </div>
  );
}

export function HeightsControl() {
  const {flat, stackHeight, set, vizStyle} = useViewer();
  const heights = useViewStyle(s => s.heights);
  const setHeight = useViewStyle(s => s.setHeight);
  const resetHeights = useViewStyle(s => s.resetHeights);
  const touched = Object.keys(heights).length > 0;
  return (
    <div className="heights">
      <Toggle checked={flat} onChange={v => set({flat: v})} label="Flat (no layer stacking)" />
      {!flat && (
        <>
          <label className="slider-row">
            <span>Layer spacing</span>
            <input type="range" min={0.5} max={10} step={0.5} value={stackHeight} onChange={e => set({stackHeight: Number(e.target.value)})} aria-label="Layer spacing" />
            <span className="mono small">{stackHeight} m</span>
          </label>
          {HEIGHT_KEYS.map(({key, name}) => {
            const v = heightScale(key, vizStyle, heights);
            const follows = key === "bboxes" && heights.bboxes === undefined;
            return (
              <label key={key} className={`slider-row sub ${heights[key] !== undefined ? "set" : ""}`} title={follows ? "Follows the object nodes until moved" : `${v} × layer spacing`}>
                <span>{name}</span>
                <input
                  type="range"
                  min={-1}
                  max={5}
                  step={0.25}
                  value={v}
                  onChange={e => setHeight(key, Number(e.target.value))}
                  onDoubleClick={() => setHeight(key, null)}
                  aria-label={`${name} height`}
                />
                <span className="mono small">{follows ? "= obj" : `${(v * stackHeight).toFixed(1)} m`}</span>
              </label>
            );
          })}
          <div className="row gap-sm">
            <button className="btn btn-xs" onClick={() => setHeight("bboxes", 0)} title="Bounding boxes at their true world height">
              Boxes at true height
            </button>
            <button className="btn btn-xs" disabled={!touched} onClick={resetHeights} title="hydra_visualizer.yaml z_offset_scale defaults">
              Reset heights
            </button>
          </div>
        </>
      )}
    </div>
  );
}

/** Object labels present in the enabled sources: name, id, count. */
function useLabelsPresent() {
  const sources = useViewer(s => s.sources);
  const revs = Object.values(sources)
    .map(s => `${s.id}:${s.enabled}:${s.rev}`)
    .join(",");
  return useMemo(() => {
    const out = new Map<string, {name: string; label: number; count: number}>();
    for (const {model} of readyModels()) {
      for (const n of model.nodes.values()) {
        if (layerInfo(n.layer, n.part).kind !== "objects" || n.label < 0) continue;
        const name = labelSpaceName(model, n) || `label ${n.label}`;
        const cur = out.get(name);
        if (cur) cur.count++;
        else out.set(name, {name, label: n.label, count: 1});
      }
    }
    return [...out.values()].sort((a, b) => b.count - a.count || a.name.localeCompare(b.name));
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [revs]);
}

export function ColorsPanel() {
  const colors = useViewStyle(s => s.colors);
  const labelColors = useViewStyle(s => s.labelColors);
  const meshColor = useViewStyle(s => s.meshColor);
  const {setRule, setLabelColor, resetColors, patch} = useViewStyle.getState();
  const labels = useLabelsPresent();
  const [showAll, setShowAll] = useState(false);
  const objMode = colors.objects?.mode ?? "default";
  const touched = Object.keys(colors).length > 0 || Object.keys(labelColors).length > 0 || meshColor.mode !== "rgb";
  return (
    <section className="vsec">
      <header className="vsec-head">
        <h4>Colors</h4>
        <button className="btn btn-xs" disabled={!touched} onClick={resetColors} title="hydra_visualizer colours everywhere">
          Reset
        </button>
      </header>
      <ul className="color-rules">
        <li className="color-rule">
          <span className="cr-name">Mesh</span>
          <select className="input input-sm" value={meshColor.mode} aria-label="Mesh colour mode" onChange={e => patch({meshColor: {...meshColor, mode: e.target.value as MeshColorMode}})}>
            <option value="rgb">RGB</option>
            <option value="uniform">Uniform</option>
            <option value="height">Height colormap</option>
          </select>
          {meshColor.mode === "uniform" ? (
            <input type="color" className="color-in" value={meshColor.color} aria-label="Mesh colour" onChange={e => patch({meshColor: {...meshColor, color: e.target.value}})} />
          ) : (
            <span className="color-in-ph" />
          )}
        </li>
        {COLOR_KEYS.map(({key, name, label}) => {
          const r = colors[key];
          const mode = r?.mode ?? "default";
          return (
            <li key={key} className="color-rule">
              <span className="cr-name">{name}</span>
              <select
                className="input input-sm"
                value={mode}
                aria-label={`${name} colour mode`}
                onChange={e => {
                  const m = e.target.value as ColorMode;
                  if (m === "default") setRule(key, null);
                  else setRule(key, {mode: m, color: r?.color ?? DEFAULT_RULE_COLOR[key]});
                }}
              >
                <option value="default">{key === "bboxes" ? "Object colour" : "Default"}</option>
                <option value="uniform">Uniform</option>
                {label && <option value="label">By label</option>}
              </select>
              {mode === "uniform" ? (
                <input type="color" className="color-in" value={r?.color ?? DEFAULT_RULE_COLOR[key]} aria-label={`${name} colour`} onChange={e => setRule(key, {color: e.target.value})} />
              ) : (
                <span className="color-in-ph" />
              )}
            </li>
          );
        })}
      </ul>
      {objMode === "label" && (
        <div className="label-palette">
          <div className="row between small muted">
            <span>{labels.length} object labels</span>
            <button className="btn btn-xs" disabled={!Object.keys(labelColors).length} onClick={() => Object.keys(labelColors).forEach(k => setLabelColor(k, null))}>
              Reset labels
            </button>
          </div>
          <ul className="label-list">
            {(showAll ? labels : labels.slice(0, 24)).map(l => {
              const c = labelColors[l.name] ?? rgbToHex(defaultLabelColor(l.label));
              return (
                <li key={l.name}>
                  <input type="color" className="color-in sm" value={c} aria-label={`Colour for ${l.name}`} onChange={e => setLabelColor(l.name, e.target.value)} />
                  <span className={`ll-name ${labelColors[l.name] ? "set" : ""}`}>{l.name}</span>
                  <span className="mono small muted">{l.count}</span>
                </li>
              );
            })}
          </ul>
          {labels.length > 24 && (
            <button className="btn btn-xs" onClick={() => setShowAll(v => !v)}>
              {showAll ? "Fewer" : `All ${labels.length}`}
            </button>
          )}
        </div>
      )}
    </section>
  );
}

/** Area display toggles (Room nodes), shown in the Layers panel when the map has areas. */
export function RegionControls() {
  const regionColorPlaces = useViewStyle(s => s.regionColorPlaces);
  const regionHulls = useViewStyle(s => s.regionHulls);
  const patch = useViewStyle(s => s.patch);
  return (
    <div className="region-controls">
      <Toggle checked={regionHulls} onChange={v => patch({regionHulls: v})} label="Area boundaries" title="Each area's boundary (Room nodes: the drawn polygon, else the ground its places cover)" />
      <Toggle
        checked={regionColorPlaces}
        onChange={v => patch({regionColorPlaces: v})}
        label="Colour members by area"
        title="Places take their area's colour (hydra ParentColorAdapter)"
      />
    </div>
  );
}

/** Robot paths: colour mode, bag labels, and a legend of each path's session, robot
 *  and source bag (from the map's trajectory ``robots``). */
export function PathControls() {
  const {sources, order, showTrajectory, pathColor, pathLabels, set} = useViewer();
  const robotColorOf = useRobotColorOf();
  const trajOpacity = useViewStyle(s => s.trajectoryOpacity);
  const patchStyle = useViewStyle(s => s.patch);
  const robots = order.flatMap(id => (sources[id]?.enabled ? sources[id].trajectoryRobots : []));
  const colors = pathColors(robots, pathColor, robotColorOf);
  return (
    <div className="path-controls">
      <Toggle checked={showTrajectory} onChange={v => set({showTrajectory: v})} label="Robot paths" />
      {showTrajectory && (
        <>
          <label className="slider-row">
            <span>Path colour</span>
            <select className="input input-sm" value={pathColor} onChange={e => set({pathColor: e.target.value as PathColorMode})} aria-label="Path colour">
              <option value="bag">Per bag</option>
              <option value="robot">Per robot</option>
              <option value="single">Single colour</option>
            </select>
          </label>
          <Toggle checked={pathLabels} onChange={v => set({pathLabels: v})} label="Label paths with their bag" title="Each path's session, robot and source bag at its start" />
          <label className="slider-row">
            <span>Opacity</span>
            <input type="range" min={0.1} max={1} step={0.05} value={trajOpacity} onChange={e => patchStyle({trajectoryOpacity: Number(e.target.value)})} aria-label="Robot path opacity" />
            <span className="mono small">{Math.round(trajOpacity * 100)}%</span>
          </label>
          {robots.some(r => r.session || r.bag) && (
            <ul className="path-legend">
              {robots.map(r => (
                <li key={r.key} title={r.bag}>
                  <span className="path-swatch" style={{background: colors?.[r.key] ?? "transparent"}} />
                  <span>
                    <b>{r.session ?? r.key}</b>
                    {r.robot && r.robot !== r.session ? ` · ${r.robot}` : ""}
                    {r.bag && <span className="mono small muted path-bag">{r.bag}</span>}
                  </span>
                </li>
              ))}
            </ul>
          )}
        </>
      )}
    </div>
  );
}

// ------------------------------------------------------------ screenshot button

function useShotContext() {
  const {sources, order, vizStyle} = useViewer();
  const bg = useBackground();
  return useCallback(() => {
    const on = order.map(id => sources[id]).filter(s => s?.enabled);
    return {sourceIds: on.map(s => s.id), sourceLabels: on.map(s => s.label), background: bg, style: vizStyle};
  }, [sources, order, vizStyle, bg]);
}

/** Toolbar camera button: click saves with the current options; the caret opens them. Shortcut: P. */
export function ScreenshotButton({onDone, compact}: {onDone: (msg: string) => void; compact?: boolean}) {
  const shot = useViewStyle(s => s.shot);
  const patch = useViewStyle(s => s.patch);
  const [open, setOpen] = useState(false);
  const [busy, setBusy] = useState(false);
  const ref = useRef<HTMLDivElement>(null);
  const ctx = useShotContext();
  const save = useCallback(async () => {
    setBusy(true);
    try {
      const r = await captureView(useViewStyle.getState().shot, ctx());
      downloadBlob(r.blob, r.filename);
      onDone(`Saved ${r.filename} (${r.width}×${r.height})`);
    } catch (e) {
      onDone(`Screenshot failed: ${e instanceof Error ? e.message : String(e)}`);
    } finally {
      setBusy(false);
    }
  }, [ctx, onDone]);
  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      const t = e.target as HTMLElement | null;
      if (e.ctrlKey || e.metaKey || e.altKey || e.key.toLowerCase() !== "p") return;
      if (t && (t.isContentEditable || /^(INPUT|TEXTAREA|SELECT)$/.test(t.tagName))) return;
      e.preventDefault();
      void save();
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, [save]);
  useEffect(() => {
    if (!open) return;
    const key = (e: KeyboardEvent) => e.key === "Escape" && setOpen(false);
    const down = (e: PointerEvent) => ref.current && !ref.current.contains(e.target as Node) && setOpen(false);
    window.addEventListener("keydown", key);
    window.addEventListener("pointerdown", down);
    return () => {
      window.removeEventListener("keydown", key);
      window.removeEventListener("pointerdown", down);
    };
  }, [open]);
  const eff = effectiveShot(shot);
  const setShot = (p: Partial<typeof shot>) => patch({shot: {...shot, ...p}});
  return (
    <div className="help-wrap shot-wrap" ref={ref}>
      <div className="shot-split">
        <button className={`btn ${compact ? "icon-only" : "btn-sm"} panel-btn`} onClick={() => void save()} disabled={busy} aria-label="Save screenshot" title="Save screenshot of the 3D view (P)">
          <svg width={compact ? 20 : 16} height={compact ? 20 : 16} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth={1.7} strokeLinecap="round" strokeLinejoin="round" aria-hidden>
            <path d="M4 8h3l2-3h6l2 3h3v11H4z" />
            <circle cx="12" cy="13" r="3.5" />
          </svg>
        </button>
        <button className={`btn ${compact ? "icon-only" : "btn-sm"} panel-btn shot-caret ${open ? "on" : ""}`} onClick={() => setOpen(o => !o)} aria-label="Screenshot options" aria-expanded={open} title="Screenshot options">
          ▾
        </button>
      </div>
      {open && (
        <div className="help-card shot-card" role="dialog" aria-label="Screenshot options">
          <h5>Screenshot</h5>
          <label className="slider-row">
            <span>Format</span>
            <select className="input input-sm" value={shot.format} aria-label="Screenshot format" onChange={e => setShot({format: e.target.value as typeof shot.format})}>
              <option value="png">PNG</option>
              <option value="jpeg">JPEG</option>
            </select>
            <span />
          </label>
          {shot.format === "jpeg" && (
            <label className="slider-row">
              <span>Quality</span>
              <input type="range" min={0.5} max={1} step={0.01} value={shot.quality} aria-label="JPEG quality" onChange={e => setShot({quality: Number(e.target.value)})} />
              <span className="mono small">{Math.round(shot.quality * 100)}</span>
            </label>
          )}
          <label className="slider-row">
            <span>Resolution</span>
            <select className="input input-sm" value={shot.scale} aria-label="Screenshot resolution" onChange={e => setShot({scale: Number(e.target.value) === 2 ? 2 : 1})}>
              <option value={1}>1× (as shown)</option>
              <option value={2}>2×</option>
            </select>
            <span />
          </label>
          <label className={`mini-check ${shot.format === "jpeg" ? "disabled" : ""}`} title={shot.format === "jpeg" ? "JPEG has no transparency: composited onto the background colour" : ""}>
            <input type="checkbox" checked={eff.transparent} disabled={shot.format === "jpeg"} onChange={e => setShot({transparent: e.target.checked})} /> Transparent background
            {shot.format === "jpeg" && <span className="muted small"> (PNG only)</span>}
          </label>
          <label className="mini-check">
            <input type="checkbox" checked={shot.labels} onChange={e => setShot({labels: e.target.checked})} /> Text labels
          </label>
          <label className="mini-check">
            <input type="checkbox" checked={shot.legend} onChange={e => setShot({legend: e.target.checked})} /> Legend (source, time, scale bar)
          </label>
          <button
            className="btn btn-sm btn-primary"
            disabled={busy}
            onClick={() => {
              setOpen(false);
              void save();
            }}
          >
            Save .{eff.ext}
          </button>
        </div>
      )}
    </div>
  );
}
