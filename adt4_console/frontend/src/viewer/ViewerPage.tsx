import {useCallback, useEffect, useLayoutEffect, useMemo, useRef, useState, type ReactNode} from "react";
import {useSearchParams} from "react-router";
import {useViewer} from "../stores/viewer";
import {Scene, type SceneCallbacks} from "./Scene";
import type {GraphLayout} from "./layout";
import type {TileStats} from "./MeshTiles";
import {DisplayPanel, Inspector, LayersPanel, SearchBox, SourcesPanel} from "./panels";
import {LiveMapPanel, MapLoadPill} from "./LiveMapPanel";
import {ColorsPanel, ScreenshotButton} from "./StylePanels";
import {ImagesTab} from "./FrameBrowser";
import {Segmented} from "../components/ui";
import {useWsStatus} from "../ws";
import {useIsPhone, useMedia} from "../lib/media";
import {Icon} from "../components/Shell";
import {BottomSheet} from "./BottomSheet";
import {ControlsHelpButton, TopViewHint} from "./ControlsHelp";
import {FloatingWindow, Splitter, TabPortal, TabSlot, useElementSize, usePaneLayout, useTabHosts, useTabTearOff} from "./Panes";
import {FLOAT_MQ, LEFT, MIN_CANVAS, PANE_TABS, RIGHT, clampWidths, dock, popOut, raise, updateWin, type PaneTab} from "./paneLayout";
import {useMapShortcuts} from "./shortcuts";
import {exitMapFull, toggleMapFull, useMapFull} from "./mapFull";
import {LoadBar} from "./LoadBar";
import {autoDetail} from "../lib/meshPlan";
import {loadGeo, useGeo} from "../stores/geo";

type Tab = PaneTab;
const TAB_TITLE: Record<Tab, string> = {inspect: "Inspect", images: "Images", map: "Map"};
/** below this map width the viewer toolbar folds into a "more" menu */
export const NARROW_MAP_W = 700;

export default function ViewerPage() {
  const v = useViewer();
  const [params] = useSearchParams();
  const [tab, setTabRaw] = useState<Tab>("inspect");
  const isPhone = useIsPhone();
  // mesh detail "auto": medium on a phone, full on a desktop (lib/meshPlan autoDetail)
  useEffect(() => {
    useViewer.getState().set({meshAutoDetail: autoDetail(isPhone)});
  }, [isPhone]);
  // pane layout: splitter widths + popped-out tabs (floating windows only on wide screens)
  const [layout, setLayout, resetLayout] = usePaneLayout();
  const wide = useMedia(FLOAT_MQ) && !isPhone;
  const floats = useCallback((t: Tab) => wide && !!layout.windows[t], [wide, layout.windows]);
  const floatsRef = useRef(floats);
  floatsRef.current = floats;
  const [leftOpen, setLeftOpenRaw] = useState(() => window.innerWidth > 1100);
  const [rightOpen, setRightOpenRaw] = useState(() => window.innerWidth > 900);
  const [searchOpen, setSearchOpen] = useState(false);
  const viewerRef = useRef<HTMLDivElement>(null);
  const area = useElementSize(viewerRef);
  const canvasRef = useRef<HTMLDivElement>(null);
  const canvasSize = useElementSize(canvasRef);
  const narrow = !isPhone && canvasSize.w > 0 && canvasSize.w < NARROW_MAP_W;
  // the top-left toolbar stops short of the top-right one (--tr-w), whatever
  // buttons that one shows
  const topRightRef = useRef<HTMLDivElement>(null);
  useLayoutEffect(() => {
    const tr = topRightRef.current;
    const canvas = canvasRef.current;
    if (!tr || !canvas) return;
    const read = () => canvas.style.setProperty("--tr-w", `${Math.ceil(tr.getBoundingClientRect().width)}px`);
    read();
    const ro = new ResizeObserver(read);
    ro.observe(tr);
    return () => ro.disconnect();
  }, [isPhone]);
  const leftRef = useRef<HTMLElement>(null);
  const rightRef = useRef<HTMLElement>(null);
  const stripRef = useRef<HTMLDivElement>(null);
  const hosts = useTabHosts(PANE_TABS);
  // phones show one panel at a time, as a bottom sheet
  const phoneRef = useRef(isPhone);
  phoneRef.current = isPhone;
  const setLeftOpen = useCallback((f: boolean | ((o: boolean) => boolean)) => {
    setLeftOpenRaw(o => {
      const next = typeof f === "function" ? f(o) : f;
      if (next && phoneRef.current) setRightOpenRaw(false);
      return next;
    });
  }, []);
  const setRightOpen = useCallback((f: boolean | ((o: boolean) => boolean)) => {
    setRightOpenRaw(o => {
      const next = typeof f === "function" ? f(o) : f;
      if (next && phoneRef.current) setLeftOpenRaw(false);
      return next;
    });
  }, []);
  /** Bring a tab forward: raise (and restore) its floating window, else select it in the side panel. */
  const setTab = useCallback(
    (t: Tab) => {
      if (floatsRef.current(t)) setLayout(l => updateWin(raise(l, t), t, {min: false}));
      else setTabRaw(t);
    },
    [setLayout],
  );
  const showTab = useCallback(
    (t: Tab) => {
      setTab(t);
      if (!floatsRef.current(t)) setRightOpen(true);
    },
    [setTab, setRightOpen],
  );
  const [stats, setStats] = useState<Record<string, TileStats>>({});
  const [bounds, setBounds] = useState<{min: number[]; max: number[]} | null>(null);
  const [toast, setToastState] = useState<{text: string; err: boolean} | null>(null);
  const setToast = useCallback((text: string | null, err = false) => setToastState(text ? {text, err} : null), []);
  const wsUp = useWsStatus(s => s.connected);
  const initDone = useRef(false);

  // catalog + initial source selection (?src=... or the first neo4j / file source)
  useEffect(() => {
    void (async () => {
      await useViewer.getState().loadCatalog();
      if (initDone.current) return;
      initDone.current = true;
      const st = useViewer.getState();
      const wanted = params.get("src");
      if (wanted) {
        if (!st.sources[wanted]) await st.loadCatalog();
        if (useViewer.getState().sources[wanted]) st.enableSource(wanted, true);
        return;
      }
      if (st.order.some(id => st.sources[id].enabled)) return;
      const first = st.order.find(id => st.sources[id].kind === "neo4j") ?? st.order.find(id => st.sources[id].kind === "file") ?? st.order[0];
      if (first) st.enableSource(first, true);
    })();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);
  // M toggles the full-screen map (viewer/shortcuts.ts)
  useMapShortcuts();
  // refresh the catalog when sources announce new versions / reconnects
  useEffect(() => {
    if (wsUp) void useViewer.getState().loadCatalog();
  }, [wsUp]);

  useEffect(() => {
    if (!toast) return;
    const id = window.setTimeout(() => setToast(null), toast.err ? 8000 : 3500);
    return () => window.clearTimeout(id);
  }, [toast, setToast]);

  const enabled = useMemo(() => v.order.map(id => v.sources[id]).filter(s => s?.enabled), [v.order, v.sources]);

  // satellite underlay: fetched only while it is on, for the first shown source
  // (its GPS anchor) over the map's extent
  const satellite = useGeo(s => s.satellite);
  const geoSrc = enabled[0]?.id ?? null;
  const geoKey = bounds ? bounds.min.concat(bounds.max).map(x => Math.round(x)).join(",") : "";
  useEffect(() => {
    if (!satellite) return;
    void loadGeo(geoSrc, bounds ? [bounds.min[0], bounds.min[1], bounds.max[0], bounds.max[1]] : null);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [satellite, geoSrc, geoKey]);

  // a picked search result: the Inspect tab (object images) comes to the front
  const showInspect = useCallback(() => {
    setTab("inspect");
    setRightOpen(true);
  }, [setTab, setRightOpen]);

  const cb: SceneCallbacks = useMemo(
    () => ({
      onNode: (src, sym) => {
        useViewer.getState().select({src, sym});
        showTab("inspect");
      },
      onPoint: () => {},
      onMiss: () => useViewer.getState().select(null),
      onStats: (src, s) => setStats(cur => ({...cur, [src]: s})),
      onLayout: (_src: string, l: GraphLayout) => {
        if (l.bounds) setBounds(b => b ?? l.bounds);
      },
    }),
    [showTab],
  );

  const totalNodes = enabled.reduce((n, s) => n + (s.model?.nodes.size ?? 0), 0);
  const totalTris = Object.entries(stats).reduce((n, [id, s]) => n + (v.sources[id]?.enabled ? s.triangles : 0), 0);
  const tileText = Object.entries(stats)
    .filter(([id]) => v.sources[id]?.enabled)
    .map(([, s]) => `${s.loaded}/${s.total}`)
    .join(" + ");

  const leftPanel = (
    <>
      <SourcesPanel stats={stats} />
      <LayersPanel />
      <DisplayPanel />
      <ColorsPanel />
      {!isPhone && (
        <section className="vsec">
          <header className="vsec-head">
            <h4>Layout</h4>
          </header>
          <div className="layout-row">
            <span className="muted small">{wide ? "Drag pane edges to resize; pop side-panel tabs out with ⧉." : "Pane widths are fixed on narrow screens."}</span>
            <button className="btn btn-xs" onClick={resetLayout} title="Default pane widths, dock all floating windows">
              Reset layout
            </button>
          </div>
        </section>
      )}
    </>
  );

  const strip = PANE_TABS;
  // the docked side panel shows the selected tab, or the first strip tab that is not floating
  const dockTab: Tab | null = strip.includes(tab) && !floats(tab) ? tab : (strip.find(t => !floats(t)) ?? null);
  const mounted = (t: Tab) => floats(t) || (rightOpen && t === dockTab);
  const pop = useCallback(
    (t: Tab, at?: {x: number; y: number}) => {
      const r = viewerRef.current?.getBoundingClientRect();
      if (!r) return;
      setLayout(l => popOut(l, t, r.width, r.height, at && {x: at.x - r.left, y: at.y - r.top}));
    },
    [setLayout],
  );
  const dockWin = (t: Tab) => {
    setLayout(l => dock(l, t));
    setTabRaw(t);
    setRightOpen(true);
  };
  const canTear = useCallback((t: Tab) => !floats(t), [floats]);
  const onStripDown = useTabTearOff(stripRef, strip, canTear, useCallback((t: Tab, x: number, y: number) => pop(t, {x, y}), [pop]));
  const tabLabel = (t: Tab): ReactNode => (
    <>
      {TAB_TITLE[t]}
      {floats(t) && (
        <span className="tab-float-mark" title="In a floating window" aria-label="(floating)">
          ⧉
        </span>
      )}
    </>
  );
  const tabContent = (t: Tab) => {
    switch (t) {
      case "inspect":
        return <Inspector />;
      case "images":
        return <ImagesTab />;
      case "map":
        return <LiveMapPanel />;
    }
  };
  const tabs = (
    <div className="vright-head" ref={stripRef} onPointerDown={wide ? onStripDown : undefined}>
      <Segmented<Tab> ariaLabel="Side panel" value={dockTab ?? tab} onChange={setTab} options={strip.map(t => ({value: t, label: tabLabel(t)}))} />
      {wide && (
        <button className="pop-btn" disabled={!dockTab} onClick={() => dockTab && pop(dockTab)} aria-label={dockTab ? `Pop out ${TAB_TITLE[dockTab]}` : "Pop out"} title="Pop out into a floating window (or drag a tab out)">
          ⧉
        </button>
      )}
    </div>
  );
  const rightBody = (
    <div className="vright-body">
      {dockTab ? <TabSlot key={dockTab} host={hosts[dockTab]} /> : <div className="dock-empty muted">Every tab is in a floating window. Dock one with ⇥ or drag its title bar onto the tab strip.</div>}
    </div>
  );
  const widths = clampWidths(layout.leftW, layout.rightW, area.w || window.innerWidth, leftOpen, rightOpen, "right");
  const splitMax = (side: "left" | "right") => Math.min(side === "left" ? LEFT.max : RIGHT.max, (area.w || window.innerWidth) - MIN_CANVAS - (side === "left" ? (rightOpen ? widths.rightW : 0) : leftOpen ? widths.leftW : 0));
  // full map (mapFull.ts): the panes close while it is on and come back as they were
  const mapFull = useMapFull(s => s.on);
  const panesBefore = useRef<{left: boolean; right: boolean} | null>(null);
  useEffect(() => {
    if (mapFull) {
      panesBefore.current = {left: leftOpen, right: rightOpen};
      setLeftOpenRaw(false);
      setRightOpenRaw(false);
    } else if (panesBefore.current) {
      setLeftOpenRaw(panesBefore.current.left);
      setRightOpenRaw(panesBefore.current.right);
      panesBefore.current = null;
    }
  }, [mapFull]); // eslint-disable-line react-hooks/exhaustive-deps
  useEffect(() => () => exitMapFull(), []);
  const mapFullBtn = (
    <button className={`btn btn-sm panel-btn ${mapFull ? "on" : ""}`} onClick={toggleMapFull} aria-pressed={mapFull} title={mapFull ? "Leave the full-screen map (M or Esc)" : "Map only, full screen: hides the navigation, top bar and side panels (M)"} data-testid="map-full">
      {mapFull ? "⤡ Exit full map" : narrow ? "⛶" : "⛶ Full map"}
    </button>
  );
  const camera = (
    <Segmented
      ariaLabel="Camera"
      value={v.topDown ? "top" : "3d"}
      onChange={x => v.set({topDown: x === "top"})}
      options={[
        {value: "3d", label: "3D"},
        {value: "top", label: "Top"},
      ]}
    />
  );

  return (
    <div ref={viewerRef} className={`viewer ${leftOpen ? "left-open" : ""} ${rightOpen ? "right-open" : ""} ${isPhone ? "phone" : ""} ${narrow ? "map-narrow" : ""}`}>
      {PANE_TABS.filter(mounted).map(t => (
        <TabPortal key={t} host={hosts[t]}>
          {tabContent(t)}
        </TabPortal>
      ))}
      {leftOpen && !isPhone && (
        <aside ref={leftRef} className="vpanel vleft" aria-label="Sources and layers" style={wide ? {width: widths.leftW} : undefined}>
          {leftPanel}
        </aside>
      )}
      {leftOpen && wide && (
        <Splitter
          side="left"
          label="Resize layers panel"
          value={widths.leftW}
          min={LEFT.min}
          max={splitMax("left")}
          target={leftRef}
          onCommit={w => setLayout(l => ({...l, leftW: w}))}
          onReset={() => setLayout(l => ({...l, leftW: LEFT.def}))}
        />
      )}
      <div className="vcanvas" ref={canvasRef}>
        {enabled.length === 0 && (
          <div className="canvas-empty">
            <div className="state-title">No source shown</div>
            <div className="muted">Enable a graph source in the {isPhone ? "layers sheet" : "left panel"}.</div>
          </div>
        )}
        <Scene sources={enabled} bounds={bounds} cb={cb} />
        <MapLoadPill />
        {isPhone ? (
          searchOpen ? (
            <div className="vtoolbar phone-bar phone-search">
              <SearchBox
                autoFocus
                onPicked={() => {
                  setSearchOpen(false);
                  showInspect();
                }}
              />
              <button className="btn icon-only" onClick={() => setSearchOpen(false)} aria-label="Close search">
                ×
              </button>
            </div>
          ) : (
            <div className="vtoolbar phone-bar">
              <button className={`btn icon-only panel-btn ${leftOpen ? "on" : ""}`} onClick={() => setLeftOpen(o => !o)} aria-label="Toggle layers panel" aria-pressed={leftOpen}>
                <Icon name="layers" size={20} />
              </button>
              <button className={`btn icon-only panel-btn ${v.query ? "on" : ""}`} onClick={() => setSearchOpen(true)} aria-label="Search graph">
                <Icon name="search" size={20} />
              </button>
              <span className="grow" />
              {camera}
              <ScreenshotButton onDone={setToast} compact />
              <ControlsHelpButton mode={v.topDown ? "top" : "3d"} withLayout={false} compact />
              <button className={`btn icon-only panel-btn ${rightOpen ? "on" : ""}`} onClick={() => setRightOpen(o => !o)} aria-label="Toggle side panel" aria-pressed={rightOpen}>
                <Icon name="panel" size={20} />
              </button>
            </div>
          )
        ) : (
          <>
            <div className="vtoolbar top-left">
              <button className="btn btn-sm panel-btn" onClick={() => setLeftOpen(o => !o)} aria-label="Toggle layers panel">
                {leftOpen ? (narrow ? "Hide" : "Hide panel") : "Layers"}
              </button>
              <SearchBox onPicked={showInspect} />
            </div>
            <div className="vtoolbar top-right" ref={topRightRef}>
              {camera}
              <ScreenshotButton onDone={setToast} />
              <ControlsHelpButton mode={v.topDown ? "top" : "3d"} withLayout={wide} />
              {mapFullBtn}
              <button className="btn btn-sm panel-btn" onClick={() => setRightOpen(o => !o)} aria-label="Toggle side panel">
                {rightOpen ? "Hide" : "Panel"}
              </button>
            </div>
          </>
        )}
        {/* notices stack under the toolbar instead of on top of each other */}
        <div className="canvas-notices">
          {toast && (
            <div className={`toast${toast.err ? " toast-err" : ""}`} role={toast.err ? "alert" : "status"}>
              {toast.text}
            </div>
          )}
        </div>
        <TopViewHint active={v.topDown} />
        <LoadBar />
        <div className="vstatus mono">
          {totalNodes.toLocaleString()} nodes{tileText ? ` · tiles ${tileText}` : ""}
          {totalTris ? ` · ${(totalTris / 1e6).toFixed(2)}M tris` : ""}
          {v.flat ? " · flat" : ""}
        </div>
      </div>
      {rightOpen && wide && (
        <Splitter
          side="right"
          label="Resize side panel"
          value={widths.rightW}
          min={RIGHT.min}
          max={splitMax("right")}
          target={rightRef}
          onCommit={w => setLayout(l => ({...l, rightW: w}))}
          onReset={() => setLayout(l => ({...l, rightW: RIGHT.def}))}
        />
      )}
      {rightOpen && !isPhone && (
        <aside ref={rightRef} className="vpanel vright" aria-label="Inspector" style={wide ? {width: widths.rightW} : undefined}>
          {tabs}
          {rightBody}
        </aside>
      )}
      {wide &&
        // stable DOM order (moving a window's node mid-click would swallow the click); stacking via z-index
        PANE_TABS.map(t => {
          const w = layout.windows[t];
          const i = layout.order.indexOf(t);
          if (!w) return null;
          return (
            <FloatingWindow
              key={t}
              title={TAB_TITLE[t]}
              win={w}
              z={60 + i}
              area={area}
              areaRef={viewerRef}
              dockTarget={stripRef}
              onChange={patch => setLayout(l => updateWin(l, t, patch))}
              onRaise={() => setLayout(l => raise(l, t))}
              onDock={() => dockWin(t)}
              onClose={() => setLayout(l => dock(l, t))}
            >
              <TabSlot host={hosts[t]} />
            </FloatingWindow>
          );
        })}
      {isPhone && leftOpen && (
        <BottomSheet key="left" label="Sources and layers" host={viewerRef} onClose={() => setLeftOpen(false)}>
          {leftPanel}
        </BottomSheet>
      )}
      {isPhone && rightOpen && (
        <BottomSheet key="right" label="Inspector" head={tabs} host={viewerRef} onClose={() => setRightOpen(false)} initial="half">
          {rightBody}
        </BottomSheet>
      )}
    </div>
  );
}
