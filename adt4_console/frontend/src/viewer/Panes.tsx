// Viewer pane chrome: splitters between the docked panels and the canvas,
// floating windows for popped-out side-panel tabs, and the "tab host" that
// keeps a tab's React tree mounted while it moves between the dock and a
// window (so panel state survives docking and dragging).
//
// Tab host: each mounted tab renders through createPortal into its own
// persistent, detached <div>; a <TabSlot> wherever the tab is shown (dock body,
// floating window, phone sheet) re-parents that div with appendChild. React
// never sees the portal target change, so nothing remounts.
//
// Drags (splitters, window move/resize) write geometry straight to the DOM on
// every pointermove and commit to React state once on release, so the panels'
// contents do not re-render while the operator drags.
import {useCallback, useEffect, useLayoutEffect, useRef, useState, type ReactNode, type RefObject} from "react";
import {createPortal} from "react-dom";
import {
  EDGES,
  clampRect,
  defaultLayout,
  loadLayout,
  resizeRect,
  saveLayout,
  snapRect,
  clearLayout,
  TITLE_H,
  type Edge,
  type FloatWin,
  type PaneLayout,
  type PaneTab,
  type WinRect,
} from "./paneLayout";
import "./panes.css";

/** Layout state loaded once from localStorage and written back on every change. */
export function usePaneLayout() {
  const [layout, setLayout] = useState<PaneLayout>(() => loadLayout());
  const first = useRef(true);
  useEffect(() => {
    if (first.current) {
      first.current = false;
      return;
    }
    saveLayout(layout);
  }, [layout]);
  const reset = useCallback(() => {
    clearLayout();
    first.current = true; // don't immediately re-save the defaults
    setLayout(defaultLayout());
  }, []);
  return [layout, setLayout, reset] as const;
}

/** Size of an element, tracked with a ResizeObserver. */
export function useElementSize(ref: RefObject<HTMLElement | null>) {
  const [size, setSize] = useState({w: 0, h: 0});
  useLayoutEffect(() => {
    const el = ref.current;
    if (!el) return;
    const read = () => setSize(s => (s.w === el.clientWidth && s.h === el.clientHeight ? s : {w: el.clientWidth, h: el.clientHeight}));
    read();
    const ro = new ResizeObserver(read);
    ro.observe(el);
    return () => ro.disconnect();
  }, [ref]);
  return size;
}

/** One persistent detached container per tab (see the header comment). */
export function useTabHosts(tabs: readonly PaneTab[]) {
  const ref = useRef<Partial<Record<PaneTab, HTMLDivElement>>>({});
  for (const t of tabs) {
    if (!ref.current[t] && typeof document !== "undefined") {
      const el = document.createElement("div");
      el.className = "tab-host";
      el.dataset.tab = t;
      ref.current[t] = el;
    }
  }
  return ref.current as Record<PaneTab, HTMLDivElement>;
}

/** Renders `children` into the tab's host container (keeps it mounted). */
export function TabPortal({host, children}: {host: HTMLElement; children: ReactNode}) {
  return createPortal(children, host);
}

/** Shows a tab host here by re-parenting its container. */
export function TabSlot({host, className}: {host: HTMLElement; className?: string}) {
  const ref = useRef<HTMLDivElement>(null);
  useLayoutEffect(() => {
    const slot = ref.current;
    if (!slot) return;
    slot.appendChild(host);
    return () => {
      if (host.parentNode === slot) slot.removeChild(host);
    };
  }, [host]);
  return <div ref={ref} className={`tab-slot ${className ?? ""}`} />;
}

/** Vertical (or horizontal) drag handle between a docked panel and the canvas.
 *  `side` says which side of the handle the panel is on, so dragging toward
 *  the panel shrinks it. Double-click resets; arrow keys nudge. */
export function Splitter({
  side,
  value,
  min,
  max,
  target,
  orientation = "vertical",
  onCommit,
  onReset,
  label,
}: {
  side: "left" | "right" | "top" | "bottom";
  value: number;
  min: number;
  max: number;
  /** the panel element whose size the handle drives live during a drag */
  target: RefObject<HTMLElement | null>;
  orientation?: "vertical" | "horizontal";
  onCommit: (v: number) => void;
  onReset: () => void;
  label: string;
}) {
  const drag = useRef<{p: number; v: number; cur: number} | null>(null);
  const horiz = orientation === "horizontal";
  const sign = side === "left" || side === "top" ? 1 : -1;
  const prop = horiz ? "height" : "width";
  const clampV = (v: number) => Math.round(Math.min(Math.max(v, min), Math.max(min, max)));

  const onDown = (e: React.PointerEvent<HTMLDivElement>) => {
    if (e.button !== 0) return;
    e.preventDefault();
    e.currentTarget.setPointerCapture?.(e.pointerId);
    drag.current = {p: horiz ? e.clientY : e.clientX, v: value, cur: value};
    document.body.classList.add(horiz ? "pane-dragging-row" : "pane-dragging-col");
  };
  const onMove = (e: React.PointerEvent) => {
    const d = drag.current;
    if (!d) return;
    d.cur = clampV(d.v + sign * ((horiz ? e.clientY : e.clientX) - d.p));
    if (target.current) target.current.style[prop] = `${d.cur}px`;
  };
  const onUp = () => {
    const d = drag.current;
    drag.current = null;
    document.body.classList.remove("pane-dragging-row", "pane-dragging-col");
    if (d && d.cur !== d.v) onCommit(d.cur);
  };
  return (
    <div
      className={`pane-splitter ${horiz ? "row" : "col"} side-${side}`}
      role="separator"
      aria-orientation={horiz ? "horizontal" : "vertical"}
      aria-label={label}
      aria-valuenow={value}
      aria-valuemin={min}
      aria-valuemax={max}
      tabIndex={0}
      title="Drag to resize · double-click to reset"
      onPointerDown={onDown}
      onPointerMove={onMove}
      onPointerUp={onUp}
      onPointerCancel={onUp}
      onDoubleClick={onReset}
      onKeyDown={e => {
        const step = e.shiftKey ? 48 : 16;
        const grow = horiz ? (side === "top" ? "ArrowDown" : "ArrowUp") : side === "left" ? "ArrowRight" : "ArrowLeft";
        const shrink = horiz ? (side === "top" ? "ArrowUp" : "ArrowDown") : side === "left" ? "ArrowLeft" : "ArrowRight";
        if (e.key === grow) onCommit(clampV(value + step));
        else if (e.key === shrink) onCommit(clampV(value - step));
        else if (e.key === "Home" || e.key === "Enter") onReset();
        else return;
        e.preventDefault();
      }}
    >
      <span />
    </div>
  );
}

const within = (r: DOMRect | undefined, x: number, y: number) => !!r && x >= r.left && x <= r.right && y >= r.top && y <= r.bottom;

/** A popped-out side-panel tab: drag by the title bar, resize from any edge or
 *  corner, snaps to and stays inside the viewer. Dropping it on the side
 *  panel's tab strip (`dockTarget`) docks it again. */
export function FloatingWindow({
  title,
  win,
  z,
  area,
  areaRef,
  dockTarget,
  onChange,
  onRaise,
  onDock,
  onClose,
  children,
}: {
  title: string;
  win: FloatWin;
  z: number;
  /** viewer size (the windows' coordinate space) */
  area: {w: number; h: number};
  areaRef: RefObject<HTMLElement | null>;
  dockTarget: RefObject<HTMLElement | null>;
  onChange: (patch: Partial<FloatWin>) => void;
  onRaise: () => void;
  onDock: () => void;
  onClose: () => void;
  children: ReactNode;
}) {
  const ref = useRef<HTMLDivElement>(null);
  const op = useRef<{kind: "move" | Edge; px: number; py: number; start: WinRect; cur: WinRect; overDock: boolean} | null>(null);
  const shown = clampRect(win, area.w || win.w, area.h || win.h, win.min);

  const apply = (r: WinRect) => {
    const el = ref.current;
    if (!el) return;
    el.style.left = `${r.x}px`;
    el.style.top = `${r.y}px`;
    el.style.width = `${r.w}px`;
    if (!win.min) el.style.height = `${r.h}px`;
  };
  const begin = (kind: "move" | Edge) => (e: React.PointerEvent<HTMLElement>) => {
    if (e.button !== 0) return;
    if (kind === "move" && (e.target as HTMLElement).closest("button")) return;
    e.preventDefault();
    e.stopPropagation();
    onRaise();
    e.currentTarget.setPointerCapture?.(e.pointerId);
    op.current = {kind, px: e.clientX, py: e.clientY, start: shown, cur: shown, overDock: false};
    ref.current?.classList.add("dragging");
  };
  const move = (e: React.PointerEvent) => {
    const o = op.current;
    if (!o) return;
    const dx = e.clientX - o.px;
    const dy = e.clientY - o.py;
    const bw = areaRef.current?.clientWidth ?? area.w;
    const bh = areaRef.current?.clientHeight ?? area.h;
    if (o.kind === "move") {
      const vis = {...o.start, x: o.start.x + dx, y: o.start.y + dy, h: win.min ? TITLE_H : o.start.h};
      const r = clampRect(snapRect(vis, bw, bh), bw, bh, win.min);
      o.cur = {...r, h: o.start.h};
      const over = within(dockTarget.current?.getBoundingClientRect(), e.clientX, e.clientY);
      if (over !== o.overDock) {
        o.overDock = over;
        dockTarget.current?.classList.toggle("dock-hover", over);
        ref.current?.classList.toggle("dock-hover", over);
      }
    } else {
      o.cur = resizeRect(o.start, o.kind, dx, dy, bw, bh);
    }
    apply(o.cur);
  };
  const end = () => {
    const o = op.current;
    op.current = null;
    ref.current?.classList.remove("dragging", "dock-hover");
    dockTarget.current?.classList.remove("dock-hover");
    if (!o) return;
    if (o.overDock) return onDock();
    const {x, y, w, h} = o.cur;
    if (x !== win.x || y !== win.y || w !== win.w || h !== win.h) onChange({x, y, w, h});
  };

  return (
    <div
      ref={ref}
      className={`float-win ${win.min ? "minimized" : ""}`}
      role="dialog"
      aria-label={title}
      style={{left: shown.x, top: shown.y, width: shown.w, height: win.min ? TITLE_H : shown.h, zIndex: z}}
      onPointerDownCapture={onRaise}
    >
      <div className="float-title" onPointerDown={begin("move")} onPointerMove={move} onPointerUp={end} onPointerCancel={end} onDoubleClick={e => !(e.target as HTMLElement).closest("button") && onChange({min: !win.min})}>
        <span className="float-name">{title}</span>
        <button className="float-btn" onClick={() => onChange({min: !win.min})} aria-label={win.min ? `Restore ${title}` : `Minimize ${title}`} title={win.min ? "Restore" : "Minimize"}>
          {win.min ? "▢" : "–"}
        </button>
        <button className="float-btn" onClick={onDock} aria-label={`Dock ${title}`} title="Dock into the side panel">
          ⇥
        </button>
        <button className="float-btn" onClick={onClose} aria-label={`Close ${title}`} title="Close">
          ×
        </button>
      </div>
      <div className="float-body" hidden={win.min}>
        {children}
      </div>
      {!win.min &&
        EDGES.map(edge => <div key={edge} className={`float-rz rz-${edge}`} onPointerDown={begin(edge)} onPointerMove={move} onPointerUp={end} onPointerCancel={end} aria-hidden />)}
    </div>
  );
}

/** Pointer-drag a tab out of the side panel's tab strip to pop it out. Clicks
 *  still reach the tab buttons (no pointer capture; window listeners only). */
export function useTabTearOff(strip: RefObject<HTMLElement | null>, tabs: readonly PaneTab[], canTear: (t: PaneTab) => boolean, onTear: (t: PaneTab, clientX: number, clientY: number) => void) {
  return useCallback(
    (e: React.PointerEvent) => {
      if (e.button !== 0 || e.pointerType === "touch") return;
      const btn = (e.target as HTMLElement).closest('button[role="tab"]');
      const el = strip.current;
      if (!btn || !el?.contains(btn)) return;
      const idx = Array.from(btn.parentElement?.children ?? []).indexOf(btn);
      const tab = tabs[idx];
      if (!tab || !canTear(tab)) return;
      const sx = e.clientX;
      const sy = e.clientY;
      const mv = (ev: PointerEvent) => {
        const r = el.getBoundingClientRect();
        const out = ev.clientY > r.bottom + 28 || ev.clientY < r.top - 28 || ev.clientX < r.left - 28 || ev.clientX > r.right + 28;
        if (out && Math.hypot(ev.clientX - sx, ev.clientY - sy) > 32) {
          stop();
          onTear(tab, ev.clientX, ev.clientY);
        }
      };
      const stop = () => {
        window.removeEventListener("pointermove", mv);
        window.removeEventListener("pointerup", stop);
        window.removeEventListener("pointercancel", stop);
      };
      window.addEventListener("pointermove", mv);
      window.addEventListener("pointerup", stop);
      window.addEventListener("pointercancel", stop);
    },
    [strip, tabs, canTear, onTear],
  );
}
