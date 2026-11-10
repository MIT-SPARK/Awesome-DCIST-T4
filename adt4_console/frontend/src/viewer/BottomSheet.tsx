// Phone layout for the viewer's side panels: a sheet docked to the bottom of
// the viewer with three heights (peek / half / full). Drag the grip to resize
// (it snaps to the nearest height, or closes when dragged below peek); tap the
// grip to cycle the heights. The sheet height is published to the viewer as
// the CSS variable --sheet-h, so floating bits (status line, pick banner) sit
// above it.
import {useEffect, useLayoutEffect, useRef, useState, type ReactNode, type RefObject} from "react";

export type Snap = "peek" | "half" | "full";
const FRACTION: Record<Snap, number> = {peek: 0.36, half: 0.6, full: 1};
const ORDER: Snap[] = ["peek", "half", "full"];
const TAP_PX = 6;

export function BottomSheet({
  label,
  head,
  host,
  onClose,
  children,
  initial = "half",
}: {
  label: string;
  head?: ReactNode;
  host: RefObject<HTMLElement | null>;
  onClose: () => void;
  children: ReactNode;
  initial?: Snap;
}) {
  const [snap, setSnap] = useState<Snap>(initial);
  const ref = useRef<HTMLElement>(null);
  const drag = useRef<{y: number; h: number; moved: boolean} | null>(null);

  const publish = (px: number) => host.current?.style.setProperty("--sheet-h", `${Math.round(px)}px`);
  const hostH = () => host.current?.clientHeight ?? window.innerHeight;

  // keep --sheet-h in sync with the rendered height (snap changes, rotation)
  useLayoutEffect(() => {
    const el = ref.current;
    if (!el) return;
    publish(el.offsetHeight);
    const ro = new ResizeObserver(() => publish(el.offsetHeight));
    ro.observe(el);
    return () => ro.disconnect();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);
  useEffect(() => {
    const h = host.current;
    return () => h?.style.setProperty("--sheet-h", "0px");
  }, [host]);

  const onDown = (e: React.PointerEvent) => {
    const el = ref.current;
    if (!el) return;
    (e.target as Element).setPointerCapture?.(e.pointerId);
    drag.current = {y: e.clientY, h: el.offsetHeight, moved: false};
  };
  const onMove = (e: React.PointerEvent) => {
    const d = drag.current;
    const el = ref.current;
    if (!d || !el) return;
    const dy = d.y - e.clientY;
    if (Math.abs(dy) > TAP_PX) d.moved = true;
    if (!d.moved) return;
    el.style.height = `${Math.max(48, Math.min(hostH(), d.h + dy))}px`;
    el.classList.add("dragging");
  };
  const onUp = () => {
    const d = drag.current;
    const el = ref.current;
    drag.current = null;
    if (!d || !el) return;
    el.classList.remove("dragging");
    if (!d.moved) {
      setSnap(s => ORDER[(ORDER.indexOf(s) + 1) % ORDER.length]);
      return;
    }
    const frac = el.offsetHeight / hostH();
    el.style.height = "";
    if (frac < FRACTION.peek * 0.6) return onClose();
    const best = ORDER.reduce((a, b) => (Math.abs(FRACTION[b] - frac) < Math.abs(FRACTION[a] - frac) ? b : a));
    setSnap(best);
  };

  return (
    <aside ref={ref} className={`vsheet snap-${snap}`} aria-label={label}>
      <div
        className="vsheet-grip"
        role="button"
        tabIndex={0}
        aria-label={`Resize ${label} (now ${snap})`}
        onPointerDown={onDown}
        onPointerMove={onMove}
        onPointerUp={onUp}
        onPointerCancel={onUp}
        onKeyDown={e => {
          if (e.key === "Enter" || e.key === " ") setSnap(s => ORDER[(ORDER.indexOf(s) + 1) % ORDER.length]);
          if (e.key === "Escape") onClose();
        }}
      >
        <span />
      </div>
      <div className="vsheet-head">
        <div className="vsheet-title">{head ?? <h3>{label}</h3>}</div>
        <button className="icon-btn" onClick={onClose} aria-label={`Close ${label}`}>
          ×
        </button>
      </div>
      <div className="vsheet-body">{children}</div>
    </aside>
  );
}
