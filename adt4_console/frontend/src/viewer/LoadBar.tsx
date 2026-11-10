// The viewer's startup-load bar: what is loading, bytes so far / total and an
// ETA from the measured throughput ("Graph ✓ · Mesh 12.4/38 MB · ~25 s left").
// The map stays usable underneath; the bar lingers briefly once all is in.
// It sits just under whatever toolbars are at the top of the map: they wrap
// to more rows, fold or move with the canvas width, so it is measured.
import {useEffect, useLayoutEffect, useRef, useState} from "react";
import {fmtEta} from "../lib/loadProgress";
import {LINGER_S, useLoading} from "../stores/loading";

/** what the bar must clear at the top of the map */
const TOP_SELECTOR = ".vtoolbar";

/** The bar's top (px in `canvas`): `gap` under the lowest top-anchored
 *  toolbar (one starting in the canvas's upper third), `fallback` if none. */
export function barTop(canvas: Element, self: Element | null, gap = 8, fallback = 54): number {
  const c = canvas.getBoundingClientRect();
  let top = -1;
  canvas.querySelectorAll(TOP_SELECTOR).forEach(el => {
    if (el === self) return;
    const r = el.getBoundingClientRect();
    if (!r.width || !r.height || r.top - c.top > c.height / 3) return;
    top = Math.max(top, r.bottom - c.top + gap);
  });
  return top < 0 ? fallback : Math.round(top);
}

export function LoadBar() {
  const ref = useRef<HTMLDivElement>(null);
  const summary = useLoading(s => s.summary);
  const finishedAt = useLoading(s => s.finishedAt);
  const [, rerender] = useState(0);
  const lingering = !summary.active && finishedAt !== null && performance.now() / 1000 - finishedAt < LINGER_S;
  useEffect(() => {
    if (!lingering) return;
    const t = setTimeout(() => rerender(n => n + 1), LINGER_S * 1000 + 50);
    return () => clearTimeout(t);
  }, [lingering, finishedAt]);
  const state = summary.active ? "active" : summary.parts.length ? "done" : "idle";
  const visible = summary.parts.length > 0 && (summary.active || lingering);
  const pct = summary.fraction === null ? null : Math.round(summary.fraction * 100);
  const eta = summary.active ? fmtEta(summary.eta) : "";
  useLayoutEffect(() => {
    const el = ref.current;
    const canvas = el?.parentElement;
    if (!visible || !el || !canvas) return;
    const place = () => {
      el.style.top = `${barTop(canvas, el)}px`;
    };
    place();
    const ro = new ResizeObserver(place);
    ro.observe(canvas);
    canvas.querySelectorAll(TOP_SELECTOR).forEach(t => ro.observe(t));
    // toolbars are swapped (search on a phone)
    const mo = new MutationObserver(() => {
      canvas.querySelectorAll(TOP_SELECTOR).forEach(t => ro.observe(t));
      place();
    });
    mo.observe(canvas, {childList: true});
    return () => {
      ro.disconnect();
      mo.disconnect();
    };
  }, [visible]);
  return (
    <div
      ref={ref}
      className={`load-bar${visible ? "" : " load-bar-hidden"}${summary.error ? " load-bar-err" : ""}`}
      data-load-progress={state}
      data-testid="load-bar"
      role="progressbar"
      aria-label="Loading the map"
      aria-valuemin={0}
      aria-valuemax={100}
      aria-valuenow={pct ?? undefined}
      aria-hidden={!visible}
    >
      <div className={`load-bar-track${pct === null && summary.active ? " indeterminate" : ""}`}>
        <div className="load-bar-fill" style={{width: `${pct ?? (summary.active ? 30 : 100)}%`}} />
      </div>
      <div className="load-bar-text mono">
        {summary.parts.map((p, i) => (
          <span key={p.label} className={`load-part${p.done ? " done" : ""}`}>
            {i > 0 && <span className="load-sep"> · </span>}
            {p.label} <b>{p.text}</b>
          </span>
        ))}
        {eta && (
          <span className="load-eta">
            <span className="load-sep"> · </span>
            {eta}
          </span>
        )}
        {summary.active && !eta && summary.bytesTotal > 0 && pct !== null && (
          <span className="load-eta muted">
            <span className="load-sep"> · </span>
            {pct}%
          </span>
        )}
      </div>
    </div>
  );
}
