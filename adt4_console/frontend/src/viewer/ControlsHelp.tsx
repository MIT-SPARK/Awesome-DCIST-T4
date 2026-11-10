// "?" button + compact controls card for the current view mode, and the
// one-time Top-view hint. Text lives in controlsHelp.ts.
import {useEffect, useRef, useState} from "react";
import {TOP_HINT, TOP_HINT_TOUCH, dismissHint, helpSections, hintDismissed, type HelpMode} from "./controlsHelp";
import {useMedia} from "../lib/media";
import {safeStorage} from "./paneLayout";

export function ControlsHelpButton({mode, withLayout, compact}: {mode: HelpMode; withLayout: boolean; compact?: boolean}) {
  const [open, setOpen] = useState(false);
  const ref = useRef<HTMLDivElement>(null);
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
  return (
    <div className="help-wrap" ref={ref}>
      <button
        className={`btn ${compact ? "icon-only" : "btn-sm"} panel-btn help-btn ${open ? "on" : ""}`}
        onClick={() => setOpen(o => !o)}
        aria-label="Viewer controls"
        aria-expanded={open}
        title="Controls"
      >
        ?
      </button>
      {open && (
        <div className="help-card" role="dialog" aria-label="Viewer controls">
          {helpSections(mode, withLayout).map(s => (
            <section key={s.title}>
              <h5>{s.title}</h5>
              <dl>
                {s.entries.map(e => (
                  <div key={e.keys} className="help-row">
                    <dt>
                      <kbd>{e.keys}</kbd>
                    </dt>
                    <dd>{e.does}</dd>
                  </div>
                ))}
              </dl>
            </section>
          ))}
        </div>
      )}
    </div>
  );
}

/** Shown the first time Top view is active, until dismissed (remembered per browser). */
export function TopViewHint({active}: {active: boolean}) {
  const [dismissed, setDismissed] = useState(() => hintDismissed(safeStorage()));
  const coarse = useMedia("(pointer: coarse)");
  if (!active || dismissed) return null;
  return (
    <div className="top-hint" role="status">
      <span>{coarse ? TOP_HINT_TOUCH : TOP_HINT}</span>
      <button
        className="icon-btn"
        aria-label="Dismiss hint"
        onClick={() => {
          dismissHint(safeStorage());
          setDismissed(true);
        }}
      >
        ×
      </button>
    </div>
  );
}
