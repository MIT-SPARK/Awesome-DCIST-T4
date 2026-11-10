import {useEffect, useRef, useState, type ReactNode} from "react";
import {ApiError, errorText} from "../lib/api";
import {fuzzyFilter} from "../lib/fuzzy";

export type Tone = "ok" | "warn" | "err" | "off" | "info";

export function Dot({tone, title}: {tone: Tone; title?: string}) {
  return <span className={`dot dot-${tone}`} title={title} aria-label={title} />;
}

export function Badge({tone = "off", children, title}: {tone?: Tone; children: ReactNode; title?: string}) {
  return (
    <span className={`badge badge-${tone}`} title={title}>
      {children}
    </span>
  );
}

export function Spinner({label}: {label?: string}) {
  return (
    <span className="spinner-wrap" role="status">
      <span className="spinner" />
      {label && <span className="muted">{label}</span>}
    </span>
  );
}

export function Loading({label = "Loading"}: {label?: string}) {
  return (
    <div className="state-box">
      <Spinner label={label} />
    </div>
  );
}

export function Empty({title, children}: {title: string; children?: ReactNode}) {
  return (
    <div className="state-box empty">
      <div className="state-title">{title}</div>
      {children && <div className="muted">{children}</div>}
    </div>
  );
}

/** Inline error, with the "service unavailable" / "backend down" cases spelled out. */
export function ErrorBox({error, onRetry, compact}: {error: unknown; onRetry?: () => void; compact?: boolean}) {
  const kind = error instanceof ApiError ? error.kind : "other";
  const title =
    kind === "unavailable" ? "Service unavailable" : kind === "down" ? "Backend unreachable" : kind === "auth" ? "Not authorised" : "Request failed";
  const detail = kind === "down" ? "The console server is not responding. Retrying automatically." : errorText(error);
  return (
    <div className={`error-box ${kind === "unavailable" ? "tone-warn" : "tone-err"} ${compact ? "compact" : ""}`} role="alert">
      <div>
        <strong>{title}</strong>
        {!compact && <div className="error-detail">{detail}</div>}
        {compact && kind !== "down" && <span className="error-detail"> {detail}</span>}
      </div>
      {onRetry && (
        <button className="btn btn-sm" onClick={onRetry}>
          Retry
        </button>
      )}
    </div>
  );
}

export function Card({title, actions, children, className = ""}: {title?: ReactNode; actions?: ReactNode; children: ReactNode; className?: string}) {
  return (
    <section className={`card ${className}`}>
      {(title || actions) && (
        <header className="card-head">
          <h3>{title}</h3>
          <div className="card-actions">{actions}</div>
        </header>
      )}
      <div className="card-body">{children}</div>
    </section>
  );
}

export function PageHeader({title, sub, actions}: {title: string; sub?: ReactNode; actions?: ReactNode}) {
  return (
    <header className="page-head">
      <div>
        <h1>{title}</h1>
        {sub && <div className="page-sub">{sub}</div>}
      </div>
      <div className="page-actions">{actions}</div>
    </header>
  );
}

export function Modal({title, children, onClose, footer, tone, wide}: {title: ReactNode; children: ReactNode; onClose?: () => void; footer?: ReactNode; tone?: Tone; wide?: boolean}) {
  useEffect(() => {
    if (!onClose) return;
    const h = (e: KeyboardEvent) => e.key === "Escape" && onClose();
    window.addEventListener("keydown", h);
    return () => window.removeEventListener("keydown", h);
  }, [onClose]);
  return (
    <div className="modal-backdrop" onMouseDown={e => e.target === e.currentTarget && onClose?.()}>
      <div className={`modal ${tone ? `modal-${tone}` : ""} ${wide ? "modal-wide" : ""}`} role="dialog" aria-modal="true">
        <header className="modal-head">
          <h2>{title}</h2>
          {onClose && (
            <button className="icon-btn" onClick={onClose} aria-label="Close">
              ×
            </button>
          )}
        </header>
        <div className="modal-body">{children}</div>
        {footer && <footer className="modal-foot">{footer}</footer>}
      </div>
    </div>
  );
}

export function ProgressBar({pct, label, tone = "info"}: {pct: number; label?: string; tone?: Tone}) {
  const p = Math.max(0, Math.min(100, pct));
  return (
    <div className="progress" title={`${label ?? ""} ${p.toFixed(0)}%`}>
      {label && <span className="progress-label mono">{label}</span>}
      <div className="progress-track">
        <div className={`progress-fill fill-${tone}`} style={{width: `${p}%`}} />
      </div>
      <span className="progress-pct mono">{p.toFixed(0)}%</span>
    </div>
  );
}

export function Sparkline({values, width = 120, height = 28, color = "var(--accent)", max}: {values: number[]; width?: number; height?: number; color?: string; max?: number}) {
  const m = Math.max(max ?? 0, ...values, 1);
  const n = Math.max(values.length, 2);
  const pts = values.map((v, i) => `${((i / (n - 1)) * width).toFixed(1)},${(height - 2 - (v / m) * (height - 4)).toFixed(1)}`);
  return (
    <svg className="spark" width={width} height={height} viewBox={`0 0 ${width} ${height}`} aria-hidden>
      {values.length > 1 && (
        <>
          <polygon points={`0,${height} ${pts.join(" ")} ${(((values.length - 1) / (n - 1)) * width).toFixed(1)},${height}`} fill={color} opacity={0.15} />
          <polyline points={pts.join(" ")} fill="none" stroke={color} strokeWidth={1.5} />
        </>
      )}
    </svg>
  );
}

export function Toggle({checked, onChange, label, title}: {checked: boolean; onChange: (v: boolean) => void; label: ReactNode; title?: string}) {
  return (
    <label className="toggle" title={title}>
      <input type="checkbox" checked={checked} onChange={e => onChange(e.target.checked)} />
      <span className="toggle-track" />
      <span className="toggle-label">{label}</span>
    </label>
  );
}

export function Segmented<T extends string>({value, options, onChange, ariaLabel}: {value: T; options: {value: T; label: ReactNode; disabled?: boolean; title?: string}[]; onChange: (v: T) => void; ariaLabel?: string}) {
  return (
    <div className="segmented" role="tablist" aria-label={ariaLabel}>
      {options.map(o => (
        <button key={o.value} role="tab" aria-selected={o.value === value} className={o.value === value ? "on" : ""} disabled={o.disabled} title={o.title} onClick={() => onChange(o.value)}>
          {o.label}
        </button>
      ))}
    </div>
  );
}

/** Text input + fuzzy-filtered option list (session pickers etc). */
export function FuzzyPicker({value, options, onChange, placeholder, ariaLabel}: {value: string; options: string[]; onChange: (v: string) => void; placeholder?: string; ariaLabel?: string}) {
  const [q, setQ] = useState("");
  const [open, setOpen] = useState(false);
  const ref = useRef<HTMLDivElement>(null);
  useEffect(() => {
    const h = (e: MouseEvent) => {
      if (ref.current && !ref.current.contains(e.target as Node)) setOpen(false);
    };
    document.addEventListener("mousedown", h);
    return () => document.removeEventListener("mousedown", h);
  }, []);
  const list = fuzzyFilter(options, q, s => s).slice(0, 80);
  return (
    <div className="fuzzy" ref={ref}>
      <input
        aria-label={ariaLabel}
        className="input mono"
        value={open ? q : value}
        placeholder={value || placeholder}
        onFocus={() => {
          setQ("");
          setOpen(true);
        }}
        onChange={e => setQ(e.target.value)}
        onKeyDown={e => {
          if (e.key === "Enter" && list[0]) {
            onChange(list[0]);
            setOpen(false);
            (e.target as HTMLInputElement).blur();
          } else if (e.key === "Escape") setOpen(false);
        }}
      />
      {open && (
        <ul className="fuzzy-list">
          {list.length === 0 && <li className="muted">No match</li>}
          {list.map(o => (
            <li
              key={o}
              className={o === value ? "on mono" : "mono"}
              onMouseDown={e => {
                e.preventDefault();
                onChange(o);
                setOpen(false);
              }}
            >
              {o}
            </li>
          ))}
        </ul>
      )}
    </div>
  );
}

export function Field({label, children, hint}: {label: string; children: ReactNode; hint?: ReactNode}) {
  return (
    <label className="field">
      <span className="field-label">{label}</span>
      {children}
      {hint && <span className="field-hint">{hint}</span>}
    </label>
  );
}

export function KV({k, children, mono}: {k: string; children: ReactNode; mono?: boolean}) {
  return (
    <div className="kv">
      <span className="kv-k">{k}</span>
      <span className={`kv-v ${mono ? "mono" : ""}`}>{children}</span>
    </div>
  );
}

/** Phone-style action sheet: slides up from the bottom over a backdrop. */
export function Sheet({title, children, onClose}: {title: ReactNode; children: ReactNode; onClose: () => void}) {
  useEffect(() => {
    const h = (e: KeyboardEvent) => e.key === "Escape" && onClose();
    window.addEventListener("keydown", h);
    return () => window.removeEventListener("keydown", h);
  }, [onClose]);
  return (
    <div className="sheet-backdrop" onClick={e => e.target === e.currentTarget && onClose()}>
      <div className="sheet" role="dialog" aria-modal="true" aria-label={typeof title === "string" ? title : undefined}>
        <div className="sheet-grip" aria-hidden />
        <header className="sheet-head">
          <h2>{title}</h2>
          <button className="icon-btn" onClick={onClose} aria-label="Close">
            ×
          </button>
        </header>
        <div className="sheet-body">{children}</div>
      </div>
    </div>
  );
}
