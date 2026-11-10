import {useEffect, useState, type ReactNode} from "react";
import {NavLink, Outlet} from "react-router";
import {useWsStatus} from "../ws";
import {useHealthStore} from "../stores/health";
import {isActive, useJobs} from "../stores/jobs";
import {onBackendState} from "../lib/api";
import {JobConfirmHost, JobsDrawer} from "./Jobs";
import {Dot, Sheet} from "./ui";
import {useTheme} from "../stores/theme";

const P = {
  maps: "M3 6l6-3 6 3 6-3v15l-6 3-6-3-6 3zM9 3v15M15 6v15",
  viewer: "M12 2l9 5v10l-9 5-9-5V7zM12 12l9-5M12 12v10M12 12L3 7",
  jobs: "M4 5h16M4 10h16M4 15h10M4 20h7",
  layers: "M12 3l9 5-9 5-9-5zM3 13l9 5 9-5",
  search: "M11 18a7 7 0 100-14 7 7 0 000 14zM20.5 20.5l-4.6-4.6",
  panel: "M4 4h16v16H4zM4 13h16",
  sun: "M12 16a4 4 0 100-8 4 4 0 000 8zM12 2v2M12 20v2M4.9 4.9l1.4 1.4M17.7 17.7l1.4 1.4M2 12h2M20 12h2M4.9 19.1l1.4-1.4M17.7 6.3l1.4-1.4",
  moon: "M20.5 14.5A8.5 8.5 0 019.5 3.5a8.5 8.5 0 1011 11z",
  collapse: "M15 6l-6 6 6 6",
  expand: "M9 6l6 6-6 6",
};

export function Icon({name, size = 18}: {name: keyof typeof P; size?: number}) {
  return (
    <svg width={size} height={size} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth={1.7} strokeLinecap="round" strokeLinejoin="round" aria-hidden>
      <path d={P[name]} />
    </svg>
  );
}

const NAV: {to: string; label: string; short?: string; icon: keyof typeof P}[] = [
  {to: "/maps", label: "Maps", icon: "maps"},
  {to: "/viewer", label: "Map viewer", short: "Viewer", icon: "viewer"},
];

function useBackendUp(): boolean {
  const [up, setUp] = useState(true);
  useEffect(() => onBackendState(ok => setUp(ok)), []);
  return up;
}

type PillTone = "ok" | "warn" | "err" | "off";
interface Health {
  key: string;
  tone: PillTone;
  label: ReactNode;
  detail: string;
}

function StatusPill({tone, label, title}: {tone: PillTone; label: ReactNode; title?: string}) {
  return (
    <span className="pill" title={title}>
      <Dot tone={tone} />
      {label}
    </span>
  );
}

function useHealth(): Health[] {
  const wsUp = useWsStatus(s => s.connected);
  const attempts = useWsStatus(s => s.attempts);
  const health = useHealthStore(s => s.health);
  const backendUp = useBackendUp();
  const neo = health ? Object.entries(health.neo4j ?? {}) : [];
  const stale = !backendUp; // last health is unknown while the server is unreachable
  return [
    {key: "server", tone: backendUp ? "ok" : "err", label: "server", detail: backendUp ? "Console server reachable" : "Console server unreachable"},
    {key: "live", tone: wsUp ? "ok" : attempts > 2 ? "err" : "warn", label: "live", detail: wsUp ? "WebSocket connected" : `WebSocket reconnecting (attempt ${attempts})`},
    ...neo.map(([n, ok]): Health => ({
      key: `neo4j:${n}`,
      tone: stale ? "off" : ok ? "ok" : "err",
      label: <span className="mono">neo4j:{n}</span>,
      detail: stale ? "Unknown" : ok ? "Graph database reachable" : "Graph database unreachable",
    })),
  ];
}

const RANK: Record<PillTone, number> = {ok: 0, off: 1, warn: 2, err: 3};

/** Phone top bar: one dot for everything; tap for the per-service list. */
function HealthButton({items}: {items: Health[]}) {
  const [open, setOpen] = useState(false);
  const worst = items.reduce<PillTone>((w, h) => (RANK[h.tone] > RANK[w] ? h.tone : w), "ok");
  const bad = items.filter(h => h.tone === "err" || h.tone === "warn").length;
  return (
    <>
      <button className="btn health-btn" onClick={() => setOpen(true)} aria-label="System status">
        <Dot tone={worst} />
        {bad ? `${bad} issue${bad > 1 ? "s" : ""}` : worst === "off" ? "status" : "all ok"}
      </button>
      {open && (
        <Sheet title="System status" onClose={() => setOpen(false)}>
          <ul className="health-list">
            {items.map(h => (
              <li key={h.key}>
                <Dot tone={h.tone} />
                <span className="strong">{h.label}</span>
                <span className="muted small">{h.detail}</span>
              </li>
            ))}
          </ul>
        </Sheet>
      )}
    </>
  );
}

/** Dark / light switch (remembered per browser). Shows what a click switches to. */
export function ThemeToggle() {
  const theme = useTheme(s => s.theme);
  const toggle = useTheme(s => s.toggle);
  const next = theme === "dark" ? "light" : "dark";
  return (
    <button className="btn theme-btn" onClick={toggle} aria-label={`Switch to ${next} theme`} title={`Switch to ${next} theme`} data-testid="theme-toggle">
      <Icon name={theme === "dark" ? "sun" : "moon"} size={16} />
      <span className="theme-btn-label">{next === "light" ? "Light" : "Dark"}</span>
    </button>
  );
}

function TopBar() {
  const items = useHealth();
  const jobs = useJobs(s => s.jobs);
  const open = useJobs(s => s.open);
  const drawerOpen = useJobs(s => s.drawerOpen);
  const close = useJobs(s => s.close);
  const active = Object.values(jobs).filter(isActive);
  const waiting = active.filter(j => j.state === "waiting").length;
  return (
    <div className="topbar">
      <div className="topbar-brand" aria-hidden>
        <Logo />
        ADT4
      </div>
      <div className="topbar-status">
        {items.map(h => (
          <StatusPill key={h.key} tone={h.tone} label={h.label} title={h.detail} />
        ))}
      </div>
      <HealthButton items={items} />
      <ThemeToggle />
      <button className={`btn jobs-btn ${waiting ? "attention" : ""}`} onClick={() => (drawerOpen ? close() : open())} aria-label="Jobs">
        <Icon name="jobs" size={16} />
        Jobs
        {active.length > 0 && <span className="count">{active.length}</span>}
        {waiting > 0 && <span className="count count-warn">{waiting} waiting</span>}
      </button>
    </div>
  );
}

const Logo = ({size = 22}: {size?: number}) => (
  <svg width={size} height={size} viewBox="0 0 16 16" aria-hidden>
    <path d="M3 12L8 3l5 9z" fill="var(--ok)" />
  </svg>
);

/** Phone navigation: one tab per page. */
function TabBar() {
  return (
    <nav className="tabbar" aria-label="Main">
      {NAV.map(n => (
        <NavLink key={n.to} to={n.to} className={({isActive}) => `tab ${isActive ? "on" : ""}`}>
          <Icon name={n.icon} size={22} />
          <span>{n.short ?? n.label}</span>
        </NavLink>
      ))}
    </nav>
  );
}

export const NAV_KEY = "adt4.nav.collapsed.v1";

/** The side nav's icon-only state, remembered per browser; with nothing stored it
 *  starts collapsed on screens up to 1366 px wide (the map needs the room). */
export function loadNavCollapsed(width = typeof window !== "undefined" ? window.innerWidth : 1600): boolean {
  try {
    const v = window.localStorage.getItem(NAV_KEY);
    if (v === "1" || v === "0") return v === "1";
  } catch {
    /* storage blocked: fall through to the width default */
  }
  return width <= 1366;
}

export function saveNavCollapsed(on: boolean) {
  try {
    window.localStorage.setItem(NAV_KEY, on ? "1" : "0");
  } catch {
    /* private window / quota: the toggle still works for this page */
  }
}

export function Shell() {
  const backendUp = useBackendUp();
  const authError = useWsStatus(s => s.authError);
  const [collapsed, setCollapsed] = useState(() => loadNavCollapsed());
  const toggle = () =>
    setCollapsed(c => {
      saveNavCollapsed(!c);
      return !c;
    });
  return (
    <div className={`shell ${collapsed ? "nav-collapsed" : ""}`}>
      <nav className="sidenav" aria-label="Main">
        <div className="brand">
          <Logo />
          <span className="brand-text">
            ADT4 <span className="muted">console</span>
          </span>
        </div>
        {NAV.map(n => (
          <NavLink key={n.to} to={n.to} className={({isActive}) => `navlink ${isActive ? "on" : ""}`} title={n.label}>
            <Icon name={n.icon} />
            <span className="navlabel">{n.label}</span>
          </NavLink>
        ))}
        <span className="grow" />
        <button className="navlink nav-toggle" onClick={toggle} aria-pressed={collapsed} aria-label={collapsed ? "Expand navigation" : "Collapse navigation"} title={collapsed ? "Show page names" : "Icons only (more room for the map)"} data-testid="nav-toggle">
          <Icon name={collapsed ? "expand" : "collapse"} />
          <span className="navlabel">Collapse</span>
        </button>
      </nav>
      <div className="main">
        <TopBar />
        {!backendUp && (
          <div className="banner banner-err" role="alert">
            Console server unreachable. Showing the last known data; reconnecting automatically.
          </div>
        )}
        {backendUp && authError && (
          <div className="banner banner-err" role="alert">
            Not authorised: open the console with <span className="mono">?token=...</span> (the server has a token configured).
          </div>
        )}
        <div className="content">
          <Outlet />
        </div>
      </div>
      <TabBar />
      <JobsDrawer />
      <JobConfirmHost />
    </div>
  );
}
