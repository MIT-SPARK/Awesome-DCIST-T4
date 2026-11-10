// Graph statistics (the RViz DsgStatsPanel numbers) for each enabled source.
import {useEffect, useState} from "react";
import {useViewer, srcPath} from "../stores/viewer";
import {errorText, getJSON} from "../lib/api";
import {Spinner} from "../components/ui";

export interface GraphStats {
  layers: {key: string; name: string; kind?: string; nodes: number}[];
  nodes?: number;
  edges: Record<string, number>;
  edges_total?: number;
  mesh: {vertices: number; faces: number; tiles: number};
  labels: {label: number; name: string; count: number; layer?: string}[];
  version: number | null;
}

/** Refetched whenever the source's version changes. */
function SourceStats({id, label, version}: {id: string; label: string; version: number | null}) {
  const [stats, setStats] = useState<GraphStats | null>(null);
  const [err, setErr] = useState<string | null>(null);
  useEffect(() => {
    const ctl = new AbortController();
    getJSON<GraphStats>(`${srcPath(id)}/stats`, ctl.signal)
      .then(s => {
        setStats(s);
        setErr(null);
      })
      .catch(e => !ctl.signal.aborted && setErr(errorText(e)));
    return () => ctl.abort();
  }, [id, version]);
  const edges = Object.entries(stats?.edges ?? {}).sort((a, b) => b[1] - a[1]);
  return (
    <div className="stats-src">
      <h4 className="sub-title">
        {label} <span className="muted mono small">{id}</span>
      </h4>
      {!stats && !err && <Spinner label="Loading stats" />}
      {err && <div className="inline-err small">{err}</div>}
      {stats && (
        <>
          <table className="table compact kv-table">
            <tbody>
              {stats.layers.map(l => (
                <tr key={l.key}>
                  <th>{l.name}</th>
                  <td className="mono">{l.nodes.toLocaleString()}</td>
                </tr>
              ))}
              {edges.map(([k, n]) => (
                <tr key={k}>
                  <th className="muted">{k === "agents-agents" ? "trajectory edges" : `${k.replace("-", " / ")} edges`}</th>
                  <td className="mono">{n.toLocaleString()}</td>
                </tr>
              ))}
              <tr>
                <th>mesh</th>
                <td className="mono">
                  {stats.mesh.tiles ? `${stats.mesh.faces.toLocaleString()} faces · ${stats.mesh.vertices.toLocaleString()} verts · ${stats.mesh.tiles} tiles` : "none"}
                </td>
              </tr>
              {stats.version !== null && stats.version !== undefined && (
                <tr>
                  <th>version</th>
                  <td className="mono">{stats.version}</td>
                </tr>
              )}
            </tbody>
          </table>
          {stats.labels.length > 0 && (
            <table className="table compact kv-table stats-labels">
              <thead>
                <tr>
                  <th>top object labels</th>
                  <td />
                </tr>
              </thead>
              <tbody>
                {stats.labels.map(l => (
                  <tr key={`${l.layer}:${l.label}`}>
                    <th>
                      {l.name || "unlabeled"} <span className="muted mono">({l.label})</span>
                    </th>
                    <td className="mono">{l.count}</td>
                  </tr>
                ))}
              </tbody>
            </table>
          )}
        </>
      )}
    </div>
  );
}

export function StatsPanel() {
  const {sources, order} = useViewer();
  const enabled = order.map(id => sources[id]).filter(s => s?.enabled && s.status === "ready");
  if (!enabled.length) return null;
  return (
    <div className="stats-panel">
      <h4 className="sub-title">Graph statistics</h4>
      {enabled.map(s => (
        <SourceStats key={s.id} id={s.id} label={s.label} version={s.version} />
      ))}
    </div>
  );
}
