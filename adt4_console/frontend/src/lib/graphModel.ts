// In-memory mirror of a scene graph (records + edges).
import type {DecodedDelta, DecodedGraph, Edge, Labelspaces, NodeRecord} from "../codec";

export const edgeKey = (a: string, b: string) => `${a}\u0000${b}`;

export interface LayerInfo {
  key: string; // "layer/part"
  name: string;
  kind: "objects" | "agents" | "subkeyframes" | "places" | "rooms" | "buildings" | "segments" | "other";
  order: number; // stacking order for Z offsets (0 = at true position)
}

export function layerInfo(layer: number, part: number): LayerInfo {
  const key = `${layer}/${part}`;
  // Hydra partitions are keyed by the node symbol's character code:
  // 2/97 'a' agent keyframes, 2/115 's' sub-keyframes, 3/1 mesh places, 3/2 traversability.
  if (layer === 2 && part === 97) return {key, name: "agents", kind: "agents", order: 0};
  if (layer === 2 && part === 115) return {key, name: "subkeyframes", kind: "subkeyframes", order: 0};
  // other lower-case partitions: more robots' keyframes (multi-robot maps: b, c, ...)
  if (layer === 2 && part > 97 && part <= 122) return {key, name: `agents ${String.fromCharCode(part)}`, kind: "agents", order: 0};
  if (layer === 2) return {key, name: part ? `objects ${part}` : "objects", kind: "objects", order: 0};
  if (layer === 3 && part === 1) return {key, name: "mesh places", kind: "places", order: 1};
  if (layer === 3 && part === 2) return {key, name: "traversability", kind: "places", order: 1};
  if (layer === 3) return {key, name: part ? `places ${part}` : "places", kind: "places", order: 1};
  if (layer === 20) return {key, name: "mesh places", kind: "places", order: 1};
  if (layer === 1) return {key, name: "segments", kind: "segments", order: 0};
  if (layer === 4) return {key, name: "rooms", kind: "rooms", order: 2};
  if (layer === 5) return {key, name: "buildings", kind: "buildings", order: 3};
  return {key, name: `layer ${key}`, kind: "other", order: 2};
}

/** Whether a layer is hidden: its own "layer/part" entry, else its kind's
 *  ("kind:places"). */
export function layerHidden(hidden: Record<string, boolean>, layer: number, part: number): boolean {
  const i = layerInfo(layer, part);
  return hidden[i.key] ?? hidden[`kind:${i.kind}`] ?? false;
}

export class GraphModel {
  nodes = new Map<string, NodeRecord>();
  edges = new Map<string, Edge>();
  labelspaces: Labelspaces = {};
  /** monotonically increasing on every change */
  rev = 0;
  lastSeq: number | null = null;

  static fromSnapshot(g: DecodedGraph): GraphModel {
    const m = new GraphModel();
    m.loadSnapshot(g);
    return m;
  }

  loadSnapshot(g: DecodedGraph) {
    this.nodes = new Map(g.nodes.map(n => [n.sym, n]));
    this.edges = new Map(g.edges.map(e => [edgeKey(e[0], e[1]), e]));
    this.labelspaces = g.labelspaces ?? {};
    this.rev++;
  }

  applyDelta(d: DecodedDelta) {
    for (const r of d.upserts) this.nodes.set(r.sym, r);
    for (const s of d.deletes) this.nodes.delete(s);
    for (const e of d.edgesDel) this.edges.delete(edgeKey(e[0], e[1]));
    for (const e of d.edgesAdd) this.edges.set(edgeKey(e[0], e[1]), e);
    if (d.deletes.length) {
      const gone = new Set(d.deletes);
      for (const [k, e] of this.edges) if (gone.has(e[0]) || gone.has(e[1])) this.edges.delete(k);
    }
    if (d.labelspaces && Object.keys(d.labelspaces).length) this.labelspaces = d.labelspaces;
    if (d.seq !== undefined) this.lastSeq = d.seq;
    this.rev++;
  }

  labelName(n: NodeRecord): string {
    if (n.label < 0) return "";
    const ls = this.labelspaces[`${n.layer}/${n.part}`] ?? this.labelspaces[`${n.layer}/0`];
    return ls?.[String(n.label)] ?? `label ${n.label}`;
  }

  /** Display text used for search: name, label name and symbol. */
  displayName(n: NodeRecord): string {
    const lbl = this.labelName(n);
    if (n.name && lbl && n.name !== lbl) return `${n.name} (${lbl})`;
    return n.name || lbl || n.sym;
  }

  layers(): Map<string, {info: LayerInfo; count: number}> {
    const out = new Map<string, {info: LayerInfo; count: number}>();
    for (const n of this.nodes.values()) {
      const key = `${n.layer}/${n.part}`;
      const cur = out.get(key);
      if (cur) cur.count++;
      else out.set(key, {info: layerInfo(n.layer, n.part), count: 1});
    }
    return out;
  }
}

/** Nearest node of a place layer to `p` (xy distance weighted, z de-emphasised). */
/** An agent (trajectory keyframe) node: layer 2 in partition 'a', or in its
 *  robot-prefix partition (sym starts with that character); mirrors the
 *  backend's ``is_agent_record``. */
export function isAgentNode(n: Pick<NodeRecord, "layer" | "part" | "sym">): boolean {
  if (n.layer !== 2) return false;
  if (n.part === 97) return true;
  return n.part !== 0 && n.part !== 115 && !!n.sym && n.sym[0] === String.fromCharCode(n.part);
}

export function nearestPlace(
  model: GraphModel | null | undefined,
  p: [number, number, number],
  opts: {maxDist?: number; kinds?: LayerInfo["kind"][]} = {},
): NodeRecord | null {
  if (!model) return null;
  const kinds = opts.kinds ?? ["places"];
  const maxD2 = (opts.maxDist ?? Infinity) ** 2;
  let best: NodeRecord | null = null;
  let bestD = Infinity;
  for (const n of model.nodes.values()) {
    if (!kinds.includes(layerInfo(n.layer, n.part).kind) || isAgentNode(n)) continue;
    const dx = n.pos[0] - p[0];
    const dy = n.pos[1] - p[1];
    const dz = (n.pos[2] - p[2]) * 0.25;
    const d = dx * dx + dy * dy + dz * dz;
    if (d < bestD) {
      bestD = d;
      best = n;
    }
  }
  return best && bestD <= maxD2 ? best : null;
}

/** Search nodes by a free-text query against names/labels/symbols. */
export function searchNodes(model: GraphModel | null | undefined, query: string, limit = 50): NodeRecord[] {
  if (!model) return [];
  const q = query.trim().toLowerCase();
  if (!q) return [];
  const hits: [number, NodeRecord][] = [];
  for (const n of model.nodes.values()) {
    const text = `${model.displayName(n)} ${n.sym}`.toLowerCase();
    const idx = text.indexOf(q);
    if (idx < 0) continue;
    const kindBias = layerInfo(n.layer, n.part).kind === "objects" ? 0 : 100;
    hits.push([idx + kindBias, n]);
  }
  hits.sort((a, b) => a[0] - b[0] || a[1].sym.localeCompare(b[1].sym, undefined, {numeric: true}));
  return hits.slice(0, limit).map(h => h[1]);
}
